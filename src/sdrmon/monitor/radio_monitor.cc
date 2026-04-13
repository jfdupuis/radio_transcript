// sdrmon/monitor/radio_monitor.cc

#include "sdrmon/monitor/radio_monitor.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "sdrmon/dsp/decimator.h"
#include "sdrmon/dsp/fm_demodulator.h"
#include "sdrmon/protocol/fleetsync_decoder.h"
#include "sdrmon/protocol/mdc1200_decoder.h"
#include "sdrmon/rtlsdr/rtlsdr_device.h"
#include "sdrmon/monitor/channel_config.h"
#include "sdrmon/monitor/radio_event.h"
#include "sdrmon/monitor/transcript_engine.h"

namespace sdrmon {

// ---------------------------------------------------------------------------
// Voice Activity Detector (energy-based)
// ---------------------------------------------------------------------------
class VoiceActivityDetector {
public:
    explicit VoiceActivityDetector(float sample_rate,
                                    float window_sec,
                                    float hangover_sec,
                                    float threshold)
        : window_samples_(static_cast<int>(sample_rate * window_sec)),
          hangover_samples_(static_cast<int>(sample_rate * hangover_sec)),
          threshold_(threshold)
    {}

    // Returns true on level transition: false→active (voice start) or
    // active→inactive (voice end).  Use active() to read current state.
    struct Update {
        bool start_event;  // just went active
        bool end_event;    // just went inactive
        bool active;
    };

    Update push(float sample)
    {
        Update u{false, false, active_};

        // Accumulate power.
        window_acc_ += sample * sample;
        window_count_++;

        if (window_count_ >= window_samples_) {
            float rms = std::sqrt(window_acc_ / static_cast<float>(window_count_));
            window_acc_   = 0.0f;
            window_count_ = 0;

            if (rms >= threshold_) {
                hangover_ = hangover_samples_;
                if (!active_) {
                    active_ = true;
                    u.active      = true;
                    u.start_event = true;
                }
            } else if (hangover_ > 0) {
                hangover_ -= window_samples_;
                if (!active_) {
                    active_ = true;
                    // extending an existing segment is not a new start
                }
            } else if (active_) {
                active_      = false;
                u.active     = false;
                u.end_event  = true;
            }
        }
        return u;
    }

    bool active() const { return active_; }
    void reset()
    {
        active_       = false;
        window_acc_   = 0.0f;
        window_count_ = 0;
        hangover_     = 0;
    }

private:
    int   window_samples_;
    int   hangover_samples_;
    float threshold_;

    bool  active_       = false;
    float window_acc_   = 0.0f;
    int   window_count_ = 0;
    int   hangover_     = 0;
};

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct RadioMonitor::Impl {
    MonitorConfig  cfg;
    EventCallback  event_cb;
    std::shared_ptr<TranscriptEngine> transcript_engine;

    std::unique_ptr<RtlSdrDevice>             device;
    std::thread                               reader_thread;

    // DSP components (one per path)
    dsp::FmDemodulator                        fm_demod;
    std::unique_ptr<dsp::Decimator>           dec_fsync; // → 8000 Hz
    std::unique_ptr<dsp::Decimator>           dec_voice; // → voice_rate

    // Protocol decoders
    std::unique_ptr<protocol::FleetSyncDecoder> fsync_dec;
    std::unique_ptr<protocol::Mdc1200Decoder>   mdc_dec;

    // VAD state
    std::unique_ptr<VoiceActivityDetector> vad;

    // Voice segment accumulator for transcription
    std::vector<float>     voice_buf;
    TimePoint              voice_start_time;

    void emit(RadioEvent ev)
    {
        if (event_cb) event_cb(std::move(ev));
    }

    void on_iq(const uint8_t *buf, uint32_t len)
    {
        // 1. FM demodulate.
        auto audio = fm_demod.process(buf, len);

        // 2a. FleetSync + MDC path: decimate to 8000 Hz, convert to uint8.
        if (cfg.channel.decode_fleetsync || cfg.channel.decode_mdc1200) {
            auto audio8k = dec_fsync->process(audio);
            std::vector<uint8_t> u8(audio8k.size());
            for (size_t i = 0; i < audio8k.size(); ++i) {
                float s = audio8k[i];
                s = std::min(std::max(s, -1.0f), 1.0f);
                u8[i] = static_cast<uint8_t>((s + 1.0f) * 127.5f);
            }
            if (cfg.channel.decode_fleetsync)
                fsync_dec->process(u8.data(), static_cast<int>(u8.size()));
            if (cfg.channel.decode_mdc1200)
                mdc_dec->process(u8.data(), static_cast<int>(u8.size()));
        }

        // 2b. Voice path: decimate to voice_rate, run VAD + optional STT.
        {
            auto voice_audio = dec_voice->process(audio);
            auto now         = Clock::now();

            for (float s : voice_audio) {
                auto u = vad->push(s);

                if (u.start_event) {
                    voice_start_time = now;
                    voice_buf.clear();
                    emit(VoiceStartEvent{now, cfg.channel.frequency_hz});
                }

                if (vad->active())
                    voice_buf.push_back(s);

                if (u.end_event) {
                    float dur = static_cast<float>(voice_buf.size()) /
                                static_cast<float>(cfg.voice_rate);
                    emit(VoiceEndEvent{now, cfg.channel.frequency_hz, dur});

                    if (cfg.channel.transcribe_voice && transcript_engine) {
                        auto buf_copy = voice_buf;
                        uint32_t rate = cfg.voice_rate;
                        uint32_t freq = cfg.channel.frequency_hz;
                        transcript_engine->transcribe(
                            buf_copy, rate,
                            [this, now, freq](const std::string &text, float conf) {
                                emit(TranscriptEvent{now, freq, text, conf});
                            });
                    }
                    voice_buf.clear();
                }
            }
        }
    }
};

// ---------------------------------------------------------------------------
// RadioMonitor
// ---------------------------------------------------------------------------
RadioMonitor::RadioMonitor(MonitorConfig config)
    : impl_(std::make_unique<Impl>())
{
    impl_->cfg = std::move(config);
}

RadioMonitor::~RadioMonitor()
{
    if (is_running()) stop();
}

void RadioMonitor::set_transcript_engine(
    std::shared_ptr<TranscriptEngine> engine)
{
    impl_->transcript_engine = std::move(engine);
}

void RadioMonitor::set_event_callback(EventCallback cb)
{
    impl_->event_cb = std::move(cb);
}

void RadioMonitor::start(uint32_t device_index)
{
    if (running_.load())
        throw std::runtime_error("RadioMonitor already running");

    const MonitorConfig &cfg = impl_->cfg;

    // Validate decimation ratios.
    if (cfg.channel.sample_rate % cfg.fleetsync_rate != 0)
        throw std::invalid_argument("sample_rate must be divisible by fleetsync_rate");
    if (cfg.channel.sample_rate % cfg.voice_rate != 0)
        throw std::invalid_argument("sample_rate must be divisible by voice_rate");

    // Build DSP chain.
    impl_->fm_demod  = dsp::FmDemodulator{};
    impl_->dec_fsync = std::make_unique<dsp::Decimator>(
        cfg.channel.sample_rate, cfg.fleetsync_rate);
    impl_->dec_voice = std::make_unique<dsp::Decimator>(
        cfg.channel.sample_rate, cfg.voice_rate);

    // Protocol decoders.
    if (cfg.channel.decode_fleetsync) {
        impl_->fsync_dec = std::make_unique<protocol::FleetSyncDecoder>(
            static_cast<int>(cfg.fleetsync_rate));
        impl_->fsync_dec->set_callback([this](const protocol::FleetSyncPacket &pkt) {
            impl_->emit(FleetSyncEvent{Clock::now(),
                                       impl_->cfg.channel.frequency_hz,
                                       pkt});
        });
    }

    if (cfg.channel.decode_mdc1200) {
        impl_->mdc_dec = std::make_unique<protocol::Mdc1200Decoder>(
            static_cast<int>(cfg.fleetsync_rate));
        impl_->mdc_dec->set_callback([this](const protocol::Mdc1200Packet &pkt) {
            impl_->emit(Mdc1200Event{Clock::now(),
                                      impl_->cfg.channel.frequency_hz,
                                      pkt});
        });
    }

    // VAD.
    impl_->vad = std::make_unique<VoiceActivityDetector>(
        static_cast<float>(cfg.voice_rate),
        cfg.vad_window_sec,
        cfg.vad_hangover_sec,
        cfg.channel.squelch_level);

    // Open RTL-SDR device.
    impl_->device = std::make_unique<RtlSdrDevice>(device_index);
    impl_->device->set_center_freq(cfg.channel.frequency_hz);
    impl_->device->set_sample_rate(cfg.channel.sample_rate);
    impl_->device->set_freq_correction(cfg.channel.freq_correction_ppm);

    if (cfg.channel.gain_tenth_db < 0) {
        impl_->device->set_gain_mode(false);
        impl_->device->set_agc_mode(true);
    } else {
        impl_->device->set_gain_mode(true);
        impl_->device->set_gain(cfg.channel.gain_tenth_db);
        impl_->device->set_agc_mode(false);
    }

    running_.store(true);

    // rtlsdr_read_async blocks, so spin it in a dedicated thread.
    impl_->reader_thread = std::thread([this]() {
        impl_->device->start(
            [this](const uint8_t *buf, uint32_t len) {
                impl_->on_iq(buf, len);
            },
            impl_->cfg.rtlsdr_buf_num,
            impl_->cfg.rtlsdr_buf_len);
        running_.store(false);
    });
}

void RadioMonitor::stop()
{
    if (!running_.load()) return;
    if (impl_->device)
        impl_->device->stop();
    if (impl_->reader_thread.joinable())
        impl_->reader_thread.join();
    running_.store(false);
}

} // namespace sdrmon
