#pragma once
// sdrmon/monitor/channel_config.h
// Per-channel configuration for the RadioMonitor.

#include <cstdint>
#include <string>

namespace sdrmon {

struct ChannelConfig {
    uint32_t    frequency_hz;    // Center frequency to tune to (Hz)
    uint32_t    sample_rate;     // RTL-SDR sample rate (e.g. 240000)
    std::string label;           // Human-readable name (e.g. "Fire Dispatch")
    int         freq_correction_ppm = 0;
    int         gain_tenth_db       = -1;     // -1 = auto-gain
    float       squelch_level       = 0.005f; // RMS threshold for voice activity
    bool        decode_fleetsync    = true;
    bool        decode_mdc1200      = true;
    bool        transcribe_voice    = false;  // requires TranscriptEngine set
};

struct MonitorConfig {
    ChannelConfig channel;

    // Audio DSP settings.
    uint32_t fleetsync_rate  = 8000;    // sample rate fed to FleetSync/MDC decoder
    uint32_t voice_rate      = 48000;   // sample rate fed to voice pipeline
    uint32_t rtlsdr_buf_num  = 0;       // 0 = library default (15)
    uint32_t rtlsdr_buf_len  = 0;       // 0 = library default (~512 kB)

    // Voice activity detection window (seconds).
    float vad_window_sec    = 0.020f;   // 20 ms RMS window
    float vad_hangover_sec  = 0.300f;   // hold-off after signal drops
};

} // namespace sdrmon
