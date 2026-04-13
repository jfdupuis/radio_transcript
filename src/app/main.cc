// sdrmon console application
// Monitors one radio channel and prints events to stdout as JSON.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "sdrmon/monitor/channel_config.h"
#include "sdrmon/monitor/radio_event.h"
#include "sdrmon/monitor/radio_monitor.h"
#include "sdrmon/rtlsdr/rtlsdr_device.h"

static std::atomic<bool> g_stop{false};

static void signal_handler(int /*sig*/)
{
    g_stop.store(true);
}

// ---- JSON helpers -----------------------------------------------------------

static std::string json_str(const std::string &s)
{
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += static_cast<char>(c);
            }
        }
    }
    out += '"';
    return out;
}

static std::string timestamp_str(sdrmon::TimePoint tp)
{
    auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp.time_since_epoch()).count();
    return std::to_string(epoch_ms);
}

static std::string hex_str(const uint8_t *data, int len)
{
    std::ostringstream oss;
    oss << "\"0x";
    oss << std::hex << std::setfill('0');
    for (int i = 0; i < len; ++i)
        oss << std::setw(2) << static_cast<unsigned>(data[i]);
    oss << '"';
    return oss.str();
}

// ---- Event formatters -------------------------------------------------------

static std::string format_fleetsync(const sdrmon::FleetSyncEvent &ev)
{
    const auto &p = ev.packet;
    std::ostringstream j;
    j << "{"
      << "\"type\":\"FLEETSYNC\","
      << "\"timestamp_ms\":" << timestamp_str(ev.timestamp) << ","
      << "\"freq_hz\":"       << ev.frequency_hz << ","
      << "\"cmd\":"           << p.cmd << ","
      << "\"subcmd\":"        << p.subcmd << ","
      << "\"from_fleet\":"    << p.from_fleet << ","
      << "\"from_unit\":"     << p.from_unit << ","
      << "\"to_fleet\":"      << p.to_fleet << ","
      << "\"to_unit\":"       << p.to_unit << ","
      << "\"all_call\":"      << (p.all_call ? "true" : "false") << ","
      << "\"payload\":"       << hex_str(p.payload, p.payload_len) << ","
      << "\"fsync2\":"        << (p.is_fsync2 ? "true" : "false") << ","
      << "\"rate_2400\":"     << (p.is_2400   ? "true" : "false");

    if (p.gps_valid) {
        j << ",\"lat\":"  << std::fixed << std::setprecision(5) << p.latitude
          << ",\"lon\":"  << std::fixed << std::setprecision(5) << p.longitude;
    }
    j << "}";
    return j.str();
}

static std::string format_mdc1200(const sdrmon::Mdc1200Event &ev)
{
    const auto &p = ev.packet;
    char buf[512];
    if (p.num_frames == 2) {
        std::snprintf(buf, sizeof(buf),
            "{\"type\":\"MDC1200\","
            "\"timestamp_ms\":%s,"
            "\"freq_hz\":%u,"
            "\"op\":\"0x%02x\","
            "\"arg\":\"0x%02x\","
            "\"unit_id\":\"0x%04x\","
            "\"extra0\":\"0x%02x\","
            "\"extra1\":\"0x%02x\","
            "\"extra2\":\"0x%02x\","
            "\"extra3\":\"0x%02x\"}",
            timestamp_str(ev.timestamp).c_str(),
            ev.frequency_hz,
            p.op, p.arg, p.unit_id,
            p.extra0, p.extra1, p.extra2, p.extra3);
    } else {
        std::snprintf(buf, sizeof(buf),
            "{\"type\":\"MDC1200\","
            "\"timestamp_ms\":%s,"
            "\"freq_hz\":%u,"
            "\"op\":\"0x%02x\","
            "\"arg\":\"0x%02x\","
            "\"unit_id\":\"0x%04x\"}",
            timestamp_str(ev.timestamp).c_str(),
            ev.frequency_hz,
            p.op, p.arg, p.unit_id);
    }
    return buf;
}

static std::string format_voice_start(const sdrmon::VoiceStartEvent &ev)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"type\":\"VOICE_START\","
        "\"timestamp_ms\":%s,"
        "\"freq_hz\":%u}",
        timestamp_str(ev.timestamp).c_str(), ev.frequency_hz);
    return buf;
}

static std::string format_voice_end(const sdrmon::VoiceEndEvent &ev)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"type\":\"VOICE_END\","
        "\"timestamp_ms\":%s,"
        "\"freq_hz\":%u,"
        "\"duration_sec\":%.3f}",
        timestamp_str(ev.timestamp).c_str(),
        ev.frequency_hz, ev.duration_sec);
    return buf;
}

static std::string format_transcript(const sdrmon::TranscriptEvent &ev)
{
    std::ostringstream j;
    j << "{"
      << "\"type\":\"TRANSCRIPT\","
      << "\"timestamp_ms\":" << timestamp_str(ev.timestamp) << ","
      << "\"freq_hz\":"       << ev.frequency_hz << ","
      << "\"text\":"          << json_str(ev.text) << ","
      << "\"confidence\":"    << std::fixed << std::setprecision(3) << ev.confidence
      << "}";
    return j.str();
}

static void print_event(const sdrmon::RadioEvent &ev)
{
    std::string json;
    std::visit([&json](const auto &e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, sdrmon::FleetSyncEvent>)
            json = format_fleetsync(e);
        else if constexpr (std::is_same_v<T, sdrmon::Mdc1200Event>)
            json = format_mdc1200(e);
        else if constexpr (std::is_same_v<T, sdrmon::VoiceStartEvent>)
            json = format_voice_start(e);
        else if constexpr (std::is_same_v<T, sdrmon::VoiceEndEvent>)
            json = format_voice_end(e);
        else if constexpr (std::is_same_v<T, sdrmon::TranscriptEvent>)
            json = format_transcript(e);
    }, ev);
    std::cout << json << '\n' << std::flush;
}

// ---- CLI parsing ------------------------------------------------------------

static void print_usage(const char *prog)
{
    std::cerr
        << "Usage: " << prog << " [OPTIONS]\n\n"
        << "Options:\n"
        << "  -f <Hz>       Center frequency in Hz (required)\n"
        << "  -r <Hz>       RTL-SDR sample rate (default: 240000)\n"
        << "  -d <idx>      Device index (default: 0)\n"
        << "  -p <ppm>      Frequency correction in ppm (default: 0)\n"
        << "  -g <gain>     Tuner gain in tenths of dB, e.g. 496 = 49.6 dB\n"
        << "                (default: automatic)\n"
        << "  -s <level>    Squelch RMS level for VAD (default: 0.005)\n"
        << "  -l <label>    Channel label for display\n"
        << "  --no-fsync    Disable FleetSync decoder\n"
        << "  --no-mdc      Disable MDC1200 decoder\n"
        << "  --list        List available RTL-SDR devices and exit\n"
        << "  -h, --help    Show this help\n\n"
        << "Example (fire dispatch at 154.310 MHz):\n"
        << "  " << prog << " -f 154310000 -g 400 -l \"Fire Dispatch\"\n\n"
        << "Output is newline-delimited JSON on stdout.\n\n";
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Defaults
    uint32_t    freq_hz    = 0;
    uint32_t    sample_rate = 240000;
    uint32_t    device_idx = 0;
    int         ppm        = 0;
    int         gain       = -1;   // auto
    float       squelch    = 0.005f;
    std::string label;
    bool        do_fsync   = true;
    bool        do_mdc     = true;
    bool        list_devs  = false;

    // Simple argument parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> const char * {
            if (i + 1 >= argc) {
                std::cerr << "Missing argument for " << arg << "\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "-f")          freq_hz     = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "-r")     sample_rate = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "-d")     device_idx  = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "-p")     ppm         = std::stoi(next());
        else if (arg == "-g")     gain        = std::stoi(next());
        else if (arg == "-s")     squelch     = std::stof(next());
        else if (arg == "-l")     label       = next();
        else if (arg == "--no-fsync") do_fsync = false;
        else if (arg == "--no-mdc")   do_mdc   = false;
        else if (arg == "--list")     list_devs = true;
        else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // List devices if requested
    if (list_devs) {
        auto devs = sdrmon::RtlSdrDevice::enumerate();
        if (devs.empty()) {
            std::cout << "No RTL-SDR devices found.\n";
        } else {
            std::cout << devs.size() << " device(s):\n";
            for (const auto &d : devs)
                std::cout << "  [" << d.index << "] " << d.name
                          << "  mfg=" << d.manufacturer
                          << "  prod=" << d.product
                          << "  serial=" << d.serial << "\n";
        }
        return 0;
    }

    if (freq_hz == 0) {
        std::cerr << "Error: center frequency (-f) is required.\n\n";
        print_usage(argv[0]);
        return 1;
    }

    // Validate that sample_rate is divisible by both target rates.
    constexpr uint32_t FSYNC_RATE = 8000;
    constexpr uint32_t VOICE_RATE = 48000;
    if (sample_rate % FSYNC_RATE != 0 || sample_rate % VOICE_RATE != 0) {
        std::cerr << "Error: sample_rate (" << sample_rate
                  << ") must be divisible by " << FSYNC_RATE
                  << " and " << VOICE_RATE << ".\n"
                  << "Suggested rates: 240000 (÷30 and ÷5)\n";
        return 1;
    }

    if (label.empty()) label = std::to_string(freq_hz) + " Hz";

    // Build configuration
    sdrmon::MonitorConfig cfg;
    cfg.channel.frequency_hz       = freq_hz;
    cfg.channel.sample_rate        = sample_rate;
    cfg.channel.label              = label;
    cfg.channel.freq_correction_ppm = ppm;
    cfg.channel.gain_tenth_db      = gain;
    cfg.channel.squelch_level      = squelch;
    cfg.channel.decode_fleetsync   = do_fsync;
    cfg.channel.decode_mdc1200     = do_mdc;
    cfg.fleetsync_rate             = FSYNC_RATE;
    cfg.voice_rate                 = VOICE_RATE;

    // Banner
    std::cerr << "sdrmon — Software Defined Radio Monitor\n"
              << "  Channel : " << label << "\n"
              << "  Frequency: " << freq_hz / 1e6 << " MHz\n"
              << "  Sample rate: " << sample_rate << " Hz\n"
              << "  Device index: " << device_idx << "\n"
              << "  FleetSync: " << (do_fsync ? "enabled" : "disabled") << "\n"
              << "  MDC1200  : " << (do_mdc   ? "enabled" : "disabled") << "\n"
              << "  Press Ctrl-C to stop.\n\n";

    try {
        sdrmon::RadioMonitor monitor(cfg);
        monitor.set_event_callback(print_event);
        monitor.start(device_idx);

        while (!g_stop.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cerr << "\nStopping...\n";
        monitor.stop();
    } catch (const std::exception &ex) {
        std::cerr << "Fatal error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
