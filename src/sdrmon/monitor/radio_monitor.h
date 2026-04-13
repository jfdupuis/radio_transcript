#pragma once
// sdrmon/monitor/radio_monitor.h
// Top-level orchestrator: RTL-SDR → FM demod → decimation → decoders + VAD.

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include "sdrmon/monitor/channel_config.h"
#include "sdrmon/monitor/radio_event.h"
#include "sdrmon/monitor/transcript_engine.h"

namespace sdrmon {

// EventCallback is called from a background thread for every decoded event.
using EventCallback = std::function<void(RadioEvent)>;

class RadioMonitor {
public:
    explicit RadioMonitor(MonitorConfig config);
    ~RadioMonitor();

    RadioMonitor(const RadioMonitor &)            = delete;
    RadioMonitor &operator=(const RadioMonitor &) = delete;

    // Optionally attach a speech-to-text backend before calling start().
    void set_transcript_engine(std::shared_ptr<TranscriptEngine> engine);

    // Register the event callback (must be set before start()).
    void set_event_callback(EventCallback cb);

    // Open the RTL-SDR device and begin streaming.
    // The device index from config.channel is used; index defaults to 0.
    void start(uint32_t device_index = 0);

    // Gracefully stop streaming and close the device.
    void stop();

    bool is_running() const { return running_.load(); }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::atomic<bool>     running_{false};
};

} // namespace sdrmon
