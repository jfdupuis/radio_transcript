#pragma once
// sdrmon/monitor/radio_event.h
// Typed events emitted by RadioMonitor.

#include <chrono>
#include <cstdint>
#include <string>
#include <variant>

#include "sdrmon/protocol/fleetsync_decoder.h"
#include "sdrmon/protocol/mdc1200_decoder.h"

namespace sdrmon {

using Clock     = std::chrono::system_clock;
using TimePoint = Clock::time_point;

// A FleetSync packet event.
struct FleetSyncEvent {
    TimePoint                  timestamp;
    uint32_t                   frequency_hz;
    protocol::FleetSyncPacket  packet;
};

// An MDC1200 packet event.
struct Mdc1200Event {
    TimePoint                 timestamp;
    uint32_t                  frequency_hz;
    protocol::Mdc1200Packet   packet;
};

// Voice activity started on a channel.
struct VoiceStartEvent {
    TimePoint timestamp;
    uint32_t  frequency_hz;
};

// Voice activity ended on a channel.
struct VoiceEndEvent {
    TimePoint   timestamp;
    uint32_t    frequency_hz;
    float       duration_sec;
};

// Speech-to-text transcript for a completed voice segment.
struct TranscriptEvent {
    TimePoint   timestamp;
    uint32_t    frequency_hz;
    std::string text;
    float       confidence; // 0..1, -1 if not available
};

// Discriminated union of all possible events.
using RadioEvent = std::variant<
    FleetSyncEvent,
    Mdc1200Event,
    VoiceStartEvent,
    VoiceEndEvent,
    TranscriptEvent>;

} // namespace sdrmon
