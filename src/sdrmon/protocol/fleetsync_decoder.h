#pragma once
// sdrmon/protocol/fleetsync_decoder.h
// C++ wrapper around the Kenwood FleetSync I/II C decoder library.

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace sdrmon::protocol {

// Data decoded from a single FleetSync packet.
struct FleetSyncPacket {
    int  cmd;          // command byte (upper 7 bits of msg[1])
    int  subcmd;       // sub-command (msg[0] & 0xfe) — valid when cmd==0x42
    int  from_fleet;   // -1 = broadcast
    int  from_unit;    // -1 = broadcast
    int  to_fleet;     // -1 = broadcast
    int  to_unit;      // -1 = broadcast
    bool all_call;     // all-call flag

    // Payload bytes (only present for cmd == 0x42).
    int           payload_len;
    uint8_t       payload[128];

    // GPS fix — valid only when subcmd == 0xf6 and gps_valid == true.
    bool   gps_valid;
    double latitude;
    double longitude;

    bool is_fsync2; // decoded via FleetSync II path
    bool is_2400;   // decoded at 2400 bps variant
};

// Callback type: invoked for every successfully decoded FleetSync packet.
using FleetSyncCallback = std::function<void(const FleetSyncPacket &)>;

// Decoder expects 8000 Hz unsigned 8-bit mono audio.
class FleetSyncDecoder {
public:
    explicit FleetSyncDecoder(int sample_rate = 8000);
    ~FleetSyncDecoder();

    FleetSyncDecoder(const FleetSyncDecoder &)            = delete;
    FleetSyncDecoder &operator=(const FleetSyncDecoder &) = delete;

    void set_callback(FleetSyncCallback cb);

    // Feed raw audio samples (unsigned 8-bit, sample_rate Hz).
    void process(const uint8_t *samples, int count);

    // Flush any pending state (call at end of stream).
    void flush();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sdrmon::protocol
