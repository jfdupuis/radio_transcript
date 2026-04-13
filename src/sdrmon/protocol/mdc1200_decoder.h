#pragma once
// sdrmon/protocol/mdc1200_decoder.h
// C++ wrapper around the Motorola MDC1200 C decoder library.

#include <cstdint>
#include <functional>
#include <memory>

namespace sdrmon::protocol {

// Data decoded from a single MDC1200 packet.
struct Mdc1200Packet {
    int            num_frames;  // 1 = single, 2 = double packet
    unsigned char  op;
    unsigned char  arg;
    unsigned short unit_id;
    // Extra bytes — only valid when num_frames == 2.
    unsigned char  extra0;
    unsigned char  extra1;
    unsigned char  extra2;
    unsigned char  extra3;
};

// Callback type invoked for every successfully decoded MDC1200 packet.
using Mdc1200Callback = std::function<void(const Mdc1200Packet &)>;

// Decoder expects 8000 Hz unsigned 8-bit mono audio.
class Mdc1200Decoder {
public:
    explicit Mdc1200Decoder(int sample_rate = 8000);
    ~Mdc1200Decoder();

    Mdc1200Decoder(const Mdc1200Decoder &)            = delete;
    Mdc1200Decoder &operator=(const Mdc1200Decoder &) = delete;

    void set_callback(Mdc1200Callback cb);

    // Feed raw audio samples (unsigned 8-bit, sample_rate Hz).
    void process(const uint8_t *samples, int count);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sdrmon::protocol
