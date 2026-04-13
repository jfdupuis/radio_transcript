#pragma once
// sdrmon/dsp/fm_demodulator.h
// Narrow-band FM demodulator for IQ sample streams.
//
// Input:  interleaved uint8 IQ pairs (from RTL-SDR), each in [0,255].
// Output: float audio samples in [-1, +1], at the same input sample rate.
//
// Use with a decimator to reduce rate after demodulation.

#include <complex>
#include <cstdint>
#include <vector>

namespace sdrmon::dsp {

class FmDemodulator {
public:
    FmDemodulator() = default;

    // Process a raw IQ buffer (interleaved uint8 I, Q pairs).
    // Returns demodulated float audio samples at the same sample rate.
    std::vector<float> process(const uint8_t *buf, uint32_t len);

    void reset();

private:
    std::complex<float> prev_{1.0f, 0.0f};
};

} // namespace sdrmon::dsp
