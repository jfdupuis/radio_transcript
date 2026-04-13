#pragma once
// sdrmon/dsp/decimator.h
// Integer-ratio decimator with a simple windowed-sinc FIR anti-alias filter.
//
// Usage:
//   Decimator dec(from_rate, to_rate);
//   std::vector<float> out = dec.process(audio_in);

#include <cstdint>
#include <vector>

namespace sdrmon::dsp {

class Decimator {
public:
    // Construct a decimator that reduces sample rate by factor = from_rate / to_rate.
    // from_rate must be an integer multiple of to_rate.
    Decimator(uint32_t from_rate, uint32_t to_rate);

    // Process input samples, return decimated output.
    std::vector<float> process(const std::vector<float> &in);

    void reset();

    uint32_t factor() const { return factor_; }

private:
    uint32_t            factor_;
    std::vector<float>  coeffs_;   // FIR coefficients (low-pass anti-alias)
    std::vector<float>  delay_;    // delay line
    uint32_t            phase_ = 0;

    void design_filter(uint32_t factor);
};

} // namespace sdrmon::dsp
