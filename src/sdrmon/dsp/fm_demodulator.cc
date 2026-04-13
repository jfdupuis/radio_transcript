// sdrmon/dsp/fm_demodulator.cc

#include "fm_demodulator.h"

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace sdrmon::dsp {

// Convert unsigned 8-bit IQ sample to complex<float> in [-1, +1].
static inline std::complex<float> iq_to_complex(uint8_t i, uint8_t q)
{
    return { (static_cast<float>(i) - 127.5f) / 127.5f,
             (static_cast<float>(q) - 127.5f) / 127.5f };
}

// FM discriminator:
//   phi_diff = arg( conj(prev) * curr )
//            = atan2(Im[prev* · curr], Re[prev* · curr])
//            = atan2(Ip*Qc - Qp*Ic, Ip*Ic + Qp*Qc)
//
// This gives instantaneous frequency deviation normalised to [-pi, pi].
// We normalise to [-1, 1] by dividing by pi.
static inline float fm_discriminate(std::complex<float> prev,
                                    std::complex<float> curr)
{
    // conj(prev) * curr
    float re = prev.real() * curr.real() + prev.imag() * curr.imag();
    float im = prev.real() * curr.imag() - prev.imag() * curr.real();
    return std::atan2(im, re) * (1.0f / 3.14159265358979f);
}

std::vector<float> FmDemodulator::process(const uint8_t *buf, uint32_t len)
{
    // Each IQ pair = 2 bytes.
    uint32_t n_samples = len / 2;
    std::vector<float> out;
    out.reserve(n_samples);

    for (uint32_t i = 0; i < n_samples; ++i) {
        auto curr = iq_to_complex(buf[2 * i], buf[2 * i + 1]);
        out.push_back(fm_discriminate(prev_, curr));
        prev_ = curr;
    }
    return out;
}

void FmDemodulator::reset()
{
    prev_ = {1.0f, 0.0f};
}

} // namespace sdrmon::dsp
