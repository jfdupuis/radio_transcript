// sdrmon/dsp/decimator.cc
// Polyphase-style integer decimator with windowed-sinc FIR anti-aliasing.

#include "decimator.h"

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace sdrmon::dsp {

namespace {

// Blackman window for minimising side-lobe leakage.
static float blackman(int n, int N)
{
    const double pi = 3.14159265358979;
    double a = static_cast<double>(n) / (N - 1);
    return static_cast<float>(0.42 - 0.5 * std::cos(2.0 * pi * a) +
                               0.08 * std::cos(4.0 * pi * a));
}

// Windowed-sinc low-pass FIR with cutoff at normalised frequency w_c (0..1).
// N is the filter order; returns N+1 coefficients (symmetric).
static std::vector<float> design_lpf(int N, double w_c)
{
    std::vector<float> h(N + 1);
    double sum = 0.0;
    int half = N / 2;
    for (int i = 0; i <= N; ++i) {
        int n = i - half;
        double sinc = (n == 0) ? 1.0 : std::sin(M_PI * w_c * n) / (M_PI * n);
        h[i] = static_cast<float>(sinc * blackman(i, N + 1));
        sum += h[i];
    }
    // Normalise DC gain to 1.0
    for (auto &c : h) c /= static_cast<float>(sum);
    return h;
}

} // namespace

Decimator::Decimator(uint32_t from_rate, uint32_t to_rate)
{
    if (from_rate == 0 || to_rate == 0 || from_rate % to_rate != 0)
        throw std::invalid_argument(
            "Decimator: from_rate must be a non-zero integer multiple of to_rate. "
            "Got " + std::to_string(from_rate) + " / " + std::to_string(to_rate));

    factor_ = from_rate / to_rate;
    design_filter(factor_);
    delay_.assign(coeffs_.size(), 0.0f);
}

void Decimator::design_filter(uint32_t factor)
{
    // Cutoff at 0.9 * Nyquist of output (leaves 10% guard band).
    double cutoff = 0.9 / static_cast<double>(factor);
    // Filter order: 64 * factor gives reasonable stopband attenuation.
    int order = static_cast<int>(64 * factor);
    if (order > 1024) order = 1024;  // cap for very large decimation ratios
    coeffs_ = design_lpf(order, cutoff);
}

std::vector<float> Decimator::process(const std::vector<float> &in)
{
    std::vector<float> out;
    out.reserve(in.size() / factor_ + 1);

    const int N = static_cast<int>(coeffs_.size());

    for (float sample : in) {
        // Push sample into delay line (circular: prepend to front by shifting).
        // Use a simple shift register; acceptable for typical buffer sizes.
        delay_.insert(delay_.begin(), sample);
        if (static_cast<int>(delay_.size()) > N)
            delay_.resize(N);

        ++phase_;
        if (phase_ >= factor_) {
            phase_ = 0;
            // Compute FIR output.
            float y = 0.0f;
            int   sz = static_cast<int>(delay_.size());
            for (int k = 0; k < std::min(N, sz); ++k)
                y += coeffs_[k] * delay_[k];
            out.push_back(y);
        }
    }
    return out;
}

void Decimator::reset()
{
    std::fill(delay_.begin(), delay_.end(), 0.0f);
    phase_ = 0;
}

} // namespace sdrmon::dsp
