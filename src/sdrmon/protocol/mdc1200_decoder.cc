// sdrmon/protocol/mdc1200_decoder.cc

#include "mdc1200_decoder.h"

#include <stdexcept>

// Third-party MDC1200 decoder (GPL-2.0)
extern "C" {
#include "mdc_decode.h"
}

namespace sdrmon::protocol {

struct Mdc1200Decoder::Impl {
    mdc_decoder_t *decoder = nullptr;
    Mdc1200Callback user_cb;

    static void mdc_cb(int numFrames,
                        unsigned char op,   unsigned char arg,
                        unsigned short unitID,
                        unsigned char extra0, unsigned char extra1,
                        unsigned char extra2, unsigned char extra3,
                        void *context)
    {
        auto *self = static_cast<Impl *>(context);
        if (!self->user_cb) return;

        Mdc1200Packet pkt{};
        pkt.num_frames = numFrames;
        pkt.op         = op;
        pkt.arg        = arg;
        pkt.unit_id    = unitID;
        pkt.extra0     = extra0;
        pkt.extra1     = extra1;
        pkt.extra2     = extra2;
        pkt.extra3     = extra3;
        self->user_cb(pkt);
    }
};

Mdc1200Decoder::Mdc1200Decoder(int sample_rate)
    : impl_(std::make_unique<Impl>())
{
    impl_->decoder = mdc_decoder_new(sample_rate);
    if (!impl_->decoder)
        throw std::runtime_error("MDC1200: failed to allocate decoder");
    mdc_decoder_set_callback(impl_->decoder, Impl::mdc_cb, impl_.get());
}

Mdc1200Decoder::~Mdc1200Decoder()
{
    if (impl_->decoder) {
        free(impl_->decoder);
        impl_->decoder = nullptr;
    }
}

void Mdc1200Decoder::set_callback(Mdc1200Callback cb)
{
    impl_->user_cb = std::move(cb);
}

void Mdc1200Decoder::process(const uint8_t *samples, int count)
{
    mdc_decoder_process_samples(
        impl_->decoder,
        const_cast<mdc_sample_t *>(samples),
        count);
}

} // namespace sdrmon::protocol
