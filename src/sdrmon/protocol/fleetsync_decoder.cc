// sdrmon/protocol/fleetsync_decoder.cc

#include "fleetsync_decoder.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>

// Third-party FleetSync decoder (GPL-2.0)
#include "fsync_decode.h"

namespace sdrmon::protocol {

// GPS payload decode — empirically-derived constants from FLEETSYNC_FORMAT.md.
static bool decode_gps(const uint8_t *payload, int len,
                        double &lat, double &lon)
{
    if (len < 15)       return false;
    if (payload[0] != 0x01) return false; // validity flag

    // Latitude: unsigned LE uint32 at [7:11].
    uint32_t enc_lat = static_cast<uint32_t>(payload[7]) |
                       (static_cast<uint32_t>(payload[8])  <<  8) |
                       (static_cast<uint32_t>(payload[9])  << 16) |
                       (static_cast<uint32_t>(payload[10]) << 24);

    // Longitude: signed LE int32 at [1:5].
    int32_t enc_lon = static_cast<int32_t>(
                          static_cast<uint32_t>(payload[1]) |
                         (static_cast<uint32_t>(payload[2]) <<  8) |
                         (static_cast<uint32_t>(payload[3]) << 16) |
                         (static_cast<uint32_t>(payload[4]) << 24));

    lat = (static_cast<double>(enc_lat) - 212806563.0) / 1109091.0;
    lon = (static_cast<double>(enc_lon) - 57556364.176) / 541176.471;
    return true;
}

// ---- Impl (hides fsync_decoder_t from public header) -------------------------

struct FleetSyncDecoder::Impl {
    fsync_decoder_t *decoder = nullptr;
    FleetSyncCallback user_cb;

    static void fsync_cb(int cmd, int subcmd,
                          int from_fleet, int from_unit,
                          int to_fleet,   int to_unit,
                          int allflag,
                          unsigned char *payload, int payload_len,
                          unsigned char * /*raw_msg*/, int /*raw_msg_len*/,
                          void *context,
                          int is_fsync2, int is_2400)
    {
        auto *self = static_cast<Impl *>(context);
        if (!self->user_cb) return;

        FleetSyncPacket pkt{};
        pkt.cmd        = cmd;
        pkt.subcmd     = subcmd;
        pkt.from_fleet = from_fleet;
        pkt.from_unit  = from_unit;
        pkt.to_fleet   = to_fleet;
        pkt.to_unit    = to_unit;
        pkt.all_call   = (allflag != 0);
        pkt.payload_len = std::min(payload_len, 128);
        if (pkt.payload_len > 0)
            std::memcpy(pkt.payload, payload, pkt.payload_len);
        pkt.is_fsync2  = (is_fsync2 != 0);
        pkt.is_2400    = (is_2400   != 0);

        pkt.gps_valid = false;
        if (subcmd == 0xf6)
            pkt.gps_valid = decode_gps(payload, payload_len,
                                        pkt.latitude, pkt.longitude);

        self->user_cb(pkt);
    }
};

// ---- FleetSyncDecoder --------------------------------------------------------

FleetSyncDecoder::FleetSyncDecoder(int sample_rate)
    : impl_(std::make_unique<Impl>())
{
    impl_->decoder = fsync_decoder_new(sample_rate);
    if (!impl_->decoder)
        throw std::runtime_error("FleetSync: failed to allocate decoder");
    fsync_decoder_set_callback(impl_->decoder, Impl::fsync_cb, impl_.get());
}

FleetSyncDecoder::~FleetSyncDecoder()
{
    if (impl_->decoder) {
        free(impl_->decoder);   // fsync_decoder_new uses malloc()
        impl_->decoder = nullptr;
    }
}

void FleetSyncDecoder::set_callback(FleetSyncCallback cb)
{
    impl_->user_cb = std::move(cb);
}

void FleetSyncDecoder::process(const uint8_t *samples, int count)
{
    fsync_decoder_process_samples(
        impl_->decoder,
        const_cast<fsync_sample_t *>(samples),
        count);
}

void FleetSyncDecoder::flush()
{
    fsync_decoder_end_samples(impl_->decoder);
}

} // namespace sdrmon::protocol
