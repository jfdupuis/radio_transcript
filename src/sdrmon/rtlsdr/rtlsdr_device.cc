// sdrmon/rtlsdr/rtlsdr_device.cc

#include "rtlsdr_device.h"

#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

// System-installed rtl-sdr header (or vendored fallback).
// The vendored header lives in //third_party/rtlsdr.
#include "rtl-sdr.h"

namespace sdrmon {

// ---- enumerate ----------------------------------------------------------------

std::vector<DeviceInfo> RtlSdrDevice::enumerate()
{
    std::vector<DeviceInfo> result;
    uint32_t count = rtlsdr_get_device_count();
    result.reserve(count);

    for (uint32_t i = 0; i < count; ++i) {
        DeviceInfo info;
        info.index = i;
        info.name  = rtlsdr_get_device_name(i);

        char mfg[256] = {}, prod[256] = {}, serial[256] = {};
        rtlsdr_get_device_usb_strings(i, mfg, prod, serial);
        info.manufacturer = mfg;
        info.product      = prod;
        info.serial       = serial;
        result.push_back(std::move(info));
    }
    return result;
}

// ---- ctor / dtor --------------------------------------------------------------

RtlSdrDevice::RtlSdrDevice(uint32_t device_index)
{
    if (rtlsdr_open(&dev_, device_index) != 0)
        throw std::runtime_error("rtlsdr_open failed for index " +
                                 std::to_string(device_index));
}

RtlSdrDevice::~RtlSdrDevice()
{
    if (running_) stop();
    if (dev_)     rtlsdr_close(dev_);
}

RtlSdrDevice::RtlSdrDevice(RtlSdrDevice &&o) noexcept
    : dev_(o.dev_), running_(o.running_), user_cb_(std::move(o.user_cb_))
{
    o.dev_     = nullptr;
    o.running_ = false;
}

RtlSdrDevice &RtlSdrDevice::operator=(RtlSdrDevice &&o) noexcept
{
    if (this != &o) {
        if (running_) stop();
        if (dev_)     rtlsdr_close(dev_);
        dev_     = o.dev_;
        running_ = o.running_;
        user_cb_ = std::move(o.user_cb_);
        o.dev_   = nullptr;
        o.running_ = false;
    }
    return *this;
}

// ---- configuration ------------------------------------------------------------

void RtlSdrDevice::set_center_freq(uint32_t freq_hz)
{
    if (rtlsdr_set_center_freq(dev_, freq_hz) != 0)
        throw std::runtime_error("Failed to set center frequency");
}

void RtlSdrDevice::set_sample_rate(uint32_t rate)
{
    if (rtlsdr_set_sample_rate(dev_, rate) != 0)
        throw std::runtime_error("Failed to set sample rate");
}

void RtlSdrDevice::set_freq_correction(int ppm)
{
    rtlsdr_set_freq_correction(dev_, ppm);
}

void RtlSdrDevice::set_gain_mode(bool manual)
{
    rtlsdr_set_tuner_gain_mode(dev_, manual ? 1 : 0);
}

void RtlSdrDevice::set_gain(int gain_tenth_db)
{
    rtlsdr_set_tuner_gain(dev_, gain_tenth_db);
}

void RtlSdrDevice::set_agc_mode(bool enable)
{
    rtlsdr_set_agc_mode(dev_, enable ? 1 : 0);
}

void RtlSdrDevice::set_offset_tuning(bool enable)
{
    rtlsdr_set_offset_tuning(dev_, enable ? 1 : 0);
}

uint32_t RtlSdrDevice::center_freq() const
{
    return rtlsdr_get_center_freq(dev_);
}

uint32_t RtlSdrDevice::sample_rate() const
{
    return rtlsdr_get_sample_rate(dev_);
}

int RtlSdrDevice::freq_correction() const
{
    return rtlsdr_get_freq_correction(dev_);
}

// ---- streaming ----------------------------------------------------------------

void RtlSdrDevice::rtlsdr_cb(unsigned char *buf, uint32_t len, void *ctx)
{
    auto *self = static_cast<RtlSdrDevice *>(ctx);
    if (self->user_cb_)
        self->user_cb_(buf, len);
}

void RtlSdrDevice::start(IqCallback callback,
                          uint32_t buf_num,
                          uint32_t buf_len)
{
    if (running_)
        throw std::runtime_error("RtlSdrDevice already running");

    user_cb_ = std::move(callback);
    rtlsdr_reset_buffer(dev_);
    running_ = true;

    // rtlsdr_read_async blocks until rtlsdr_cancel_async is called.
    // We run it directly here; the caller is responsible for calling
    // this from a dedicated thread.
    int ret = rtlsdr_read_async(dev_, rtlsdr_cb, this, buf_num, buf_len);
    running_ = false;
    (void)ret;
}

void RtlSdrDevice::stop()
{
    if (!running_) return;
    rtlsdr_cancel_async(dev_);
}

} // namespace sdrmon
