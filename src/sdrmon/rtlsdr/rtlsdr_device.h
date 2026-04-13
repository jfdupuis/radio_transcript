#pragma once
// sdrmon/rtlsdr/rtlsdr_device.h
// C++ wrapper around the librtlsdr C API.

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// Forward-declare the C type outside any namespace so it matches rtl-sdr.h.
struct rtlsdr_dev;

namespace sdrmon {

// Callback type: delivers raw interleaved IQ bytes from the RTL-SDR.
// Bytes are interleaved: I0, Q0, I1, Q1, ... in unsigned 8-bit format.
using IqCallback = std::function<void(const uint8_t *buf, uint32_t len)>;

struct DeviceInfo {
    uint32_t    index;
    std::string name;
    std::string manufacturer;
    std::string product;
    std::string serial;
};

class RtlSdrDevice {
public:
    // List all connected RTL-SDR devices.
    static std::vector<DeviceInfo> enumerate();

    // Construct targeting a device by index (0-based).
    explicit RtlSdrDevice(uint32_t device_index = 0);
    ~RtlSdrDevice();

    // Non-copyable, movable.
    RtlSdrDevice(const RtlSdrDevice &)            = delete;
    RtlSdrDevice &operator=(const RtlSdrDevice &) = delete;
    RtlSdrDevice(RtlSdrDevice &&)                 noexcept;
    RtlSdrDevice &operator=(RtlSdrDevice &&)      noexcept;

    // Configuration — call before start().
    void set_center_freq(uint32_t freq_hz);
    void set_sample_rate(uint32_t rate);
    void set_freq_correction(int ppm);
    void set_gain_mode(bool manual);       // false = automatic AGC
    void set_gain(int gain_tenth_db);      // e.g. 496 = 49.6 dB
    void set_agc_mode(bool enable);
    void set_offset_tuning(bool enable);

    uint32_t center_freq() const;
    uint32_t sample_rate() const;
    int      freq_correction() const;

    // Asynchronous streaming.
    // The callback is invoked from the reader thread.
    // buf_num=0 uses the default (15), buf_len=0 uses default (16*32*512).
    void start(IqCallback callback,
               uint32_t buf_num = 0,
               uint32_t buf_len = 0);

    void stop();

    bool is_running() const { return running_; }

private:
    ::rtlsdr_dev      *dev_ = nullptr;
    bool               running_ = false;

    static void rtlsdr_cb(unsigned char *buf, uint32_t len, void *ctx);
    IqCallback user_cb_;
};

} // namespace sdrmon
