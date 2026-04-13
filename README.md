# sdrmon — Software-Defined Radio Monitor

A C++17 library and console application for monitoring radio traffic, decoding
FleetSync I/II and MDC1200 digital packets, and generating voice transcripts using
an RTL-SDR dongle.

## Architecture

```
RTL-SDR dongle
     │  IQ @ 240 kHz (uint8)
     ▼
FmDemodulator       ← arctangent discriminator
     │  float audio @ 240 kHz
     ├──── Decimator ÷5 ──► 48 kHz ──► VoiceActivityDetector ──► TranscriptEngine
     │
     └──── Decimator ÷30 ─► 8 kHz  ──► FleetSyncDecoder
                                    └── Mdc1200Decoder

All decoders emit RadioEvents (std::variant) via an EventCallback.
```

The core logic lives in `src/sdrmon/` as a reusable library. The console
binary in `src/app/` wraps it and prints newline-delimited JSON to stdout.

## Dependencies

### Linux (Debian / Ubuntu)

```bash
sudo apt update
sudo apt install librtlsdr-dev rtl-sdr
```

### macOS (Homebrew)

```bash
brew install librtlsdr
```

### Build tool

Bazel 7 or later with Bzlmod support:

```bash
# Linux
curl -fsSL https://github.com/bazelbuild/bazelisk/releases/latest/download/bazelisk-linux-amd64 \
  -o /usr/local/bin/bazel && chmod +x /usr/local/bin/bazel

# macOS
brew install bazelisk
```

## Building

```bash
# Debug build
bazel build //src/app:sdrmon

# Optimised release
bazel build -c opt //src/app:sdrmon

# Cross-compile for macOS Apple Silicon (from an ARM Mac)
bazel build --config=macos_arm64 //src/app:sdrmon
```

The resulting binary is at `bazel-bin/src/app/sdrmon`.

## Usage

```
sdrmon [OPTIONS]

Options:
  -f <hz>      Centre frequency in Hz   (default: 462562500)
  -r <hz>      Sample rate in Hz        (default: 240000)
  -d <idx>     RTL-SDR device index     (default: 0)
  -p <ppm>     Frequency correction     (default: 0)
  -g <db>      Gain in dB, 0=auto       (default: 0 / auto)
  -s <level>   Squelch RMS threshold    (default: 0.01)
  -l <label>   Channel label string     (default: ch0)
  --no-fsync   Disable FleetSync decoder
  --no-mdc     Disable MDC1200 decoder
  --list       List available RTL-SDR devices and exit
```

### Examples

```bash
# Monitor a FleetSync fleet on 462.5625 MHz
./bazel-bin/src/app/sdrmon -f 462562500 -l "Site-A"

# List connected devices
./bazel-bin/src/app/sdrmon --list

# Fixed 40 dB gain, MDC1200 only
./bazel-bin/src/app/sdrmon -f 155000000 -g 40 --no-fsync -l "PD-Dispatch"
```

## JSON Output Format

Events are written to stdout as newline-delimited JSON objects.

### FleetSync packet

```json
{
  "type": "fleetsync",
  "ts": 1720000000.123,
  "label": "Site-A",
  "cmd": 2,
  "subcmd": 0,
  "from_fleet": 1,
  "from_unit": 42,
  "to_fleet": 1,
  "to_unit": 0,
  "fsync2": false
}
```

FleetSync GPS packet includes extra fields:
```json
{
  "type": "fleetsync",
  ...
  "lat": 37.7749,
  "lon": -122.4194
}
```

### MDC1200 packet

```json
{
  "type": "mdc1200",
  "ts": 1720000000.456,
  "label": "PD-Dispatch",
  "op": 1,
  "arg": 0,
  "unit_id": 1234
}
```

### Voice activity

```json
{"type": "voice_start", "ts": 1720000001.000, "label": "Site-A"}
{"type": "voice_end",   "ts": 1720000005.500, "label": "Site-A"}
```

### Transcript (when a `TranscriptEngine` backend is provided)

```json
{
  "type": "transcript",
  "ts": 1720000005.600,
  "label": "Site-A",
  "text": "Unit 42 on scene."
}
```

## Extending with a Transcript Backend

Implement the abstract `sdrmon::TranscriptEngine` interface:

```cpp
#include "sdrmon/monitor/transcript_engine.h"

class WhisperEngine : public sdrmon::TranscriptEngine {
public:
    void feed_audio(const float* samples, int count, int sample_rate) override;
    std::string get_transcript() override;
    bool has_result() override;
    void reset() override;
};
```

Then pass it to the monitor:

```cpp
sdrmon::MonitorConfig cfg = ...;
sdrmon::RadioMonitor monitor(cfg);
monitor.set_transcript_engine(std::make_unique<WhisperEngine>());
monitor.set_event_callback(my_callback);
monitor.start(0);
```

## Project Structure

```
MODULE.bazel                   Bzlmod module definition
.bazelrc                       Build flags, platform configs

third_party/
  rtlsdr/                      Vendored rtl-sdr.h + system library link
  fsync_mdc/                   FleetSync I/II + MDC1200 C decoder (GPL-2.0)

src/
  sdrmon/                      Reusable monitoring library
    rtlsdr/                    RTL-SDR device RAII wrapper
    dsp/                       FM demodulator + windowed-sinc decimator
    protocol/                  C++ wrappers for FleetSync + MDC1200
    monitor/                   Orchestrator, VAD, event types, STT interface
  app/
    main.cc                    Console binary (JSON output)
```

## Licences

- `third_party/fsync_mdc/`: **GPL-2.0** — adapted from
  [jfdupuis/fsync-mdc1200-decode](https://github.com/jfdupuis/fsync-mdc1200-decode)
  which is itself derived from work by Matthew Kaufman and others.
- All other source files in this repository are also released under **GPL-2.0** to
  comply with the share-alike requirement of the vendored library.
