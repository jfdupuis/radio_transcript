#pragma once
// sdrmon/monitor/transcript_engine.h
// Abstract interface for speech-to-text backends.
// Implement this to plug in Whisper.cpp, Google STT, etc.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sdrmon {

// Callback called with the transcribed text and a confidence score (0..1, or
// -1 if unsupported by the backend).
using TranscriptResultCb =
    std::function<void(const std::string &text, float confidence)>;

class TranscriptEngine {
public:
    virtual ~TranscriptEngine() = default;

    // Transcribe a block of PCM audio samples.
    // samples: float PCM, sample_rate Hz, mono.
    // on_result will be called (possibly asynchronously) with the result.
    virtual void transcribe(const std::vector<float> &samples,
                             uint32_t                  sample_rate,
                             TranscriptResultCb        on_result) = 0;
};

} // namespace sdrmon
