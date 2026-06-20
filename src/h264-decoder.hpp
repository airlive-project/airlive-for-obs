// h264-decoder.hpp — FFmpeg libavcodec H.264 decode for the Airlive wire.
//
// WHY FFmpeg and not VideoToolbox: OBS plugins are cross-platform C/C++ against
// libobs. FFmpeg's h264 decoder runs identically on Win/macOS/Linux.
//
// Strategy (brief Option A — convert to Annex-B): the wire carries AVCC
// (4-byte length-prefixed NALs) plus a separate parameter-set packet. We turn
// each sample into Annex-B (00 00 00 01 start codes) and prepend the cached
// SPS/PPS in front of every keyframe. FFmpeg's inline parameter-set handling
// then absorbs any mid-stream format change for free — no decoder rebuild, no
// extradata. We still cache+compare SPS/PPS so the prepend blob is rebuilt only
// when the bytes actually change.
//
// COLOR: the wire is a baked, display-ready Rec.709-ish H.264 proxy (LUT
// already applied on the phone). We pass through the decoded planes and tag
// Rec.709 — we never apply a Log curve.

#pragma once

#include <cstdint>
#include <functional>
#include <vector>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;

namespace airlive {

class H264Decoder {
public:
    // Called once per decoded frame. `frame` is valid only for the call
    // duration (OBS copies on output). pts_us is the capture timestamp.
    using FrameSink = std::function<void(const AVFrame *frame, int64_t pts_us)>;

    explicit H264Decoder(FrameSink sink);
    ~H264Decoder();

    H264Decoder(const H264Decoder &) = delete;
    H264Decoder &operator=(const H264Decoder &) = delete;

    // type 0 — parameter sets ([2-byte BE len][NAL]...). Rebuilds the Annex-B
    // SPS/PPS prepend blob only if the bytes changed since last time.
    void setParameterSets(const uint8_t *payload, size_t len);

    // type 1 — one access unit in AVCC. Converted to Annex-B and decoded.
    void decodeSample(const uint8_t *payload, size_t len, int64_t pts_us);

    // Flush + drop cached parameter sets (call on disconnect).
    void reset();

private:
    bool ensureOpen();
    void send(const uint8_t *annexb, size_t len, int64_t pts_us, bool keyframe);
    void drainFrames(int64_t pts_us);

    FrameSink sink_;
    AVCodecContext *ctx_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *pkt_ = nullptr;

    std::vector<uint8_t> paramSetsAnnexB_; // 00000001 SPS 00000001 PPS ...
    std::vector<uint8_t> scratch_;         // reused per-sample Annex-B buffer
};

} // namespace airlive
