#include "h264-decoder.hpp"

#include <climits>
#include <cstring>

#include <obs-module.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
}

#include "wire.hpp"

namespace airlive {

namespace {
constexpr uint8_t kStartCode[4] = {0x00, 0x00, 0x00, 0x01};

inline uint16_t be16(const uint8_t *p) { return uint16_t(p[0]) << 8 | uint16_t(p[1]); }
inline uint32_t be32(const uint8_t *p) {
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | uint32_t(p[3]);
}

void appendStartCodeNal(std::vector<uint8_t> &out, const uint8_t *nal, size_t len) {
    out.insert(out.end(), kStartCode, kStartCode + 4);
    out.insert(out.end(), nal, nal + len);
}
} // namespace

H264Decoder::H264Decoder(FrameSink sink) : sink_(std::move(sink)) {}

H264Decoder::~H264Decoder() {
    if (pkt_) av_packet_free(&pkt_);
    if (frame_) av_frame_free(&frame_);
    if (ctx_) avcodec_free_context(&ctx_);
}

bool H264Decoder::ensureOpen() {
    if (ctx_)
        return true;

    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        blog(LOG_ERROR, "[airlive] H.264 decoder not found in this FFmpeg build");
        return false;
    }
    ctx_ = avcodec_alloc_context3(codec);
    if (!ctx_)
        return false;

    // Latency is the whole point: decode each access unit immediately, no
    // multi-frame delay or threaded frame reordering.
    ctx_->thread_count = 1;
    ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;

    if (avcodec_open2(ctx_, codec, nullptr) < 0) {
        blog(LOG_ERROR, "[airlive] avcodec_open2 failed");
        avcodec_free_context(&ctx_);
        return false;
    }
    frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();
    if (!frame_ || !pkt_) {
        // OOM on either alloc: tear EVERYTHING down (all the frees are
        // null-safe). Otherwise ctx_ stays non-null, the next ensureOpen()
        // short-circuits to `return true`, and decodeSample() dereferences a
        // null pkt_/frame_ → crash. All-or-nothing, like the avcodec_open2 path.
        av_frame_free(&frame_);
        av_packet_free(&pkt_);
        avcodec_free_context(&ctx_);
        return false;
    }
    return true;
}

void H264Decoder::setParameterSets(const uint8_t *payload, size_t len) {
    // Parse [2-byte BE len][NAL] entries into an Annex-B blob.
    std::vector<uint8_t> blob;
    size_t pos = 0;
    while (pos + 2 <= len) {
        const uint16_t nalLen = be16(payload + pos);
        pos += 2;
        if (nalLen == 0 || pos + nalLen > len)
            break; // truncated/corrupt parameter-set packet — stop here
        appendStartCodeNal(blob, payload + pos, nalLen);
        pos += nalLen;
    }
    if (blob.empty())
        return;

    // The phone re-sends the parameter sets on every I-frame. Only act when the
    // bytes actually change (resolution/profile/fps switch) — otherwise this is
    // a no-op and we avoid churn.
    if (blob == paramSetsAnnexB_)
        return;

    paramSetsAnnexB_ = std::move(blob);
    blog(LOG_INFO, "[airlive] new SPS/PPS (%zu bytes Annex-B)", paramSetsAnnexB_.size());
}

void H264Decoder::decodeSample(const uint8_t *payload, size_t len, int64_t pts_us) {
    if (!ensureOpen())
        return;

    // Walk the AVCC NALs once: detect whether this access unit contains an IDR,
    // and rewrite each 4-byte length prefix as an Annex-B start code.
    scratch_.clear();
    bool isKeyframe = false;

    size_t pos = 0;
    while (pos + kAvccLengthBytes <= len) {
        const uint32_t nalLen = be32(payload + pos);
        pos += kAvccLengthBytes;
        if (nalLen == 0 || pos + nalLen > len) {
            blog(LOG_WARNING, "[airlive] malformed AVCC NAL — dropping access unit");
            return;
        }
        const uint8_t nalType = payload[pos] & 0x1F;
        if (nalType == kNalTypeIDR)
            isKeyframe = true;
        appendStartCodeNal(scratch_, payload + pos, nalLen);
        pos += nalLen;
    }
    if (scratch_.empty())
        return;

    // Prepend SPS/PPS to every keyframe so a decoder that just (re)opened, or a
    // viewer that joined mid-stream, can lock on without external extradata.
    if (isKeyframe && !paramSetsAnnexB_.empty()) {
        std::vector<uint8_t> withHeaders;
        withHeaders.reserve(paramSetsAnnexB_.size() + scratch_.size());
        withHeaders.insert(withHeaders.end(), paramSetsAnnexB_.begin(), paramSetsAnnexB_.end());
        withHeaders.insert(withHeaders.end(), scratch_.begin(), scratch_.end());
        send(withHeaders.data(), withHeaders.size(), pts_us, isKeyframe);
    } else {
        send(scratch_.data(), scratch_.size(), pts_us, isKeyframe);
    }
}

void H264Decoder::send(const uint8_t *annexb, size_t len, int64_t pts_us, bool keyframe) {
    if (len == 0 || len > size_t(INT_MAX)) // far above the 16 MiB framing ceiling — defensive
        return;

    // Give FFmpeg an OWNED, ref-counted packet. avcodec_send_packet may retain a
    // reference to the data past the call (threaded/future decoders), so it must
    // not point at our transient scratch buffer. av_new_packet allocates a
    // ref-counted buffer; we copy the Annex-B bytes in and unref after sending.
    if (av_new_packet(pkt_, int(len)) < 0)
        return;
    memcpy(pkt_->data, annexb, len);
    pkt_->pts = pts_us;
    pkt_->dts = pts_us;
    // Hint the keyframe to the decoder, mirroring OBS's own FFmpeg path
    // (win-dshow/ffmpeg-decode.c, which sets AV_PKT_FLAG_KEY via obs_avc_keyframe).
    pkt_->flags = keyframe ? AV_PKT_FLAG_KEY : 0;

    const int rc = avcodec_send_packet(ctx_, pkt_);
    av_packet_unref(pkt_);
    if (rc < 0) {
        blog(LOG_DEBUG, "[airlive] avcodec_send_packet rc=%d (transient)", rc);
        return;
    }
    drainFrames(pts_us);
}

void H264Decoder::drainFrames(int64_t pts_us) {
    for (;;) {
        const int rc = avcodec_receive_frame(ctx_, frame_);
        if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF)
            return;
        if (rc < 0) {
            blog(LOG_WARNING, "[airlive] avcodec_receive_frame rc=%d", rc);
            return;
        }
        // Prefer the frame's own PTS; fall back to the packet PTS for decoders
        // that don't propagate it.
        const int64_t out_pts = (frame_->pts != AV_NOPTS_VALUE) ? frame_->pts : pts_us;
        sink_(frame_, out_pts);
        av_frame_unref(frame_);
    }
}

void H264Decoder::reset() {
    if (ctx_)
        avcodec_flush_buffers(ctx_);
    paramSetsAnnexB_.clear();
}

} // namespace airlive
