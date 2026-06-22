// airlive-source.cpp — the OBS source "OBS Airlive".
//
// One source == one advertised Bonjour instance == one iPhone slot. This file
// owns the OBS-facing surface: registration, properties, the AVFrame ->
// obs_source_frame2 bridge, a fixed presentation-delay buffer, connection
// status display, and Studio-Mode tally (setCue) back to the iPhone.
//
// Threading:
//   - connection worker thread  -> decodes, runs the delay FIFO, does the pixel
//                                   mapping + obs_source_output_video2();
//                                   parses control JSON, calls onControl()
//   - OBS core thread           -> activate/deactivate/show/hide (tally),
//                                   properties, create/update/destroy
// Decode → delay → output are all synchronous on the worker thread, so the FIFO
// and the per-source sws context have a single owner — no locks needed.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <obs-module.h>
#include <util/platform.h>

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include "airlive-connection.hpp"
#include "secret-store.hpp"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("airlive-obs", "en-US")

using namespace airlive;

namespace {

constexpr const char *kSettingDeviceName = "device_name";
constexpr const char *kSettingSourceName = "source_name";
constexpr const char *kSettingDelayMs = "delay_ms";
constexpr const char *kSettingSid = "sid"; // hidden — stable per-source id
// Receiver-password auth (STREAM-AUTH-SPEC). Off by default.
constexpr const char *kSettingRequireAuth = "require_auth";
constexpr const char *kSettingPassword = "auth_password";

// Phase-2a camera-control properties — each maps to a type-2 set-command sent to
// the iPhone when the operator changes it. SEND-only (live readback goes to the
// status text, not back into these, to avoid a feedback loop).
constexpr const char *kCamAE = "cam_ae";
constexpr const char *kCamISO = "cam_iso";
constexpr const char *kCamShutter = "cam_shutter";
constexpr const char *kCamAWB = "cam_awb";
constexpr const char *kCamWB = "cam_wb";
constexpr const char *kCamTint = "cam_tint";
constexpr const char *kCamAF = "cam_af";
constexpr const char *kCamFocus = "cam_focus";
constexpr const char *kCamLens = "cam_lens";
constexpr const char *kCamFPS = "cam_fps";
constexpr const char *kCamZoom = "cam_zoom";
constexpr const char *kCamLUT = "cam_lut";

constexpr int kDefaultDelayMs = 120; // Studio-style fixed delay (Normal preset)
constexpr int kMaxDelayMs = 400;     // matches the highest preset (Safe)
constexpr size_t kSafetyCapFrames = 600; // anti-runaway if timestamps stop advancing

// A decoded frame held in the delay FIFO — an FFmpeg ref-counted clone (cheap,
// refs the buffer, no pixel copy) tagged with its capture PTS.
struct QueuedFrame {
    AVFrame *frame;
    int64_t pts_us;
};

// One OBS source instance.
struct AirliveSourceCtx {
    obs_source_t *source = nullptr;
    std::string did; // per-install group key (shared across all sources)
    std::string sid; // per-source stable id
    std::string dev; // group display name
    std::string src; // source display name
    std::unique_ptr<AirliveConnection> conn;

    std::atomic<int> delayMs{kDefaultDelayMs};
    std::atomic<bool> delayReset{false}; // set by source_update when delay changes

    // Fixed-delay FIFO — mirrors OBS's own async-delay-filter: keep `delay`
    // worth of frames (measured by their timestamps) and emit the oldest, so the
    // output is always `delay` behind reality. Touched ONLY on the worker
    // thread — decode → emit are synchronous there, so no locks/extra thread.
    std::deque<QueuedFrame> dq;
    bool delayReached = false;  // false while the buffer is still filling
    int64_t lastQueuedPts = 0;  // for timestamp-jump detection

    // sws fallback — only used when the decoded format isn't one OBS takes
    // directly. Touched ONLY on the worker thread.
    SwsContext *sws = nullptr;
    std::vector<uint8_t> swsBuf;
    int swsW = 0, swsH = 0;
    int swsSrcFmt = -1;

    // Output rotation (clockwise degrees: 0/90/180/270) from the iPhone's
    // StateSnapshot.outputRotation. Presentation-only: 90/270 swap W/H (portrait).
    std::atomic<int> rotationDeg{0};
    std::vector<uint8_t> rotBuf;   // rotated I420 output (worker thread only)
    std::vector<uint8_t> i420Buf;  // sws→I420 staging for non-420p input
    SwsContext *swsRot = nullptr;  // converts a non-420p frame to I420 before rotating
    int swsRotW = 0, swsRotH = 0, swsRotFmt = -1;

    // Live stats (from decoded frames). statW/H/Fps are read by get_properties.
    std::atomic<uint32_t> statW{0}, statH{0};
    std::atomic<double> statFps{0.0};
    int64_t lastPtsUs = 0; // fps EMA state — touched only on the worker thread
    double fpsEma = 0.0;

    // Remote camera state (from the iPhone's type-2 JSON snapshot) — for the
    // read-only status readout.
    std::mutex stateMutex;
    std::string rDevice, rResolution, rColorSpace, rLens;
    double rIso = 0, rShutter = 0, rWB = 0;
    int rFps = 0;
    bool rAE = false, rAWB = false;

    // Camera-control last-sent values: used to (a) detect which control the
    // operator actually changed in update(), and (b) seed the auto/manual
    // grey-out state when the properties dialog opens. Touched on the OBS thread.
    bool ccInit = false;
    bool cAE = true, cAWB = true, cAF = true, cLUT = false;
    int cISO = 100, cShutter = 50, cWB = 5600, cTint = 0, cFPS = 30;
    double cFocus = 0.5, cZoom = 1.0;
    std::string cLens = "1x";

    // Tally state — guarded because OBS fires the callbacks on its own thread.
    std::mutex tallyMutex;
    std::string lastCue{"none"};

    ~AirliveSourceCtx() {
        if (sws)
            sws_freeContext(sws);
        if (swsRot)
            sws_freeContext(swsRot);
    }
};

// ---- identity helpers -----------------------------------------------------

std::string genUuid() {
    std::random_device rd;
    std::seed_seq seq{rd(), rd(), rd(), rd()}; // seed_seq avoids a degenerate all-zero seed
    std::mt19937_64 gen(seq);
    uint8_t b[16];
    const uint64_t r0 = gen(), r1 = gen(); // memcpy, not reinterpret_cast — no aliasing UB
    memcpy(b, &r0, 8);
    memcpy(b + 8, &r1, 8);
    b[6] = (b[6] & 0x0F) | 0x40; // version 4
    b[8] = (b[8] & 0x3F) | 0x80; // variant 1
    char out[37];
    snprintf(out, sizeof(out),
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13],
             b[14], b[15]);
    return std::string(out);
}

// The did groups every source from this install under one header on the phone,
// so it must be stable across launches. Persist it in the module config dir.
std::string loadOrCreateDid() {
    if (char *dir = obs_module_config_path(nullptr)) {
        os_mkdirs(dir);
        bfree(dir);
    }
    std::string did;
    char *path = obs_module_config_path("install-id.txt");
    if (path) {
        if (os_file_exists(path)) {
            if (char *contents = os_quick_read_utf8_file(path)) {
                did = contents;
                bfree(contents);
                while (!did.empty() && (did.back() == '\n' || did.back() == '\r' || did.back() == ' '))
                    did.pop_back();
            }
        }
        if (did.empty()) {
            did = genUuid();
            os_quick_write_utf8_file(path, did.c_str(), did.size(), false);
        }
        bfree(path);
    }
    return did.empty() ? genUuid() : did;
}

// ---- AVFrame -> obs_source_frame2 -----------------------------------------
// Color helpers mirror OBS's own FFmpeg decoder
// (plugins/win-dshow/ffmpeg-decode.c) so behaviour matches every other OBS
// camera source.

enum video_colorspace avframeColorspace(const AVFrame *f) {
    switch (f->colorspace) {
    case AVCOL_SPC_BT709:
        return (f->color_trc == AVCOL_TRC_IEC61966_2_1) ? VIDEO_CS_SRGB : VIDEO_CS_709;
    case AVCOL_SPC_FCC:
    case AVCOL_SPC_BT470BG:
    case AVCOL_SPC_SMPTE170M:
    case AVCOL_SPC_SMPTE240M:
        return VIDEO_CS_601;
    case AVCOL_SPC_BT2020_NCL:
        return (f->color_trc == AVCOL_TRC_ARIB_STD_B67) ? VIDEO_CS_2100_HLG : VIDEO_CS_2100_PQ;
    default:
        // Baked SDR proxy — treat unspecified as Rec.709 rather than letting OBS guess.
        return VIDEO_CS_709;
    }
}

uint8_t avframeTRC(const AVFrame *f) {
    switch (f->color_trc) {
    case AVCOL_TRC_BT709:
    case AVCOL_TRC_GAMMA22:
    case AVCOL_TRC_GAMMA28:
    case AVCOL_TRC_SMPTE170M:
    case AVCOL_TRC_SMPTE240M:
    case AVCOL_TRC_IEC61966_2_1:
        return VIDEO_TRC_SRGB;
    case AVCOL_TRC_SMPTE2084:
        return VIDEO_TRC_PQ;
    case AVCOL_TRC_ARIB_STD_B67:
        return VIDEO_TRC_HLG;
    default:
        return VIDEO_TRC_DEFAULT;
    }
}

// ---- output rotation (StateSnapshot.outputRotation, clockwise) -------------

int normalizeRotation(int d) { return (d == 90 || d == 180 || d == 270) ? d : 0; }

// Rotate one 8-bit plane CLOCKWISE into a tightly-packed dst (stride = dst width).
void rotatePlaneCW(const uint8_t *s, int sw, int sh, int sstride, uint8_t *d, int deg) {
    if (deg == 180) {
        for (int y = 0; y < sh; ++y) {
            const uint8_t *sr = s + size_t(sh - 1 - y) * sstride;
            uint8_t *dr = d + size_t(y) * sw;
            for (int x = 0; x < sw; ++x)
                dr[x] = sr[sw - 1 - x];
        }
    } else { // 90 / 270 → dst is sh wide, sw tall
        const int dw = sh, dh = sw;
        for (int y = 0; y < dh; ++y) {
            uint8_t *dr = d + size_t(y) * dw;
            if (deg == 90)
                for (int x = 0; x < dw; ++x)
                    dr[x] = s[size_t(sh - 1 - x) * sstride + y];
            else // 270
                for (int x = 0; x < dw; ++x)
                    dr[x] = s[size_t(x) * sstride + (sw - 1 - y)];
        }
    }
}

// Expose the frame as I420 planes — directly if it already is, else sws-convert
// once into i420Buf. Keeps rotation a pure pixel reorder (no colour math), so
// rotated output matches the non-rotated path exactly.
bool ensureI420(AirliveSourceCtx *ctx, const AVFrame *f, const uint8_t *&y, const uint8_t *&u,
                const uint8_t *&v, int &ly, int &lu, int &lv) {
    const int w = f->width, h = f->height, fmt = f->format;
    if (fmt == AV_PIX_FMT_YUV420P || fmt == AV_PIX_FMT_YUVJ420P) {
        y = f->data[0]; u = f->data[1]; v = f->data[2];
        ly = f->linesize[0]; lu = f->linesize[1]; lv = f->linesize[2];
        return true;
    }
    const int cw = (w + 1) / 2, ch = (h + 1) / 2;
    if (!ctx->swsRot || ctx->swsRotW != w || ctx->swsRotH != h || ctx->swsRotFmt != fmt) {
        if (ctx->swsRot)
            sws_freeContext(ctx->swsRot);
        ctx->swsRot = sws_getContext(w, h, AVPixelFormat(fmt), w, h, AV_PIX_FMT_YUV420P,
                                     SWS_BILINEAR, nullptr, nullptr, nullptr);
        ctx->swsRotW = w; ctx->swsRotH = h; ctx->swsRotFmt = fmt;
        ctx->i420Buf.resize(size_t(w) * h + 2 * size_t(cw) * ch);
    }
    if (!ctx->swsRot)
        return false;
    uint8_t *dy = ctx->i420Buf.data();
    uint8_t *du = dy + size_t(w) * h;
    uint8_t *dv = du + size_t(cw) * ch;
    uint8_t *dst[3] = {dy, du, dv};
    int dstStride[3] = {w, cw, cw};
    sws_scale(ctx->swsRot, f->data, f->linesize, 0, h, dst, dstStride);
    y = dy; u = du; v = dv; ly = w; lu = cw; lv = cw;
    return true;
}

void outputRotated(AirliveSourceCtx *ctx, const AVFrame *f, int64_t pts_us, int rot) {
    const uint8_t *sy, *su, *sv;
    int ly, lu, lv;
    if (!ensureI420(ctx, f, sy, su, sv, ly, lu, lv))
        return;
    const int w = f->width, h = f->height, cw = (w + 1) / 2, ch = (h + 1) / 2;
    const int outW = (rot == 180) ? w : h; // 90/270 swap to portrait
    const int outH = (rot == 180) ? h : w;
    const int ocw = (outW + 1) / 2, och = (outH + 1) / 2;
    const size_t ySize = size_t(outW) * outH, cSize = size_t(ocw) * och;
    ctx->rotBuf.resize(ySize + 2 * cSize);
    uint8_t *dy = ctx->rotBuf.data();
    uint8_t *du = dy + ySize;
    uint8_t *dv = du + cSize;
    rotatePlaneCW(sy, w, h, ly, dy, rot);
    rotatePlaneCW(su, cw, ch, lu, du, rot);
    rotatePlaneCW(sv, cw, ch, lv, dv, rot);

    obs_source_frame2 frame = {};
    frame.width = uint32_t(outW);
    frame.height = uint32_t(outH);
    frame.timestamp = uint64_t(pts_us) * 1000ull;
    frame.format = VIDEO_FORMAT_I420;
    frame.data[0] = dy; frame.linesize[0] = uint32_t(outW);
    frame.data[1] = du; frame.linesize[1] = uint32_t(ocw); // chroma stride = chroma width
    frame.data[2] = dv; frame.linesize[2] = uint32_t(ocw);

    const bool isJpeg = (f->format == AV_PIX_FMT_YUVJ420P);
    const enum video_range_type range =
        (f->color_range == AVCOL_RANGE_JPEG || isJpeg) ? VIDEO_RANGE_FULL : VIDEO_RANGE_PARTIAL;
    const enum video_colorspace cs = avframeColorspace(f);
    frame.range = range;
    frame.trc = avframeTRC(f);
    video_format_get_parameters_for_format(cs, range, frame.format, frame.color_matrix,
                                           frame.color_range_min, frame.color_range_max);
    obs_source_output_video2(ctx->source, &frame);
}

// Map a decoded AVFrame and hand it to OBS. Runs on the worker thread only.
void outputFrame(AirliveSourceCtx *ctx, const AVFrame *f, int64_t pts_us) {
    if (const int rot = normalizeRotation(ctx->rotationDeg.load())) {
        outputRotated(ctx, f, pts_us, rot); // clockwise presentation rotation (matches Studio)
        return;
    }
    // frame2 (not the legacy frame) — the v1 obs_source_output_video path does
    // not honour partial range, and H.264 is limited-range. See obs.h:1413.
    obs_source_frame2 frame = {};
    frame.width = uint32_t(f->width);
    frame.height = uint32_t(f->height);
    frame.timestamp = uint64_t(pts_us) * 1000ull; // µs -> ns (OBS wants ns)

    const int fmt = f->format;
    // YUVJ420P/YUVJ444P are FFmpeg's legacy "full-range" pixel formats; some
    // decoder versions leave color_range UNSPECIFIED for them, which would
    // otherwise tag full-range pixels as limited → a washed-out / bright image.
    const bool isJpegFmt = (fmt == AV_PIX_FMT_YUVJ420P || fmt == AV_PIX_FMT_YUVJ444P);
    const enum video_range_type range =
        (f->color_range == AVCOL_RANGE_JPEG || isJpegFmt) ? VIDEO_RANGE_FULL : VIDEO_RANGE_PARTIAL;
    const enum video_colorspace cs = avframeColorspace(f);

    if (fmt == AV_PIX_FMT_YUV420P || fmt == AV_PIX_FMT_YUVJ420P) {
        frame.format = VIDEO_FORMAT_I420;
        for (int i = 0; i < 3; ++i) {
            frame.data[i] = f->data[i];
            frame.linesize[i] = uint32_t(f->linesize[i]);
        }
    } else if (fmt == AV_PIX_FMT_NV12) {
        frame.format = VIDEO_FORMAT_NV12;
        for (int i = 0; i < 2; ++i) {
            frame.data[i] = f->data[i];
            frame.linesize[i] = uint32_t(f->linesize[i]);
        }
    } else {
        // Anything else (e.g. a 10-bit variant) — convert once to NV12.
        const int w = f->width, h = f->height;
        if (!ctx->sws || ctx->swsW != w || ctx->swsH != h || ctx->swsSrcFmt != fmt) {
            if (ctx->sws)
                sws_freeContext(ctx->sws);
            ctx->sws = sws_getContext(w, h, AVPixelFormat(fmt), w, h, AV_PIX_FMT_NV12,
                                      SWS_BILINEAR, nullptr, nullptr, nullptr);
            ctx->swsW = w;
            ctx->swsH = h;
            ctx->swsSrcFmt = fmt;
            // NV12 = w*h luma + w*ceil(h/2) interleaved-UV. Use (h+1)/2 so an
            // odd height can't undersize the chroma plane (1080p is even, but be
            // correct for any source the decoder might hand us).
            ctx->swsBuf.resize(size_t(w) * h + size_t(w) * ((h + 1) / 2));
        }
        if (!ctx->sws)
            return;
        uint8_t *dst[2] = {ctx->swsBuf.data(), ctx->swsBuf.data() + size_t(w) * h};
        int dstStride[2] = {w, w};
        sws_scale(ctx->sws, f->data, f->linesize, 0, h, dst, dstStride);
        frame.format = VIDEO_FORMAT_NV12;
        frame.data[0] = dst[0];
        frame.linesize[0] = uint32_t(w);
        frame.data[1] = dst[1];
        frame.linesize[1] = uint32_t(w);
    }

    frame.range = range;
    frame.trc = avframeTRC(f);
    video_format_get_parameters_for_format(cs, range, frame.format, frame.color_matrix,
                                           frame.color_range_min, frame.color_range_max);

    obs_source_output_video2(ctx->source, &frame);
}

// ---- fixed-delay FIFO ------------------------------------------------------

void flushDelayQueue(AirliveSourceCtx *ctx) {
    for (auto &q : ctx->dq)
        av_frame_free(&q.frame);
    ctx->dq.clear();
    ctx->delayReached = false;
}

// A discontinuity in capture timestamps (stream restart, big gap) makes the
// buffered span meaningless — flush and refill on one.
bool isTimestampJump(int64_t ts, int64_t prev) {
    return ts < prev || (ts - prev) > 1000000; // backwards, or >1 s gap (µs)
}

// Per decoded frame, on the connection worker thread. Same timestamp-interval
// delay as OBS's async-delay-filter: buffer frames until the span
// (newest.pts - oldest.pts) reaches the delay, then emit the oldest for each new
// frame — steady state is one-in/one-out, always `delay` behind reality.
// (The previous wall-clock hold added no visible latency: OBS plays async frames
//  by their timestamp cadence, so a constant per-frame hold is invisible on air.)
void onDecodedFrame(AirliveSourceCtx *ctx, const AVFrame *f, int64_t pts_us) {
    // Report the OUTPUT dimensions (swapped for 90/270) so OBS sizes the source
    // to the rotated, portrait frame.
    const int rot = normalizeRotation(ctx->rotationDeg.load());
    const bool portrait = (rot == 90 || rot == 270);
    ctx->statW.store(uint32_t(portrait ? f->height : f->width));
    ctx->statH.store(uint32_t(portrait ? f->width : f->height));
    if (ctx->lastPtsUs != 0 && pts_us > ctx->lastPtsUs) {
        const double inst = 1e6 / double(pts_us - ctx->lastPtsUs);
        ctx->fpsEma = (ctx->fpsEma == 0.0) ? inst : (ctx->fpsEma * 0.9 + inst * 0.1);
        ctx->statFps.store(ctx->fpsEma);
    }
    ctx->lastPtsUs = pts_us;

    // Delay changed (re-converge to new target) or timestamps jumped → refill.
    if (ctx->delayReset.exchange(false) || isTimestampJump(pts_us, ctx->lastQueuedPts))
        flushDelayQueue(ctx);
    ctx->lastQueuedPts = pts_us;

    const int64_t intervalUs = int64_t(ctx->delayMs.load()) * 1000;

    AVFrame *clone = av_frame_clone(f);
    if (!clone)
        return;
    ctx->dq.push_back({clone, pts_us});

    // Keep filling until we hold `delay` worth of timeline; then emit the oldest.
    // delay 0 → span 0 >= 0 → emit immediately (one-in/one-out, no added latency).
    const int64_t span = ctx->dq.back().pts_us - ctx->dq.front().pts_us;
    if (!ctx->delayReached && span < intervalUs && ctx->dq.size() <= kSafetyCapFrames)
        return; // still filling — nothing on air yet
    ctx->delayReached = true;

    QueuedFrame out = ctx->dq.front();
    ctx->dq.pop_front();
    outputFrame(ctx, out.frame, out.pts_us);
    av_frame_free(&out.frame);
}

// ---- tally (Studio Mode -> setCue) ----------------------------------------

// Send the current cue to the iPhone. force=true sends even if unchanged (used
// when the phone (re)connects, so it learns it's already live).
//
// active → "program" (on air); else showing → "preview" (staged in Studio Mode);
// else "none". The cue is a closed set of literals, so building the JSON by
// concatenation is safe — no escaping needed.
void ensureTally(AirliveSourceCtx *ctx, bool force) {
    if (!ctx->conn)
        return;
    const bool active = obs_source_active(ctx->source);
    const bool showing = obs_source_showing(ctx->source);
    const char *cue = active ? "program" : (showing ? "preview" : "none");

    bool valueChanged, needSend;
    {
        std::lock_guard<std::mutex> lk(ctx->tallyMutex);
        valueChanged = (cue != ctx->lastCue);
        needSend = force || valueChanged; // force re-asserts on (re)connect
    }
    if (!needSend)
        return;

    const bool ok = ctx->conn->sendControl(
        std::string("{\"type\":\"setCue\",\"stringValue\":\"") + cue + "\"}");
    if (ok) {
        // Commit lastCue ONLY after a confirmed send — a dropped cue stays
        // "unsent" so the next evaluation retries it (no optimistic dedup).
        std::lock_guard<std::mutex> lk(ctx->tallyMutex);
        ctx->lastCue = cue;
    }
    // Log only real transitions / drops — never the ~1 Hz force re-asserts.
    if (valueChanged || !ok)
        blog(LOG_INFO, "[airlive] tally -> %s%s", cue, ok ? "" : " (DROPPED, will retry)");
}

// ---- control channel (status) ---------------------------------------------

// Parse an inbound type-2 JSON ControlMessage. We only read the state snapshot
// for status display, then (re)assert tally so a freshly-connected phone learns
// whether it's live.
void onControl(AirliveSourceCtx *ctx, const char *json, size_t len) {
    const std::string s(json, len); // null-terminate for the JSON parser
    obs_data_t *msg = obs_data_create_from_json(s.c_str());
    if (!msg)
        return;
    bool gotState = false;
    if (obs_data_t *st = obs_data_get_obj(msg, "state")) {
        {
            std::lock_guard<std::mutex> lk(ctx->stateMutex);
            ctx->rDevice = obs_data_get_string(st, "deviceModel");
            ctx->rResolution = obs_data_get_string(st, "resolution");
            ctx->rColorSpace = obs_data_get_string(st, "colorSpace");
            ctx->rLens = obs_data_get_string(st, "lens");
            ctx->rIso = obs_data_get_double(st, "iso");
            ctx->rShutter = obs_data_get_double(st, "shutterDenom");
            ctx->rWB = obs_data_get_double(st, "wbKelvin");
            ctx->rFps = int(obs_data_get_int(st, "fps"));
            ctx->rAE = obs_data_get_bool(st, "exposureAuto");
            ctx->rAWB = obs_data_get_bool(st, "whiteBalanceAuto");
        }
        // Additive presentation rotation (clockwise; absent/0 = landscape).
        ctx->rotationDeg.store(normalizeRotation(int(obs_data_get_int(st, "outputRotation"))));
        obs_data_release(st);
        gotState = true;
    }
    obs_data_release(msg);

    // Re-assert tally only on an actual state snapshot (the phone announcing
    // itself), not on any other type-2 message — otherwise we'd emit a
    // redundant setCue for messages that aren't a connect/state event.
    if (gotState)
        ensureTally(ctx, /*force=*/true);
}

// ---- camera control (Phase 2a — send set-commands) ------------------------

// Each command is a tiny type-2 JSON ControlMessage. We build it with obs_data
// (correct escaping + number formatting, same lib we parse with) and ship it
// over the existing control channel. Field names match the iPhone's Codable
// ControlMessage (floatValue/intValue/stringValue/boolValue/lutName).
void sendCmd(AirliveSourceCtx *ctx, obs_data_t *d) {
    if (ctx->conn)
        ctx->conn->sendControl(obs_data_get_json(d));
    obs_data_release(d);
}
void sendCmdFloat(AirliveSourceCtx *ctx, const char *type, double v) {
    obs_data_t *d = obs_data_create();
    obs_data_set_string(d, "type", type);
    obs_data_set_double(d, "floatValue", v);
    sendCmd(ctx, d);
}
void sendCmdInt(AirliveSourceCtx *ctx, const char *type, int v) {
    obs_data_t *d = obs_data_create();
    obs_data_set_string(d, "type", type);
    obs_data_set_int(d, "intValue", v);
    sendCmd(ctx, d);
}
void sendCmdBool(AirliveSourceCtx *ctx, const char *type, bool v) {
    obs_data_t *d = obs_data_create();
    obs_data_set_string(d, "type", type);
    obs_data_set_bool(d, "boolValue", v);
    sendCmd(ctx, d);
}
void sendCmdString(AirliveSourceCtx *ctx, const char *type, const char *v) {
    obs_data_t *d = obs_data_create();
    obs_data_set_string(d, "type", type);
    obs_data_set_string(d, "stringValue", v);
    sendCmd(ctx, d);
}

// Read the control values from settings. When sendDiffs is true, send a command
// for each one the operator actually changed since last time; always update the
// cached "last value". On create we call with sendDiffs=false so loading a saved
// scene doesn't blast the camera with its persisted settings.
void applyCameraControls(AirliveSourceCtx *ctx, obs_data_t *settings, bool sendDiffs) {
    const bool ae = obs_data_get_bool(settings, kCamAE);
    const int iso = int(obs_data_get_int(settings, kCamISO));
    const int sh = int(obs_data_get_int(settings, kCamShutter));
    const bool awb = obs_data_get_bool(settings, kCamAWB);
    const int wb = int(obs_data_get_int(settings, kCamWB));
    const int tint = int(obs_data_get_int(settings, kCamTint));
    const bool af = obs_data_get_bool(settings, kCamAF);
    const double focus = obs_data_get_double(settings, kCamFocus);
    const std::string lens = obs_data_get_string(settings, kCamLens);
    const int fps = int(obs_data_get_int(settings, kCamFPS));
    const double zoom = obs_data_get_double(settings, kCamZoom);
    const bool lut = obs_data_get_bool(settings, kCamLUT);

    if (sendDiffs && ctx->ccInit) {
        if (ae != ctx->cAE) sendCmdBool(ctx, "setExposureAuto", ae);
        if (iso != ctx->cISO) sendCmdFloat(ctx, "setISO", iso);
        if (sh != ctx->cShutter) sendCmdFloat(ctx, "setShutter", sh);
        if (awb != ctx->cAWB) sendCmdBool(ctx, "setWhiteBalanceAuto", awb);
        if (wb != ctx->cWB) sendCmdFloat(ctx, "setWB", wb);
        if (tint != ctx->cTint) sendCmdFloat(ctx, "setTint", tint);
        if (af != ctx->cAF) sendCmdBool(ctx, "setFocusAuto", af);
        if (focus != ctx->cFocus) sendCmdFloat(ctx, "setFocusPosition", focus);
        if (lens != ctx->cLens) sendCmdString(ctx, "setLens", lens.c_str());
        if (fps != ctx->cFPS) sendCmdInt(ctx, "setFPS", fps);
        if (zoom != ctx->cZoom) sendCmdFloat(ctx, "setZoom", zoom);
        if (lut != ctx->cLUT) sendCmdBool(ctx, "setLUT", lut);
    }

    ctx->cAE = ae; ctx->cISO = iso; ctx->cShutter = sh; ctx->cAWB = awb;
    ctx->cWB = wb; ctx->cTint = tint; ctx->cAF = af; ctx->cFocus = focus;
    ctx->cLens = lens; ctx->cFPS = fps; ctx->cZoom = zoom; ctx->cLUT = lut;
    ctx->ccInit = true;
}

// ---- connection lifecycle -------------------------------------------------

ServiceIdentity makeIdentity(const AirliveSourceCtx *ctx) {
    return ServiceIdentity{ctx->did, ctx->dev, ctx->sid, ctx->src};
}

// Push the receiver-password auth config to the connection (STREAM-AUTH-SPEC §4).
// Auth engages only when the toggle is on AND a password is set; OFF by default.
//
// The password is kept in the OS secret store (macOS Keychain), NEVER in the
// scene-collection JSON: if the operator typed a new one, we move it into the
// Keychain (keyed by the stable sid) and SCRUB the settings field so nothing
// plaintext persists on disk. A blank field means "keep the stored password"
// (use the Clear button to remove it). A change applies to the NEXT connection.
void applyAuth(AirliveSourceCtx *ctx, obs_data_t *settings) {
    if (!ctx->conn)
        return;
    const char *typed = obs_data_get_string(settings, kSettingPassword);
    if (typed && *typed) {
        secretStoreSet(ctx->sid, typed);
        obs_data_set_string(settings, kSettingPassword, ""); // never persist plaintext
    }
    const bool require = obs_data_get_bool(settings, kSettingRequireAuth);
    ctx->conn->setAuth(require, secretStoreGet(ctx->sid));
}

// (Re)build the connection with the current identity. Called on create and
// whenever a renameable property changes (new names must hit the TXT record).
void rebuildConnection(AirliveSourceCtx *ctx) {
    ctx->conn.reset();
    // The lambdas below capture ctx by RAW pointer. This is safe ONLY because
    // conn.reset()/~AirliveConnection joins the worker thread before ctx is ever
    // freed (source_destroy: conn.reset() → flush → delete ctx), so no callback
    // can fire after ctx dies. If async/deferred teardown is ever introduced,
    // replace this with a weak_ptr or a cancel token — do not loosen it.
    AirliveSourceCtx *self = ctx;
    ctx->conn = std::make_unique<AirliveConnection>(
        makeIdentity(ctx),
        [self](const AVFrame *f, int64_t pts) { onDecodedFrame(self, f, pts); },
        [self](const char *j, size_t n) { onControl(self, j, n); });
    ctx->conn->start();
}

// ---- obs_source_info callbacks --------------------------------------------

const char *source_get_name(void *) {
    return obs_module_text("Airlive.SourceName");
}

void apply_settings(AirliveSourceCtx *ctx, obs_data_t *settings) {
    ctx->dev = obs_data_get_string(settings, kSettingDeviceName);

    // The iPhone-facing label defaults to the OBS source's OWN name — the unique
    // name the operator typed when adding it (and can rename in the Sources
    // list). Saved once so it persists; the operator can still override it in
    // this source's properties. No counter (that produced ever-growing numbers
    // unrelated to how many sources actually exist).
    ctx->src = obs_data_get_string(settings, kSettingSourceName);
    if (ctx->src.empty()) {
        const char *sname = obs_source_get_name(ctx->source);
        ctx->src = (sname && *sname) ? sname : "OBS Source";
        obs_data_set_string(settings, kSettingSourceName, ctx->src.c_str());
    }

    int delay = int(obs_data_get_int(settings, kSettingDelayMs));
    delay = delay < 0 ? 0 : (delay > kMaxDelayMs ? kMaxDelayMs : delay);
    if (delay != ctx->delayMs.exchange(delay))
        ctx->delayReset.store(true); // re-converge the FIFO to the new target

    // The sid must survive restarts. Generate once and write it back into the
    // source's settings so it's saved with the scene collection.
    const char *sid = obs_data_get_string(settings, kSettingSid);
    if (!sid || !*sid) {
        ctx->sid = genUuid();
        obs_data_set_string(settings, kSettingSid, ctx->sid.c_str());
    } else {
        ctx->sid = sid;
    }
}

void *source_create(obs_data_t *settings, obs_source_t *source) {
    auto *ctx = new AirliveSourceCtx();
    ctx->source = source;
    ctx->did = loadOrCreateDid();
    apply_settings(ctx, settings);
    applyCameraControls(ctx, settings, /*sendDiffs=*/false); // seed cache, don't command
    rebuildConnection(ctx);
    applyAuth(ctx, settings); // push auth config to the fresh connection
    return ctx;
}

void source_destroy(void *data) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    ctx->conn.reset();    // joins worker — nothing touches the FIFO after this
    flushDelayQueue(ctx); // free any frames still buffered
    delete ctx;
}

void source_update(void *data, obs_data_t *settings) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    const std::string prevDev = ctx->dev, prevSrc = ctx->src;
    apply_settings(ctx, settings); // delay changes apply immediately (read per-frame)
    applyCameraControls(ctx, settings, /*sendDiffs=*/true); // send whatever the operator changed
    if (ctx->dev != prevDev || ctx->src != prevSrc)
        rebuildConnection(ctx); // re-advertise under the new display names
    applyAuth(ctx, settings); // (re-)push auth — also re-pushes after a rebuild
}

void source_get_defaults(obs_data_t *settings) {
    obs_data_set_default_string(settings, kSettingDeviceName, "OBS Airlive");
    // No default for source_name — apply_settings adopts the OBS source name once.
    obs_data_set_default_int(settings, kSettingDelayMs, kDefaultDelayMs);
    obs_data_set_default_bool(settings, kSettingRequireAuth, false); // open by default

    obs_data_set_default_bool(settings, kCamAE, true);
    obs_data_set_default_int(settings, kCamISO, 100);
    obs_data_set_default_int(settings, kCamShutter, 50);
    obs_data_set_default_bool(settings, kCamAWB, true);
    obs_data_set_default_int(settings, kCamWB, 5600);
    obs_data_set_default_int(settings, kCamTint, 0);
    obs_data_set_default_bool(settings, kCamAF, true);
    obs_data_set_default_double(settings, kCamFocus, 0.5);
    obs_data_set_default_string(settings, kCamLens, "1x");
    obs_data_set_default_int(settings, kCamFPS, 30);
    obs_data_set_default_double(settings, kCamZoom, 1.0);
    obs_data_set_default_bool(settings, kCamLUT, false);
}

// Tally callbacks — recompute and push the cue on every activation/visibility change.
void source_activate(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_deactivate(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_show(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_hide(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }

std::string buildStatusText(AirliveSourceCtx *ctx) {
    std::string s;
    const bool conn = ctx->conn && ctx->conn->connected();
    s += conn ? "● Connected" : "○ Waiting for iPhone…";

    if (conn) {
        const uint32_t w = ctx->statW.load(), h = ctx->statH.load();
        const int fps = int(ctx->statFps.load() + 0.5);
        if (w && h) {
            s += "\n" + std::to_string(w) + "×" + std::to_string(h);
            if (fps > 0)
                s += " @ " + std::to_string(fps) + " fps";
        }
        if (const int rot = ctx->rotationDeg.load())
            s += "\nRotation: " + std::to_string(rot) + "° " +
                 ((rot == 90 || rot == 270) ? "(portrait)" : "(flipped)");
        std::lock_guard<std::mutex> lk(ctx->stateMutex);
        if (!ctx->rDevice.empty())
            s += "\nDevice: " + ctx->rDevice;
        if (!ctx->rResolution.empty() || !ctx->rColorSpace.empty())
            s += "\nCamera: " + ctx->rResolution + (ctx->rColorSpace.empty() ? "" : " " + ctx->rColorSpace);
        if (!ctx->rLens.empty())
            s += "\nLens: " + ctx->rLens;
        // Live exposure / WB readout (actual camera values, not the slider state).
        if (ctx->rIso > 0)
            s += "\nISO " + std::to_string(int(ctx->rIso + 0.5)) + (ctx->rAE ? " (auto)" : "");
        if (ctx->rShutter > 0)
            s += "  ·  1/" + std::to_string(int(ctx->rShutter + 0.5));
        if (ctx->rWB > 0)
            s += "\nWB " + std::to_string(int(ctx->rWB + 0.5)) + "K" + (ctx->rAWB ? " (auto)" : "");
    }
    // Auth status (independent of connection) — confirms a password is stored,
    // since the password field blanks itself after saving (it's in the Keychain).
    {
        obs_data_t *st = obs_source_get_settings(ctx->source);
        const bool req = obs_data_get_bool(st, kSettingRequireAuth);
        obs_data_release(st);
        const bool hasPw = !secretStoreGet(ctx->sid).empty();
        if (req && hasPw)
            s += "\n🔒 Password required";
        else if (req && !hasPw)
            s += "\n⚠️ Require password is ON but none set — channel stays OPEN";
    }
    return s;
}

// Grey out the manual controls whose auto-mode is on. Fires when an auto toggle
// changes; returning true tells OBS to refresh the property view.
bool cam_auto_modified(obs_properties_t *props, obs_property_t *, obs_data_t *settings) {
    const bool ae = obs_data_get_bool(settings, kCamAE);
    const bool awb = obs_data_get_bool(settings, kCamAWB);
    const bool af = obs_data_get_bool(settings, kCamAF);
    obs_property_set_enabled(obs_properties_get(props, kCamISO), !ae);
    obs_property_set_enabled(obs_properties_get(props, kCamShutter), !ae);
    obs_property_set_enabled(obs_properties_get(props, kCamWB), !awb);
    obs_property_set_enabled(obs_properties_get(props, kCamTint), !awb);
    obs_property_set_enabled(obs_properties_get(props, kCamFocus), !af);
    return true;
}

bool refresh_clicked(obs_properties_t *, obs_property_t *, void *) {
    return true; // returning true rebuilds the property view -> re-reads status
}

// Delete the stored password from the Keychain and push the now-empty config to
// the connection (auth effectively off until a new password is set).
bool clear_password_clicked(obs_properties_t *, obs_property_t *, void *data) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    secretStoreDelete(ctx->sid);
    obs_data_t *s = obs_source_get_settings(ctx->source);
    obs_data_set_string(s, kSettingPassword, "");
    const bool require = obs_data_get_bool(s, kSettingRequireAuth);
    obs_data_release(s);
    if (ctx->conn)
        ctx->conn->setAuth(require, ""); // no password → auth disengages
    return true; // rebuild the view (status reflects the cleared state)
}

obs_properties_t *source_get_properties(void *data) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    obs_properties_t *props = obs_properties_create();

    if (ctx) {
        const std::string status = buildStatusText(ctx);
        obs_properties_add_text(props, "status", status.c_str(), OBS_TEXT_INFO);
        obs_properties_add_button2(props, "refresh", obs_module_text("Airlive.Refresh"),
                                   refresh_clicked, ctx);
    }

    // Fixed-delay presets — mirror the Studio app's LatencyPreset values 1:1
    // (real industry standards, not round numbers). A discrete list (not a
    // slider) so the operator picks an exact, meaningful value; OBS applies the
    // choice immediately via source_update.
    obs_property_t *delay = obs_properties_add_list(props, kSettingDelayMs,
                                                    obs_module_text("Airlive.Delay"),
                                                    OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(delay, "Unbuffered (+0 ms)", 0);  // OBS async_unbuffered / WebRTC min=0
    obs_property_list_add_int(delay, "Normal (+120 ms)", 120);  // SRT default jitter buffer
    obs_property_list_add_int(delay, "Smooth (+200 ms)", 200);  // WebRTC interactive upper bound
    obs_property_list_add_int(delay, "Safe (+400 ms)", 400);    // WebRTC buffer-against-glitches
    obs_property_set_long_description(
        delay, "Extra buffer on top of base latency. \"+0\" isn't true zero — it's \"show on "
               "decode\". Higher = smoother on bad networks.");
    obs_properties_add_text(props, kSettingDeviceName, obs_module_text("Airlive.DeviceName"),
                            OBS_TEXT_DEFAULT);
    obs_properties_add_text(props, kSettingSourceName, obs_module_text("Airlive.SourceLabel"),
                            OBS_TEXT_DEFAULT);

    // ---- Security: receiver-password auth (STREAM-AUTH-SPEC §4) ----
    // Off by default — the stream stays open until the operator turns this on AND
    // sets a password. Closes the same-LAN-prankster threat (slot-grab / fake-feed
    // injection); it is ACCESS control, not encryption. The camera answers an HMAC
    // challenge — the password never crosses the wire. The password is stored in
    // the macOS Keychain (see applyAuth) — never plaintext in the scene file.
    obs_properties_add_text(props, "sec_hdr", "— Security —", OBS_TEXT_INFO);
    obs_properties_add_bool(props, kSettingRequireAuth, "Require password");
    obs_property_t *pw = obs_properties_add_text(props, kSettingPassword, "Password", OBS_TEXT_PASSWORD);
    obs_property_set_long_description(
        pw, "Cameras must enter this to connect. Stored in the macOS Keychain (the field "
            "blanks after saving). Changing it takes effect on the next connection "
            "(re-tap Live on the iPhone). ASCII only.");
    obs_properties_add_button2(props, "clear_password", "Clear stored password",
                               clear_password_clicked, ctx);

    // ---- Camera control (Phase 2a) ----
    // Send-only knobs: changing one ships a set-command to the iPhone. The
    // actual camera values are in the status readout above (Refresh to update).
    obs_properties_add_text(props, "cam_hdr", "— Camera control —", OBS_TEXT_INFO);

    obs_property_t *ae = obs_properties_add_bool(props, kCamAE, "Auto exposure");
    obs_property_t *iso = obs_properties_add_int_slider(props, kCamISO, "ISO", 20, 12800, 1);
    obs_property_t *shutter =
        obs_properties_add_int_slider(props, kCamShutter, "Shutter 1/x", 24, 8000, 1);

    obs_property_t *awb = obs_properties_add_bool(props, kCamAWB, "Auto white balance");
    obs_property_t *wb = obs_properties_add_int_slider(props, kCamWB, "White balance (K)", 2300, 10000, 50);
    obs_property_t *tint = obs_properties_add_int_slider(props, kCamTint, "Tint", -150, 150, 1);

    obs_property_t *af = obs_properties_add_bool(props, kCamAF, "Auto focus");
    obs_property_t *focus = obs_properties_add_float_slider(props, kCamFocus, "Focus", 0.0, 1.0, 0.01);

    obs_property_t *lens =
        obs_properties_add_list(props, kCamLens, "Lens", OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
    for (const char *l : {"0.5x", "1x", "2x", "3x", "5x"})
        obs_property_list_add_string(lens, l, l);

    obs_property_t *fps =
        obs_properties_add_list(props, kCamFPS, "FPS", OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    for (int f : {24, 25, 30, 50, 60})
        obs_property_list_add_int(fps, std::to_string(f).c_str(), f);

    obs_properties_add_float_slider(props, kCamZoom, "Zoom", 1.0, 15.0, 0.1);
    obs_properties_add_bool(props, kCamLUT, "LUT on");

    // Auto toggles drive the grey-out of their manual partners — live (callback)
    // and on open (seed from the last-known auto state in ctx).
    obs_property_set_modified_callback(ae, cam_auto_modified);
    obs_property_set_modified_callback(awb, cam_auto_modified);
    obs_property_set_modified_callback(af, cam_auto_modified);
    if (ctx) {
        obs_property_set_enabled(iso, !ctx->cAE);
        obs_property_set_enabled(shutter, !ctx->cAE);
        obs_property_set_enabled(wb, !ctx->cAWB);
        obs_property_set_enabled(tint, !ctx->cAWB);
        obs_property_set_enabled(focus, !ctx->cAF);
    }
    return props;
}

struct obs_source_info airlive_source_info = []() {
    struct obs_source_info info = {};
    // Distinct id so OBS never confuses this with any older Airlive
    // screen-mirroring plugin already installed on the machine.
    info.id = "airlive_obs_source";
    info.type = OBS_SOURCE_TYPE_INPUT;
    info.output_flags = OBS_SOURCE_ASYNC_VIDEO | OBS_SOURCE_DO_NOT_DUPLICATE;
    info.icon_type = OBS_ICON_TYPE_CAMERA;
    info.get_name = source_get_name;
    // Async sources get their dimensions from the output frames, so these are
    // optional — but some OBS internals/plugins query them before the first
    // frame; back them with the live stats so they report real dims once video
    // is flowing (0 until then, standard for async sources).
    info.get_width = [](void *d) -> uint32_t { return static_cast<AirliveSourceCtx *>(d)->statW.load(); };
    info.get_height = [](void *d) -> uint32_t { return static_cast<AirliveSourceCtx *>(d)->statH.load(); };
    info.create = source_create;
    info.destroy = source_destroy;
    info.update = source_update;
    info.get_defaults = source_get_defaults;
    info.get_properties = source_get_properties;
    info.activate = source_activate;
    info.deactivate = source_deactivate;
    info.show = source_show;
    info.hide = source_hide;
    return info;
}();

} // namespace

bool obs_module_load(void) {
#ifdef _WIN32
    // Start Winsock once for the whole module (not per source/worker) so the
    // reference count is balanced and we never tear Winsock out from under OBS.
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#else
    // Ignore SIGPIPE process-wide: a send() to a peer that just vanished must
    // never terminate OBS. Our own sends already pass MSG_NOSIGNAL (Linux) /
    // run on SO_NOSIGPIPE sockets (macOS); this is a belt-and-suspenders guard
    // against any future or third-party send path that forgets the flag.
    signal(SIGPIPE, SIG_IGN);
#endif
    obs_register_source(&airlive_source_info);
    blog(LOG_INFO, "[airlive-obs] OBS Airlive loaded");
    return true;
}

void obs_module_unload(void) {
#ifdef _WIN32
    WSACleanup();
#endif
    blog(LOG_INFO, "[airlive-obs] OBS Airlive unloaded");
}

MODULE_EXPORT const char *obs_module_description(void) {
    return "OBS Airlive — iPhone (Airlive Camera) as an async video source over LAN";
}

MODULE_EXPORT const char *obs_module_name(void) {
    return "OBS Airlive";
}
