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
constexpr const char *kSettingOrder = "order"; // operator's channel order → Bonjour "ord" (#6)
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
constexpr const char *kCamLUT = "cam_lut";           // legacy on/off bool (cameras with no availableLuts)
// Parity-with-Bridge additions — each maps to a set-command whose EXACT verb / value-field / raw
// string values match the iPhone's ControlMessage (a mismatched string is a SILENT no-op camera-side).
// The list controls (res / colour space / stabilization / LUT-by-name) are populated verbatim from
// the caps the camera SENDS, so their option strings are correct by construction.
constexpr const char *kCamEV = "cam_ev";             // -> setExposureBias (floatValue)
constexpr const char *kCamIsoComp = "cam_iso_comp";  // -> setIsoCompensation (boolValue)
constexpr const char *kCamRes = "cam_res";           // -> setResolution (stringValue) — caps.resolutions
constexpr const char *kCamColorSpace = "cam_colorspace"; // -> setColorSpace (stringValue) — caps.colorSpaces
constexpr const char *kCamStab = "cam_stab";         // -> setStabilization (stringValue) — caps.stabilizations
constexpr const char *kCamLutName = "cam_lut_name";  // -> setLUT (lutName; "" = clear) — caps.availableLuts

constexpr int kDefaultDelayMs = 60; // LAN default (~2 frames @30fps). Our no-B-frame stream on a LAN
                                    // has far less jitter than the 120ms SRT/WAN figure; presets
                                    // 0/120/200/400 still let a bad-network operator dial back up.
constexpr int kMaxDelayMs = 400;     // matches the highest preset (Safe)
constexpr size_t kSafetyCapFrames = 600; // anti-runaway if timestamps stop advancing

// No decoded frame for this long ⇒ paint black. The on-air source lost signal, dropped, or the
// Bridge deliberately cut its program to black (it just stops forwarding). Long enough to ride out
// a single dropped frame / keyframe gap at 30 fps, short enough to feel like a deliberate cut.
constexpr int64_t kNoSignalBlackNs = 600'000'000; // 600 ms

// Monotonic wall-clock in ns — for stall detection only (NOT frame timestamps, which stay on the
// capture PTS timeline). steady_clock never jumps backwards on an NTP step.
inline int64_t nowMonoNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

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
    std::string role = "obs"; // "obs" (direct iPhone) or "obs-bridge" (Bridge program)
    std::string ord = "0";    // operator's channel order → Bonjour TXT "ord" (asymmetry #6)
    // connMutex guards the `conn` POINTER itself (not the object — AirliveConnection is
    // internally thread-safe). libobs serializes update/video_tick/activate/show on the ONE
    // graphics thread (obs-source.c: video sources defer info.update into obs_source_video_tick,
    // which also fires activate/show), but rebuildConnection's reset/reassign in update still
    // races the OTHER threads that read conn: the UI thread (buildStatusText via get_properties,
    // clear_password_clicked) and the worker thread (ensureTally via onControl). NEVER destroy a
    // connection while holding this mutex — ~AirliveConnection joins the worker, whose callbacks
    // take it (deadlock); see teardownConnection.
    std::mutex connMutex;
    std::unique_ptr<AirliveConnection> conn;

    // Peer (camera) protocol generation from its one-shot hello (PROTOCOL-COMPAT-SPEC §2).
    // 0 = no hello seen yet / legacy peer (proto=1). Read/written under stateMutex. Used for the
    // update-prompt detection; feature availability is decided by the hello caps, never these.
    int rProto = 0, rMinProto = 0;
    std::vector<std::string> rCaps; // camera's advertised feature caps — future gating + matrix C1

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
    // Parity-with-Bridge readback (StateSnapshot fields) — feed the new control widgets' current
    // selection, exactly like rLens/rIso above.  Guarded by stateMutex.
    double rExposureBias = 0;              // StateSnapshot.exposureBias
    bool rIsoComp = false;                 // StateSnapshot.isoCompensation
    std::string rStabilization;            // StateSnapshot.stabilization ("Standard"/"High")
    std::string rLutName;                  // StateSnapshot.lutName (current selection, "" when cleared)
    bool rLutEnabled = false;              // StateSnapshot.lutEnabled (selected != nil)
    // Device-read capability ranges (from StateSnapshot.capabilities) so the remote-control
    // sliders match THIS phone instead of a hardcoded table.  Defaults = the previous hardcoded
    // slider bounds, so an old camera that sends no `capabilities` = unchanged behaviour.
    // (Guarded by stateMutex, like the r* fields above.)
    double capIsoMin = 20, capIsoMax = 12800;
    double capShutterMin = 24, capShutterMax = 8000;
    double capWbMin = 2300, capWbMax = 10000;
    double capTintMin = -150, capTintMax = 150;
    // Parity-with-Bridge caps.  EV bounds default to the DeviceCapabilities defaults (-2..+2), so an
    // old camera that sends no evBias keeps a sane EV slider.  zoomMax 0 = "not provided" → the zoom
    // slider keeps its hardcoded max.
    double capEvBiasMin = -2, capEvBiasMax = 2;
    double capZoomMax = 0;
    // Device-read OPTION lists (availableLenses, capabilities.supportedFps).  Empty until a
    // state arrives → the properties UI falls back to the standard ladders.
    std::vector<std::string> capLenses;
    std::vector<int> capFps;
    // Parity-with-Bridge option lists (all nested in capabilities).  Empty = the camera doesn't offer
    // it (legacy or unsupported) → that control is HIDDEN, never shown as a dead picker.
    std::vector<std::string> capResolutions;   // "1080" / "4K"
    std::vector<std::string> capColorSpaces;   // "Rec.709"/"P3 D65"/"Rec.2020 HLG"/"Apple Log"
    std::vector<std::string> capStabilizations; // "Standard"/"High"
    std::vector<std::string> capAvailableLuts; // phone's quick-access LUT names
    bool capsReceived = false; // a StateSnapshot.capabilities object has arrived (vs. the defaults)
    // Set by the "Refresh" button (UI thread); consumed by onControl (worker thread) to issue the
    // actual obs_source_update_properties() — a property callback can't reload inline without UAF.
    std::atomic<bool> refreshRequested{false};

    // Set false at the very top of source_destroy so no thread pokes the OBS UI once teardown
    // has begun — see the CRASH note on obs_source_update_properties in source_video_tick.
    std::atomic<bool> alive{true};

    // No-signal→black + live status, driven from video_tick. The two atomics are written on the
    // worker thread (real frame emit) and read on the video thread; the plain fields below are
    // touched ONLY in video_tick (single owner, no locks).
    std::atomic<int64_t> lastFrameMonoNs{0};  // nowMonoNs() of the last REAL emitted frame (0 = none yet)
    std::atomic<uint64_t> lastEmittedTsNs{0}; // last frame.timestamp handed to OBS (black continues it)
    bool tickBlanked = false;                 // a black frame is currently shown (emit once per gap)
    std::vector<uint8_t> blackBuf;            // reusable black I420 (built lazily, rebuilt on resize)
    int blackW = 0, blackH = 0;               // dims blackBuf currently holds

    // Camera-control last-sent values: used to (a) detect which control the
    // operator actually changed in update(), and (b) seed the auto/manual
    // grey-out state when the properties dialog opens. Touched on the OBS thread.
    bool ccInit = false;
    bool cAE = true, cAWB = true, cAF = true, cLUT = false;
    int cISO = 100, cShutter = 50, cWB = 5600, cTint = 0, cFPS = 30;
    double cFocus = 0.5, cZoom = 1.0;
    std::string cLens = "1x";
    // Parity-with-Bridge send-cache — same diff-then-send discipline as the fields above.  The list
    // controls default to "" (no camera value seen yet); "" for the LUT name is the "None" selection.
    bool cIsoComp = false;
    double cEV = 0.0;
    std::string cRes, cColorSpace, cStab, cLutName;

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
    ctx->lastFrameMonoNs.store(nowMonoNs());     // video_tick no-signal detection
    ctx->lastEmittedTsNs.store(frame.timestamp); // keep any black frame on this same timeline
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
    ctx->lastFrameMonoNs.store(nowMonoNs());     // video_tick no-signal detection
    ctx->lastEmittedTsNs.store(frame.timestamp); // keep any black frame on this same timeline
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

    // Re-converge after a FORWARD timestamp gap (a lens/format switch pauses the
    // camera ~0.3-0.5 s; a network stall does the same). Such a gap inflates the
    // buffered span far past the target, and emitting one-per-frame would carry
    // that gap as several frames of EXTRA latency on EVERY switch ("при смене линзы
    // каждый раз добавляется задержка"). Instead drop the stale head in one shot so
    // latency snaps back to the target. The margin is wide enough that ordinary
    // capture jitter never trips it in steady state. (Backward jumps / >1 s gaps are
    // already flushed by isTimestampJump above.)
    const int64_t reconvergeMarginUs = 250000; // 250 ms over the target
    while (ctx->dq.size() > 1 &&
           (ctx->dq.back().pts_us - ctx->dq.front().pts_us) > intervalUs + reconvergeMarginUs) {
        av_frame_free(&ctx->dq.front().frame);
        ctx->dq.pop_front();
    }

    QueuedFrame out = ctx->dq.front();
    ctx->dq.pop_front();
    outputFrame(ctx, out.frame, out.pts_us);
    av_frame_free(&out.frame);

    // Re-enter "filling" once the buffer falls below the target span — otherwise a forward gap that
    // drains the queue (reconverge above, or a stall) leaves delayReached stuck true, so the FIFO
    // emits one-in/one-out at ~0 depth and the operator's chosen jitter buffer silently collapses
    // for the rest of the session. Resetting lets the next frames rebuild the delay-worth span.
    if (ctx->dq.empty() ||
        (ctx->dq.back().pts_us - ctx->dq.front().pts_us) < intervalUs)
        ctx->delayReached = false;
}

// ---- tally (Studio Mode -> setCue) ----------------------------------------

// Send the current cue to the iPhone. force=true sends even if unchanged (used
// when the phone (re)connects, so it learns it's already live).
//
// active → "program" (on air); else showing → "preview" (staged in Studio Mode);
// else "none". The cue is a closed set of literals, so building the JSON by
// concatenation is safe — no escaping needed.
void ensureTally(AirliveSourceCtx *ctx, bool force) {
    // Hold connMutex across the whole check-then-send: the worker thread lands here from
    // onControl while update (graphics thread) may be swapping `conn` in rebuildConnection.
    // Cheap to hold — sendControl only ENQUEUES (never blocks on the socket).
    std::lock_guard<std::mutex> ck(ctx->connMutex);
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
// Minimal extractors for a JSON array of strings / ints by key, from a WELL-FORMED control
// message (Swift JSONEncoder output).  Needed because `obs_data` can't read JSON arrays of
// primitives and jansson isn't exposed to OBS plugins.  NOT a general JSON parser, but it IS
// escape-aware: it only treats `]` as the array terminator OUTSIDE a string, and honours `\"`
// / `\\` inside a value — because `availableLuts` carries user-chosen `.cube` filenames that can
// contain `]` or `"` (e.g. `Rec709 [Warm].cube`).  Returns the first match (keys are unique).
static std::vector<std::string> extractJsonStringArray(const std::string &j, const char *key) {
    std::vector<std::string> out;
    const std::string pat = std::string("\"") + key + "\"";
    size_t k = j.find(pat);
    if (k == std::string::npos) return out;
    size_t lb = j.find('[', k);
    if (lb == std::string::npos) return out;
    size_t i = lb + 1;
    while (i < j.size()) {
        // Between elements: skip until a string opens (") or the array closes (]).
        // A ']' here is genuinely the terminator — we are not inside a string.
        while (i < j.size() && j[i] != '"' && j[i] != ']') i++;
        if (i >= j.size() || j[i] == ']') break;
        // Consume one JSON string, honouring \" and \\ so ']' / '"' in the value are literal.
        ++i; // past opening quote
        std::string val;
        while (i < j.size() && j[i] != '"') {
            if (j[i] == '\\' && i + 1 < j.size()) {
                char c = j[i + 1];
                val.push_back(c == 'n' ? '\n' : c == 't' ? '\t' : c == 'r' ? '\r' : c);
                i += 2;
            } else {
                val.push_back(j[i++]);
            }
        }
        if (i >= j.size()) break; // unterminated string
        ++i;                      // past closing quote
        out.push_back(val);
    }
    return out;
}
static std::vector<int> extractJsonIntArray(const std::string &j, const char *key) {
    std::vector<int> out;
    const std::string pat = std::string("\"") + key + "\"";
    size_t k = j.find(pat);
    if (k == std::string::npos) return out;
    size_t lb = j.find('[', k);
    size_t rb = (lb == std::string::npos) ? std::string::npos : j.find(']', lb);
    if (lb == std::string::npos || rb == std::string::npos) return out;
    size_t i = lb + 1;
    while (i < rb) {
        while (i < rb && (j[i] == ' ' || j[i] == ',')) i++;
        if (i >= rb) break;
        size_t start = i;
        while (i < rb && j[i] != ',') i++;
        try { out.push_back(std::stoi(j.substr(start, i - start))); } catch (...) {}
    }
    return out;
}

void onControl(AirliveSourceCtx *ctx, const char *json, size_t len) {
    const std::string s(json, len); // null-terminate for the JSON parser
    obs_data_t *msg = obs_data_create_from_json(s.c_str());
    if (!msg)
        return;

    // One-shot hello (PROTOCOL-COMPAT-SPEC §2): the camera's introduction — app/version + its
    // protocol generation + FEATURE caps. Snapshot the ints for the update-prompt detection; a
    // camera that predates hello simply never sends this (legacy → proto stays 1).
    if (obs_data_t *h = obs_data_get_obj(msg, "hello")) {
        const int proto = int(obs_data_get_int(h, "proto"));
        const int minProto = int(obs_data_get_int(h, "minProto"));
        const char *app = obs_data_get_string(h, "app");
        const char *ver = obs_data_get_string(h, "appVersion");
        // caps is an array of strings inside the hello object; obs_data can't read primitive
        // arrays, so use the same substring extractor as availableLenses/supportedFps.
        const std::vector<std::string> caps = extractJsonStringArray(s, "caps");
        std::string capsJoined;
        for (size_t i = 0; i < caps.size(); ++i)
            capsJoined += (i ? "," : "") + caps[i];
        {
            std::lock_guard<std::mutex> lk(ctx->stateMutex);
            ctx->rProto = proto ? proto : 1;
            ctx->rMinProto = minProto ? minProto : 1;
            ctx->rCaps = caps;
        }
        blog(LOG_INFO, "[airlive] hello: app=%s %s proto=%d minProto=%d caps=[%s]",
             app && *app ? app : "?", ver && *ver ? ver : "?", proto, minProto, capsJoined.c_str());
        // Update-prompt DETECTION (§6) — LOG ONLY for now. The properties-panel red text lands
        // later (per §7 rollout). NEVER a modal, NEVER obs_source_update_properties from this
        // worker thread on a plugin-owned source mid-callback — that's the known macOS-26 SEGV.
        if (proto && proto < kProtoMinGeneration)
            blog(LOG_WARNING, "[airlive] camera proto %d < our minProto %d — camera app needs an update",
                 proto, kProtoMinGeneration);
        if (minProto && kProtoGeneration < minProto)
            blog(LOG_WARNING, "[airlive] our proto %d < camera minProto %d — THIS plugin needs an update",
                 kProtoGeneration, minProto);
        obs_data_release(h);
    }

    bool gotState = false;
    if (obs_data_t *st = obs_data_get_obj(msg, "state")) {
        std::string capsLog; // built under the lock, emitted after release (no I/O under stateMutex)
        bool discreteChanged = false; // lens/fps/auto/first-caps changed → push a live dialog reload
        {
            std::lock_guard<std::mutex> lk(ctx->stateMutex);
            const std::string oRLens = ctx->rLens;
            const int oRFps = ctx->rFps;
            const bool oRAE = ctx->rAE, oRAWB = ctx->rAWB;
            // Old discrete format states — a change pushes a live dialog reload (below) so the seeded
            // pickers refresh, exactly like lens/fps.
            const std::string oRRes = ctx->rResolution, oRCS = ctx->rColorSpace, oRStab = ctx->rStabilization;
            const std::string oRLutName = ctx->rLutName;
            const bool oRLutEnabled = ctx->rLutEnabled;
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
            // Parity-with-Bridge state (additive fields; missing key → 0/false/"" default).
            ctx->rExposureBias = obs_data_get_double(st, "exposureBias");
            ctx->rIsoComp = obs_data_get_bool(st, "isoCompensation");
            ctx->rStabilization = obs_data_get_string(st, "stabilization");
            ctx->rLutName = obs_data_get_string(st, "lutName");
            ctx->rLutEnabled = obs_data_get_bool(st, "lutEnabled");
            // Device-read capability RANGES (additive object).  `has_user_value` guards each so a
            // missing key keeps the default (and so tint's negative min isn't clobbered to 0).
            const bool oCapsReceived = ctx->capsReceived;
            const double oIsoMax = ctx->capIsoMax;
            const size_t oLensN = ctx->capLenses.size(), oFpsN = ctx->capFps.size();
            if (obs_data_t *caps = obs_data_get_obj(st, "capabilities")) {
                ctx->capsReceived = true; // this phone reports real ranges, not our defaults
                if (obs_data_has_user_value(caps, "isoMin"))          ctx->capIsoMin     = obs_data_get_double(caps, "isoMin");
                if (obs_data_has_user_value(caps, "isoMax"))          ctx->capIsoMax     = obs_data_get_double(caps, "isoMax");
                if (obs_data_has_user_value(caps, "shutterMinDenom")) ctx->capShutterMin = obs_data_get_double(caps, "shutterMinDenom");
                if (obs_data_has_user_value(caps, "shutterMaxDenom")) ctx->capShutterMax = obs_data_get_double(caps, "shutterMaxDenom");
                if (obs_data_has_user_value(caps, "wbTempMin"))       ctx->capWbMin      = obs_data_get_double(caps, "wbTempMin");
                if (obs_data_has_user_value(caps, "wbTempMax"))       ctx->capWbMax      = obs_data_get_double(caps, "wbTempMax");
                if (obs_data_has_user_value(caps, "wbTintMin"))       ctx->capTintMin    = obs_data_get_double(caps, "wbTintMin");
                if (obs_data_has_user_value(caps, "wbTintMax"))       ctx->capTintMax    = obs_data_get_double(caps, "wbTintMax");
                if (obs_data_has_user_value(caps, "evBiasMin"))       ctx->capEvBiasMin  = obs_data_get_double(caps, "evBiasMin");
                if (obs_data_has_user_value(caps, "evBiasMax"))       ctx->capEvBiasMax  = obs_data_get_double(caps, "evBiasMax");
                if (obs_data_has_user_value(caps, "zoomMax"))         ctx->capZoomMax    = obs_data_get_double(caps, "zoomMax");
                obs_data_release(caps);
            }
            // Option lists are JSON arrays of primitives — obs_data can't read those and jansson
            // isn't exposed to plugins, so they're pulled from the raw message by the targeted
            // extractors above. Only overwrite when present so an old camera keeps the fallback ladder.
            auto lenses = extractJsonStringArray(s, "availableLenses");
            if (!lenses.empty()) ctx->capLenses = std::move(lenses);
            auto fpsList = extractJsonIntArray(s, "supportedFps");
            if (!fpsList.empty()) ctx->capFps = std::move(fpsList);
            // Parity-with-Bridge option lists — same targeted-extractor pattern (obs_data can't read
            // JSON primitive arrays). Each key is unique in the message and its values contain no `"`
            // or `]` (enum rawValues / short strings / LUT filenames), so the substring scan is safe.
            // Only overwrite when present, so a legacy camera keeps these EMPTY → the control hides.
            auto resList = extractJsonStringArray(s, "resolutions");
            if (!resList.empty()) ctx->capResolutions = std::move(resList);
            auto csList = extractJsonStringArray(s, "colorSpaces");
            if (!csList.empty()) ctx->capColorSpaces = std::move(csList);
            auto stabList = extractJsonStringArray(s, "stabilizations");
            if (!stabList.empty()) ctx->capStabilizations = std::move(stabList);
            auto lutList = extractJsonStringArray(s, "availableLuts");
            if (!lutList.empty()) ctx->capAvailableLuts = std::move(lutList);
            // Format the "caps changed" line here (cheap, CPU-only) but DON'T blog yet —
            // emit it after the lock releases so file I/O never runs inside the critical
            // section (matches ensureTally). Logged once per ACTUAL change so the operator
            // can confirm in the OBS log that THIS phone's caps arrived vs. default ranges.
            if (ctx->capIsoMax != oIsoMax || ctx->capLenses.size() != oLensN ||
                ctx->capFps.size() != oFpsN) {
                char buf[256];
                snprintf(buf, sizeof(buf),
                         "[airlive] capabilities from phone: ISO %d-%d, shutter 1/%d-1/%d, "
                         "WB %d-%dK, tint %d..%d, lenses=%zu, fps=%zu",
                         int(ctx->capIsoMin), int(ctx->capIsoMax), int(ctx->capShutterMin),
                         int(ctx->capShutterMax), int(ctx->capWbMin), int(ctx->capWbMax),
                         int(ctx->capTintMin), int(ctx->capTintMax), ctx->capLenses.size(),
                         ctx->capFps.size());
                capsLog = buf;
            }
            // Only a DISCRETE change pushes a live dialog reload (below). iso/shutter/wb drift
            // continuously under auto-exposure — reloading on those would rebuild the dialog ~1 Hz
            // and fight an operator mid-edit. A first capabilities arrival also counts (ranges show).
            discreteChanged = (ctx->rLens != oRLens) || (ctx->rFps != oRFps) ||
                              (ctx->rAE != oRAE) || (ctx->rAWB != oRAWB) ||
                              (ctx->rResolution != oRRes) || (ctx->rColorSpace != oRCS) ||
                              (ctx->rStabilization != oRStab) || (ctx->rLutName != oRLutName) ||
                              (ctx->rLutEnabled != oRLutEnabled) ||
                              (!oCapsReceived && ctx->capsReceived);
        }
        if (!capsLog.empty())
            blog(LOG_INFO, "%s", capsLog.c_str());
        // Make the OPEN Properties dialog (if any) re-run get_properties so the status text, the
        // capability ranges and the control-widget reseed reflect the new state. MUST come from this
        // worker thread, NOT a property callback: from the UI thread the signal handler re-enters and
        // frees the live widgets mid-callback (UAF). Off-thread the frontend queues the reload safely.
        // No-op when no dialog is open. Also serves the manual "Refresh" button (sets refreshRequested).
        // `alive` gate: never poke the UI once teardown started (see the crash note in video_tick).
        if (ctx->alive.load() && (discreteChanged || ctx->refreshRequested.exchange(false)))
            obs_source_update_properties(ctx->source);
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

// The live camera disconnected — reset the hello epoch so the NEXT connection is read fresh.
// Without this, a LEGACY camera reconnecting after a modern one (it sends no hello) would be
// misread against the previous peer's proto/caps; PROTOCOL-COMPAT-SPEC §B requires a per-epoch
// reset. Runs on the worker thread; guarded like the other r* fields.
void onDisconnect(AirliveSourceCtx *ctx) {
    std::lock_guard<std::mutex> lk(ctx->stateMutex);
    ctx->rProto = 0;
    ctx->rMinProto = 0;
    ctx->rCaps.clear();
    // Clear cached device-read option lists + reset the additive ranges to their defaults, so a
    // DIFFERENT phone reconnecting to this same OBS slot never shows the previous camera's stale
    // dropdowns/bounds (matches the Bridge nil-ing peerCaps on disconnect). The next StateSnapshot
    // repopulates them. (The static iso/shutter/wb/tint bounds are left alone — the next state's
    // capabilities object overwrites them within ~1 s and they degrade to sane sliders meanwhile.)
    ctx->capLenses.clear();
    ctx->capFps.clear();
    ctx->capResolutions.clear();
    ctx->capColorSpaces.clear();
    ctx->capStabilizations.clear();
    ctx->capAvailableLuts.clear();
    ctx->capZoomMax = 0;
    ctx->capEvBiasMin = -2;
    ctx->capEvBiasMax = 2;
}

// ---- camera control (Phase 2a — send set-commands) ------------------------

// Each command is a tiny type-2 JSON ControlMessage. We build it with obs_data
// (correct escaping + number formatting, same lib we parse with) and ship it
// over the existing control channel. Field names match the iPhone's Codable
// ControlMessage (floatValue/intValue/stringValue/boolValue/lutName).
void sendCmd(AirliveSourceCtx *ctx, obs_data_t *d) {
    {
        std::lock_guard<std::mutex> lk(ctx->connMutex); // vs. rebuildConnection's swap
        if (ctx->conn)
            ctx->conn->sendControl(obs_data_get_json(d));
    }
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
// setLUT is special: the LUT name rides a DEDICATED `lutName` field (NOT stringValue), alongside a
// `boolValue` enable flag — matching the iPhone's ControlMessage.setLUT(name:enabled:).  A NON-empty
// name selects that LUT; an EMPTY name is the camera's explicit "clear the LUT" signal (the "None"
// entry in the dropdown).  (Verified against AirliveCore/Packet.swift setLUT + CaptureEngine handler.)
void sendCmdLut(AirliveSourceCtx *ctx, const std::string &name) {
    obs_data_t *d = obs_data_create();
    obs_data_set_string(d, "type", "setLUT");
    obs_data_set_bool(d, "boolValue", !name.empty());
    obs_data_set_string(d, "lutName", name.c_str());
    sendCmd(ctx, d);
}

// True if the camera advertised feature-cap token `cap` in its one-shot hello (rCaps). Used to
// DOUBLE-gate the format pickers exactly like the Bridge (hasCap + a non-empty option list), so a
// camera that lists options but never advertised the token still hides that control. rCaps is
// cleared on disconnect, so a stale token can't survive a peer swap.
bool hasCap(AirliveSourceCtx *ctx, const char *cap) {
    std::lock_guard<std::mutex> lk(ctx->stateMutex);
    for (const auto &c : ctx->rCaps)
        if (c == cap)
            return true;
    return false;
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
    // Parity-with-Bridge controls.  Hidden controls (empty caps) keep their default value, which
    // always equals the cache, so their diff never fires — a legacy camera is never sent a command
    // for a control it doesn't expose.
    const double ev = obs_data_get_double(settings, kCamEV);
    const bool isoComp = obs_data_get_bool(settings, kCamIsoComp);
    const std::string res = obs_data_get_string(settings, kCamRes);
    const std::string colorSpace = obs_data_get_string(settings, kCamColorSpace);
    const std::string stab = obs_data_get_string(settings, kCamStab);
    const std::string lutName = obs_data_get_string(settings, kCamLutName);

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
        if (lut != ctx->cLUT) {
            // The legacy on/off bool must go through sendCmdLut, NOT sendCmdBool: a bare
            // {"setLUT",boolValue:true} carries no lutName, and camera-side a nil name only CLEARS
            // (when boolValue==false) — a true bool with no name matches neither branch (silent
            // no-op). ON re-selects the camera's CURRENT LUT by name; OFF clears (empty name). Mirrors
            // the Bridge fallback. (Reachable for any camera without a curated availableLuts quick-list.)
            std::string curName;
            { std::lock_guard<std::mutex> lk(ctx->stateMutex); curName = ctx->rLutName; }
            sendCmdLut(ctx, lut ? curName : "");
        }
        if (ev != ctx->cEV) sendCmdFloat(ctx, "setExposureBias", ev);
        if (isoComp != ctx->cIsoComp) sendCmdBool(ctx, "setIsoCompensation", isoComp);
        if (res != ctx->cRes) sendCmdString(ctx, "setResolution", res.c_str());
        if (colorSpace != ctx->cColorSpace) sendCmdString(ctx, "setColorSpace", colorSpace.c_str());
        if (stab != ctx->cStab) sendCmdString(ctx, "setStabilization", stab.c_str());
        if (lutName != ctx->cLutName) sendCmdLut(ctx, lutName); // "" = None (clear)
    }

    ctx->cAE = ae; ctx->cISO = iso; ctx->cShutter = sh; ctx->cAWB = awb;
    ctx->cWB = wb; ctx->cTint = tint; ctx->cAF = af; ctx->cFocus = focus;
    ctx->cLens = lens; ctx->cFPS = fps; ctx->cZoom = zoom; ctx->cLUT = lut;
    ctx->cEV = ev; ctx->cIsoComp = isoComp; ctx->cRes = res;
    ctx->cColorSpace = colorSpace; ctx->cStab = stab; ctx->cLutName = lutName;
    ctx->ccInit = true;
}

// ---- connection lifecycle -------------------------------------------------

ServiceIdentity makeIdentity(const AirliveSourceCtx *ctx) {
    return ServiceIdentity{ctx->did, ctx->dev, ctx->sid, ctx->src, ctx->role, ctx->ord};
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
    // Keychain I/O (SecItem* IPC round-trips) runs BEFORE taking connMutex: holding the lock
    // across a slow securityd would stall the worker's tally path (ensureTally takes connMutex
    // on every camera state message). Same ordering as clear_password_clicked. Doing the store+
    // scrub unconditionally also fixes a leak: the old early-return on a null conn left the
    // typed password plaintext in the scene-collection settings.
    const char *typed = obs_data_get_string(settings, kSettingPassword);
    if (typed && *typed) {
        secretStoreSet(ctx->sid, typed);
        obs_data_set_string(settings, kSettingPassword, ""); // never persist plaintext
    }
    const bool require = obs_data_get_bool(settings, kSettingRequireAuth);
    std::string pw = secretStoreGet(ctx->sid);
    std::lock_guard<std::mutex> lk(ctx->connMutex); // vs. the UI/worker readers of `conn`
    if (ctx->conn)
        ctx->conn->setAuth(require, std::move(pw));
}

// Swap the connection out under connMutex, then destroy it OUTSIDE the lock: the destructor
// (~AirliveConnection → stop) JOINS the worker thread, and the worker's own callbacks
// (onControl → ensureTally) take connMutex — joining while holding it would deadlock. Readers
// that grab the lock after the swap see nullptr and bail; none can hold a dying pointer.
void teardownConnection(AirliveSourceCtx *ctx) {
    std::unique_ptr<AirliveConnection> old;
    {
        std::lock_guard<std::mutex> lk(ctx->connMutex);
        old = std::move(ctx->conn);
    }
    // `old` destroyed on return — the worker join runs with the lock released.
}

// (Re)build the connection with the current identity. Called on create and
// whenever a renameable property changes (new names must hit the TXT record).
void rebuildConnection(AirliveSourceCtx *ctx) {
    teardownConnection(ctx);
    // The lambdas below capture ctx by RAW pointer. This is safe ONLY because
    // teardownConnection/~AirliveConnection joins the worker thread before ctx is
    // ever freed (source_destroy: teardown → flush → delete ctx), so no callback
    // can fire after ctx dies. If async/deferred teardown is ever introduced,
    // replace this with a weak_ptr or a cancel token — do not loosen it.
    AirliveSourceCtx *self = ctx;
    auto fresh = std::make_unique<AirliveConnection>(
        makeIdentity(ctx),
        [self](const AVFrame *f, int64_t pts) { onDecodedFrame(self, f, pts); },
        [self](const char *j, size_t n) { onControl(self, j, n); },
        [self]() { onDisconnect(self); });
    std::lock_guard<std::mutex> lk(ctx->connMutex);
    ctx->conn = std::move(fresh);
    ctx->conn->start(); // start only SPAWNS the worker (never waits on it) — safe under the lock
}

// ---- obs_source_info callbacks --------------------------------------------

const char *source_get_name(void *) {
    return obs_module_text("Airlive.SourceName");
}

const char *source_get_name_bridge(void *) {
    // "Airlive Bridge" — sorts right next to "Airlive Camera" in OBS's source menu
    // (both under "Airlive"), instead of being scattered under "O".
    return obs_module_text("Airlive.SourceNameBridge");
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
    // Operator's channel order → advertised as Bonjour TXT "ord" so the phone sorts multi-source
    // by position, not just by name (asymmetry #6). Default 0 keeps every source equal (name sort).
    ctx->ord = std::to_string(obs_data_get_int(settings, kSettingOrder));

    // The Bridge-program source adds NO delay — the Bridge already switched and timed the program,
    // so it flies straight through (no operator delay control on that source, see get_properties).
    int delay = (ctx->role == "obs-bridge") ? 0 : int(obs_data_get_int(settings, kSettingDelayMs));
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

void *createWithRole(obs_data_t *settings, obs_source_t *source, const char *role) {
    auto *ctx = new AirliveSourceCtx();
    ctx->source = source;
    ctx->role = role;
    // Drop libobs's OWN async-frame FIFO (timestamp-paced playout) that otherwise stacks ~1 capture
    // frame + up to ~1 compositor tick ON TOP of our delay FIFO. Unbuffered = libobs drains to the
    // newest frame each composite tick, so our delay_ms FIFO is the SINGLE place latency is added and
    // "Unbuffered (+0 ms)" finally means 0. Our presets remain the jitter buffer for bad networks.
    obs_source_set_async_unbuffered(source, true);
    ctx->did = loadOrCreateDid();
    apply_settings(ctx, settings);
    applyCameraControls(ctx, settings, /*sendDiffs=*/false); // seed cache, don't command
    rebuildConnection(ctx);
    applyAuth(ctx, settings); // push auth config to the fresh connection
    return ctx;
}

// "OBS Airlive" — receives directly from an iPhone (Airlive Camera).
void *source_create(obs_data_t *settings, obs_source_t *source) {
    return createWithRole(settings, source, "obs");
}

// "OBS Airlive Bridge" — receives the PROGRAM from the Airlive Bridge app over a
// same-machine loopback link (127.0.0.1, no Bonjour, invisible to the iPhone).  A
// SEPARATE source from the direct-iPhone one so both can run at once.
void *source_create_bridge(obs_data_t *settings, obs_source_t *source) {
    return createWithRole(settings, source, "obs-bridge");
}

void source_destroy(void *data) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    ctx->alive.store(false); // stop any UI poke from a still-draining callback (crash guard)
    teardownConnection(ctx); // joins worker (lock released) — nothing touches the FIFO after this
    flushDelayQueue(ctx); // free any frames still buffered
    delete ctx;
}

void source_update(void *data, obs_data_t *settings) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    const std::string prevDev = ctx->dev, prevSrc = ctx->src, prevOrd = ctx->ord;
    apply_settings(ctx, settings); // delay changes apply immediately (read per-frame)
    applyCameraControls(ctx, settings, /*sendDiffs=*/true); // send whatever the operator changed
    // Channel order (Bonjour TXT "ord") must also force a rebuild — it lives in the advertisement, so
    // without this the live BonjourService keeps advertising the OLD ord until an unrelated rename.
    if (ctx->dev != prevDev || ctx->src != prevSrc || ctx->ord != prevOrd)
        rebuildConnection(ctx); // re-advertise under the new display names / channel order
    applyAuth(ctx, settings); // (re-)push auth — also re-pushes after a rebuild
}

void source_get_defaults(obs_data_t *settings) {
    // iPhone-FACING device name KEEPS "OBS" on purpose: the OBS menu/source is now
    // "Airlive Camera", but the camera operator must still see this receiver is OBS (so
    // the director knows where the signal lands).  The OBS display rename does NOT touch
    // this Bonjour-advertised name.
    obs_data_set_default_string(settings, kSettingDeviceName, "OBS Airlive");
    // No default for source_name — apply_settings adopts the OBS source name once.
    obs_data_set_default_int(settings, kSettingDelayMs, kDefaultDelayMs);
    obs_data_set_default_int(settings, kSettingOrder, 0); // default channel order = 0 (equal, name sort)
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
    obs_data_set_default_double(settings, kCamEV, 0.0);
    obs_data_set_default_bool(settings, kCamIsoComp, false);
    // Res / colour space / stabilization / LUT-name default to "" — the pickers are populated AND
    // seeded from the camera's reported caps/state; "" means "no camera value yet" and, for the LUT
    // name, the "None" (cleared) selection.  "" always equals the seeded cache → no spurious command.
    obs_data_set_default_string(settings, kCamRes, "");
    obs_data_set_default_string(settings, kCamColorSpace, "");
    obs_data_set_default_string(settings, kCamStab, "");
    obs_data_set_default_string(settings, kCamLutName, "");
}

// Tally callbacks — recompute and push the cue on every activation/visibility change.
void source_activate(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_deactivate(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_show(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }
void source_hide(void *data) { ensureTally(static_cast<AirliveSourceCtx *>(data), false); }

std::string buildStatusText(AirliveSourceCtx *ctx) {
    std::string s;
    bool conn = false;
    {
        // get_properties runs on the UI thread; update (graphics thread) may be mid-rebuild.
        std::lock_guard<std::mutex> lk(ctx->connMutex);
        conn = ctx->conn && ctx->conn->connected();
    }
    const bool bridge = (ctx->role == "obs-bridge");
    s += conn ? (bridge ? "● Receiving program from Airlive Bridge" : "● Connected")
              : (bridge ? "○ Waiting for Airlive Bridge…" : "○ Waiting for iPhone…");

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

        // Device-read capability ranges/options — makes it VISIBLE whether the
        // remote-control sliders/lists below reflect THIS phone or fall back to the
        // defaults (answers "are these values current?"). Refresh / reopen re-reads.
        if (ctx->capsReceived) {
            s += "\nRanges: ISO " + std::to_string(int(ctx->capIsoMin)) + "–" +
                 std::to_string(int(ctx->capIsoMax)) + " · 1/" +
                 std::to_string(int(ctx->capShutterMin)) + "–1/" +
                 std::to_string(int(ctx->capShutterMax)) + " · WB " +
                 std::to_string(int(ctx->capWbMin)) + "–" + std::to_string(int(ctx->capWbMax)) + "K";
            if (!ctx->capLenses.empty()) {
                s += "\nLenses: ";
                for (size_t i = 0; i < ctx->capLenses.size(); ++i)
                    s += (i ? ", " : "") + ctx->capLenses[i];
            }
            if (!ctx->capFps.empty()) {
                s += "\nFPS: ";
                for (size_t i = 0; i < ctx->capFps.size(); ++i)
                    s += (i ? ", " : "") + std::to_string(ctx->capFps[i]);
            }
        } else {
            s += "\n(Capabilities: defaults — none received from this camera yet)";
        }
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

bool refresh_clicked(obs_properties_t *, obs_property_t *, void *data) {
    // A property callback runs on the UI thread, where obs_source_update_properties() would
    // re-enter and free these very widgets mid-callback (UAF). So flag it instead: the connection
    // worker (onControl) issues the real reload on the next state message (≤1 s while connected).
    if (auto *ctx = static_cast<AirliveSourceCtx *>(data))
        ctx->refreshRequested.store(true);
    return true; // also re-render the existing widgets immediately (harmless)
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
    std::lock_guard<std::mutex> lk(ctx->connMutex); // UI thread vs. update's rebuild (graphics)
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

    // The Bridge-program source has NO operator settings — it only RECEIVES the already-switched,
    // already-timed program from the Airlive Bridge app. No fixed delay (program flies straight
    // through), no iPhone-facing device/source name (it's not a camera target), no per-camera control,
    // no password. Just the status readout above; rename it in OBS's own Sources list.
    if (ctx && ctx->role == "obs-bridge")
        return props;

    // Fixed-delay presets — mirror the Studio app's LatencyPreset values 1:1
    // (real industry standards, not round numbers). A discrete list (not a
    // slider) so the operator picks an exact, meaningful value; OBS applies the
    // choice immediately via source_update.
    obs_property_t *delay = obs_properties_add_list(props, kSettingDelayMs,
                                                    obs_module_text("Airlive.Delay"),
                                                    OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(delay, "Unbuffered (+0 ms)", 0);  // libobs unbuffered / WebRTC min=0
    obs_property_list_add_int(delay, "LAN (+60 ms)", 60);       // DEFAULT — kDefaultDelayMs (~2 frames)
    obs_property_list_add_int(delay, "Normal (+120 ms)", 120);  // SRT default jitter buffer / WAN
    obs_property_list_add_int(delay, "Smooth (+200 ms)", 200);  // WebRTC interactive upper bound
    obs_property_list_add_int(delay, "Safe (+400 ms)", 400);    // WebRTC buffer-against-glitches
    obs_property_set_long_description(
        delay, "Extra buffer on top of base latency. \"+0\" isn't true zero — it's \"show on "
               "decode\". Higher = smoother on bad networks.");
    obs_properties_add_text(props, kSettingDeviceName, obs_module_text("Airlive.DeviceName"),
                            OBS_TEXT_DEFAULT);
    obs_properties_add_text(props, kSettingSourceName, obs_module_text("Airlive.SourceLabel"),
                            OBS_TEXT_DEFAULT);
    // Channel order → Bonjour "ord" so the phone can sort multi-source by position (asymmetry #6).
    obs_properties_add_int(props, kSettingOrder, "Channel order", 0, 999, 1);

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
    // EV compensation — always shown; bounds from caps.evBiasMin/Max (the ctx defaults are the
    // camera's own -2..+2 fallback). NOT greyed out under auto exposure: like the Bridge, EV biases
    // the AE target while AE is on. Created with the fallback bounds, then overridden below.
    obs_property_t *ev = obs_properties_add_float_slider(props, kCamEV, "EV", -2.0, 2.0, 0.1);

    obs_property_t *awb = obs_properties_add_bool(props, kCamAWB, "Auto white balance");
    obs_property_t *wb = obs_properties_add_int_slider(props, kCamWB, "White balance (K)", 2300, 10000, 50);
    obs_property_t *tint = obs_properties_add_int_slider(props, kCamTint, "Tint", -150, 150, 1);

    // Override the slider bounds with this phone's device-read capabilities (from the last state
    // snapshot) so they match the actual camera — Refresh / reopen the dialog after connecting to
    // pick them up.  No state yet → defaults above (unchanged).
    if (ctx) {
        double isoMin, isoMax, shMin, shMax, wbMin, wbMax, tMin, tMax, evMin, evMax;
        {
            std::lock_guard<std::mutex> lk(ctx->stateMutex);
            isoMin = ctx->capIsoMin; isoMax = ctx->capIsoMax;
            shMin  = ctx->capShutterMin; shMax = ctx->capShutterMax;
            wbMin  = ctx->capWbMin; wbMax = ctx->capWbMax;
            tMin   = ctx->capTintMin; tMax = ctx->capTintMax;
            evMin  = ctx->capEvBiasMin; evMax = ctx->capEvBiasMax;
        }
        // Clamp lo<=hi: a partial/hostile capabilities object can supply one bound while its pair
        // falls back to a default (e.g. isoMin=5000 with isoMax=3200 default), which would set an
        // INVERTED, unusable slider limit. min/max keeps it sane no matter what the sender sends.
        auto setLim = [](obs_property_t *p, double a, double b, int step) {
            obs_property_int_set_limits(p, int(a < b ? a : b), int(a < b ? b : a), step);
        };
        setLim(iso,     isoMin, isoMax, 1);
        setLim(shutter, shMin,  shMax,  1);
        setLim(wb,      wbMin,  wbMax,  50);
        setLim(tint,    tMin,   tMax,   1);
        // EV is a float (0.1-EV steps) — same lo<=hi guard against an inverted caps pair.
        obs_property_float_set_limits(ev, evMin < evMax ? evMin : evMax, evMin < evMax ? evMax : evMin, 0.1);
    }

    // ISO compensation — a plain "look" toggle, always available (no caps gate).
    obs_properties_add_bool(props, kCamIsoComp, "ISO compensation");

    obs_property_t *af = obs_properties_add_bool(props, kCamAF, "Auto focus");
    obs_property_t *focus = obs_properties_add_float_slider(props, kCamFocus, "Focus", 0.0, 1.0, 0.01);

    obs_property_t *lens =
        obs_properties_add_list(props, kCamLens, "Lens", OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
    obs_property_t *fps =
        obs_properties_add_list(props, kCamFPS, "FPS", OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    // Snapshot every device-read option list + zoomMax under one lock, then build the combos.  The
    // NEW lists (res / colour space / stabilization / LUT) are shown ONLY when the camera actually
    // sent them — an empty list means "this camera doesn't offer it", so the control is HIDDEN rather
    // than shown as a dead picker (a legacy camera must never get an empty format menu).
    std::vector<std::string> lenses, resList, csList, stabList, lutList;
    std::vector<int> fpsList;
    double zoomMaxLim = 15.0; // hardcoded fallback when the camera reports no zoomMax
    if (ctx) {
        std::lock_guard<std::mutex> lk(ctx->stateMutex);
        lenses = ctx->capLenses;
        fpsList = ctx->capFps;
        resList = ctx->capResolutions;
        csList = ctx->capColorSpaces;
        stabList = ctx->capStabilizations;
        lutList = ctx->capAvailableLuts;
        if (ctx->capZoomMax > 1.0) zoomMaxLim = ctx->capZoomMax;
    }
    // Lens / FPS keep their standard-ladder fallback (they always existed); the new lists do NOT.
    if (lenses.empty()) lenses = {"0.5x", "1x", "2x", "3x", "5x"};
    if (fpsList.empty()) fpsList = {24, 25, 30, 50, 60};
    for (const auto &l : lenses) obs_property_list_add_string(lens, l.c_str(), l.c_str());
    for (int f : fpsList) obs_property_list_add_int(fps, std::to_string(f).c_str(), f);

    // Resolution / colour space / stabilization — populated VERBATIM from the caps the camera sent,
    // so the exact wire strings ("1080"/"4K", "Rec.709"/"P3 D65"/"Rec.2020 HLG"/"Apple Log",
    // "Standard"/"High") are correct by construction. Colour space + stabilization ALSO require the
    // hello cap token (Bridge parity: hasCap + non-empty list); resolution stays list-gated only
    // (Bridge doesn't hasCap-gate it). Hidden when the camera offered none.
    if (!resList.empty()) {
        obs_property_t *res = obs_properties_add_list(props, kCamRes, "Resolution",
                                                      OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
        for (const auto &r : resList) obs_property_list_add_string(res, r.c_str(), r.c_str());
    }
    if (ctx && hasCap(ctx, "colorSpace") && !csList.empty()) {
        obs_property_t *cs = obs_properties_add_list(props, kCamColorSpace, "Colour space",
                                                     OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
        for (const auto &c : csList) obs_property_list_add_string(cs, c.c_str(), c.c_str());
    }
    if (ctx && hasCap(ctx, "stabilization") && !stabList.empty()) {
        obs_property_t *stab = obs_properties_add_list(props, kCamStab, "Stabilization",
                                                       OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
        for (const auto &sv : stabList) obs_property_list_add_string(stab, sv.c_str(), sv.c_str());
    }

    // Zoom — max from caps.zoomMax when the camera reports it (>1), else the 15× fallback.
    obs_properties_add_float_slider(props, kCamZoom, "Zoom", 1.0, zoomMaxLim, 0.1);

    // LUT — by NAME from the phone's quick-access list (with a "None" clear entry) when the camera
    // advertised the "lut" cap AND sent availableLuts (Bridge parity); otherwise fall back to the
    // legacy on/off bool (older cameras / no quick-list). "None" carries an EMPTY name → the camera
    // clears the LUT (see sendCmdLut). Names come from the camera, never invented.  (POI / tap-to-focus
    // is intentionally NOT ported — OBS Properties has no video tap surface; the Focus slider above
    // covers manual focus.)
    if (ctx && hasCap(ctx, "lut") && !lutList.empty()) {
        obs_property_t *lut = obs_properties_add_list(props, kCamLutName, "LUT",
                                                      OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);
        obs_property_list_add_string(lut, "None", ""); // "" = clear
        for (const auto &n : lutList) obs_property_list_add_string(lut, n.c_str(), n.c_str());
    } else {
        obs_properties_add_bool(props, kCamLUT, "LUT on");
    }

    // Reflect the camera's ACTUAL current state in the control widgets — so the lens combo,
    // ISO, shutter, WB, fps and the auto toggles open at what the phone is really doing instead
    // of a static saved default ("линза по умолчанию не та что выбрана"). The received state
    // already feeds the read-only status text; THIS feeds it into the editable widgets. Safe to
    // run on every get_properties: OBS only calls get_properties on dialog OPEN or a worker-issued
    // reload (never on a widget edit — see refresh_clicked / onControl), so it can't clobber an
    // in-flight edit. The send-cache (ctx->c*) is synced too, so seeding never looks like an
    // operator change in source_update → no command is bounced back.
    if (ctx) {
        std::string aLens, aRes, aCS, aStab, aLutName;
        std::vector<std::string> capL, capRes, capCS, capStab, capLuts;
        double aIso = 0, aSh = 0, aWb = 0, aEV = 0;
        int aFps = 0;
        bool aAE = true, aAWB = true, aIsoComp = false, aLutEnabled = false, have = false;
        {
            std::lock_guard<std::mutex> lk(ctx->stateMutex);
            have = !ctx->rLens.empty() || ctx->rIso > 0; // a real snapshot has arrived
            aLens = ctx->rLens; aIso = ctx->rIso; aSh = ctx->rShutter; aWb = ctx->rWB;
            aFps = ctx->rFps; aAE = ctx->rAE; aAWB = ctx->rAWB;
            capL = ctx->capLenses;
            aRes = ctx->rResolution; aCS = ctx->rColorSpace; aStab = ctx->rStabilization;
            aLutName = ctx->rLutName; aLutEnabled = ctx->rLutEnabled; aIsoComp = ctx->rIsoComp;
            aEV = ctx->rExposureBias;
            capRes = ctx->capResolutions; capCS = ctx->capColorSpaces;
            capStab = ctx->capStabilizations; capLuts = ctx->capAvailableLuts;
        }
        if (have) {
            // Only seed the lens if the camera's current lens is actually SELECTABLE in the combo
            // (the device list, else the fallback ladder). Seeding an unlisted lens — e.g. the front
            // camera's "Front", which isn't in the back-lens availableLenses — makes the combo snap to
            // index 0 and then bounce the phone to that wrong lens on the next apply.
            const std::vector<std::string> lensList =
                capL.empty() ? std::vector<std::string>{"0.5x", "1x", "2x", "3x", "5x"} : capL;
            bool lensSelectable = false;
            for (const auto &l : lensList)
                if (l == aLens) { lensSelectable = true; break; }

            // Same selectable-guard for the FPS combo, checked against `fpsList` — exactly what
            // was rendered into the combo above. Camera-side fps and supportedFps go through the
            // same Int(rounded()) (broadcastState), so a modern camera's fps is always listed;
            // the mismatches are a legacy camera (empty supportedFps → our fallback ladder) at a
            // rate off the ladder, the streaming ≤30-filter transient, or a hostile peer. Seeding
            // an unlisted value is never safe: the properties view fires ControlChanged on a
            // not-found value ("trigger a settings update if the index was not found"), writes
            // item 0 into the settings and ships a spurious setFPS — and a mid-take
            // frame-duration change corrupts the recording on the phone.
            bool fpsSelectable = false;
            for (int f : fpsList)
                if (f == aFps) { fpsSelectable = true; break; }

            // Write each setting ONLY when it differs from what's stored, so merely opening the
            // dialog never dirties / overwrites the saved scene-collection settings when the camera
            // already matches. The send-cache (ctx->c*) is always synced to the actual so a later
            // source_update never sends a spurious command.
            obs_data_t *st = obs_source_get_settings(ctx->source);
            const int iiso = int(aIso + 0.5), ish = int(aSh + 0.5), iwb = int(aWb + 0.5);
            if (!aLens.empty() && lensSelectable) {
                if (aLens != obs_data_get_string(st, kCamLens)) obs_data_set_string(st, kCamLens, aLens.c_str());
                ctx->cLens = aLens;
            }
            if (aIso > 0) { if (iiso != obs_data_get_int(st, kCamISO))     obs_data_set_int(st, kCamISO, iiso);     ctx->cISO = iiso; }
            if (aSh > 0)  { if (ish  != obs_data_get_int(st, kCamShutter)) obs_data_set_int(st, kCamShutter, ish);  ctx->cShutter = ish; }
            if (aWb > 0)  { if (iwb  != obs_data_get_int(st, kCamWB))      obs_data_set_int(st, kCamWB, iwb);       ctx->cWB = iwb; }
            if (aFps > 0 && fpsSelectable) { if (aFps != int(obs_data_get_int(st, kCamFPS))) obs_data_set_int(st, kCamFPS, aFps); ctx->cFPS = aFps; }
            if (aAE  != obs_data_get_bool(st, kCamAE))  obs_data_set_bool(st, kCamAE, aAE);
            if (aAWB != obs_data_get_bool(st, kCamAWB)) obs_data_set_bool(st, kCamAWB, aAWB);
            ctx->cAE = aAE; ctx->cAWB = aAWB;

            // Parity-with-Bridge discrete controls — seed the picker to the camera's CURRENT value,
            // but ONLY when that value is one of the offered options (same selectable-guard as lens:
            // seeding an unlisted value snaps the combo to index 0 and bounces the camera there on the
            // next apply). Empty option list ⇒ the control is hidden ⇒ nothing to seed. Cache synced so
            // merely opening the dialog never looks like an operator change.
            auto seedList = [&](const char *key, const std::string &val,
                                const std::vector<std::string> &opts, std::string &cache) {
                if (val.empty() || opts.empty()) return;
                bool ok = false;
                for (const auto &o : opts) if (o == val) { ok = true; break; }
                if (!ok) return;
                if (val != obs_data_get_string(st, key)) obs_data_set_string(st, key, val.c_str());
                cache = val;
            };
            seedList(kCamRes, aRes, capRes, ctx->cRes);
            seedList(kCamColorSpace, aCS, capCS, ctx->cColorSpace);
            seedList(kCamStab, aStab, capStab, ctx->cStab);
            // LUT name: the current selection when it's in the quick-list; "None" (empty) otherwise.
            // The dropdown only exists when capLuts is non-empty, so skip seeding for legacy cameras
            // (they show the on/off bool instead, whose seeding is unchanged from before).
            if (!capLuts.empty()) {
                std::string wantLut = (aLutEnabled && !aLutName.empty()) ? aLutName : "";
                bool ok = wantLut.empty(); // "None" is always selectable
                for (const auto &n : capLuts) if (n == wantLut) { ok = true; break; }
                if (ok) {
                    if (wantLut != obs_data_get_string(st, kCamLutName))
                        obs_data_set_string(st, kCamLutName, wantLut.c_str());
                    ctx->cLutName = wantLut;
                }
            }
            // ISO compensation reflects state like the auto toggles.
            if (aIsoComp != obs_data_get_bool(st, kCamIsoComp)) obs_data_set_bool(st, kCamIsoComp, aIsoComp);
            ctx->cIsoComp = aIsoComp;
            // EV compensation — seeded like ISO/shutter/WB (a slider that reflects the camera). 0 is a
            // valid neutral value, so it is seeded unconditionally within `have` (no >0 guard).
            if (aEV != obs_data_get_double(st, kCamEV)) obs_data_set_double(st, kCamEV, aEV);
            ctx->cEV = aEV;
            obs_data_release(st);
        }
    }

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

// Shared builder for both source types — identical except id / display name /
// create (which sets the Bonjour role).
// Paint one opaque-black I420 frame at w×h. Full-range Y=0 / U=V=128 = pure black. The buffer is
// reused across ticks and rebuilt only on a resize. Timestamp continues the emitted timeline so the
// async source treats black as the NEWEST frame (holds it), and the first real frame on resume — with
// its own advancing capture PTS — still supersedes it. Emitted at most ONCE per stall (tickBlanked),
// so lastEmittedTsNs only advances +33 ms per gap; a resumed capture PTS is normally far larger and
// wins. Only theoretical exception (never seen): a multi-minute stall AND an upstream PTS reset to ~0
// on reconnect could let this synthetic ts outrun the fresh PTS for one frame. video_tick thread only.
void emitBlack(AirliveSourceCtx *ctx, uint32_t w, uint32_t h) {
    const uint32_t cw = (w + 1) / 2, chh = (h + 1) / 2;
    const size_t ySize = size_t(w) * h, cSize = size_t(cw) * chh;
    if (ctx->blackW != int(w) || ctx->blackH != int(h) || ctx->blackBuf.size() != ySize + 2 * cSize) {
        ctx->blackBuf.assign(ySize + 2 * cSize, 0);
        memset(ctx->blackBuf.data() + ySize, 128, 2 * cSize); // Y stays 0, chroma neutral
        ctx->blackW = int(w);
        ctx->blackH = int(h);
    }
    uint8_t *y = ctx->blackBuf.data();
    obs_source_frame2 frame = {};
    frame.width = w;
    frame.height = h;
    frame.timestamp = ctx->lastEmittedTsNs.load() + 33'000'000ull; // +~1 frame @30
    ctx->lastEmittedTsNs.store(frame.timestamp);
    frame.format = VIDEO_FORMAT_I420;
    frame.data[0] = y;             frame.linesize[0] = w;
    frame.data[1] = y + ySize;     frame.linesize[1] = cw;
    frame.data[2] = y + ySize + cSize; frame.linesize[2] = cw;
    frame.range = VIDEO_RANGE_FULL;
    frame.trc = VIDEO_TRC_DEFAULT;
    video_format_get_parameters_for_format(VIDEO_CS_DEFAULT, VIDEO_RANGE_FULL, frame.format,
                                           frame.color_matrix, frame.color_range_min,
                                           frame.color_range_max);
    obs_source_output_video2(ctx->source, &frame);
}

// Runs every OBS video frame (video thread, NOT the UI thread). Two jobs:
//  1. Live status — reload an OPEN Properties dialog when the connection flips (Waiting↔Receiving)
//     or on an explicit Refresh. For the Bridge source this is the ONLY status driver: it sends no
//     state messages, so onControl (the camera source's reload path) never fires. Safe off the UI
//     thread, exactly like the onControl reload.
//  2. No-signal → BLACK — when decoded frames stop (source dropped, on-air camera lost signal, or the
//     Bridge cut its program to black by just not forwarding), paint black so OBS shows a clean feed
//     instead of freezing on the last picture. Emitted once per gap; cleared when real frames resume.
void source_video_tick(void *data, float /*seconds*/) {
    auto *ctx = static_cast<AirliveSourceCtx *>(data);
    if (!ctx || !ctx->alive.load())
        return;

    // ⚠️ CRASH — reload the Properties panel ONLY on an explicit Refresh click, never automatically.
    // obs_source_update_properties() rebuilds the panel on the UI thread (and buildStatusText reads
    // the Keychain); if that fires while OBS is showing a modal (e.g. deleting this source), macOS 26
    // re-enters SwiftUI/CoreUI's one-time systemAssetManager init and SEGVs OBS — a 100%-repro crash
    // on removing the Bridge source. The status text still refreshes on panel-open and on Refresh.
    if (ctx->refreshRequested.exchange(false))
        obs_source_update_properties(ctx->source);

    const uint32_t w = ctx->statW.load(), h = ctx->statH.load();
    if (!w || !h)
        return; // never received a frame → OBS already shows its default (black) with nothing to size
    const int64_t last = ctx->lastFrameMonoNs.load();
    const bool stalled = last != 0 && (nowMonoNs() - last) > kNoSignalBlackNs;
    if (stalled && !ctx->tickBlanked) {
        emitBlack(ctx, w, h);
        ctx->tickBlanked = true;
    } else if (!stalled && ctx->tickBlanked) {
        ctx->tickBlanked = false; // real frames are flowing again
    }
}

struct obs_source_info makeSourceInfo(const char *id,
                                      const char *(*getName)(void *),
                                      void *(*create)(obs_data_t *, obs_source_t *)) {
    struct obs_source_info info = {};
    info.id = id;
    info.type = OBS_SOURCE_TYPE_INPUT;
    info.output_flags = OBS_SOURCE_ASYNC_VIDEO | OBS_SOURCE_DO_NOT_DUPLICATE;
    info.icon_type = OBS_ICON_TYPE_CAMERA;
    info.get_name = getName;
    // Async sources get their dimensions from the output frames; back the queries
    // with live stats so they report real dims once video flows (0 until then).
    info.get_width = [](void *d) -> uint32_t { return static_cast<AirliveSourceCtx *>(d)->statW.load(); };
    info.get_height = [](void *d) -> uint32_t { return static_cast<AirliveSourceCtx *>(d)->statH.load(); };
    info.create = create;
    info.destroy = source_destroy;
    info.update = source_update;
    info.video_tick = source_video_tick; // live status reload + no-signal→black
    info.get_defaults = source_get_defaults;
    info.get_properties = source_get_properties;
    info.activate = source_activate;
    info.deactivate = source_deactivate;
    info.show = source_show;
    info.hide = source_hide;
    return info;
}

// "OBS Airlive" — direct iPhone source.  Distinct id so OBS never confuses it
// with any older Airlive screen-mirroring plugin.
struct obs_source_info airlive_source_info =
    makeSourceInfo("airlive_obs_source", source_get_name, source_create);

// "OBS Airlive Bridge" — receives the Airlive Bridge program (role "obs-bridge"),
// runs alongside the direct source without slot conflict.
struct obs_source_info airlive_bridge_source_info =
    makeSourceInfo("airlive_obs_bridge_source", source_get_name_bridge, source_create_bridge);

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
    obs_register_source(&airlive_bridge_source_info);
    blog(LOG_INFO, "[airlive-obs] OBS Airlive + OBS Airlive Bridge loaded");
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
