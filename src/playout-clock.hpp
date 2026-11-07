// playout-clock.hpp — the fixed playout delay, the same rules as the Airlive Bridge.
//
// WHY THE PLUGIN KEEPS ITS OWN SCHEDULE: libobs's buffered async playout cannot hold a delay. Its
// clock advances only while frames are queued (obs-source.c: get_closest_frame returns before
// ready_async_frame on an empty queue), so every gap - a lens switch, a Wi-Fi dip - stopped the clock
// and added its whole length to the delay, for good. The operator saw 120 ms and got seconds. So the
// source runs unbuffered and THIS clock decides when each frame plays.
//
// A frame plays at: capture time + the fastest network path + the chosen delay. The path is measured
// for `kCalibrationSeconds` after (re)anchoring, then held: afterwards the schedule follows clock
// drift only, at most `kMaxDriftRate` either way, never the network. A slower spell makes frames late
// (they play at once, the delay is unchanged and comes back by itself); a faster one makes them wait
// longer. Re-anchored only on a new stream, a delay change, a picture back after silence on a slower
// path, or a path more than `kAheadReanchorThreshold` faster than the lock - each one logged.
//
// Pure logic, no OBS and no clock of its own (times are passed in), so it is tested on its own:
// tests/playout-clock-kat.cpp. Times are seconds. Worker thread only.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <utility>

namespace airlive {

/// What to do with a sample while the picture waits after a silence.
enum class ResumeVerdict {
    Drop,         // not playable: drop it, and ask for a keyframe (the caller spaces the requests)
    Show,         // the keyframe the picture was waiting for
};

class PlayoutClock {
public:
    using Log = std::function<void(const std::string &)>;

    explicit PlayoutClock(Log log) : log_(std::move(log)) {}

    /// A new connection: the next frame starts a new timeline.
    void reset() {
        closeLateSpell("on a new timeline");
        anchorSet_ = false;
        anchorReason_ = "new stream";
        awaitingResume_ = false;
    }

    /// Apply the chosen delay. Returns true when it really changed: the frames queued under the old
    /// delay must then be dropped (cleaner than re-timing them).
    bool setBuffer(double seconds) {
        if (std::abs(seconds - buffer_) < kDelayChangeEpsilon)
            return false;
        buffer_ = seconds;
        reanchor("delay setting changed");
        return true;
    }

    /// The moment this frame plays. `dropQueued` is set when the timeline was taken again and what
    /// is queued belongs to the old one.
    double playoutTime(double pts, double now, bool &dropQueued) {
        if (!anchorSet_)
            anchor(pts, now);
        double delta = tripDelta(pts, now);
        if (locked_ && delta - skew_ < -kAheadReanchorThreshold) {
            anchorReason_ = "path " + std::to_string(int((skew_ - delta) * 1000)) +
                            " ms faster than the locked delay";
            anchor(pts, now);
            dropQueued = true;
            delta = 0;
        }

        if (now < calibratingUntil_) {
            skew_ = std::min(skew_, delta); // measuring: the fastest frame so far defines the path
        } else {
            if (!locked_) {
                locked_ = true;
                lockedSkew_ = lastLoggedSkew_ = skew_;
                log_("delay locked: " + std::to_string(int(std::lround(buffer_ * 1000))) +
                     " ms over the fastest path (" + std::to_string(int(std::lround(skew_ * 1000))) +
                     " ms vs the first frame)");
            }
            followClockDrift(delta, now);
        }
        lastSkewUpdate_ = now;
        const double playout = anchorMac_ + buffer_ + (pts - anchorPts_) + skew_;
        noteTimeliness(playout - now, now);
        return playout;
    }

    /// The camera spoke again after a silence. A link that drops again before the picture is back
    /// does not restart the give-up wait.
    void beginResume(double now) {
        if (!awaitingResume_)
            resumeWaitStart_ = now;
        awaitingResume_ = true;
        resumeBucketStart_ = 0;
        resumeBucketFloor_ = resumePreviousFloor_ = kInfinity;
        resumePathSettled_ = false;
    }

    /// One sample while the picture waits. What the network held arrives first, keyframes included;
    /// none of that can play in its slot. The picture comes back at the first keyframe that CAN, on
    /// the same timeline, so the delay is what it was and nothing is fast-forwarded. A keyframe that
    /// is still late brings it back only once the path has settled (it really is slower now), or
    /// after `kResumeGiveUpSeconds` on a link that never settles.
    ResumeVerdict resumeSample(double pts, double now, bool isKeyframe) {
        const double late = anchorSet_ ? tripDelta(pts, now) - skew_ : 0;
        noteResumeArrival(late, now);
        if (!isKeyframe)
            return ResumeVerdict::Drop;
        if (late > buffer_ + kFrameSlack) {
            const double waited = now - resumeWaitStart_;
            if (resumePathSettled_) {
                reanchor("picture back on a slower path, +" + std::to_string(int(late * 1000)) + " ms");
            } else if (waited >= kResumeGiveUpSeconds) {
                reanchor("picture back after " + std::to_string(int(waited * 1000)) +
                         " ms waiting, path never settled, +" + std::to_string(int(late * 1000)) + " ms");
            } else {
                return ResumeVerdict::Drop;
            }
        }
        awaitingResume_ = false;
        return ResumeVerdict::Show;
    }

    /// How many frames may wait: the delay plus the margin a faster path adds, at 60 fps (the most
    /// a camera sends), plus one being decoded and one being shown.
    size_t queueCapacity() const {
        return size_t(std::ceil((buffer_ + kAheadReanchorThreshold) * 60.0)) + 2;
    }

    bool isLocked() const { return locked_; }
    double skew() const { return skew_; }

    /// Two crystals drift apart by tens of ppm at most; the network moves far faster, so a limit of
    /// 200 ppm (0.2 ms per second) means the delay can follow the first and never the second.
    static constexpr double kMaxDriftRate = 200e-6;
    /// Long enough to see the fastest path through the once-a-second Wi-Fi pauses.
    static constexpr double kCalibrationSeconds = 2.0;
    /// The window the drift follower looks at once locked.
    static constexpr double kDriftWindowSeconds = 10.0;
    /// Once locked, a frame this much earlier than the schedule means the calibration missed the
    /// real path: frames would pile up past the queue. Less than this just waits longer.
    static constexpr double kAheadReanchorThreshold = 0.25;
    /// One frame at the wire's 30 fps.
    static constexpr double kFrameSlack = 1.0 / 30.0;
    /// After a silence: how far the fastest arrival of one second may differ from the second before
    /// for the path to count as settled (two frames: Wi-Fi clumps move single arrivals by ~100 ms,
    /// never the fastest one).
    static constexpr double kResumeSettleTolerance = 2.0 / 30.0;
    static constexpr double kResumeBucketSeconds = 1.0;
    /// Safe because the camera keeps at most ~2 s of backlog: nothing stale is still draining.
    static constexpr double kResumeGiveUpSeconds = 4.0;

private:
    static constexpr double kInfinity = std::numeric_limits<double>::infinity();
    /// Half a millisecond: below the 1 ms step of any delay, so only a real change counts.
    static constexpr double kDelayChangeEpsilon = 0.0005;
    /// A drift worth a log line: under one frame, at most one line per 100 s at the full rate.
    static constexpr double kSkewLogStep = 0.020;
    /// On-time frames for this long close a late spell: two of the once-a-second Wi-Fi pauses.
    static constexpr double kOnTimeConfirmSeconds = 2.0;

    double tripDelta(double pts, double now) const { return (now - anchorMac_) - (pts - anchorPts_); }

    void reanchor(const std::string &reason) {
        anchorSet_ = false;
        anchorReason_ = reason;
    }

    void anchor(double pts, double now) {
        closeLateSpell("on a new timeline");
        anchorPts_ = pts;
        anchorMac_ = now;
        anchorSet_ = true;
        skew_ = 0;
        window_.clear();
        calibratingUntil_ = now + kCalibrationSeconds;
        locked_ = false;
        lastSkewUpdate_ = now;
        log_("delay re-anchored (" + anchorReason_ + ")");
        anchorReason_ = "new stream";
    }

    void followClockDrift(double delta, double now) {
        window_.emplace_back(now, delta);
        while (!window_.empty() && window_.front().first < now - kDriftWindowSeconds)
            window_.pop_front();
        double target = delta;
        for (const auto &s : window_)
            target = std::min(target, s.second);
        const double step = kMaxDriftRate * std::max(0.0, now - lastSkewUpdate_);
        skew_ += std::min(std::max(target - skew_, -step), step);

        if (std::abs(skew_ - lastLoggedSkew_) < kSkewLogStep)
            return;
        lastLoggedSkew_ = skew_;
        const int movedMs = int(std::lround((skew_ - lockedSkew_) * 1000));
        log_("schedule " + std::string(movedMs >= 0 ? "+" : "") + std::to_string(movedMs) +
             " ms since the delay locked (clock drift)");
    }

    void noteResumeArrival(double late, double now) {
        if (resumeBucketStart_ == 0)
            resumeBucketStart_ = now;
        if (now - resumeBucketStart_ >= kResumeBucketSeconds) {
            resumePathSettled_ = std::abs(resumeBucketFloor_ - resumePreviousFloor_) <= kResumeSettleTolerance;
            resumePreviousFloor_ = resumeBucketFloor_;
            resumeBucketFloor_ = kInfinity;
            resumeBucketStart_ = now;
        }
        resumeBucketFloor_ = std::min(resumeBucketFloor_, late);
    }

    /// Frames that miss their slot play at once and the delay stays locked, so a network slower
    /// than the delay is never hidden by stretching it, and never silent either.
    void noteTimeliness(double spare, double now) {
        if (spare < -kFrameSlack) {
            if (!lateSpellOpen_) {
                lateSpellOpen_ = true;
                lateSpellStart_ = now;
                lateSpellWorst_ = 0;
                lateSpellFrames_ = 0;
                log_("network slower than the delay: frames are missing their slot (" +
                     std::to_string(int(-spare * 1000)) + " ms late)");
            }
            lateSpellWorst_ = std::max(lateSpellWorst_, -spare);
            ++lateSpellFrames_;
            lateSpellLastLate_ = now;
        } else if (lateSpellOpen_ && now - lateSpellLastLate_ >= kOnTimeConfirmSeconds) {
            closeLateSpell("on time again");
        }
    }

    void closeLateSpell(const char *how) {
        if (!lateSpellOpen_)
            return;
        lateSpellOpen_ = false;
        char lasted[32];
        std::snprintf(lasted, sizeof(lasted), "%.1f", lateSpellLastLate_ - lateSpellStart_);
        log_(std::string("frames ") + how + " - " + std::to_string(lateSpellFrames_) +
             " missed their slot over " + lasted + " s, up to " +
             std::to_string(int(lateSpellWorst_ * 1000)) + " ms late");
    }

    Log log_;
    double buffer_ = -1; // no delay applied yet: the first setBuffer always counts as a change

    bool anchorSet_ = false;
    std::string anchorReason_ = "new stream";
    double anchorPts_ = 0, anchorMac_ = 0;
    double skew_ = 0; // the fastest path, relative to the anchor frame's own trip
    double calibratingUntil_ = 0;
    bool locked_ = false;
    double lastSkewUpdate_ = 0, lockedSkew_ = 0, lastLoggedSkew_ = 0;
    std::deque<std::pair<double, double>> window_; // (arrival, trip delta) for the drift follower

    bool awaitingResume_ = false;
    double resumeWaitStart_ = 0, resumeBucketStart_ = 0;
    double resumeBucketFloor_ = kInfinity, resumePreviousFloor_ = kInfinity;
    bool resumePathSettled_ = false;

    bool lateSpellOpen_ = false;
    double lateSpellStart_ = 0, lateSpellLastLate_ = 0, lateSpellWorst_ = 0;
    int lateSpellFrames_ = 0;
};

} // namespace airlive
