// playout-clock-kat.cpp — the delay must hold whatever the network does.
//
// Each case feeds the clock a stream of (capture time, arrival time) and checks the one promise:
// the schedule stays capture + path + delay. The first case is the defect this clock replaced:
// a gap in the frames (lens switch, Wi-Fi dip) must add nothing to the delay.
#include "playout-clock.hpp"

#include <cstdio>
#include <cstdlib>

using namespace airlive;

static int failures = 0;
static void check(bool ok, const char *what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        ++failures;
}
static bool near(double a, double b, double tol = 0.002) { return std::abs(a - b) <= tol; }

static constexpr double kFrame = 1.0 / 30.0;
static constexpr double kPath = 0.020; // fixed network path of the simulated link

// Plays `seconds` of frames from capture time `t`, each arriving kPath later plus `extra`.
static double feed(PlayoutClock &c, double t, double seconds, double extra, double *lastDelay) {
    bool drop = false;
    for (double end = t + seconds; t < end; t += kFrame) {
        const double arrival = t + kPath + extra;
        *lastDelay = c.playoutTime(t, arrival, drop) - t;
    }
    return t;
}

int main() {
    const auto quiet = [](const std::string &) {};
    double d = 0;

    { // Gaps add nothing: 20 lens switches of 1.2 s, the delay is what it was.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        double t = feed(c, 1000, 3, 0, &d);
        const double locked = d;
        check(near(locked, kPath + 0.120), "locks at path + delay");
        for (int i = 0; i < 20; ++i)
            t = feed(c, t + 1.2, 5, 0, &d);
        check(near(d, locked), "20 gaps of 1.2 s leave the delay unchanged");
    }
    { // A slow spell makes frames late and moves nothing.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        double t = feed(c, 1000, 3, 0, &d);
        const double locked = d;
        t = feed(c, t, 5, 0.300, &d);
        t = feed(c, t, 12, 0, &d);
        check(near(d, locked, 0.003), "a 300 ms spell does not stretch the delay");
    }
    { // A small faster path is margin, never a jump.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        double t = feed(c, 1000, 3, 0.100, &d);
        const double locked = d;
        t = feed(c, t, 5, 0, &d);
        check(near(d, locked, 0.003), "a path 100 ms faster keeps the locked delay");
    }
    { // Clock drift is followed, at its own speed.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        bool drop = false;
        double t = 1000, delay = 0;
        const double ppm = 20e-6;
        for (; t < 1000 + 600; t += kFrame)
            delay = c.playoutTime(t, t + kPath + (t - 1000) * ppm, drop) - t - (t - 1000) * ppm;
        check(near(delay, kPath + 0.120, 0.004), "20 ppm of drift is tracked for 10 minutes");
    }
    { // A delay change re-anchors and asks for the queue to go.
        PlayoutClock c(quiet);
        check(c.setBuffer(0.120), "first delay counts as a change");
        check(!c.setBuffer(0.120), "the same delay does not");
        double t = feed(c, 1000, 3, 0, &d);
        check(c.setBuffer(0.400), "a new delay does");
        t = feed(c, t, 3, 0, &d);
        check(near(d, kPath + 0.400), "locks at the new delay");
    }
    { // Resume: a stale keyframe never brings the picture back, an on-time one does, same timeline.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        double t = feed(c, 1000, 3, 0, &d);
        const double locked = d;
        c.beginResume(t + 9);
        check(c.resumeSample(t, t + 9, true) == ResumeVerdict::Drop, "a stale keyframe is dropped");
        check(c.resumeSample(t + 9, t + 9.1, false) == ResumeVerdict::Drop, "a P-frame is dropped");
        check(c.resumeSample(t + 9.1, t + 9.1 + kPath, true) == ResumeVerdict::Show,
              "an on-time keyframe shows");
        feed(c, t + 9.2, 2, 0, &d);
        check(near(d, locked, 0.003), "the delay after the outage is the delay before it");
    }
    { // Resume on a link that never settles: back within the give-up, never stuck on black.
        PlayoutClock c(quiet);
        c.setBuffer(0.120);
        double t = feed(c, 1000, 3, 0, &d);
        c.beginResume(t);
        const bool early = c.resumeSample(t, t + 1, true) == ResumeVerdict::Show;
        check(!early, "a late keyframe waits");
        double now = t + PlayoutClock::kResumeGiveUpSeconds + 0.1;
        check(c.resumeSample(t, now, true) == ResumeVerdict::Show, "the give-up brings it back");
    }
    if (failures)
        std::printf("%d FAILED\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
