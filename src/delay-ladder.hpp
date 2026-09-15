// delay-ladder.hpp — the playout-buffer ladder, defined ONCE.
//
// It lives in its own header for a dull but real reason: the ladder is consumed in two places that
// must never disagree - the properties picker the operator reads, and the snapper that repairs a
// stored value which is not on the ladder. When they were two lists, a rung added to one and
// forgotten in the other would leave OBS showing an EMPTY picker with no way to tell what was in
// effect. One definition, two readers, and a test that walks the whole range.
//
// The VALUES are shared with the rest of the product: the same rungs and the same default the
// Airlive Bridge offers (LatencyPreset in the Bridge's Sources/Model/OutputSettings.swift). Each is
// a real shipping standard rather than a round number:
//   0   libobs unbuffered / WebRTC playout-delay min=0
//   60  under two frames at 30 fps: absorbs ONE late packet and nothing more
//   120 SRT's default SRTO_LATENCY, the industry's fixed-latency baseline
//   200 the top of WebRTC's interactive-streaming band
//   400 WebRTC's buffer-against-glitches figure
//
// The 60 ms rung has a history worth keeping straight. This plugin once had it alone, defaulted to
// it, and argued that our no-B-frame stream jitters less on a LAN than the 120 ms SRT figure
// assumes. It was removed - not because the reasoning was wrong, but because a rung that exists in
// only one half of a product makes the product behave two ways depending on which half the operator
// is looking at, and it named a value the Bridge could not express at all. The Bridge added it for
// the whole desk in 1.3.0, on the same reasoning, so it comes back here as part of the shared
// ladder rather than as this plugin's private opinion. The default stays 120.
#pragma once

#include <cstddef>

namespace airlive {

struct DelayRung {
    int ms;
    const char *label;   // shown in the properties picker, exactly as the Bridge names it
};

inline constexpr DelayRung kDelayLadder[] = {
    {0,   "Unbuffered (0 ms)"},
    {60,  "Low (60 ms)"},
    {120, "Normal (120 ms)"},
    {200, "High (200 ms)"},
    {400, "Safe (400 ms)"},
};

constexpr int kDefaultDelayMs = 120;   // SRT's default jitter buffer - the Bridge's default too
constexpr int kMaxDelayMs = 400;       // the highest rung (Safe)

/// Snap a stored delay onto the shipping ladder.
///
/// A stored value that is not on the ladder leaves OBS showing an EMPTY picker, with the operator
/// unable to tell what is in effect. Ties go UP: a slightly larger buffer is the safe direction on
/// a network we know nothing about.
inline int snapDelayToPreset(int ms)
{
    int best = kDefaultDelayMs, bestDist = -1;
    for (const DelayRung &r : kDelayLadder) {
        const int dist = ms > r.ms ? ms - r.ms : r.ms - ms;
        if (bestDist < 0 || dist < bestDist || (dist == bestDist && r.ms > best)) {
            best = r.ms;
            bestDist = dist;
        }
    }
    return best;
}

} // namespace airlive
