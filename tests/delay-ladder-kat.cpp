// delay-ladder-kat.cpp — the playout ladder has to survive being edited.
//
// The ladder is read in two places (the properties picker and the snapper) and shares its values
// with the Airlive Bridge. This walks the whole reachable range and asserts three things: every
// snapped value IS a rung, it is the NEAREST rung, and a tie resolves upward. That is exactly the
// drift a future edit would introduce by adding a rung to one reader and not the other.
#include "delay-ladder.hpp"

#include <cstdio>
#include <cstdlib>

using namespace airlive;

static int failures = 0;

static void check(bool ok, const char *what, int in, int got, int want)
{
    if (ok) return;
    std::printf("FAIL  %s: snap(%d) = %d, expected %d\n", what, in, got, want);
    ++failures;
}

int main()
{
    // The rungs the whole product agrees on. Spelled out rather than read from the header, so a
    // change to the ladder has to be made HERE too - which is the point of the test.
    const int expected[] = {0, 60, 120, 200, 400};
    const size_t n = sizeof(expected) / sizeof(expected[0]);

    for (size_t i = 0; i < n; ++i) {
        const int r = expected[i];
        check(snapDelayToPreset(r) == r, "a rung snaps to itself", r, snapDelayToPreset(r), r);
    }

    // Every value an operator or an old settings file could carry, well past the top rung.
    for (int ms = -50; ms <= 600; ++ms) {
        const int got = snapDelayToPreset(ms);

        bool isRung = false;
        for (size_t i = 0; i < n; ++i)
            if (got == expected[i]) isRung = true;
        check(isRung, "snaps onto the ladder", ms, got, -1);

        // Nearest, with ties going up.
        int want = expected[0];
        int bestDist = ms > want ? ms - want : want - ms;
        for (size_t i = 1; i < n; ++i) {
            const int p = expected[i];
            const int d = ms > p ? ms - p : p - ms;
            if (d < bestDist || (d == bestDist && p > want)) { want = p; bestDist = d; }
        }
        check(got == want, "nearest rung, ties up", ms, got, want);
    }

    // The two tie points this ladder actually has, named so a failure reads plainly.
    check(snapDelayToPreset(30) == 60, "30 ms ties 0/60 and goes up", 30, snapDelayToPreset(30), 60);
    check(snapDelayToPreset(90) == 120, "90 ms ties 60/120 and goes up", 90, snapDelayToPreset(90), 120);

    // The default has to be ON the ladder, or a fresh source shows an empty picker.
    bool defaultIsRung = false;
    for (size_t i = 0; i < n; ++i)
        if (kDefaultDelayMs == expected[i]) defaultIsRung = true;
    check(defaultIsRung, "the default is a rung", kDefaultDelayMs, kDefaultDelayMs, -1);
    check(kMaxDelayMs == expected[n - 1], "kMaxDelayMs is the top rung", kMaxDelayMs, kMaxDelayMs,
          expected[n - 1]);

    // The picker's own list must BE the ladder, in order - the picker is the other reader.
    size_t i = 0;
    for (const DelayRung &r : kDelayLadder) {
        check(i < n && r.ms == expected[i], "picker row matches the ladder", int(i), r.ms,
              i < n ? expected[i] : -1);
        ++i;
    }
    check(i == n, "picker has every rung and no more", int(i), int(i), int(n));

    if (failures) {
        std::printf("delay-ladder KAT: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("delay-ladder KAT: pass\n");
    return 0;
}
