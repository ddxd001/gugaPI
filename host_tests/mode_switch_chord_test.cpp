#include <assert.h>
#include <stdint.h>

#include "app/mode_switch_chord.h"

namespace {

const uint32_t kHoldMs = 1000U;

void TestSingleButtonDoesNotStart(void)
{
    app::ModeSwitchChordState state = { false, false, 0U };

    assert(app::ModeSwitchChord_Update(
               &state, true, false, 10U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, false, true, 2000U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(!app::ModeSwitchChord_IsActive(&state));
}

void TestHoldThresholdAndReleaseLatch(void)
{
    app::ModeSwitchChordState state = { false, false, 0U };

    assert(app::ModeSwitchChord_Update(
               &state, true, true, 100U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 1099U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 1100U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_TRIGGERED);

    assert(app::ModeSwitchChord_Update(
               &state, true, true, 2500U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, false, true, 2510U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_IsActive(&state));
    assert(app::ModeSwitchChord_Update(
               &state, false, false, 2520U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(!app::ModeSwitchChord_IsActive(&state));

    assert(app::ModeSwitchChord_Update(
               &state, true, true, 2600U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
}

void TestEarlyReleaseCancelsUntilBothReleased(void)
{
    app::ModeSwitchChordState state = { false, false, 0U };

    assert(app::ModeSwitchChord_Update(
               &state, true, true, 0U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
    assert(app::ModeSwitchChord_Update(
               &state, true, false, 500U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_IsActive(&state));
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 2000U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, false, false, 2010U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 2020U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
}

void TestStaggeredPressUsesOverlapTime(void)
{
    app::ModeSwitchChordState state = { false, false, 0U };

    assert(app::ModeSwitchChord_Update(
               &state, true, false, 100U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 600U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 1599U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, 1600U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_TRIGGERED);
}

void TestMillisWraparound(void)
{
    app::ModeSwitchChordState state = { false, false, 0U };
    const uint32_t start = 0xFFFFFF00U;

    assert(app::ModeSwitchChord_Update(
               &state, true, true, start, kHoldMs) ==
           app::MODE_SWITCH_CHORD_STARTED);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, start + 999U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_NONE);
    assert(app::ModeSwitchChord_Update(
               &state, true, true, start + 1000U, kHoldMs) ==
           app::MODE_SWITCH_CHORD_TRIGGERED);
}

} /* namespace */

int main(void)
{
    TestSingleButtonDoesNotStart();
    TestHoldThresholdAndReleaseLatch();
    TestEarlyReleaseCancelsUntilBothReleased();
    TestStaggeredPressUsesOverlapTime();
    TestMillisWraparound();
    return 0;
}
