/* Host-only regression test. Keep this outside the CCS gugaPI project tree so
 * its main() is never linked into the target firmware. */
#include <assert.h>
#include <stdint.h>

#include "drivers/grayscale/grayscale_processing.h"

namespace {

drivers::GrayscaleCalibration MakeCalibration()
{
    drivers::GrayscaleCalibration calibration = {};
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        calibration.white[i] = 4000U;
        calibration.black[i] = 400U;
    }
    calibration.threshold = 500U;
    calibration.hysteresis = 300U;
    calibration.position_floor = 100U;
    calibration.min_line_strength = 600U;
    calibration.track_mask = 0x3CU;
    return calibration;
}

void Fill(uint16_t raw[drivers::GRAYSCALE_CHANNEL_COUNT], uint16_t value)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        raw[i] = value;
    }
}

drivers::GrayscaleProcessedData Process(
    const uint16_t raw[drivers::GRAYSCALE_CHANNEL_COUNT],
    const drivers::GrayscaleCalibration &calibration,
    drivers::GrayscaleProcessingState *state)
{
    drivers::GrayscaleProcessedData result = {};
    assert(drivers::Grayscale_ProcessWithState(raw,
                                               &calibration,
                                               state,
                                               &result) ==
           drivers::DRIVER_OK);
    return result;
}

} /* namespace */

int main()
{
    const drivers::GrayscaleCalibration calibration = MakeCalibration();
    drivers::GrayscaleProcessingState state = {};
    uint16_t raw[drivers::GRAYSCALE_CHANNEL_COUNT] = {};

    /* Centered line: the inner pair gives a centered, valid position. */
    Fill(raw, 3900U);
    raw[3] = 800U;
    raw[4] = 800U;
    drivers::GrayscaleProcessedData result = Process(raw,
                                                      calibration,
                                                      &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.line_position == 0);
    assert(result.selected_mask == 0x18U);

    /* Outer road evidence remains visible but cannot move the centroid. */
    Fill(raw, 3900U);
    raw[0] = 800U;
    raw[4] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.active_mask == 0x11U);
    assert(result.selected_mask == 0x10U);
    assert(result.line_position == -1000);

    /* The new default tracking core includes channels 1 and 6, while the
     * outermost pair remains reserved for road/intersection evidence. */
    drivers::GrayscaleCalibration six_channel = calibration;
    six_channel.track_mask = drivers::GRAYSCALE_DEFAULT_TRACK_MASK;
    state = {};
    Fill(raw, 3900U);
    raw[1] = 800U;
    result = Process(raw, six_channel, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.selected_mask == 0x02U);
    assert(result.line_position == 2000);

    Fill(raw, 3900U);
    raw[6] = 800U;
    result = Process(raw, six_channel, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.selected_mask == 0x40U);
    assert(result.line_position == -2000);

    Fill(raw, 3900U);
    raw[0] = 800U;
    raw[4] = 800U;
    result = Process(raw, six_channel, &state);
    assert(result.active_mask == 0x11U);
    assert(result.selected_mask == 0x10U);
    assert(result.line_position == -1000);

    /* Continue the established four-channel compatibility regressions. */
    state = {};

    /* The previously troublesome 0xF8 road pattern contains only three core
     * channels. Outer width is road evidence and must not invalidate the
     * continuous tracking position. */
    Fill(raw, 3900U);
    for (uint8_t i = 3U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        raw[i] = 800U;
    }
    result = Process(raw, calibration, &state);
    assert(result.active_mask == 0xF8U);
    assert(result.selected_mask == 0x38U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);

    /* Two separated core peaks are explicit ambiguous geometry. */
    Fill(raw, 3900U);
    raw[2] = 800U;
    raw[5] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_MULTIPLE);
    assert(!result.position_valid);
    assert(result.invalid_frames == 1U);

    /* Consecutive invalid complete frames are counted for the application's
     * independent stop gate. */
    result = Process(raw, calibration, &state);
    assert(result.invalid_frames == 2U);

    /* A good frame resets the consecutive-invalid counter. */
    Fill(raw, 3900U);
    raw[4] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.invalid_frames == 0U);

    /* A line centered between sensors is accepted from adjacent moderate
     * evidence even when neither channel reaches the digital-on threshold. */
    Fill(raw, 3900U);
    raw[3] = 2560U;
    raw[4] = 2560U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.line_position == 0);

    /* A single partial response can acquire a narrow line directly from a
     * cold state. This mirrors the measured vehicle frame: normalized=530,
     * signal=430, below threshold_on=650 but above the derived analogue
     * acquisition minimum of 400. */
    state = {};
    Fill(raw, 3900U);
    raw[4] = 2092U; /* normalized=530, signal=430 */
    result = Process(raw, calibration, &state);
    assert(result.active_mask == 0U);
    assert(result.selected_mask == 0x10U);
    assert(result.line_strength == 430U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.line_position == -1000);
    assert(result.position_confidence >= 300U);

    /* Two adjacent responses below threshold_off=350 combine through the
     * continuous weighted centroid and acquire a centered line.  A physical
     * sensor gap gets a lower, still geometry-bounded cold-start threshold. */
    state = {};
    Fill(raw, 3900U);
    raw[3] = 2920U; /* normalized=300, signal=200 */
    raw[4] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.active_mask == 0U);
    assert(result.selected_mask == 0x18U);
    assert(result.line_strength == 400U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.line_position == 0);

    /* Reproduce the measured start-line frame: normalized values are about
     * 133 and 251, so the signals above the floor total only 184.  Because
     * they are adjacent this must still acquire from a cold state. */
    state = {};
    Fill(raw, 3900U);
    raw[2] = 3520U; /* normalized=133, signal=33 */
    raw[3] = 3096U; /* normalized=251, signal=151 */
    result = Process(raw, calibration, &state);
    assert(result.active_mask == 0U);
    assert(result.selected_mask == 0x0CU);
    assert(result.line_strength == 184U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.line_detected);
    assert(result.position_valid);
    assert(result.position_confidence >= 300U);

    /* Adjacent geometry alone is insufficient: total signal below the
     * derived two-channel minimum (150 for this calibration) stays lost. */
    state = {};
    Fill(raw, 3900U);
    raw[2] = 3370U; /* normalized=175, signal=75 */
    raw[3] = 3374U; /* normalized=173, signal=73 */
    result = Process(raw, calibration, &state);
    assert(result.selected_mask == 0x0CU);
    assert(result.line_strength == 148U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    /* Analogue acquisition remains bounded by total energy and narrow,
     * contiguous geometry. */
    state = {};
    Fill(raw, 3900U);
    raw[4] = 2204U; /* normalized=498, signal=398: below minimum */
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    state = {};
    Fill(raw, 3900U);
    raw[2] = 2920U;
    raw[5] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.line_strength == 400U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    state = {};
    Fill(raw, 3900U);
    raw[2] = 3100U; /* normalized=250, signal=150 */
    raw[3] = 3100U;
    raw[4] = 3100U;
    result = Process(raw, calibration, &state);
    assert(result.line_strength == 450U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    state = {};
    Fill(raw, 3900U);
    result = Process(raw, calibration, &state);
    assert(result.selected_mask == 0U);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    /* An already-active channel remains valid inside the hysteresis band,
     * even just below the cold analogue-acquisition minimum. */
    Fill(raw, 3900U);
    raw[4] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    raw[4] = 2204U; /* normalized=498, signal=398 */
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);

    /* The same sub-minimum isolated evidence cannot start a new detection. */
    state = {};
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    Fill(raw, 3900U);
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);

    /* A previously acquired line may cross a physical sensor gap. A single
     * connected weak core segment remains position-valid for a bounded eight
     * frames, using analogue interpolation rather than retaining a stale
     * position indefinitely. */
    state = {};
    Fill(raw, 3900U);
    raw[4] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.weak_tracking_frames == 0U);

    Fill(raw, 3900U);
    raw[4] = 2920U; /* normalized=300, signal above floor=200 */
    for (uint8_t frame = 1U;
         frame <= drivers::GRAYSCALE_MAX_WEAK_TRACKING_FRAMES;
         frame++) {
        result = Process(raw, calibration, &state);
        assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
        assert(result.position_valid);
        assert(result.line_position == -1000);
        assert(result.weak_tracking_frames == frame);
        assert(result.invalid_frames == 0U);
        assert(result.position_confidence <= 250U);
    }

    /* Weak tracking is bounded. The next weak frame starts the normal invalid
     * counter and cannot run forever like the reference implementation. */
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);
    assert(result.weak_tracking_frames == 0U);
    assert(result.invalid_frames == 1U);

    /* Strong evidence immediately reacquires and resets both counters. */
    Fill(raw, 3900U);
    raw[4] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.weak_tracking_frames == 0U);
    assert(result.invalid_frames == 0U);

    /* A weak segment on a disconnected core channel cannot jump across the
     * array from the previously tracked line. */
    state = {};
    Fill(raw, 3900U);
    raw[2] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    Fill(raw, 3900U);
    raw[5] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    /* No analogue signal above the calibrated floor is still an immediate
     * invalid frame even after a valid track. */
    state = {};
    Fill(raw, 3900U);
    raw[3] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    Fill(raw, 3900U);
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(result.invalid_frames == 1U);

    /* A short completely blank interval no longer destroys the spatial
     * recovery anchor. A connected weak segment may reappear within the same
     * eight-frame recovery budget. */
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(result.invalid_frames == 2U);
    Fill(raw, 3900U);
    raw[3] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.position_valid);
    assert(result.line_position == 1000);
    assert(result.weak_tracking_frames == 3U);
    assert(result.invalid_frames == 0U);

    /* Blank and accepted weak frames share one budget; alternating them
     * cannot extend weak recovery forever. */
    Fill(raw, 3900U);
    for (uint8_t age = 4U;
         age <= drivers::GRAYSCALE_MAX_WEAK_TRACKING_FRAMES;
         age++) {
        if ((age & 1U) == 0U) {
            result = Process(raw, calibration, &state);
            assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
        } else {
            raw[3] = 2920U;
            result = Process(raw, calibration, &state);
            assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
            assert(result.weak_tracking_frames == age);
            raw[3] = 3900U;
        }
    }
    raw[3] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    /* Strong evidence starts a fresh recovery window. */
    raw[3] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.weak_tracking_frames == 0U);
    assert(result.invalid_frames == 0U);

    /* Ambiguous geometry cancels the recovery anchor. A later weak signal is
     * not allowed to revive an old segment after a multiple-line event. */
    Fill(raw, 3900U);
    raw[2] = 800U;
    raw[5] = 800U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_MULTIPLE);
    Fill(raw, 3900U);
    raw[3] = 2920U;
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_LOST);
    assert(!result.position_valid);

    /* All four core channels indicate a wide/crossing pattern; position and
     * outer-channel road classification remain separate. */
    Fill(raw, 3900U);
    for (uint8_t i = 2U; i <= 5U; i++) {
        raw[i] = 800U;
    }
    result = Process(raw, calibration, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_WIDE);
    assert(!result.position_valid);

    /* Per-channel normalization also supports the opposite sensor polarity. */
    drivers::GrayscaleCalibration inverse = calibration;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        inverse.white[i] = 400U;
        inverse.black[i] = 4000U;
    }
    state = {};
    Fill(raw, 500U);
    raw[4] = 3600U;
    result = Process(raw, inverse, &state);
    assert(result.track_state == drivers::GRAYSCALE_TRACK_VALID);
    assert(result.line_position == -1000);

    return 0;
}
