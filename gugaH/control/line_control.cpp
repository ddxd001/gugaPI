#include "control/line_control.h"

#include <limits.h>

namespace gugah {
namespace {

int32_t Clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

int16_t ClampRpm(int32_t value)
{
    return static_cast<int16_t>(Clamp32(value, -1000, 1000));
}

int32_t StepToward(int32_t current, int32_t target, int32_t step)
{
    if (current < target) {
        const int32_t remaining = target - current;
        return current + ((remaining < step) ? remaining : step);
    }
    if (current > target) {
        const int32_t remaining = current - target;
        return current - ((remaining < step) ? remaining : step);
    }
    return current;
}

} /* namespace */

void LineControl_Init(LineControlState *state)
{
    if (state == 0) {
        return;
    }
    *state = {};
    state->initialized = true;
}

bool LineControl_IsTrackUsable(
    const drivers::GrayscaleProcessedData *line)
{
    return (line != 0) && line->line_detected && line->position_valid &&
           ((line->track_state == drivers::GRAYSCALE_TRACK_VALID) ||
            (line->track_state == drivers::GRAYSCALE_TRACK_WIDE)) &&
           (line->calibration_fault_mask == 0U);
}

bool LineControl_SearchRight(LineControlState *state,
                             int16_t forward_rpm,
                             uint32_t now_ms)
{
    if ((state == 0) || !state->initialized || (forward_rpm <= 0)) {
        return false;
    }

    /* Positive wheel RPM means forward.  A faster left wheel therefore
     * searches along a forward-right arc without pivoting in place. */
    int32_t turn_rpm = static_cast<int32_t>(forward_rpm) / 3;
    if (turn_rpm < 1) {
        turn_rpm = 1;
    }
    state->line_valid = false;
    state->failed = false;
    state->last_update_ms = now_ms;
    state->last_correction_rpm = -turn_rpm;
    state->left_rpm = ClampRpm(
        static_cast<int32_t>(forward_rpm) + turn_rpm);
    state->right_rpm = ClampRpm(
        static_cast<int32_t>(forward_rpm) - turn_rpm);
    return true;
}

bool LineControl_Update(LineControlState *state,
                        const drivers::GrayscaleProcessedData *line,
                        int16_t base_rpm,
                        uint32_t now_ms,
                        const HConfig *config)
{
    if ((state == 0) || !state->initialized || (config == 0)) {
        return false;
    }

    uint32_t elapsed_ms = now_ms - state->last_update_ms;
    if ((elapsed_ms == 0U) || (elapsed_ms > 100U)) {
        elapsed_ms = 2U;
    }
    state->last_update_ms = now_ms;

    const bool was_line_valid = state->line_valid;
    const bool usable = LineControl_IsTrackUsable(line);
    if (!usable) {
        if (state->invalid_since_ms == 0U) {
            state->invalid_since_ms = now_ms;
        }
        if ((now_ms - state->invalid_since_ms) >
            config->line_lost_grace_ms) {
            state->failed = true;
            state->line_valid = false;
            state->left_rpm = 0;
            state->right_rpm = 0;
            return false;
        }
        state->line_valid = false;
    } else {
        state->invalid_since_ms = 0U;
        state->line_valid = true;
    }

    int32_t requested = state->last_correction_rpm;
    if (usable && (line->track_state == drivers::GRAYSCALE_TRACK_VALID)) {
        if (!was_line_valid) {
            /* Reacquisition starts from the new measurement so the first
             * derivative term cannot kick the vehicle away from the line. */
            state->last_position = line->line_position;
        }
        const int32_t derivative =
            static_cast<int32_t>(
                (static_cast<int64_t>(
                     line->line_position - state->last_position) *
                 1000LL) /
                elapsed_ms);
        requested = static_cast<int32_t>(
            (static_cast<int64_t>(config->line_kp_milli) *
                 line->line_position +
             static_cast<int64_t>(config->line_kd_milli) *
                 derivative) /
            1000LL);
        state->last_position = line->line_position;
    }

    requested = Clamp32(requested,
                        -config->line_max_correction_rpm,
                        config->line_max_correction_rpm);
    int32_t slew_step = static_cast<int32_t>(
        (static_cast<uint64_t>(
             config->line_correction_slew_rpm_s) *
         elapsed_ms) /
        1000U);
    if (slew_step < 1) {
        slew_step = 1;
    }
    state->last_correction_rpm =
        StepToward(state->last_correction_rpm, requested, slew_step);
    state->left_rpm = ClampRpm(
        static_cast<int32_t>(base_rpm) -
        state->last_correction_rpm);
    state->right_rpm = ClampRpm(
        static_cast<int32_t>(base_rpm) +
        state->last_correction_rpm);
    return true;
}

} /* namespace gugah */
