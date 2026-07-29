#ifndef APP_MODE_SWITCH_CHORD_H_
#define APP_MODE_SWITCH_CHORD_H_

#include <stdbool.h>
#include <stdint.h>

namespace app {

enum ModeSwitchChordEvent {
    MODE_SWITCH_CHORD_NONE = 0,
    MODE_SWITCH_CHORD_STARTED,
    MODE_SWITCH_CHORD_TRIGGERED
};

struct ModeSwitchChordState {
    bool tracking;
    bool latched;
    uint32_t started_ms;
};

inline void ModeSwitchChord_Reset(ModeSwitchChordState *state)
{
    if (state == 0) {
        return;
    }
    state->tracking = false;
    state->latched = false;
    state->started_ms = 0U;
}

inline bool ModeSwitchChord_IsActive(const ModeSwitchChordState *state)
{
    return (state != 0) && (state->tracking || state->latched);
}

inline ModeSwitchChordEvent ModeSwitchChord_Update(
    ModeSwitchChordState *state,
    bool button1_pressed,
    bool button3_pressed,
    uint32_t now_ms,
    uint32_t hold_ms)
{
    if (state == 0) {
        return MODE_SWITCH_CHORD_NONE;
    }

    const bool both_pressed = button1_pressed && button3_pressed;
    if (state->latched) {
        if (!button1_pressed && !button3_pressed) {
            state->latched = false;
        }
        return MODE_SWITCH_CHORD_NONE;
    }

    if (!both_pressed) {
        if (state->tracking) {
            state->tracking = false;
            state->latched = true;
        }
        return MODE_SWITCH_CHORD_NONE;
    }

    if (!state->tracking) {
        state->tracking = true;
        state->started_ms = now_ms;
        return MODE_SWITCH_CHORD_STARTED;
    }

    if ((uint32_t) (now_ms - state->started_ms) >= hold_ms) {
        state->tracking = false;
        state->latched = true;
        return MODE_SWITCH_CHORD_TRIGGERED;
    }

    return MODE_SWITCH_CHORD_NONE;
}

} /* namespace app */

#endif /* APP_MODE_SWITCH_CHORD_H_ */
