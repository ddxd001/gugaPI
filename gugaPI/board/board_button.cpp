#include "board/board_button.h"

#include "board/board_pins.h"
#include "drivers/button/button.h"

namespace board {

namespace {

static const drivers::ButtonConfig g_buttonConfigs[BOARD_BUTTON_COUNT] = {
    {
        BOARD_BUTTON1_PORT,
        BOARD_BUTTON1_PIN,
        (BOARD_BUTTON_ACTIVE_LOW != 0U),
        BOARD_BUTTON_DEBOUNCE_MS
    },
    {
        BOARD_BUTTON2_PORT,
        BOARD_BUTTON2_PIN,
        (BOARD_BUTTON_ACTIVE_LOW != 0U),
        BOARD_BUTTON_DEBOUNCE_MS
    },
    {
        BOARD_BUTTON3_PORT,
        BOARD_BUTTON3_PIN,
        (BOARD_BUTTON_ACTIVE_LOW != 0U),
        BOARD_BUTTON_DEBOUNCE_MS
    }
};

static drivers::ButtonContext g_buttonContexts[BOARD_BUTTON_COUNT];

bool IsValidButtonId(BoardButtonId id)
{
    return ((uint32_t) id < (uint32_t) BOARD_BUTTON_COUNT);
}

} /* namespace */

drivers::DriverStatus Board_ButtonsInit(void)
{
    for (uint32_t i = 0U; i < (uint32_t) BOARD_BUTTON_COUNT; i++) {
        const drivers::DriverStatus status = drivers::Button_Init(
            &g_buttonContexts[i],
            &g_buttonConfigs[i]);

        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ButtonsUpdate(uint32_t now_ms)
{
    for (uint32_t i = 0U; i < (uint32_t) BOARD_BUTTON_COUNT; i++) {
        const drivers::DriverStatus status = drivers::Button_Update(
            &g_buttonContexts[i],
            now_ms);

        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ButtonReadRaw(BoardButtonId id, bool *pressed)
{
    if (!IsValidButtonId(id)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::Button_ReadRaw(&g_buttonContexts[id], pressed);
}

const char *Board_ButtonGetName(BoardButtonId id)
{
    switch (id) {
    case BOARD_BUTTON_1:
        return "button1";
    case BOARD_BUTTON_2:
        return "button2";
    case BOARD_BUTTON_3:
        return "button3";
    default:
        return "unknown";
    }
}

bool Board_ButtonIsReady(BoardButtonId id)
{
    if (!IsValidButtonId(id)) {
        return false;
    }

    return drivers::Button_IsReady(&g_buttonContexts[id]);
}

bool Board_ButtonIsPressed(BoardButtonId id)
{
    if (!IsValidButtonId(id)) {
        return false;
    }

    return drivers::Button_IsPressed(&g_buttonContexts[id]);
}

bool Board_ButtonWasPressed(BoardButtonId id)
{
    if (!IsValidButtonId(id)) {
        return false;
    }

    return drivers::Button_WasPressed(&g_buttonContexts[id]);
}

bool Board_ButtonWasReleased(BoardButtonId id)
{
    if (!IsValidButtonId(id)) {
        return false;
    }

    return drivers::Button_WasReleased(&g_buttonContexts[id]);
}

} /* namespace board */
