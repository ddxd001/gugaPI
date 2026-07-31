#include "board/board_led.h"

#include "board/board_pins.h"
#include "drivers/led/led.h"

namespace board {
namespace {

static const drivers::LedConfig g_ledConfigs[BOARD_LED_COUNT] = {
    {
        BOARD_LED1_PORT,
        BOARD_LED1_PIN,
        (BOARD_LED1_ACTIVE_LOW != 0U),
        (BOARD_LED1_INITIAL_ON != 0U)
    },
    {
        BOARD_LED2_PORT,
        BOARD_LED2_PIN,
        (BOARD_LED2_ACTIVE_LOW != 0U),
        (BOARD_LED2_INITIAL_ON != 0U)
    },
    {
        BOARD_LED3_PORT,
        BOARD_LED3_PIN,
        (BOARD_LED3_ACTIVE_LOW != 0U),
        (BOARD_LED3_INITIAL_ON != 0U)
    }
};

static drivers::LedContext g_ledContexts[BOARD_LED_COUNT] = {
    { 0, false, false },
    { 0, false, false },
    { 0, false, false }
};

bool IsLedIdValid(BoardLedId id)
{
    return static_cast<uint8_t>(id) < BOARD_LED_COUNT;
}

drivers::LedContext *LedContextFor(BoardLedId id)
{
    if (!IsLedIdValid(id)) {
        return 0;
    }

    return &g_ledContexts[static_cast<uint8_t>(id)];
}

} /* namespace */

drivers::DriverStatus Board_LedsInit(void)
{
    for (uint8_t index = 0U; index < BOARD_LED_COUNT; index++) {
        const drivers::DriverStatus status =
            drivers::Led_Init(&g_ledContexts[index], &g_ledConfigs[index]);
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_LedSet(BoardLedId id, bool on)
{
    drivers::LedContext *ctx = LedContextFor(id);
    if (ctx == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::Led_Set(ctx, on);
}

drivers::DriverStatus Board_LedOn(BoardLedId id)
{
    return Board_LedSet(id, true);
}

drivers::DriverStatus Board_LedOff(BoardLedId id)
{
    return Board_LedSet(id, false);
}

drivers::DriverStatus Board_LedToggle(BoardLedId id)
{
    drivers::LedContext *ctx = LedContextFor(id);
    if (ctx == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::Led_Toggle(ctx);
}

bool Board_LedIsReady(BoardLedId id)
{
    drivers::LedContext *ctx = LedContextFor(id);
    return drivers::Led_IsReady(ctx);
}

bool Board_LedIsOn(BoardLedId id)
{
    drivers::LedContext *ctx = LedContextFor(id);
    return drivers::Led_IsOn(ctx);
}

uint8_t Board_LedCount(void)
{
    return BOARD_LED_COUNT;
}

drivers::DriverStatus Board_StatusLedInit(void)
{
    return Board_LedsInit();
}

drivers::DriverStatus Board_StatusLedOn(void)
{
    return Board_LedOn(BOARD_LED_ID_1);
}

drivers::DriverStatus Board_StatusLedOff(void)
{
    return Board_LedOff(BOARD_LED_ID_1);
}

drivers::DriverStatus Board_StatusLedToggle(void)
{
    return Board_LedToggle(BOARD_LED_ID_1);
}

bool Board_StatusLedIsReady(void)
{
    return Board_LedIsReady(BOARD_LED_ID_1);
}

bool Board_StatusLedIsOn(void)
{
    return Board_LedIsOn(BOARD_LED_ID_1);
}

} /* namespace board */
