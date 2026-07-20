#include "board/board.h"

#include "board/board_button.h"

#include "board/board_buzzer.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_grayscale.h"
#include "board/board_gy931.h"
#include "board/board_ina219.h"
#include "board/board_imu.h"
#include "board/board_led.h"
#include "board/board_lora.h"
#include "board/board_motor_driver.h"
#include "board/board_oled.h"
#include "board/board_pins.h"
#include "config/feature_config.h"

namespace board {

namespace {

drivers::DriverStatus g_initStatus = drivers::DRIVER_OK;
const char *g_failedDriver = nullptr;

/* Record the first failing driver; later failures do not overwrite it so the
 * caller can report the root cause. Every init result is examined, never
 * silently discarded. */
void TrackInit(const char *name, drivers::DriverStatus status)
{
    if ((status != drivers::DRIVER_OK) && (g_failedDriver == nullptr)) {
        g_failedDriver = name;
        g_initStatus = status;
    }
}

} /* namespace */

drivers::DriverStatus Board_Init(void)
{
    /*
     * SysConfig already performed low-level clock, power, and pin mux setup.
     * Put custom board bring-up here: power rails, enables, default GPIO
     * states, and board-level self checks.
     */
    g_initStatus = drivers::DRIVER_OK;
    g_failedDriver = nullptr;

#if FEATURE_ENABLE_STATUS_LED
    TrackInit("status_led", Board_StatusLedInit());
#endif

#if FEATURE_ENABLE_BUZZER
    TrackInit("buzzer", Board_BuzzerInit());
#endif

#if FEATURE_ENABLE_BUTTONS
    TrackInit("buttons", Board_ButtonsInit());
#endif

#if FEATURE_ENABLE_FRAM
    TrackInit("fram", Board_FramInit());
#endif

#if FEATURE_ENABLE_INA219
    TrackInit("ina219", Board_Ina219Init());
#endif

#if FEATURE_ENABLE_OLED
    TrackInit("oled", Board_OledInit());
#endif

#if FEATURE_ENABLE_GY931
    TrackInit("gy931", Board_Gy931Init());
#endif

#if FEATURE_ENABLE_IMU
    TrackInit("imu", Board_ImuInit());
#endif

#if FEATURE_ENABLE_LORA
    TrackInit("lora", Board_LoraInit());
#endif

#if FEATURE_ENABLE_MOTOR_DRIVER
    TrackInit("motor_driver", Board_MotorDriverInit());
#endif
#if FEATURE_ENABLE_GRAYSCALE
    TrackInit("grayscale", Board_GrayscaleInit());
#endif

    return g_initStatus;
}

const char *Board_GetFailedDriver(void)
{
    return g_failedDriver;
}

void Board_LateInit(void)
{
    /*
     * Use this hook after all drivers are initialized, for devices that need
     * a known system state before being enabled.
     */
}

} /* namespace board */
