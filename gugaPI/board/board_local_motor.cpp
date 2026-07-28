#include "board/board_local_motor.h"

#include "board/board_pins.h"
#include "config/feature_config.h"
#include "drivers/drv8876/drv8876.h"
#include "drivers/wheel_encoder/wheel_encoder.h"

namespace board {

#if FEATURE_ENABLE_LOCAL_MOTOR && FEATURE_LOCAL_MOTOR_RIGHT_ONLY
namespace {

static const drivers::Drv8876Config g_rightDriveConfig = {
    BOARD_MOTOR_PWM_INST,
    BOARD_RIGHT_MOTOR_PWM_INDEX,
    BOARD_RIGHT_MOTOR_PH_PORT,
    BOARD_RIGHT_MOTOR_PH_PIN,
    BOARD_RIGHT_MOTOR_NSLEEP_PORT,
    BOARD_RIGHT_MOTOR_NSLEEP_PIN,
    BOARD_MOTOR_PWM_PERIOD_COUNTS,
    BOARD_MOTOR_WAKE_DELAY_MS
};

static const drivers::WheelEncoderConfig g_rightEncoderConfig = {
    BOARD_RIGHT_MOTOR_QEI_INST,
    BOARD_RIGHT_MOTOR_QEI_PHA_PORT,
    BOARD_RIGHT_MOTOR_QEI_PHA_PIN,
    BOARD_RIGHT_MOTOR_QEI_PHB_PORT,
    BOARD_RIGHT_MOTOR_QEI_PHB_PIN,
    1,
    BOARD_ENCODER_SAMPLE_PERIOD_MS
};

drivers::Drv8876Context g_rightDrive = {};
drivers::WheelEncoderContext g_rightEncoder = {};
bool g_initialized = false;

} /* namespace */

drivers::DriverStatus Board_LocalMotorInit(void)
{
    /* Repeat the SysConfig-safe state before any timer is started. */
    DL_GPIO_clearPins(BOARD_LEFT_MOTOR_NSLEEP_PORT,
                      BOARD_LEFT_MOTOR_NSLEEP_PIN |
                          BOARD_RIGHT_MOTOR_NSLEEP_PIN);
    DL_GPIO_clearPins(BOARD_LEFT_MOTOR_PH_PORT,
                      BOARD_LEFT_MOTOR_PH_PIN |
                          BOARD_RIGHT_MOTOR_PH_PIN);
    DL_TimerG_setCaptureCompareValue(BOARD_MOTOR_PWM_INST,
                                     BOARD_MOTOR_PWM_PERIOD_COUNTS,
                                     BOARD_RIGHT_MOTOR_PWM_INDEX);

    drivers::DriverStatus status =
        drivers::Drv8876_Init(&g_rightDrive, &g_rightDriveConfig);
    if (status == drivers::DRIVER_OK) {
        status = drivers::WheelEncoder_Init(&g_rightEncoder,
                                             &g_rightEncoderConfig,
                                             0U);
    }
    if (status != drivers::DRIVER_OK) {
        (void) Board_LocalMotorSleepAll();
        g_initialized = false;
        return status;
    }

    DL_TimerG_startCounter(BOARD_MOTOR_PWM_INST);
    g_initialized = true;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_LocalMotorProcessEncoders(uint32_t now_ms)
{
    if (!g_initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    return drivers::WheelEncoder_Process(&g_rightEncoder, now_ms);
}

drivers::DriverStatus Board_LocalMotorGetEncoder(
    LocalMotorWheel wheel,
    LocalMotorEncoderSnapshot *snapshot)
{
    if (snapshot == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (wheel == LOCAL_MOTOR_LEFT) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if (wheel != LOCAL_MOTOR_RIGHT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (!g_initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    drivers::WheelEncoderSnapshot raw = {};
    const drivers::DriverStatus status =
        drivers::WheelEncoder_GetSnapshot(&g_rightEncoder, &raw);
    if (status == drivers::DRIVER_OK) {
        snapshot->count = raw.count;
        snapshot->counts_per_second = raw.counts_per_second;
        snapshot->state = raw.state;
    }
    return status;
}

drivers::DriverStatus Board_LocalMotorRun(LocalMotorWheel wheel,
                                          bool reverse,
                                          uint16_t duty_q8,
                                          uint32_t now_ms)
{
    if (wheel == LOCAL_MOTOR_LEFT) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if (wheel != LOCAL_MOTOR_RIGHT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (!g_initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    /* Logical wheel direction is converted to the physical PH level here;
     * motor_output_invert_flags remains the per-wheel installation calibration. */
    return drivers::Drv8876_Run(
        &g_rightDrive,
        reverse ? drivers::DRV8876_PHASE_HIGH : drivers::DRV8876_PHASE_LOW,
        duty_q8,
        now_ms);
}

drivers::DriverStatus Board_LocalMotorSleep(LocalMotorWheel wheel)
{
    if (wheel == LOCAL_MOTOR_LEFT) {
        DL_GPIO_clearPins(BOARD_LEFT_MOTOR_NSLEEP_PORT,
                          BOARD_LEFT_MOTOR_NSLEEP_PIN);
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if (wheel != LOCAL_MOTOR_RIGHT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    return drivers::Drv8876_Sleep(&g_rightDrive);
}

drivers::DriverStatus Board_LocalMotorSleepAll(void)
{
    const drivers::DriverStatus right_status =
        drivers::Drv8876_Sleep(&g_rightDrive);
    /* The unsupported left bridge has no driver context, so always force both
     * hardware sleep pins low independently of right-driver init state. */
    DL_GPIO_clearPins(BOARD_LEFT_MOTOR_NSLEEP_PORT,
                      BOARD_LEFT_MOTOR_NSLEEP_PIN |
                          BOARD_RIGHT_MOTOR_NSLEEP_PIN);
    return (right_status == drivers::DRIVER_ERROR_NOT_INITIALIZED)
               ? drivers::DRIVER_OK
               : right_status;
}

bool Board_LocalMotorIsReady(void)
{
    return g_initialized;
}

bool Board_LocalMotorIsAwake(LocalMotorWheel wheel)
{
    if (!g_initialized || (wheel != LOCAL_MOTOR_RIGHT)) {
        return false;
    }
    return drivers::Drv8876_IsAwake(&g_rightDrive);
}

#else

drivers::DriverStatus Board_LocalMotorInit(void)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus Board_LocalMotorProcessEncoders(uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus Board_LocalMotorGetEncoder(
    LocalMotorWheel,
    LocalMotorEncoderSnapshot *)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus Board_LocalMotorRun(LocalMotorWheel,
                                          bool,
                                          uint16_t,
                                          uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus Board_LocalMotorSleep(LocalMotorWheel)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus Board_LocalMotorSleepAll(void)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

bool Board_LocalMotorIsReady(void)
{
    return false;
}

bool Board_LocalMotorIsAwake(LocalMotorWheel)
{
    return false;
}

#endif

} /* namespace board */
