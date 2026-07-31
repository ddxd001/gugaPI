#include "control/sensor_hub.h"

#include "board/board_ball_vision.h"
#include "board/board_grayscale.h"
#include "board/board_imu.h"
#include "control/vision_state.h"

namespace gugah {
namespace {

drivers::GrayscaleProcessingState g_line_state = {};
drivers::GrayscaleProcessedData g_line = {};
uint16_t g_gray_raw[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint8_t g_gray_next_channel = 0U;
bool g_line_valid = false;
bool g_line_frame_ready = false;

drivers::BallVisionParser g_vision_parser = {};
VisionState g_vision_state = {};
drivers::BallVisionFrame g_vision_candidate = {};
bool g_vision_candidate_valid = false;

ImuFeedback g_imu = {};
uint32_t g_last_imu_ms = 0U;
int32_t g_yaw_remainder = 0;
bool g_ready = false;
uint32_t g_error_count = 0U;
uint16_t g_vision_min_confidence = 500U;

/* Fixed-point CORDIC atan2, result in signed milli-degrees. */
int32_t Atan2MilliDeg(int32_t y, int32_t x)
{
    if ((x == 0) && (y == 0)) {
        return 0;
    }
    static const int32_t angles_udeg[24] = {
        45000000, 26565051, 14036243, 7125016, 3576334, 1789911,
        895174, 447614, 223811, 111906, 55953, 27976,
        13988, 6994, 3497, 1748, 874, 437, 218, 109, 55, 27, 14, 7
    };
    int64_t xi = static_cast<int64_t>(x) << 20U;
    int64_t yi = static_cast<int64_t>(y) << 20U;
    int32_t angle_udeg = 0;
    for (uint8_t i = 0U; i < 24U; i++) {
        const int64_t old_x = xi;
        if (yi >= 0) {
            xi += yi >> i;
            yi -= old_x >> i;
            angle_udeg += angles_udeg[i];
        } else {
            xi -= yi >> i;
            yi += old_x >> i;
            angle_udeg -= angles_udeg[i];
        }
    }
    return angle_udeg / 1000;
}

int32_t WrapSigned180(int32_t angle_mdeg)
{
    while (angle_mdeg > 180000) {
        angle_mdeg -= 360000;
    }
    while (angle_mdeg < -180000) {
        angle_mdeg += 360000;
    }
    return angle_mdeg;
}

void UpdateYaw(uint32_t now_ms)
{
    if (g_last_imu_ms == 0U) {
        g_last_imu_ms = now_ms;
        return;
    }
    const uint32_t elapsed_ms = now_ms - g_last_imu_ms;
    g_last_imu_ms = now_ms;
    if ((elapsed_ms == 0U) || (elapsed_ms > 100U)) {
        g_yaw_remainder = 0;
        return;
    }
    const int64_t scaled_delta =
        static_cast<int64_t>(g_imu.gyro_z_mdps) * elapsed_ms +
        g_yaw_remainder;
    const int32_t delta_mdeg = static_cast<int32_t>(scaled_delta / 1000LL);
    g_yaw_remainder = static_cast<int32_t>(scaled_delta % 1000LL);
    g_imu.yaw_mdeg = WrapSigned180(g_imu.yaw_mdeg + delta_mdeg);
}

} /* namespace */

bool SensorHub_Init(const HConfig *config)
{
    if (config == 0) {
        return false;
    }
    drivers::BallVisionParser_Init(&g_vision_parser);
    VisionState_Init(&g_vision_state);
    g_vision_candidate = {};
    g_vision_candidate_valid = false;
    g_line_state = {};
    g_line = {};
    g_gray_next_channel = 0U;
    g_line_valid = false;
    g_line_frame_ready = false;
    g_imu = {};
    g_last_imu_ms = 0U;
    g_yaw_remainder = 0;
    const drivers::DriverStatus gray = board::Board_GrayscaleInit();
    const drivers::DriverStatus vision = board::Board_BallVisionInit();
    const drivers::DriverStatus imu = board::Board_ImuInit();
    if (gray == drivers::DRIVER_OK) {
        (void)board::Board_GrayscalePrepareChannel(0U);
    }
    g_ready = (gray == drivers::DRIVER_OK) &&
              (vision == drivers::DRIVER_OK) &&
              (imu == drivers::DRIVER_OK);
    if (!g_ready) {
        g_error_count++;
    }
    return g_ready;
}

void SensorHub_Update1ms(uint32_t now_ms, const HConfig *config)
{
    (void)now_ms;
    if (config == 0) {
        return;
    }
    uint8_t completed_channel = 0U;
    uint16_t raw = 0U;
    const drivers::DriverStatus result =
        board::Board_GrayscaleTakeCompletedConversion(
            &completed_channel, &raw);
    if (result == drivers::DRIVER_OK) {
        g_gray_raw[completed_channel] = raw;
        if (completed_channel ==
            (drivers::GRAYSCALE_CHANNEL_COUNT - 1U)) {
            g_line_valid =
                (drivers::Grayscale_ProcessWithState(
                     g_gray_raw, &config->grayscale,
                     &g_line_state, &g_line) == drivers::DRIVER_OK);
            g_line_frame_ready = true;
        }
        g_gray_next_channel = static_cast<uint8_t>(
            (completed_channel + 1U) %
            drivers::GRAYSCALE_CHANNEL_COUNT);
    } else if ((result != drivers::DRIVER_ERROR_BUSY) &&
               (result != drivers::DRIVER_ERROR_NOT_INITIALIZED)) {
        g_error_count++;
    }

    const drivers::DriverStatus start =
        board::Board_GrayscaleStartPreparedConversion(
            g_gray_next_channel);
    if ((start != drivers::DRIVER_OK) &&
        (start != drivers::DRIVER_ERROR_BUSY)) {
        g_error_count++;
    }
}

void SensorHub_Update5ms(uint32_t now_ms, const HConfig *config)
{
    if (config == 0) {
        return;
    }
    drivers::Icm45686SensorData raw = {};
    const drivers::DriverStatus status = board::Board_ImuRead(&raw);
    if (status != drivers::DRIVER_OK) {
        g_error_count++;
        return;
    }
    const int32_t ax = drivers::Icm45686_AccelMilliG(
        raw.accel_x, drivers::ICM45686_ACCEL_FS_4G);
    const int32_t ay = drivers::Icm45686_AccelMilliG(
        raw.accel_y, drivers::ICM45686_ACCEL_FS_4G);
    const int32_t az = drivers::Icm45686_AccelMilliG(
        raw.accel_z, drivers::ICM45686_ACCEL_FS_4G);
    g_imu.accel_x_mg = ax;
    g_imu.accel_y_mg = ay;
    g_imu.accel_z_mg = az;
    g_imu.gyro_x_mdps = drivers::Icm45686_GyroMilliDps(
        raw.gyro_x, drivers::ICM45686_GYRO_FS_1000DPS);
    g_imu.gyro_y_mdps = drivers::Icm45686_GyroMilliDps(
        raw.gyro_y, drivers::ICM45686_GYRO_FS_1000DPS);
    g_imu.gyro_z_mdps = drivers::Icm45686_GyroMilliDps(
        raw.gyro_z, drivers::ICM45686_GYRO_FS_1000DPS) -
        config->imu_gyro_bias_z_mdps;
    g_imu.temperature_centi_c = drivers::Icm45686_TempCentiC(raw.temp);
    g_imu.valid = true;
    g_imu.pitch_mdeg = Atan2MilliDeg(-ax, az);
    UpdateYaw(now_ms);
    g_imu.received_ms = now_ms;
}

void SensorHub_ServiceVision(uint32_t now_ms, const HConfig *config)
{
    if (config == 0) {
        return;
    }
    g_vision_min_confidence = config->vision_min_confidence;
    uint8_t byte = 0U;
    drivers::BallVisionFrame frame = {};
    while (board::Board_BallVisionReadByte(&byte)) {
        if (drivers::BallVisionParser_FeedByte(
                &g_vision_parser, byte, now_ms, &frame)) {
            const bool sequence_confirmed =
                g_vision_candidate_valid &&
                (static_cast<uint8_t>(
                     frame.sequence - g_vision_candidate.sequence) == 1U);
            g_vision_candidate = frame;
            g_vision_candidate_valid = true;
            if (sequence_confirmed) {
                VisionState_Accept(
                    &g_vision_state, &frame, g_vision_min_confidence);
            }
        }
    }
}

const drivers::GrayscaleProcessedData *SensorHub_GetLine(void)
{
    return g_line_valid ? &g_line : 0;
}

bool SensorHub_TakeLineFrameReady(void)
{
    const bool ready = g_line_frame_ready;
    g_line_frame_ready = false;
    return ready;
}

VisionFeedback SensorHub_GetVision(uint32_t now_ms)
{
    return VisionState_Get(
        &g_vision_state, now_ms, g_vision_min_confidence);
}

ImuFeedback SensorHub_GetImu(void)
{
    return g_imu;
}

bool SensorHub_IsReady(void)
{
    return g_ready;
}

uint32_t SensorHub_GetErrorCount(void)
{
    return g_error_count;
}

const uint16_t *SensorHub_GetGrayscaleRaw(void)
{
    return g_gray_raw;
}

const drivers::BallVisionParserStats *SensorHub_GetVisionStats(void)
{
    return &g_vision_parser.stats;
}

bool SensorHub_GetLatestVisionFrame(drivers::BallVisionFrame *frame)
{
    if ((frame == 0) || !g_vision_candidate_valid) {
        return false;
    }
    *frame = g_vision_candidate;
    return true;
}

bool SensorHub_InjectVision(int16_t position_0p1mm,
                            uint16_t confidence,
                            uint16_t source_delay_ms,
                            uint32_t now_ms)
{
    if ((position_0p1mm < -1250) ||
        (position_0p1mm > 1250) ||
        (confidence > 1000U)) {
        return false;
    }
    drivers::BallVisionFrame frame = {};
    frame.sequence =
        static_cast<uint8_t>(g_vision_state.latest_frame.sequence + 1U);
    frame.flags =
        drivers::BALL_VISION_FLAG_BALL_FOUND |
        drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
        drivers::BALL_VISION_FLAG_CAMERA_OK;
    frame.position_0p1mm = position_0p1mm;
    frame.confidence = confidence;
    frame.source_delay_ms = source_delay_ms;
    frame.received_ms = now_ms;
    VisionState_Accept(
        &g_vision_state, &frame, g_vision_min_confidence);
    g_vision_candidate = frame;
    g_vision_candidate_valid = true;
    return true;
}

void SensorHub_ClearVision(void)
{
    VisionState_Init(&g_vision_state);
    drivers::BallVisionParser_ResetStream(&g_vision_parser);
    g_vision_candidate = {};
    g_vision_candidate_valid = false;
}

} /* namespace gugah */
