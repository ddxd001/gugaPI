#ifndef GUGAH_CONTROL_CONTROL_TYPES_H_
#define GUGAH_CONTROL_CONTROL_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/ball_vision/ball_vision_protocol.h"
#include "drivers/grayscale/grayscale_processing.h"

namespace gugah {

struct ChassisFeedback {
    bool valid;
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    int16_t left_rpm;
    int16_t right_rpm;
    uint32_t received_ms;
};

struct ImuFeedback {
    bool valid;
    int32_t pitch_mdeg;
    int32_t yaw_mdeg;
    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;
    int32_t gyro_x_mdps;
    int32_t gyro_y_mdps;
    int32_t gyro_z_mdps;
    int32_t temperature_centi_c;
    uint32_t received_ms;
};

struct DmFeedback {
    bool valid;
    int32_t position_mrad;
    int32_t velocity_mrad_s;
    int32_t torque_mnm;
    uint8_t state;
    uint8_t mos_temperature_c;
    uint8_t coil_temperature_c;
    uint32_t received_ms;
};

struct VisionFeedback {
    bool communication_online;
    bool ball_usable;
    drivers::BallVisionFrame frame;
    uint32_t frame_age_ms;
    uint32_t ball_age_ms;
};

enum MotionCommandMode : uint8_t {
    MOTION_COMMAND_NONE = 0U,
    MOTION_COMMAND_SPEED,
    MOTION_COMMAND_POSITION,
    MOTION_COMMAND_STOP
};

struct MotionCommand {
    MotionCommandMode mode;
    int16_t left_rpm;
    int16_t right_rpm;
    int32_t left_position_count;
    int32_t right_position_count;
};

} /* namespace gugah */

#endif /* GUGAH_CONTROL_CONTROL_TYPES_H_ */
