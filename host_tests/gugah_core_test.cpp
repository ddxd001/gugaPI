#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/h_app.h"
#include "config/h_config.h"
#include "control/ball_control.h"
#include "control/course_control.h"
#include "control/vision_state.h"
#include "drivers/ball_vision/ball_vision_protocol.h"

namespace {

gugah::HConfig DefaultConfig()
{
    gugah::HConfig config = {};
    gugah::HConfig_Defaults(&config);
    assert(gugah::HConfig_Validate(&config));
    return config;
}

drivers::GrayscaleProcessedData ValidLine(int16_t position)
{
    drivers::GrayscaleProcessedData line = {};
    line.line_detected = true;
    line.position_valid = true;
    line.track_state = drivers::GRAYSCALE_TRACK_VALID;
    line.line_position = position;
    line.active_mask = 0x18U;
    return line;
}

gugah::ChassisFeedback Chassis(uint32_t now,
                               int32_t left_count,
                               int32_t right_count,
                               int16_t rpm)
{
    gugah::ChassisFeedback feedback = {};
    feedback.valid = true;
    feedback.left_encoder_count = left_count;
    feedback.right_encoder_count = right_count;
    feedback.left_rpm = rpm;
    feedback.right_rpm = rpm;
    feedback.received_ms = now;
    return feedback;
}

int16_t CameraPositionForPhysical(int16_t position)
{
    int16_t camera_position = -2000;
    int32_t best_error = INT32_MAX;
    for (int32_t raw = -2000; raw <= 2000; raw++) {
        const int32_t calibrated =
            gugah::Ball_CalibrateCameraPosition0p1mm(
                static_cast<int16_t>(raw));
        const int32_t error = (calibrated >= position)
            ? (calibrated - position) : (position - calibrated);
        if (error < best_error) {
            best_error = error;
            camera_position = static_cast<int16_t>(raw);
        }
    }
    return camera_position;
}

gugah::BallInput BallInput(uint32_t now, int16_t position)
{
    gugah::BallInput input = {};
    input.now_ms = now;
    input.vision.communication_online = true;
    input.vision.ball_usable = true;
    input.vision.ball_age_ms = 0U;
    input.vision.frame.sequence =
        static_cast<uint8_t>(now / 10U);
    input.vision.frame.flags = 0x07U;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(position);
    input.vision.frame.confidence = 1000U;
    input.vision.frame.received_ms = now;
    input.imu.valid = true;
    input.imu.received_ms = now;
    input.dm.valid = true;
    input.dm.state = 1U;
    input.dm.received_ms = now;
    return input;
}

void TestConfigRecord()
{
    const gugah::HConfig config = DefaultConfig();
    assert(config.grayscale.hysteresis == 300U);
    assert(config.grayscale.min_line_strength == 600U);
    assert(config.grayscale.track_mask == 0xFFU);
    assert(config.wheel_radius_um == 33050U);
    assert(config.left_counts_per_rev == 1456U);
    assert(config.right_counts_per_rev == 1456U);
    assert(config.motor_output_invert_flags == 3U);
    assert(config.motor_encoder_invert_flags == 1U);
    assert(config.motor_speed_kp_q4_4 == 2U);
    assert(config.motor_speed_ki_q4_4 == 2U);
    assert(config.motor_speed_accel_rpm_s == 1500U);
    assert(config.motor_speed_decel_rpm_s == 2000U);
    assert(config.motor_position_kp_q4_4 == 15U);
    assert(config.motor_position_max_rpm == 40U);
    assert(config.motor_position_tolerance_counts == 3U);
    assert(config.vision_position_invert == 1U);
    assert(config.vision_min_confidence == 100U);
    assert(config.ball_angle_slew_mdeg_s == 30000);
    assert(config.ball_position_tolerance_0p1mm == 100);
    assert(config.ball_velocity_tolerance_0p1mm_s == 100);
    assert(config.ball_kp_mdeg_per_0p1mm == 2300);
    assert(config.ball_kd_mdeg_per_0p1mm_s == 8);
    assert(config.ball_ki_mdeg_per_0p1mm_s == 1);
    assert(config.ball_max_angle_mdeg == 6000);
    assert(config.ball_model_roll_gain_permille == 714U);
    assert(config.ball_model_response_ms == 800U);
    assert(config.ball_model_plan_accel_0p1mm_s2 == 100U);
    assert(config.ball_model_max_velocity_0p1mm_s == 200U);
    assert(config.ball_observer_alpha_permille == 500U);
    assert(config.ball_observer_beta_permille == 80U);
    assert(config.ball_pid_kp_mdeg_per_mm == 40);
    assert(config.ball_pid_ki_mdeg_per_mm_s == 20);
    assert(config.ball_pid_kd_mdeg_per_mm_s == 20);
    assert(config.ball_pid_integral_limit_mdeg == 1500);
    assert(config.ball_curve_origin_0p1mm == 148);
    assert(config.ball_hold_position_0p1mm[0] == -1000);
    assert(config.ball_hold_position_0p1mm[4] == 1000);
    assert(config.ball_hold_angle_mdeg[0] == -918);
    assert(config.ball_hold_angle_mdeg[2] == -118);
    assert(config.ball_hold_angle_mdeg[4] == 682);
    assert(config.ball_accel_ff_mdeg_per_mm_s2 == 0);
    assert(config.dm_position_mrad[0] == 230);
    assert(config.dm_position_mrad[1] == -150);
    assert(config.dm_position_mrad[2] == -570);
    assert(config.dm_position_mrad[3] == -1070);
    assert(config.dm_position_mrad[4] == -1490);
    gugah::HConfigRecord record = {};
    gugah::HConfig_BuildRecord(&config, &record);
    gugah::HConfig parsed = {};
    assert(gugah::HConfig_ParseRecord(&record, &parsed));
    assert(parsed.lap_distance_mm == 6142U);
    record.payload.cruise_rpm++;
    assert(!gugah::HConfig_ParseRecord(&record, &parsed));

    /* Schema-2 records keep all commissioned fields and receive safe model
     * defaults for the fields appended by the model controller. */
    gugah::HConfigRecord schema2 = {};
    const uint16_t schema2_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_model_roll_gain_permille));
    schema2.magic = gugah::H_CONFIG_MAGIC;
    schema2.schema_version = 2U;
    schema2.payload_length = schema2_length;
    memcpy(&schema2.payload, &config, schema2_length);
    const uint32_t schema2_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema2.payload),
        schema2_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema2.payload) + schema2_length,
           &schema2_crc, sizeof(schema2_crc));
    assert(gugah::HConfig_ParseRecord(&schema2, &parsed));
    assert(parsed.wheel_radius_um == config.wheel_radius_um);
    assert(parsed.ball_model_roll_gain_permille == 714U);
    assert(parsed.ball_model_response_ms == 800U);
    assert(parsed.ball_pitch_gain_permille == 0);
    assert(parsed.ball_accel_ff_mdeg_per_mm_s2 == 0);

    /* Schema-3 records retain the commissioned model values and receive
     * the bounded residual PID defaults appended by schema 4. */
    gugah::HConfigRecord schema3 = {};
    const uint16_t schema3_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_pid_kp_mdeg_per_mm));
    schema3.magic = gugah::H_CONFIG_MAGIC;
    schema3.schema_version = 3U;
    schema3.payload_length = schema3_length;
    memcpy(&schema3.payload, &config, schema3_length);
    const uint32_t schema3_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema3.payload),
        schema3_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema3.payload) + schema3_length,
           &schema3_crc, sizeof(schema3_crc));
    assert(gugah::HConfig_ParseRecord(&schema3, &parsed));
    assert(parsed.ball_model_response_ms ==
           config.ball_model_response_ms);
    assert(parsed.ball_pid_kp_mdeg_per_mm == 40);
    assert(parsed.ball_pid_ki_mdeg_per_mm_s == 20);
    assert(parsed.ball_pid_kd_mdeg_per_mm_s == 20);
    assert(parsed.ball_pid_integral_limit_mdeg == 1500);
    assert(parsed.ball_curve_origin_0p1mm == 148);

    /* Schema-5 records receive the new holding-angle table and the new
     * alpha-beta observer/PD gains instead of reusing incompatible values. */
    gugah::HConfigRecord schema5 = {};
    const uint16_t schema5_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_hold_position_0p1mm));
    schema5.magic = gugah::H_CONFIG_MAGIC;
    schema5.schema_version = 5U;
    schema5.payload_length = schema5_length;
    memcpy(&schema5.payload, &config, schema5_length);
    schema5.payload.ball_observer_beta_permille = 600U;
    schema5.payload.ball_pid_kp_mdeg_per_mm = 80;
    const uint32_t schema5_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema5.payload),
        schema5_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema5.payload) + schema5_length,
           &schema5_crc, sizeof(schema5_crc));
    assert(gugah::HConfig_ParseRecord(&schema5, &parsed));
    assert(parsed.ball_observer_beta_permille == 80U);
    assert(parsed.ball_pid_kp_mdeg_per_mm == 40);
    assert(parsed.ball_pid_kd_mdeg_per_mm_s == 20);
    assert(parsed.ball_hold_angle_mdeg[2] == -118);

    /* Schema 6 used the old linear mechanism map.  Loading it preserves all
     * other parameters but installs the measured asymmetric linkage map. */
    gugah::HConfigRecord schema6 = {};
    schema6.magic = gugah::H_CONFIG_MAGIC;
    schema6.schema_version = 6U;
    schema6.payload_length = static_cast<uint16_t>(sizeof(gugah::HConfig));
    schema6.payload = config;
    const int16_t schema6_old_dm[5] = { 500, 0, -500, -1000, -1500 };
    for (uint8_t i = 0U; i < 5U; i++) {
        schema6.payload.dm_position_mrad[i] = schema6_old_dm[i];
    }
    schema6.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema6.payload),
        schema6.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema6, &parsed));
    assert(parsed.dm_position_mrad[0] == 230);
    assert(parsed.dm_position_mrad[2] == -570);
    assert(parsed.dm_position_mrad[4] == -1490);

    /* Schema 7 commissioned the observer with integral disabled.  Upgrade
     * that exact state to the bounded static-error trim introduced by 8. */
    gugah::HConfigRecord schema7 = {};
    schema7.magic = gugah::H_CONFIG_MAGIC;
    schema7.schema_version = 7U;
    schema7.payload_length = static_cast<uint16_t>(sizeof(gugah::HConfig));
    schema7.payload = config;
    schema7.payload.ball_pid_ki_mdeg_per_mm_s = 0;
    schema7.payload.ball_pid_integral_limit_mdeg = 0;
    schema7.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema7.payload),
        schema7.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema7, &parsed));
    assert(parsed.ball_pid_ki_mdeg_per_mm_s == 20);
    assert(parsed.ball_pid_integral_limit_mdeg == 1500);

    /* Schema-4 records contain the residual PID but predate the adjustable
     * curvature origin. */
    gugah::HConfigRecord schema4 = {};
    const uint16_t schema4_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_curve_origin_0p1mm));
    schema4.magic = gugah::H_CONFIG_MAGIC;
    schema4.schema_version = 4U;
    schema4.payload_length = schema4_length;
    memcpy(&schema4.payload, &config, schema4_length);
    const uint32_t schema4_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema4.payload),
        schema4_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema4.payload) + schema4_length,
           &schema4_crc, sizeof(schema4_crc));
    assert(gugah::HConfig_ParseRecord(&schema4, &parsed));
    assert(parsed.ball_pid_kp_mdeg_per_mm ==
           config.ball_pid_kp_mdeg_per_mm);
    assert(parsed.ball_curve_origin_0p1mm == 148);

    /* Legacy increasing maps are corrected without discarding other values. */
    gugah::HConfig legacy = config;
    const int16_t old_positions[5] = {
        -1000, -500, 0, 500, 1000
    };
    for (uint8_t i = 0U; i < 5U; i++) {
        legacy.dm_position_mrad[i] = old_positions[i];
    }
    legacy.vision_position_invert = 0U;
    legacy.vision_min_confidence = 500U;
    legacy.ball_angle_slew_mdeg_s = 60000;
    legacy.ball_kp_mdeg_per_0p1mm = 50;
    legacy.ball_kd_mdeg_per_0p1mm_s = 0;
    legacy.ball_ki_mdeg_per_0p1mm_s = 0;
    legacy.ball_max_angle_mdeg = 8000;
    legacy.ball_degraded_angle_mdeg = 3000;
    legacy.cruise_rpm = 123U;
    gugah::HConfig_BuildRecord(&legacy, &record);
    assert(gugah::HConfig_ParseRecord(&record, &parsed));
    assert(parsed.cruise_rpm == 123U);
    assert(parsed.vision_position_invert == 1U);
    assert(parsed.vision_min_confidence == 100U);
    assert(parsed.ball_angle_slew_mdeg_s == 30000);
    assert(parsed.ball_kp_mdeg_per_0p1mm == 2300);
    assert(parsed.ball_kd_mdeg_per_0p1mm_s == 8);
    assert(parsed.ball_ki_mdeg_per_0p1mm_s == 1);
    assert(parsed.ball_max_angle_mdeg == 6000);
    assert(parsed.dm_position_mrad[0] == 230);
    assert(parsed.dm_position_mrad[2] == -570);
    assert(parsed.dm_position_mrad[4] == -1490);
}

void TestVisionProtocol()
{
    const uint8_t frame[drivers::BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x2AU, 0x07U, 0xF4U, 0x01U,
        0x52U, 0x03U, 0x12U, 0x00U, 0xD4U, 0x3DU
    };
    drivers::BallVisionParser parser = {};
    drivers::BallVisionParser_Init(&parser);
    drivers::BallVisionFrame decoded = {};
    for (uint8_t i = 0U; i < sizeof(frame); i++) {
        const bool complete = drivers::BallVisionParser_FeedByte(
            &parser, frame[i], 100U, &decoded);
        assert(complete == (i == (sizeof(frame) - 1U)));
    }
    assert(decoded.sequence == 0x2AU);
    assert(decoded.position_0p1mm == 500);
    assert(decoded.confidence == 850U);
    assert(decoded.source_delay_ms == 18U);

    /* Reset at every possible byte offset of an active 12-byte stream.  The
     * parser must discard the partial tail and lock to the next full frame. */
    for (uint8_t offset = 1U; offset < sizeof(frame); offset++) {
        drivers::BallVisionParser offset_parser = {};
        drivers::BallVisionParser_Init(&offset_parser);
        drivers::BallVisionFrame offset_decoded = {};
        bool complete = false;
        for (uint8_t i = offset; i < sizeof(frame); i++) {
            complete |= drivers::BallVisionParser_FeedByte(
                &offset_parser, frame[i], 90U, &offset_decoded);
        }
        for (uint8_t i = 0U; i < sizeof(frame); i++) {
            complete |= drivers::BallVisionParser_FeedByte(
                &offset_parser, frame[i], 100U, &offset_decoded);
        }
        assert(complete);
        assert(offset_decoded.sequence == 0x2AU);
        assert(offset_decoded.position_0p1mm == 500);
    }

    uint8_t corrupted[sizeof(frame)] = {};
    memcpy(corrupted, frame, sizeof(frame));
    corrupted[5] ^= 0x01U;
    for (uint8_t i = 0U; i < sizeof(corrupted); i++) {
        assert(!drivers::BallVisionParser_FeedByte(
            &parser, corrupted[i], 120U, &decoded));
    }
    assert(parser.stats.crc_errors == 1U);
}

void TestVisionStateKeepsLastBall()
{
    gugah::VisionState state = {};
    gugah::VisionState_Init(&state);
    drivers::BallVisionFrame ball = {};
    ball.sequence = 1U;
    ball.flags = drivers::BALL_VISION_FLAG_BALL_FOUND |
                 drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
                 drivers::BALL_VISION_FLAG_CAMERA_OK;
    ball.position_0p1mm = 250;
    ball.confidence = 900U;
    ball.source_delay_ms = 5U;
    ball.received_ms = 100U;
    gugah::VisionState_Accept(&state, &ball, 500U);

    drivers::BallVisionFrame missed = ball;
    missed.sequence = 2U;
    missed.flags = drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
                   drivers::BALL_VISION_FLAG_CAMERA_OK;
    missed.received_ms = 150U;
    gugah::VisionState_Accept(&state, &missed, 500U);
    gugah::VisionFeedback feedback =
        gugah::VisionState_Get(&state, 170U, 500U);
    assert(feedback.communication_online);
    assert(feedback.ball_usable);
    assert(feedback.frame.sequence == 1U);
    assert(feedback.ball_age_ms == 75U);

    feedback = gugah::VisionState_Get(&state, 221U, 500U);
    assert(feedback.communication_online);
    assert(!feedback.ball_usable);
    assert(feedback.ball_age_ms == 126U);
}

void TestLineControl()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::LineControlState state = {};
    gugah::LineControl_Init(&state);
    drivers::GrayscaleProcessedData line = ValidLine(1000);
    assert(gugah::LineControl_Update(
        &state, &line, 100, 10U, &config));
    assert(state.left_rpm < state.right_rpm);
    line.position_valid = false;
    assert(gugah::LineControl_Update(
        &state, &line, 100, 50U, &config));
    assert(!gugah::LineControl_Update(
        &state, &line, 100, 201U, &config));
}

void TestCourseH2RequiresMarker()
{
    gugah::HConfig config = DefaultConfig();
    config.finish_gate_mm = 100U;
    config.approach_start_mm = 80U;
    config.lap_distance_mm = 5000U;
    config.sensor_to_reference_mm = 0;
    gugah::CourseState state = {};
    gugah::Course_Init(&state);
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 50);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H2, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::CourseInput input = { 100U, &line, &chassis, true };
    chassis = Chassis(100U, 10000, 10000, 50);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);

    line.track_state = drivers::GRAYSCALE_TRACK_WIDE;
    line.active_mask = 0x7EU;
    input.now_ms = 102U;
    chassis.received_ms = 102U;
    (void)gugah::Course_Update(&state, &input, &config);
    input.now_ms = 104U;
    chassis.received_ms = 104U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_POSITION);
    assert(state.phase == gugah::COURSE_FINAL_POSITION);

    chassis.left_rpm = 0;
    chassis.right_rpm = 0;
    chassis.left_encoder_count = state.final_left_count;
    chassis.right_encoder_count = state.final_right_count;
    input.now_ms = 110U;
    chassis.received_ms = 110U;
    (void)gugah::Course_Update(&state, &input, &config);
    input.now_ms = 320U;
    chassis.received_ms = 320U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.phase == gugah::COURSE_COMPLETE);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
}

void TestCourseRunsOncePerGrayscaleFrame()
{
    gugah::HConfig config = DefaultConfig();
    config.line_lost_grace_ms = 100U;
    config.lap_distance_mm = 5000U;
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 50);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H2, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(1000);
    gugah::CourseInput input = { 8U, &line, &chassis, true };
    chassis.received_ms = 8U;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(state.line_control.last_update_ms == 8U);

    line.line_position = -1000;
    input.now_ms = 10U;
    input.line_frame_new = false;
    chassis.received_ms = 10U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_NONE);
    assert(state.line_control.last_update_ms == 8U);

    input.now_ms = 16U;
    input.line_frame_new = true;
    chassis.received_ms = 16U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(state.line_control.last_update_ms == 16U);

    input.now_ms = 117U;
    input.line_frame_new = false;
    chassis.received_ms = 117U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.phase == gugah::COURSE_FAILED);
    assert(state.failure == gugah::COURSE_FAILURE_LINE_LOST);
}

void TestBallControlAndFeedforward()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_accel_ff_mdeg_per_mm_s2 = 2;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 500);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    assert(state.dm_target_mrad == -570);
    assert(state.beam_target_mdeg == 0);
    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    input.chassis_accel_mm_s2 = 0;
    const gugah::BallOutput output =
        gugah::Ball_Update(&state, &input, &config);
    assert(output.command_valid);
    assert(state.target_velocity_0p1mm_s == 0);
    assert(state.velocity_error_0p1mm_s == 0);
    assert(state.desired_acceleration_0p1mm_s2 < 0);
    assert(state.rail_compensation_mdeg == 0);
    assert(state.pid_p_mdeg == -2000);
    assert(state.pid_i_mdeg == 0);
    assert(state.beam_target_mdeg < state.rail_compensation_mdeg);
    assert(gugah::Ball_MapBeamToDm(&config, -2000) == -360);
    assert(gugah::Ball_HoldAngleMdeg(&config, -750) == 0);
    assert(gugah::Ball_HoldAngleMdeg(&config, 250) == 0);

    const int32_t integral_before = state.integral_error_0p1mm_ms;
    input.now_ms = 100U;
    input.vision.ball_age_ms = 80U;
    input.imu.received_ms = 100U;
    input.dm.received_ms = 100U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.integral_error_0p1mm_ms == integral_before);

    input.now_ms = 141U;
    input.vision.ball_age_ms = 121U;
    input.imu.received_ms = 141U;
    input.dm.received_ms = 141U;
    const gugah::BallOutput failed =
        gugah::Ball_Update(&state, &input, &config);
    assert(failed.stop_chassis);
    assert(state.result == gugah::BALL_RESULT_VISION_LOST);
}

void TestBallCameraPositionCalibration()
{
    static const int16_t camera_points[11] = {
        -1000, -853, -664, -500, -300, -89,
        130, 367, 581, 808, 1000
    };
    for (uint8_t i = 0U; i < 11U; i++) {
        const int16_t expected = static_cast<int16_t>(
            -1000 + static_cast<int16_t>(i) * 200);
        assert(gugah::Ball_CalibrateCameraPosition0p1mm(
                   camera_points[i]) == expected);
    }
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(20) == 99);
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(-1100) < -1100);
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(1100) > 1100);
}

void TestBallRetargetIsBumpless()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 300);
    assert(gugah::Ball_StartMove(
        &state, 0, 2400U, &input, &config));
    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    (void)gugah::Ball_Update(&state, &input, &config);
    const int32_t beam_before = state.beam_target_mdeg;
    const int32_t estimate_before = state.estimated_position_0p1mm;
    assert(beam_before != 0);

    state.mode = gugah::BALL_HOLD;
    state.result = gugah::BALL_RESULT_SUCCESS;
    assert(gugah::Ball_StartMove(
        &state, 500, 2400U, &input, &config));
    assert(state.beam_target_mdeg == beam_before);
    assert(state.estimated_position_0p1mm == estimate_before);
    assert(state.has_sample);
    assert(state.target_position_0p1mm == 500);
    assert(state.result == gugah::BALL_RESULT_RUNNING);
}

void TestBallObserverUsesPositionResidual()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 30U;
    input.vision.frame.received_ms = 30U;
    input.vision.frame.sequence++;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(100);
    input.imu.received_ms = 30U;
    input.dm.received_ms = 30U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.measured_velocity_0p1mm_s == 5000);
    assert(state.estimated_velocity_0p1mm_s > 0);
    assert(state.estimated_velocity_0p1mm_s <
           state.measured_velocity_0p1mm_s);
    const int32_t position_at_frame = state.estimated_position_0p1mm;

    input.now_ms = 35U;
    input.imu.received_ms = 35U;
    input.dm.received_ms = 35U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.estimated_position_0p1mm > position_at_frame);
    assert(state.estimated_position_0p1mm < 130);

    input.now_ms = 50U;
    input.vision.frame.received_ms = 50U;
    input.vision.frame.sequence++;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(200);
    input.imu.received_ms = 50U;
    input.dm.received_ms = 50U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.measured_position_0p1mm == 200);
    assert(state.estimated_velocity_0p1mm_s > 0);
    assert(state.estimated_position_0p1mm > position_at_frame);
}

void TestBallEstimatorInterpolatesLowSpeedMotion()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    /* Five 50 Hz samples moving by 0.2 mm per frame represent 10 mm/s. */
    for (uint8_t i = 1U; i < 5U; i++) {
        input.now_ms = 10U + static_cast<uint32_t>(i) * 20U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.vision.frame.position_0p1mm = CameraPositionForPhysical(
            static_cast<int16_t>(i * 2U));
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::Ball_Update(&state, &input, &config);
    }
    assert(state.measured_velocity_0p1mm_s == 100);
    assert(state.estimated_velocity_0p1mm_s > 0);
    assert(state.estimated_velocity_0p1mm_s <
           state.measured_velocity_0p1mm_s);

    const int32_t position_q8_before = state.observer_position_q8;
    input.now_ms += 2U;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.observer_position_q8 > position_q8_before);
}

void TestBallOverspeedUsesFastBrake()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    assert(state.beam_target_mdeg == 0);

    input.now_ms = 30U;
    input.vision.frame.received_ms = 30U;
    input.vision.frame.sequence++;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(100);
    input.imu.received_ms = 30U;
    input.dm.received_ms = 30U;
    (void)gugah::Ball_Update(&state, &input, &config);

    assert(state.estimated_velocity_0p1mm_s >
           config.ball_model_max_velocity_0p1mm_s);
    assert(state.velocity_error_0p1mm_s < 0);
    assert(state.integral_error_0p1mm_ms == 0);
    assert(state.pid_i_mdeg == 0);
    assert(state.pid_d_mdeg < 0);
    assert(state.beam_target_mdeg < -500);
}

void TestBallOuterPdHasZeroVelocityTarget()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 1000);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    input.now_ms = 15U;
    input.vision.frame.received_ms = 15U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 15U;
    input.dm.received_ms = 15U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.target_velocity_0p1mm_s == 0);
    assert(state.velocity_error_0p1mm_s == 0);
    assert(state.pid_p_mdeg == -4000);
    assert(state.pid_correction_mdeg == -4000);
}

void TestBallPredictedVelocityDoesNotCancelBreakaway()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 500);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.stiction_compensation_mdeg == 0);

    /* A breakaway pulse is only armed after 400 ms of measured standstill. */
    input.now_ms = 430U;
    input.vision.frame.received_ms = 430U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 430U;
    input.dm.received_ms = 430U;
    (void)gugah::Ball_Update(&state, &input, &config);
    const int32_t compensation_before =
        state.stiction_compensation_mdeg;
    assert(compensation_before < 0);

    /* A tilted beam creates a large model-predicted velocity without a new
     * measured position.  Breakaway must continue until vision confirms
     * that the ball is genuinely rolling. */
    input.now_ms = 500U;
    input.dm.position_mrad = -1000;
    input.imu.received_ms = 500U;
    input.dm.received_ms = 500U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.measured_velocity_0p1mm_s == 0);
    assert(state.observer_velocity_0p1mm_s != 0);
    assert(state.estimated_velocity_0p1mm_s != 0);
    assert(state.stiction_compensation_mdeg < compensation_before);
}

void TestBallPidCorrectsInsideAcceptedBand()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 80);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.position_error_0p1mm == -80);
    assert(state.stiction_compensation_mdeg == 0);
    assert(state.pid_p_mdeg == -320);
    assert(state.pid_correction_mdeg < 0);

    /* A stationary 8 mm error is acceptable for scoring.  Small PD remains
     * active, but the large breakaway pulse is prohibited inside the band. */
    input.now_ms = 430U;
    input.vision.frame.received_ms = 430U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 430U;
    input.dm.received_ms = 430U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.stiction_compensation_mdeg == 0);
}

void TestBallIntegralRemovesStaticErrorWithoutWindup()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_observer_alpha_permille = 1000U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 90);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    /* A stationary 9 mm residual should learn a persistent correction. */
    for (uint16_t i = 1U; i <= 200U; i++) {
        input.now_ms = 10U + static_cast<uint32_t>(i) * 10U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::Ball_Update(&state, &input, &config);
    }
    assert(state.pid_i_mdeg < -250);
    assert(state.pid_i_mdeg > -400);
    const int32_t learned_integral =
        state.integral_error_0p1mm_ms;

    /* Camera noise inside 1 mm must not keep winding the trim. */
    input.vision.frame.position_0p1mm = CameraPositionForPhysical(5);
    for (uint8_t i = 0U; i < 20U; i++) {
        input.now_ms += 10U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::Ball_Update(&state, &input, &config);
    }
    assert(state.integral_error_0p1mm_ms == learned_integral);

    /* Opposite low-speed error unloads the learned bias. */
    input.vision.frame.position_0p1mm = CameraPositionForPhysical(-90);
    for (uint16_t i = 0U; i < 200U; i++) {
        input.now_ms += 10U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::Ball_Update(&state, &input, &config);
    }
    assert(state.pid_i_mdeg > -50);
}

void TestBallDampingWorksAcrossFullTravel()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_observer_alpha_permille = 1000U;
    config.ball_observer_beta_permille = 1000U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 500, &input, &config));

    /* Velocity damping is active over the full travel, not only near the
     * target.  A fast ball must request braking immediately. */
    input.now_ms = 30U;
    input.vision.frame.received_ms = 30U;
    input.vision.frame.sequence++;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(120);
    input.imu.received_ms = 30U;
    input.dm.received_ms = 30U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.position_error_0p1mm >
           config.ball_position_tolerance_0p1mm);
    assert(state.target_velocity_0p1mm_s == 0);
    assert(state.estimated_velocity_0p1mm_s > 0);
    assert(state.desired_acceleration_0p1mm_s2 < 0);
    assert(state.rail_compensation_mdeg == 0);
    assert(state.pid_p_mdeg > 0);
    assert(state.pid_d_mdeg < 0);
    assert(state.pid_correction_mdeg < 0);
    assert(state.beam_target_mdeg < 0);
}

void TestBallDampingWorksNearTarget()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_observer_alpha_permille = 1000U;
    config.ball_observer_beta_permille = 1000U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -570;
    assert(gugah::Ball_StartHold(&state, 500, &input, &config));

    /* The same observer-velocity damping remains active near the target. */
    input.now_ms = 30U;
    input.vision.frame.received_ms = 30U;
    input.vision.frame.sequence++;
    input.vision.frame.position_0p1mm =
        CameraPositionForPhysical(350);
    input.imu.received_ms = 30U;
    input.dm.received_ms = 30U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.position_error_0p1mm == 150);
    assert(state.estimated_velocity_0p1mm_s > 0);
    assert(state.desired_acceleration_0p1mm_s2 < 0);
    assert(state.pid_d_mdeg < 0);
}

void TestH3CentersFromArbitraryInitialPosition()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;

    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 685);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    input.chassis_fault = true;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.h3_stage == 0U);
    assert(state.ball.target_position_0p1mm == 0);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 110U;
    input.vision.frame.received_ms = 110U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 110U;
    input.dm.received_ms = 110U;
    const gugah::HAppOutput center_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(center_output.motion.mode == gugah::MOTION_COMMAND_NONE);
    assert(state.h3_stage == 1U);
    assert(state.ball.target_position_0p1mm == 500);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 120U;
    input.vision.frame.received_ms = 120U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 120U;
    input.dm.received_ms = 120U;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h3_stage == 2U);
    assert(state.ball.target_position_0p1mm == -500);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 130U;
    input.vision.frame.received_ms = 130U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 130U;
    input.dm.received_ms = 130U;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_PASS);
}

void TestH3ReportsDmTimeoutSeparately()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 500);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    assert(gugah::HApp_Start(&state, &input, &config));

    state.ball.mode = gugah::BALL_FAILED;
    state.ball.result = gugah::BALL_RESULT_DM_STALE;
    input.now_ms = 110U;
    const gugah::HAppOutput output =
        gugah::HApp_Update(&state, &input, &config);
    assert(output.motion.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.run_state == gugah::H_STATE_FAIL);
    assert(state.failure == gugah::H_FAILURE_DM);
}

void TestH4TimeFreezesAtB()
{
    gugah::HConfig config = DefaultConfig();
    config.h4_b_distance_mm = 1400U;
    config.h4_stop_distance_mm = 1600U;
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 50);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H4, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::CourseInput input = { 7900U, &line, &chassis, true };
    chassis = Chassis(7900U, 10000, 10000, 50);
    input.chassis = &chassis;
    (void)gugah::Course_Update(&state, &input, &config);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == 7900U);

    input.now_ms = 8200U;
    chassis = Chassis(8200U, 12000, 12000, 0);
    input.chassis = &chassis;
    const gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.phase == gugah::COURSE_STOPPING);
    input.now_ms = 8210U;
    chassis.received_ms = 8210U;
    (void)gugah::Course_Update(&state, &input, &config);
    assert(state.phase == gugah::COURSE_COMPLETE);
}

void TestH6Buttons()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    gugah::HAppInput input = {};
    input.now_ms = 10U;
    drivers::GrayscaleProcessedData line = ValidLine(0);
    input.line = &line;
    for (uint8_t i = 0U; i < 4U; i++) {
        input.buttons.b1_short = true;
        (void)gugah::HApp_Update(&state, &input, &config);
        input.buttons.b1_short = false;
    }
    assert(state.selected_problem == gugah::H_PROBLEM_6);
    input.buttons.b3_increment = true;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h6_target_0p1mm == 10);
    for (uint16_t i = 0U; i < 200U; i++) {
        (void)gugah::HApp_Update(&state, &input, &config);
    }
    assert(state.h6_target_0p1mm == 1000);
}

} /* namespace */

int main()
{
    TestConfigRecord();
    TestVisionProtocol();
    TestVisionStateKeepsLastBall();
    TestLineControl();
    TestCourseH2RequiresMarker();
    TestCourseRunsOncePerGrayscaleFrame();
    TestBallCameraPositionCalibration();
    TestBallControlAndFeedforward();
    TestBallRetargetIsBumpless();
    TestBallObserverUsesPositionResidual();
    TestBallEstimatorInterpolatesLowSpeedMotion();
    TestBallOverspeedUsesFastBrake();
    TestBallOuterPdHasZeroVelocityTarget();
    TestBallPredictedVelocityDoesNotCancelBreakaway();
    TestBallPidCorrectsInsideAcceptedBand();
    TestBallIntegralRemovesStaticErrorWithoutWindup();
    TestBallDampingWorksAcrossFullTravel();
    TestBallDampingWorksNearTarget();
    TestH3CentersFromArbitraryInitialPosition();
    TestH3ReportsDmTimeoutSeparately();
    TestH4TimeFreezesAtB();
    TestH6Buttons();
    puts("gugaH core tests passed");
    return 0;
}
