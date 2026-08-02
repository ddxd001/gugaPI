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

int32_t CountsForMillimeters(int32_t millimeters,
                             const gugah::HConfig &config)
{
    const int64_t numerator = static_cast<int64_t>(millimeters) *
        config.left_counts_per_rev * 1000000000LL;
    const int64_t denominator = 2LL * 3141593LL *
        config.wheel_radius_um;
    return static_cast<int32_t>(
        (numerator + denominator / 2LL) / denominator);
}

gugah::ImuFeedback Imu(uint32_t now, int32_t yaw_mdeg = 0)
{
    gugah::ImuFeedback feedback = {};
    feedback.valid = true;
    feedback.yaw_mdeg = yaw_mdeg;
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
    assert(config.ball_imu_beam_kp_permille == 0);
    assert(config.ball_imu_beam_limit_0p1deg == 0U);
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
    assert(config.ball_pid_kp_mdeg_per_mm == 55);
    assert(config.ball_pid_ki_mdeg_per_mm_s == 0);
    assert(config.ball_pid_kd_mdeg_per_mm_s == 20);
    assert(config.ball_pid_integral_limit_mdeg == 1500);
    assert(config.ball_zero_offset_0p1mm == 0);
    assert(config.ball_hold_position_0p1mm[0] == -1000);
    assert(config.ball_hold_position_0p1mm[4] == 1000);
    assert(config.ball_hold_angle_mdeg[0] == -918);
    assert(config.ball_hold_angle_mdeg[1] == -165);
    assert(config.ball_hold_angle_mdeg[2] == 560);
    assert(config.ball_hold_angle_mdeg[3] == 986);
    assert(config.ball_hold_angle_mdeg[4] == 682);
    assert(config.ball_accel_ff_mdeg_per_mm_s2 == 0);
    assert(config.beam_angle_mdeg[0] == -8836);
    assert(config.beam_angle_mdeg[1] == -3942);
    assert(config.beam_angle_mdeg[2] == 0);
    assert(config.beam_angle_mdeg[3] == 3093);
    assert(config.beam_angle_mdeg[4] == 6094);
    assert(config.dm_position_mrad[0] == 300);
    assert(config.dm_position_mrad[1] == -200);
    assert(config.dm_position_mrad[2] == -617);
    assert(config.dm_position_mrad[3] == -1000);
    assert(config.dm_position_mrad[4] == -1382);
    assert(config.h4_launch_ramp_rpm_s == 40U);
    assert(config.h4_stop_ramp_rpm_s == 40U);
    assert(config.h4_brake_distance_mm == 1000U);
    assert(config.h4_stop_distance_mm == 1700U);
    assert(config.h4_heading_kp == 1000);
    assert(config.h4_heading_max_correction_rpm == 30);
    assert(config.imu_gyro_bias_z_mdps == 0);
    assert(config.ball_chassis_ff_permille == 1000);
    assert(config.h5_launch_ramp_rpm_s == 40U);
    assert(config.h5_stop_ramp_rpm_s == 40U);
    assert(config.h5_brake_distance_mm == 5840U);
    assert(config.h5_cruise_rpm == 95U);
    assert(config.h5_approach_rpm == 63U);
    assert(config.course_straight_line_kp_milli == 18U);
    assert(config.course_straight_line_kd_milli == 12U);
    assert(config.course_straight_line_max_correction_rpm == 25U);
    assert(config.course_straight_line_slew_rpm_s == 400U);
    assert(config.ball_zero_offset_0p1mm == 0);
    assert(config.h2_loop_offset_mm == -100);
    gugah::HConfig grayscale_config = config;
    uint16_t white_surface[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
    uint16_t black_surface[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        white_surface[i] = static_cast<uint16_t>(3000U + i);
        black_surface[i] = static_cast<uint16_t>(1000U + i);
    }
    assert(gugah::HConfig_CaptureGrayscaleSurface(
        &grayscale_config, white_surface, true));
    assert(gugah::HConfig_CaptureGrayscaleSurface(
        &grayscale_config, black_surface, false));
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        assert(grayscale_config.grayscale.white[i] == white_surface[i]);
        assert(grayscale_config.grayscale.black[i] == black_surface[i]);
    }
    const uint16_t saved_white0 =
        grayscale_config.grayscale.white[0];
    assert(!gugah::HConfig_CaptureGrayscaleSurface(
        &grayscale_config, black_surface, true));
    assert(grayscale_config.grayscale.white[0] == saved_white0);
    gugah::HConfig h2_limit_config = config;
    h2_limit_config.h2_loop_offset_mm =
        gugah::H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM;
    assert(gugah::HConfig_Validate(&h2_limit_config));
    h2_limit_config.h2_loop_offset_mm =
        gugah::H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM + 1;
    assert(!gugah::HConfig_Validate(&h2_limit_config));
    gugah::HConfigRecord record = {};
    gugah::HConfig_BuildRecord(&config, &record);
    gugah::HConfig parsed = {};
    assert(gugah::HConfig_ParseRecord(&record, &parsed));
    assert(parsed.lap_distance_mm == 6142U);

    gugah::HConfig zero_config = config;
    zero_config.ball_zero_offset_0p1mm = -70;
    zero_config.h2_loop_offset_mm = 90;
    gugah::HConfigRecord zero_record = {};
    gugah::HConfig_BuildRecord(&zero_config, &zero_record);
    assert(gugah::HConfig_ParseRecord(&zero_record, &parsed));
    assert(parsed.ball_zero_offset_0p1mm == -70);
    assert(parsed.h2_loop_offset_mm == 90);

    gugah::HConfig old_speed_config = config;
    old_speed_config.h5_cruise_rpm = 100U;
    old_speed_config.h5_approach_rpm = 55U;
    gugah::HConfigRecord old_speed_record = {};
    gugah::HConfig_BuildRecord(&old_speed_config, &old_speed_record);
    assert(gugah::HConfig_ParseRecord(&old_speed_record, &parsed));
    assert(parsed.h5_cruise_rpm == 95U);
    assert(parsed.h5_approach_rpm == 63U);

    gugah::HConfig old_level_config = config;
    const int16_t old_level_positions[5] = {
        230, -150, -570, -1070, -1490
    };
    for (uint8_t i = 0U; i < 5U; i++) {
        old_level_config.dm_position_mrad[i] = old_level_positions[i];
    }
    gugah::HConfigRecord old_level_record = {};
    gugah::HConfig_BuildRecord(&old_level_config, &old_level_record);
    assert(gugah::HConfig_ParseRecord(&old_level_record, &parsed));
    assert(parsed.dm_position_mrad[0] == 300);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1382);

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
    assert(parsed.ball_imu_beam_kp_permille == 0);
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
    assert(parsed.ball_zero_offset_0p1mm == 0);

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
    assert(parsed.ball_hold_angle_mdeg[2] == 560);

    /* Schema 6 used the old linear mechanism map.  Loading it preserves all
     * other parameters but installs the measured asymmetric linkage map. */
    gugah::HConfigRecord schema6 = {};
    const uint16_t schema6_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h4_launch_ramp_rpm_s));
    schema6.magic = gugah::H_CONFIG_MAGIC;
    schema6.schema_version = 6U;
    schema6.payload_length = schema6_length;
    schema6.payload = config;
    const int16_t schema6_old_dm[5] = { 500, 0, -500, -1000, -1500 };
    for (uint8_t i = 0U; i < 5U; i++) {
        schema6.payload.dm_position_mrad[i] = schema6_old_dm[i];
    }
    const uint32_t schema6_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema6.payload),
        schema6.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema6.payload) + schema6_length,
           &schema6_crc, sizeof(schema6_crc));
    assert(gugah::HConfig_ParseRecord(&schema6, &parsed));
    assert(parsed.dm_position_mrad[0] == 300);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1382);

    /* Schema 7 commissioned the observer with integral disabled.  Upgrade
     * that exact state to the bounded static-error trim introduced by 8. */
    gugah::HConfigRecord schema7 = {};
    schema7.magic = gugah::H_CONFIG_MAGIC;
    schema7.schema_version = 7U;
    schema7.payload_length = schema6_length;
    schema7.payload = config;
    schema7.payload.ball_pid_ki_mdeg_per_mm_s = 0;
    schema7.payload.ball_pid_integral_limit_mdeg = 0;
    const uint32_t schema7_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema7.payload),
        schema7.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema7.payload) + schema6_length,
           &schema7_crc, sizeof(schema7_crc));
    assert(gugah::HConfig_ParseRecord(&schema7, &parsed));
    assert(parsed.ball_pid_ki_mdeg_per_mm_s == 20);
    assert(parsed.ball_pid_integral_limit_mdeg == 1500);

    /* Schema 8 predates the H4-only launch ramp.  Preserve all existing
     * control tuning and append the conservative 40 RPM/s default. */
    gugah::HConfigRecord schema8 = {};
    schema8.magic = gugah::H_CONFIG_MAGIC;
    schema8.schema_version = 8U;
    schema8.payload_length = schema6_length;
    schema8.payload = config;
    const uint32_t schema8_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema8.payload),
        schema8.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema8.payload) + schema6_length,
           &schema8_crc, sizeof(schema8_crc));
    assert(gugah::HConfig_ParseRecord(&schema8, &parsed));
    assert(parsed.ball_pid_kp_mdeg_per_mm ==
           config.ball_pid_kp_mdeg_per_mm);
    assert(parsed.ball_pid_ki_mdeg_per_mm_s ==
           config.ball_pid_ki_mdeg_per_mm_s);
    assert(parsed.h4_launch_ramp_rpm_s == 40U);
    assert(parsed.h4_stop_ramp_rpm_s == 40U);

    /* Schema 20 used this byte slot as an inactive curvature origin.  It
     * must migrate to an uncalibrated zero rather than shifting O by 14.8
     * mm after the firmware update. */
    gugah::HConfigRecord schema20 = {};
    schema20.magic = gugah::H_CONFIG_MAGIC;
    schema20.schema_version = 20U;
    schema20.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema20.payload = config;
    schema20.payload.ball_zero_offset_0p1mm = 148;
    schema20.payload.h4_brake_distance_mm = 1100U;
    schema20.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema20.payload),
        schema20.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema20, &parsed));
    assert(parsed.ball_zero_offset_0p1mm == 0);
    assert(parsed.h4_brake_distance_mm == 1000U);

    /* Schema 21 already owns the zero field.  Its calibration survives the
     * H4 brake-point migration introduced by schema 22. */
    gugah::HConfigRecord schema21 = {};
    schema21.magic = gugah::H_CONFIG_MAGIC;
    schema21.schema_version = 21U;
    schema21.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema21.payload = config;
    schema21.payload.ball_zero_offset_0p1mm = -70;
    schema21.payload.h4_brake_distance_mm = 1100U;
    schema21.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema21.payload),
        schema21.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema21, &parsed));
    assert(parsed.ball_zero_offset_0p1mm == -70);
    assert(parsed.h4_brake_distance_mm == 1000U);

    /* Schema 22 still used this storage slot as an inactive sensor offset.
     * Schema 23 must preserve all other tuning while installing the
     * commissioned -100 mm H2 lap-stop offset. */
    gugah::HConfigRecord schema22 = {};
    schema22.magic = gugah::H_CONFIG_MAGIC;
    schema22.schema_version = 22U;
    schema22.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema22.payload = config;
    schema22.payload.h2_loop_offset_mm = 0;
    schema22.payload.ball_zero_offset_0p1mm = -70;
    schema22.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema22.payload),
        schema22.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema22, &parsed));
    assert(parsed.h2_loop_offset_mm == -100);
    assert(parsed.ball_zero_offset_0p1mm == -70);

    /* Schema 23 stored retired finish/approach gates in the four slots now
     * used by the H5/H6 straight controller.  They must migrate to gains and
     * limits, never retain the old 5200/5250 mm values. */
    gugah::HConfigRecord schema23 = {};
    schema23.magic = gugah::H_CONFIG_MAGIC;
    schema23.schema_version = 23U;
    schema23.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema23.payload = config;
    schema23.payload.course_straight_line_kp_milli = 5200U;
    schema23.payload.course_straight_line_kd_milli = 5250U;
    schema23.payload.course_straight_line_max_correction_rpm = 5200U;
    schema23.payload.course_straight_line_slew_rpm_s = 5250U;
    schema23.payload.h2_loop_offset_mm = 80;
    schema23.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema23.payload),
        schema23.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema23, &parsed));
    assert(parsed.course_straight_line_kp_milli == 18U);
    assert(parsed.course_straight_line_kd_milli == 12U);
    assert(parsed.course_straight_line_max_correction_rpm == 25U);
    assert(parsed.course_straight_line_slew_rpm_s == 400U);
    assert(parsed.h2_loop_offset_mm == 80);

    /* Schema 9 stores the launch ramp but predates the H4 soft-stop ramp. */
    gugah::HConfigRecord schema9 = {};
    const uint16_t schema9_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h4_stop_ramp_rpm_s));
    schema9.magic = gugah::H_CONFIG_MAGIC;
    schema9.schema_version = 9U;
    schema9.payload_length = schema9_length;
    schema9.payload = config;
    schema9.payload.h4_launch_ramp_rpm_s = 45U;
    schema9.payload.h4_stop_distance_mm = 1650U;
    const uint32_t schema9_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema9.payload),
        schema9.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema9.payload) + schema9_length,
           &schema9_crc, sizeof(schema9_crc));
    assert(gugah::HConfig_ParseRecord(&schema9, &parsed));
    assert(parsed.h4_launch_ramp_rpm_s == 45U);
    assert(parsed.h4_stop_ramp_rpm_s == 60U);
    assert(parsed.h4_brake_distance_mm == 1000U);
    assert(parsed.h4_stop_distance_mm == 1700U);

    /* Schema 12 has the pre-B brake point but predates H4 yaw hold. */
    gugah::HConfigRecord schema12 = {};
    schema12.magic = gugah::H_CONFIG_MAGIC;
    schema12.schema_version = 12U;
    schema12.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h4_heading_kp));
    schema12.payload = config;
    const uint32_t schema12_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema12.payload),
        schema12.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema12.payload) +
               schema12.payload_length,
           &schema12_crc, sizeof(schema12_crc));
    assert(gugah::HConfig_ParseRecord(&schema12, &parsed));
    assert(parsed.h4_brake_distance_mm == 1000U);
    assert(parsed.h4_heading_kp == 1000);
    assert(parsed.h4_heading_max_correction_rpm == 30);
    assert(parsed.imu_gyro_bias_z_mdps == 0);

    /* Schema 13 has H4 yaw hold but predates the adjustable chassis
     * acceleration feedforward scale. */
    gugah::HConfigRecord schema13 = {};
    schema13.magic = gugah::H_CONFIG_MAGIC;
    schema13.schema_version = 13U;
    schema13.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_chassis_ff_permille));
    schema13.payload = config;
    const uint32_t schema13_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema13.payload),
        schema13.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema13.payload) +
               schema13.payload_length,
           &schema13_crc, sizeof(schema13_crc));
    assert(gugah::HConfig_ParseRecord(&schema13, &parsed));
    assert(parsed.h4_heading_kp == 1000);
    assert(parsed.ball_chassis_ff_permille == 1000);

    /* Schema 14 has chassis feedforward but predates the H5 ramps. */
    gugah::HConfigRecord schema14 = {};
    schema14.magic = gugah::H_CONFIG_MAGIC;
    schema14.schema_version = 14U;
    schema14.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h5_launch_ramp_rpm_s));
    schema14.payload = config;
    const uint32_t schema14_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema14.payload),
        schema14.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema14.payload) +
               schema14.payload_length,
           &schema14_crc, sizeof(schema14_crc));
    assert(gugah::HConfig_ParseRecord(&schema14, &parsed));
    assert(parsed.ball_chassis_ff_permille == 1000);
    assert(parsed.h5_launch_ramp_rpm_s == 40U);
    assert(parsed.h5_stop_ramp_rpm_s == 40U);

    /* Schema 15 ramps H5 only after A; schema 16 adds pre-A braking. */
    gugah::HConfigRecord schema15 = {};
    schema15.magic = gugah::H_CONFIG_MAGIC;
    schema15.schema_version = 15U;
    schema15.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h5_brake_distance_mm));
    schema15.payload = config;
    const uint32_t schema15_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema15.payload),
        schema15.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema15.payload) +
               schema15.payload_length,
           &schema15_crc, sizeof(schema15_crc));
    assert(gugah::HConfig_ParseRecord(&schema15, &parsed));
    assert(parsed.h5_launch_ramp_rpm_s == 40U);
    assert(parsed.h5_stop_ramp_rpm_s == 40U);
    assert(parsed.h5_brake_distance_mm == 5840U);

    /* Schema 16's former pitch bytes migrate to disabled compatibility
     * fields; the beam IMU remains diagnostic-only. */
    gugah::HConfigRecord schema16 = {};
    schema16.magic = gugah::H_CONFIG_MAGIC;
    schema16.schema_version = 16U;
    schema16.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema16.payload = config;
    schema16.payload.ball_imu_beam_kp_permille = 0;
    schema16.payload.ball_imu_beam_limit_0p1deg = 0U;
    const uint32_t schema16_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema16.payload),
        schema16.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema16.payload) +
               schema16.payload_length,
           &schema16_crc, sizeof(schema16_crc));
    assert(gugah::HConfig_ParseRecord(&schema16, &parsed));
    assert(parsed.h5_brake_distance_mm == config.h5_brake_distance_mm);
    assert(parsed.ball_imu_beam_kp_permille == 0);
    assert(parsed.ball_imu_beam_limit_0p1deg == 0U);

    /* Schema 17/18 maps migrate to the five IMU-measured monotonic knots. */
    gugah::HConfigRecord schema17 = {};
    schema17.magic = gugah::H_CONFIG_MAGIC;
    schema17.schema_version = 17U;
    schema17.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema17.payload = config;
    const int16_t schema17_positions[5] = {
        260, -120, -550, -1020, -1450
    };
    for (uint8_t i = 0U; i < 5U; i++) {
        schema17.payload.dm_position_mrad[i] = schema17_positions[i];
    }
    schema17.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema17.payload),
        schema17.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema17, &parsed));
    assert(parsed.beam_angle_mdeg[0] == -8836);
    assert(parsed.dm_position_mrad[0] == 300);
    assert(parsed.dm_position_mrad[1] == -200);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[3] == -1000);
    assert(parsed.dm_position_mrad[4] == -1382);

    gugah::HConfigRecord schema18 = schema17;
    schema18.schema_version = 18U;
    schema18.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema18.payload),
        schema18.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema18, &parsed));
    assert(parsed.beam_angle_mdeg[4] == 6094);
    assert(parsed.dm_position_mrad[0] == 300);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1382);

    /* Schema 20 replaces only the exact former H4 defaults. */
    gugah::HConfigRecord schema19 = {};
    schema19.magic = gugah::H_CONFIG_MAGIC;
    schema19.schema_version = 19U;
    schema19.payload_length = static_cast<uint16_t>(
        sizeof(gugah::HConfig));
    schema19.payload = config;
    schema19.payload.h4_cruise_rpm = 120U;
    schema19.payload.h4_launch_ramp_rpm_s = 60U;
    schema19.payload.h4_stop_ramp_rpm_s = 60U;
    schema19.payload_crc32 = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema19.payload),
        schema19.payload_length);
    assert(gugah::HConfig_ParseRecord(&schema19, &parsed));
    assert(parsed.h4_cruise_rpm == 100U);
    assert(parsed.h4_launch_ramp_rpm_s == 40U);
    assert(parsed.h4_stop_ramp_rpm_s == 40U);

    /* Schema 10's exact first soft-stop defaults migrate to the gentler
     * symmetric ramp and its longer hard-stop boundary. */
    gugah::HConfigRecord schema10 = {};
    schema10.magic = gugah::H_CONFIG_MAGIC;
    schema10.schema_version = 10U;
    schema10.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h4_brake_distance_mm));
    schema10.payload = config;
    schema10.payload.h4_stop_ramp_rpm_s = 180U;
    schema10.payload.h4_stop_distance_mm = 1650U;
    const uint32_t schema10_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema10.payload),
        schema10.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema10.payload) +
               schema10.payload_length,
           &schema10_crc, sizeof(schema10_crc));
    assert(gugah::HConfig_ParseRecord(&schema10, &parsed));
    assert(parsed.h4_stop_ramp_rpm_s == 60U);
    assert(parsed.h4_brake_distance_mm == 1000U);
    assert(parsed.h4_stop_distance_mm == 1700U);

    /* Schema 11's gentle ramp still began at B and used a far-away hard
     * boundary.  Schema 12 adds the pre-B braking point. */
    gugah::HConfigRecord schema11 = {};
    schema11.magic = gugah::H_CONFIG_MAGIC;
    schema11.schema_version = 11U;
    schema11.payload_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, h4_brake_distance_mm));
    schema11.payload = config;
    schema11.payload.h4_stop_ramp_rpm_s = 60U;
    schema11.payload.h4_stop_distance_mm = 2050U;
    const uint32_t schema11_crc = gugah::HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&schema11.payload),
        schema11.payload_length);
    memcpy(reinterpret_cast<uint8_t *>(&schema11.payload) +
               schema11.payload_length,
           &schema11_crc, sizeof(schema11_crc));
    assert(gugah::HConfig_ParseRecord(&schema11, &parsed));
    assert(parsed.h4_stop_ramp_rpm_s == 60U);
    assert(parsed.h4_brake_distance_mm == 1000U);
    assert(parsed.h4_stop_distance_mm == 1700U);

    /* Schema-4 records contain the residual PID but predate CAL_ZERO. */
    gugah::HConfigRecord schema4 = {};
    const uint16_t schema4_length = static_cast<uint16_t>(
        offsetof(gugah::HConfig, ball_zero_offset_0p1mm));
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
    assert(parsed.ball_pid_kp_mdeg_per_mm == 40);
    assert(parsed.ball_zero_offset_0p1mm == 0);

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
    assert(parsed.dm_position_mrad[0] == 300);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1382);
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
    assert(feedback.ball_usable);
    assert(feedback.ball_age_ms == 126U);

    feedback = gugah::VisionState_Get(&state, 500U, 500U);
    assert(!feedback.communication_online);
    assert(feedback.ball_usable);
    assert(feedback.ball_age_ms == 405U);
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

void TestLineControlDerivativeFilter()
{
    const gugah::HConfig config = DefaultConfig();
    const gugah::LineControlTuning gentle = {
        0, 50, 500, 65000U, 200U
    };
    const gugah::LineControlTuning raw = {
        0, 50, 500, 65000U, 1000U
    };
    drivers::GrayscaleProcessedData line = ValidLine(0);

    gugah::LineControlState gentle_state = {};
    gugah::LineControl_Init(&gentle_state);
    assert(gugah::LineControl_UpdateTuned(
        &gentle_state, &line, 100, 10U, &config, &gentle));
    line.line_position = 100;
    assert(gugah::LineControl_UpdateTuned(
        &gentle_state, &line, 100, 18U, &config, &gentle));
    assert(gentle_state.raw_derivative_per_s == 12500);
    assert(gentle_state.filtered_derivative_per_s == 2500);
    assert(gentle_state.last_correction_rpm == 125);

    line.line_position = 0;
    gugah::LineControlState raw_state = {};
    gugah::LineControl_Init(&raw_state);
    assert(gugah::LineControl_UpdateTuned(
        &raw_state, &line, 100, 10U, &config, &raw));
    line.line_position = 100;
    assert(gugah::LineControl_UpdateTuned(
        &raw_state, &line, 100, 18U, &config, &raw));
    assert(raw_state.filtered_derivative_per_s == 12500);
    assert(raw_state.last_correction_rpm == 500);
    assert(gentle_state.last_correction_rpm <
           raw_state.last_correction_rpm);
}

void TestCourseH2UsesCalibratedAverageOdometryWithoutTimeout()
{
    gugah::HConfig config = DefaultConfig();
    config.lap_distance_mm = 5000U;
    /* A non-default value proves the H2 target follows the calibrated
     * offset rather than the former compiled -100 mm constant. */
    config.h2_loop_offset_mm = -80;
    gugah::CourseState state = {};
    gugah::Course_Init(&state);
    static const int32_t kLeftStart = 100;
    static const int32_t kRightStart = 200;
    gugah::ChassisFeedback chassis =
        Chassis(0U, kLeftStart, kRightStart, 50);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H2, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    line.track_state = drivers::GRAYSCALE_TRACK_WIDE;
    line.active_mask = 0x7EU;
    gugah::CourseInput input = { 100U, &line, &chassis, true, 0 };
    const int32_t before_lap = CountsForMillimeters(4690, config);
    chassis = Chassis(100U,
                      kLeftStart + before_lap,
                      kRightStart + before_lap,
                      50);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(state.finish_confirm_frames == 0U);

    /* Even two valid A-marker frames cannot stop H2 before the configured
     * encoder lap distance. */
    input.now_ms = 102U;
    chassis.received_ms = 102U;
    (void)gugah::Course_Update(&state, &input, &config);
    input.now_ms = 104U;
    chassis.received_ms = 104U;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm ==
           static_cast<int16_t>(config.cruise_rpm));
    assert(command.right_rpm ==
           static_cast<int16_t>(config.cruise_rpm));
    assert(state.phase == gugah::COURSE_CRUISE);

    const int32_t final_approach_start =
        CountsForMillimeters(4720, config);
    const int32_t one_lap = CountsForMillimeters(4920, config);
    /* Enter the final position phase after the former 20 s H2 deadline. */
    input.now_ms = 20002U;
    chassis = Chassis(input.now_ms,
                      kLeftStart + final_approach_start,
                      kRightStart + final_approach_start,
                      50);
    input.chassis = &chassis;
    input.line_frame_new = true;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm ==
           static_cast<int16_t>(config.approach_rpm));
    assert(command.right_rpm ==
           static_cast<int16_t>(config.approach_rpm));
    assert(state.phase == gugah::COURSE_APPROACH);

    /* Near the target the PD correction is limited to half of the reduced
     * base speed, so neither wheel reverses and stalls average odometry. */
    input.now_ms = 20100U;
    const int32_t near_target = CountsForMillimeters(4910, config);
    chassis = Chassis(input.now_ms,
                      kLeftStart + near_target,
                      kRightStart + near_target,
                      20);
    input.chassis = &chassis;
    line = ValidLine(1000);
    input.line = &line;
    input.line_frame_new = true;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm > 0);
    assert(command.right_rpm > 0);
    assert((command.left_rpm + command.right_rpm) / 2 == 17);

    input.now_ms = 20220U;
    chassis = Chassis(input.now_ms,
                      kLeftStart + one_lap,
                      kRightStart + one_lap,
                      15);
    input.chassis = &chassis;
    input.line_frame_new = false;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.phase == gugah::COURSE_COMPLETE);
    assert(state.pass_ms == input.now_ms);
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
    gugah::CourseInput input = { 8U, &line, &chassis, true, 0 };
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

void TestH2SearchesRightAfterLineLossAndReacquires()
{
    gugah::HConfig config = DefaultConfig();
    config.line_lost_grace_ms = 20U;
    config.lap_distance_mm = 5000U;
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 50);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H2, &chassis, 0U, &config));

    drivers::GrayscaleProcessedData line = ValidLine(0);
    line.line_detected = false;
    line.position_valid = false;
    line.track_state = drivers::GRAYSCALE_TRACK_LOST;
    gugah::CourseInput input = { 8U, &line, &chassis, true, 0 };
    chassis.received_ms = input.now_ms;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm > command.right_rpm);
    assert(command.right_rpm > 0);
    assert((command.left_rpm + command.right_rpm) / 2 ==
           config.cruise_rpm);
    assert(state.phase == gugah::COURSE_CRUISE);

    /* Repeated valid ADC frames remain recoverable beyond the old grace
     * period; only a stalled scan is still a fault. */
    input.now_ms = 80U;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm > command.right_rpm);
    assert(state.phase == gugah::COURSE_CRUISE);

    line = ValidLine(-1000);
    input.now_ms = 88U;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(state.line_control.line_valid);
    assert(!state.line_control.failed);
    assert(state.phase == gugah::COURSE_CRUISE);
}

void TestH5AndH6SearchRightAfterLineLossAndReacquire()
{
    const gugah::CourseKind kinds[] = {
        gugah::COURSE_H5, gugah::COURSE_H6
    };
    for (uint8_t i = 0U; i < 2U; i++) {
        gugah::HConfig config = DefaultConfig();
        config.line_lost_grace_ms = 20U;
        gugah::CourseState state = {};
        gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 50);
        assert(gugah::Course_Start(
            &state, kinds[i], &chassis, 0U, &config));

        drivers::GrayscaleProcessedData line = ValidLine(0);
        line.line_detected = false;
        line.position_valid = false;
        line.track_state = drivers::GRAYSCALE_TRACK_LOST;
        const uint32_t first_ms = 1000U;
        gugah::CourseInput input = {
            first_ms, &line, &chassis, true, 0
        };
        input.now_ms = 8U;
        chassis.received_ms = input.now_ms;
        gugah::MotionCommand command =
            gugah::Course_Update(&state, &input, &config);
        assert(command.mode == gugah::MOTION_COMMAND_SPEED);
        assert(command.left_rpm == 0);
        assert(command.right_rpm == 0);
        assert(state.phase == gugah::COURSE_CRUISE);

        input.now_ms = first_ms;
        chassis.received_ms = input.now_ms;
        command = gugah::Course_Update(&state, &input, &config);
        const int16_t expected_rpm = static_cast<int16_t>(
            config.h5_launch_ramp_rpm_s);
        assert(command.mode == gugah::MOTION_COMMAND_SPEED);
        assert(command.left_rpm > command.right_rpm);
        assert(command.right_rpm > 0);
        assert((command.left_rpm + command.right_rpm) / 2 ==
               expected_rpm);
        assert(state.phase == gugah::COURSE_CRUISE);

        /* Valid lost-line frames remain recoverable beyond the old grace
         * time for H5/H6 as well. */
        input.now_ms = first_ms + 80U;
        chassis.received_ms = input.now_ms;
        command = gugah::Course_Update(&state, &input, &config);
        assert(command.mode == gugah::MOTION_COMMAND_SPEED);
        assert(command.left_rpm > command.right_rpm);
        assert(state.phase == gugah::COURSE_CRUISE);

        line = ValidLine(-500);
        input.now_ms += 8U;
        chassis.received_ms = input.now_ms;
        command = gugah::Course_Update(&state, &input, &config);
        assert(command.mode == gugah::MOTION_COMMAND_SPEED);
        assert(state.line_control.line_valid);
        assert(!state.line_control.failed);
        assert(state.phase == gugah::COURSE_CRUISE);
    }
}

void TestBallControlAndFeedforward()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_accel_ff_mdeg_per_mm_s2 = 2;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 500);
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    assert(state.dm_target_mrad == -617);
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
    assert(state.rail_compensation_mdeg == 986);
    assert(state.pid_p_mdeg == -2750);
    assert(state.pid_i_mdeg == 0);
    assert(state.beam_target_mdeg < state.rail_compensation_mdeg);
    assert(gugah::Ball_MapBeamToDm(&config, -2000) == -405);
    assert(gugah::Ball_MapBeamToDm(&config, -8836) == 300);
    assert(gugah::Ball_MapBeamToDm(&config, 6094) == -1382);
    assert(gugah::Ball_MapBeamToDm(&config, -10000) == 300);
    assert(gugah::Ball_MapBeamToDm(&config, 10000) == -1382);
    assert(gugah::Ball_HoldAngleMdeg(&config, -750) == -542);
    assert(gugah::Ball_HoldAngleMdeg(&config, 250) == 773);

    input.now_ms = 30U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    input.chassis_accel_mm_s2 = 208;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.chassis_feedforward_mdeg > 1000);
    assert(state.chassis_feedforward_mdeg < 1400);

    input.now_ms = 40U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    input.chassis_accel_mm_s2 = -208;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.chassis_feedforward_mdeg < -1000);
    assert(state.chassis_feedforward_mdeg > -1400);

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
    const gugah::BallOutput stale =
        gugah::Ball_Update(&state, &input, &config);
    assert(!stale.stop_chassis);
    assert(state.result == gugah::BALL_RESULT_SUCCESS);
}

void TestBallCameraPositionCalibration()
{
    static const int16_t camera_points[11] = {
        -1028, -887, -713, -554, -360, -154,
        58, 274, 503, 709, 921
    };
    for (uint8_t i = 0U; i < 11U; i++) {
        const int16_t expected = static_cast<int16_t>(
            -1000 + static_cast<int16_t>(i) * 200);
        assert(gugah::Ball_CalibrateCameraPosition0p1mm(
                   camera_points[i]) == expected);
    }
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(20) == 164);
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(-1100) < -1100);
    assert(gugah::Ball_CalibrateCameraPosition0p1mm(1100) > 1100);
}

void TestBallZeroOffsetChangesControlOrigin()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_zero_offset_0p1mm = 200;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 200);
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    assert(state.measured_position_0p1mm == 0);
    assert(state.estimated_position_0p1mm == 0);

    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    (void)gugah::Ball_Update(&state, &input, &config);
    /* Rail compensation remains tied to the physical +20 mm location even
     * though that point is now controller coordinate zero. */
    assert(state.rail_compensation_mdeg == 730);
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
    input.dm.position_mrad = -617;
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
    input.dm.position_mrad = -617;
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
    input.dm.position_mrad = -617;
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
    assert(state.beam_target_mdeg < 0);
}

void TestBallOuterPdHasZeroVelocityTarget()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 1000);
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));
    input.now_ms = 15U;
    input.vision.frame.received_ms = 15U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 15U;
    input.dm.received_ms = 15U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.target_velocity_0p1mm_s == 0);
    assert(state.velocity_error_0p1mm_s == 0);
    assert(state.pid_p_mdeg == -5500);
    assert(state.pid_correction_mdeg == -4500);
}

void TestBallPredictedVelocityDoesNotCancelBreakaway()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 500);
    input.dm.position_mrad = -617;
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
    input.dm.position_mrad = -1382;
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
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 20U;
    input.vision.frame.received_ms = 20U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 20U;
    input.dm.received_ms = 20U;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.position_error_0p1mm == -80);
    assert(state.stiction_compensation_mdeg == 0);
    assert(state.pid_p_mdeg == -440);
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

void TestBallImuBeamCompensationIsDisabled()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_kp_mdeg_per_0p1mm = 0;
    config.ball_accel_ff_mdeg_per_mm_s2 = 0;
    config.ball_pid_kp_mdeg_per_mm = 0;
    config.ball_pid_ki_mdeg_per_mm_s = 0;
    config.ball_pid_kd_mdeg_per_mm_s = 0;
    config.ball_pid_integral_limit_mdeg = 0;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 0);
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 20U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.imu.beam_mdeg = 1000;
    input.dm.received_ms = input.now_ms;
    const gugah::BallOutput output =
        gugah::Ball_Update(&state, &input, &config);
    assert(output.command_valid);
    assert(state.beam_target_mdeg == 300);
    assert(state.imu_beam_mdeg == 1000);
    assert(state.imu_beam_error_mdeg == -700);
    assert(state.imu_beam_compensation_mdeg == 0);
    assert(state.beam_command_mdeg == 300);
    assert(output.dm_target_mrad ==
           gugah::Ball_MapBeamToDm(&config, 300));

    config.ball_imu_beam_kp_permille = 2000;
    config.ball_imu_beam_limit_0p1deg = 5U;
    input.now_ms = 30U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.imu_beam_compensation_mdeg == 0);
    assert(state.beam_command_mdeg == 560);

}

void TestBallIntegralRemovesStaticErrorWithoutWindup()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_observer_alpha_permille = 1000U;
    config.ball_pid_ki_mdeg_per_mm_s = 20;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 90);
    input.dm.position_mrad = -617;
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

void TestVisionAgeDoesNotChangeIntegralOrAngleLimit()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_pid_ki_mdeg_per_mm_s = 20;

    gugah::BallState integral_state = {};
    gugah::Ball_Init(&integral_state);
    gugah::BallInput integral_input = BallInput(10U, 100);
    integral_input.vision.ball_age_ms = 5000U;
    integral_input.vision.communication_online = false;
    integral_input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(
        &integral_state, 0, &integral_input, &config));
    integral_input.now_ms = 110U;
    integral_input.vision.ball_age_ms = 5100U;
    integral_input.imu.received_ms = integral_input.now_ms;
    integral_input.dm.received_ms = integral_input.now_ms;
    const gugah::BallOutput integral_output = gugah::Ball_Update(
        &integral_state, &integral_input, &config);
    assert(integral_output.command_valid);
    assert(integral_state.pid_i_mdeg < 0);

    gugah::BallState angle_state = {};
    gugah::Ball_Init(&angle_state);
    gugah::BallInput angle_input = BallInput(10U, 1000);
    angle_input.vision.ball_age_ms = 5000U;
    angle_input.vision.communication_online = false;
    angle_input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(
        &angle_state, -1000, &angle_input, &config));
    angle_input.now_ms = 110U;
    angle_input.vision.ball_age_ms = 5100U;
    angle_input.imu.received_ms = angle_input.now_ms;
    angle_input.dm.received_ms = angle_input.now_ms;
    const gugah::BallOutput angle_output = gugah::Ball_Update(
        &angle_state, &angle_input, &config);
    assert(angle_output.command_valid);
    assert(angle_state.beam_target_mdeg <
           -config.ball_degraded_angle_mdeg);
    assert(angle_state.beam_target_mdeg >=
           -config.ball_max_angle_mdeg);
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
    input.dm.position_mrad = -617;
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
    assert(state.rail_compensation_mdeg == 662);
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
    input.dm.position_mrad = -617;
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

void TestH3RunsPositiveCenterNegativeSequence()
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
    assert(state.ball.target_position_0p1mm == 500);
    assert(state.ball.timeout_ms == 30000U);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 110U;
    input.vision.frame.received_ms = 110U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 110U;
    input.dm.received_ms = 110U;
    const gugah::HAppOutput positive_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(positive_output.motion.mode == gugah::MOTION_COMMAND_NONE);
    assert(positive_output.buzzer_pulse);
    assert(state.h3_stage == 1U);
    assert(state.ball.target_position_0p1mm == 0);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 120U;
    input.vision.frame.received_ms = 120U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 120U;
    input.dm.received_ms = 120U;
    const gugah::HAppOutput center_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(!center_output.buzzer_pulse);
    assert(state.h3_stage == 2U);
    assert(state.ball.target_position_0p1mm == -500);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 130U;
    input.vision.frame.received_ms = 130U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 130U;
    input.dm.received_ms = 130U;
    const gugah::HAppOutput final_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(final_output.buzzer_pulse);
    assert(state.run_state == gugah::H_STATE_PASS);
    assert(state.h3_stage == 2U);
    assert(state.ball.mode == gugah::BALL_HOLD);
    assert(state.ball.target_position_0p1mm == -500);

    /* PASS keeps the final -50 mm hold controller alive. */
    input.now_ms = 140U;
    input.vision.frame.received_ms = 140U;
    input.vision.frame.sequence++;
    input.imu.received_ms = 140U;
    input.dm.received_ms = 140U;
    const gugah::BallOutput hold_output =
        gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(hold_output.command_valid);
}

void TestH3HasNoVisionAgeTimeout()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.vision.ball_age_ms = 80U;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;

    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.ball.mode == gugah::BALL_MOVE);

    gugah::HAppState old_frame_state = {};
    gugah::HApp_Init(&old_frame_state);
    old_frame_state.selected_problem = gugah::H_PROBLEM_3;
    input.vision.ball_age_ms = 5000U;
    input.vision.communication_online = false;
    assert(gugah::HApp_Start(&old_frame_state, &input, &config));
    assert(old_frame_state.run_state == gugah::H_STATE_RUNNING);
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

void TestH3StageDeadlinesAdvanceWithoutFailure()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 1000);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    assert(gugah::HApp_Start(&state, &input, &config));

    /* The +50 leg is allowed to use the complete 1.5-second window. */
    input.now_ms = 1599U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput before_positive_deadline =
        gugah::HApp_Update(&state, &input, &config);
    assert(!before_positive_deadline.buzzer_pulse);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.h3_stage == 0U);
    assert(state.ball.target_position_0p1mm == 500);

    /* At 1.5 seconds, abandon +50 without declaring failure and target O. */
    input.now_ms = 1600U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput positive_timeout_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(positive_timeout_output.buzzer_pulse);
    assert(state.h3_stage == 1U);
    assert(state.ball.target_position_0p1mm == 0);
    assert(state.run_state == gugah::H_STATE_RUNNING);

    /* O likewise receives at most one second. */
    input.now_ms = 2599U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput before_center_deadline =
        gugah::HApp_Update(&state, &input, &config);
    assert(!before_center_deadline.buzzer_pulse);
    assert(state.h3_stage == 1U);
    assert(state.ball.target_position_0p1mm == 0);

    input.now_ms = 2600U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput center_timeout_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(!center_timeout_output.buzzer_pulse);
    assert(state.h3_stage == 2U);
    assert(state.ball.target_position_0p1mm == -500);
    assert(state.run_state == gugah::H_STATE_RUNNING);

    /* Reaching -50 passes and leaves the final hold controller active. */
    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 2700U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput pass_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(pass_output.result_changed);
    assert(pass_output.buzzer_pulse);
    assert(state.run_state == gugah::H_STATE_PASS);
    assert(state.result_time_ms == 2600U);
    assert(state.ball.mode == gugah::BALL_HOLD);
    assert(state.ball.target_position_0p1mm == -500);
}

void TestH3ForcedCompletionKeepsNegativeHold()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    assert(gugah::HApp_Start(&state, &input, &config));

    /* Put the sequence on its final leg without declaring it settled. */
    state.h3_stage = 2U;
    state.active_ball_target_0p1mm = -500;
    state.ball.target_position_0p1mm = -500;
    state.ball.mode = gugah::BALL_MOVE;
    state.ball.result = gugah::BALL_RESULT_IDLE;

    input.now_ms = 4898U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput before_deadline =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(!before_deadline.buzzer_pulse);

    /* Start the 80 ms second pulse at 4.8 s, leaving 20 ms margin before
     * the 4.9-second requirement. */
    input.now_ms = 4900U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput completed =
        gugah::HApp_Update(&state, &input, &config);
    assert(completed.result_changed);
    assert(completed.buzzer_pulse);
    assert(state.run_state == gugah::H_STATE_PASS);
    assert(state.result_time_ms == 4800U);
    assert(state.active_ball_target_0p1mm == -500);
    assert(state.ball.target_position_0p1mm == -500);

    input.now_ms = 4910U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::BallOutput hold_output =
        gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(hold_output.command_valid);
    assert(state.ball.target_position_0p1mm == -500);
}

void TestH3StageStartFailureIsReportedImmediately()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_3;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    assert(gugah::HApp_Start(&state, &input, &config));

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 110U;
    input.vision.ball_usable = false;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput output =
        gugah::HApp_Update(&state, &input, &config);
    assert(output.result_changed);
    assert(state.run_state == gugah::H_STATE_FAIL);
    assert(state.failure == gugah::H_FAILURE_BALL);
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
    gugah::ImuFeedback imu = Imu(7900U);
    gugah::CourseInput input = {
        7900U, &line, &chassis, true, &imu
    };
    chassis = Chassis(7900U, 10000, 10000, 50);
    input.chassis = &chassis;
    (void)gugah::Course_Update(&state, &input, &config);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == 7900U);

    input.now_ms = 8200U;
    imu.received_ms = input.now_ms;
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

void TestH4SoftLaunchAndHeadingLimit()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H4, &chassis, 0U, &config));
    gugah::ImuFeedback imu = Imu(500U);
    gugah::CourseInput input = {
        500U, 0, &chassis, false, &imu
    };
    chassis.received_ms = input.now_ms;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);
    assert(state.h4_commanded_accel_mm_s2 > 100);

    input.now_ms = 1000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 40);
    assert(command.right_rpm == 40);

    input.now_ms = 2000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 80);
    assert(command.right_rpm == 80);
    assert(state.h4_commanded_accel_mm_s2 > 100);

    input.now_ms = 2500U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 100);
    assert(command.right_rpm == 100);
    assert(state.h4_commanded_accel_mm_s2 == 0);

    /* H4 ignores grayscale completely.  A large yaw error at 30 RPM still
     * keeps both wheels forward by limiting correction to base/2. */
    gugah::Course_Init(&state);
    chassis.received_ms = 0U;
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H4, &chassis, 0U, &config));
    input.now_ms = 1U;
    imu = Imu(input.now_ms, 0);
    input.imu = &imu;
    input.chassis = &chassis;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 0);
    assert(command.right_rpm == 0);

    input.now_ms = 500U;
    imu = Imu(input.now_ms, -30000);
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 10);
    assert(command.right_rpm == 30);
    assert(state.h4_yaw_error_mdeg == 30000);
    assert(state.h4_heading_correction_rpm == 10);
    assert(state.h4_commanded_accel_mm_s2 > 100);
}

void TestH4SoftBrakeBeginsBeforeB()
{
    gugah::HConfig config = DefaultConfig();
    config.h4_b_distance_mm = 1400U;
    config.h4_brake_distance_mm = 1000U;
    config.h4_stop_distance_mm = 1700U;
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 100);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H4, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::ImuFeedback imu = Imu(3000U);
    gugah::CourseInput input = {
        3000U, &line, &chassis, true, &imu
    };
    chassis = Chassis(input.now_ms, 8000, 8000, 100);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(state.h4_braking);
    assert(!state.passed_b_or_a);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 100);
    assert(command.right_rpm == 100);
    assert(state.h4_commanded_accel_mm_s2 < -100);

    input.now_ms = 3333U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 87);
    assert(command.right_rpm == 87);

    input.now_ms = 4000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 60);
    assert(command.right_rpm == 60);

    input.now_ms = 5000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);
    assert(state.h4_commanded_accel_mm_s2 == 0);

    /* The minimum crawl crosses B, then the final 20 RPM is ramped to zero
     * instead of becoming a stop impulse. */
    input.now_ms = 5100U;
    imu.received_ms = input.now_ms;
    chassis = Chassis(input.now_ms, 10000, 10000, 20);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);
    assert(state.h4_commanded_accel_mm_s2 < -100);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == 5100U);

    input.now_ms = 5600U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.phase == gugah::COURSE_STOPPING);
    assert(state.pass_ms == 5100U);
}

void TestH5SoftLaunchKeepsCruiseBeforeFinish()
{
    gugah::HConfig config = DefaultConfig();
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H5, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::CourseInput input = {
        500U, &line, &chassis, true, 0
    };
    chassis.received_ms = input.now_ms;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);
    assert(state.h5_commanded_accel_mm_s2 > 100);
    assert(state.h5_commanded_accel_mm_s2 < 200);

    const int32_t straight_count =
        CountsForMillimeters(900, config);
    input.now_ms = 3000U;
    chassis = Chassis(
        input.now_ms, straight_count, straight_count, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 95);
    assert(command.right_rpm == 95);
    assert(state.phase == gugah::COURSE_CRUISE);
    assert(state.h5_commanded_accel_mm_s2 == 0);
}

void TestH5CurveSpeedUsesOdometryAndRamps()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H5, &chassis, 0U, &config));
    /* A deliberately large line error proves that road classification no
     * longer depends on the grayscale position. */
    drivers::GrayscaleProcessedData line = ValidLine(1000);
    gugah::CourseInput input = {
        3000U, &line, &chassis, true, 0
    };
    const int32_t straight_count =
        CountsForMillimeters(900, config);
    chassis = Chassis(
        input.now_ms, straight_count, straight_count, 95);
    input.chassis = &chassis;

    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 0U);
    assert(state.line_control.applied_tuning.kp_milli == 18);
    assert(state.line_control.applied_tuning.kd_milli == 12);

    /* The computed preparation point is about 1028 mm: 292 mm of gentle
     * 30 RPM/s braking plus a 180 mm settling margin before B at 1500 mm. */
    input.now_ms += 8U;
    const int32_t curve_approach_count =
        CountsForMillimeters(1040, config);
    chassis = Chassis(input.now_ms,
                      curve_approach_count,
                      curve_approach_count, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 0U);
    assert(state.line_control.applied_tuning.kp_milli == 18);
    assert(state.h5_road_ramp_rpm_s == -30);
    assert(state.h5_speed_limit_millirpm < 95000);
    assert((command.left_rpm + command.right_rpm) / 2 < 95);
    assert(state.h5_commanded_accel_mm_s2 < -100);
    assert(state.h5_commanded_accel_mm_s2 > -200);

    const int32_t first_curve_count =
        CountsForMillimeters(1500, config);
    for (uint16_t i = 0U; i < 160U; i++) {
        input.now_ms += 8U;
        chassis = Chassis(input.now_ms,
                          first_curve_count,
                          first_curve_count, 63);
        input.chassis = &chassis;
        command = gugah::Course_Update(&state, &input, &config);
    }
    assert(state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 1000U);
    assert(state.line_control.applied_tuning.kp_milli == 25);
    assert(state.line_control.applied_tuning.kd_milli == 50);
    assert(state.h5_speed_limit_millirpm == 63000);
    assert((command.left_rpm + command.right_rpm) / 2 == 63);

    line.line_position = 0;
    input.now_ms += 8U;
    const int32_t before_c_count =
        CountsForMillimeters(3070, config);
    chassis = Chassis(
        input.now_ms, before_c_count, before_c_count, 63);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 1000U);

    /* C is at 1500 + 1571 = 3071 mm; recovery starts by mileage even if
     * the line is already perfectly centred. */
    input.now_ms += 8U;
    const int32_t after_c_count =
        CountsForMillimeters(3071, config);
    chassis = Chassis(input.now_ms, after_c_count, after_c_count, 63);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 1000U);
    assert(state.h5_commanded_accel_mm_s2 > 100);
    assert(state.h5_commanded_accel_mm_s2 < 200);

    const int32_t second_straight_count =
        CountsForMillimeters(3400, config);
    for (uint16_t i = 0U; i < 170U; i++) {
        input.now_ms += 8U;
        chassis = Chassis(input.now_ms,
                          second_straight_count,
                          second_straight_count, 95);
        input.chassis = &chassis;
        command = gugah::Course_Update(&state, &input, &config);
    }
    assert(state.h5_speed_limit_millirpm == 95000);
    assert((command.left_rpm + command.right_rpm) / 2 == 95);
    assert(state.line_curve_factor_permille == 0U);

    input.now_ms += 8U;
    const int32_t before_second_curve =
        CountsForMillimeters(4080, config);
    chassis = Chassis(input.now_ms,
                      before_second_curve,
                      before_second_curve, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);

    input.now_ms += 8U;
    const int32_t second_curve_approach =
        CountsForMillimeters(4110, config);
    chassis = Chassis(input.now_ms,
                      second_curve_approach,
                      second_curve_approach, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.h5_curve_mode);
    assert(state.line_curve_factor_permille == 0U);

    /* Steering gains begin their own 250 mm blend close to the physical
     * D entry at 4571 mm, independent of the earlier speed reduction. */
    input.now_ms += 8U;
    const int32_t steering_transition =
        CountsForMillimeters(4500, config);
    chassis = Chassis(input.now_ms,
                      steering_transition,
                      steering_transition, 80);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.line_curve_factor_permille > 0U);
    assert(state.line_curve_factor_permille < 1000U);
    assert(state.line_control.applied_tuning.kp_milli > 18);
    assert(state.line_control.applied_tuning.kp_milli < 25);
}

void TestH6MatchesH5ChassisStrategy()
{
    gugah::HConfig config = DefaultConfig();
    /* These retired H6 chassis values deliberately disagree with H5.  The
     * side-by-side commands below prove they no longer affect H6. */
    config.h6_cruise_rpm = 7U;
    config.h6_approach_rpm = 6U;
    config.h6_finish_gate_mm = 1U;
    config.h6_approach_start_mm = 1U;

    gugah::CourseState h5 = {};
    gugah::CourseState h6 = {};
    gugah::ChassisFeedback start = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &h5, gugah::COURSE_H5, &start, 0U, &config));
    assert(gugah::Course_Start(
        &h6, gugah::COURSE_H6, &start, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(100);

    const uint32_t times_ms[] = { 1000U, 2000U, 3000U, 4000U };
    const int32_t distances_mm[] = { 900, 1500, 3400, 4500 };
    for (uint8_t i = 0U; i < 4U; i++) {
        const int32_t count =
            CountsForMillimeters(distances_mm[i], config);
        gugah::ChassisFeedback chassis = Chassis(
            times_ms[i], count, count, 0);
        gugah::CourseInput h5_input = {
            times_ms[i], &line, &chassis, true, 0
        };
        gugah::CourseInput h6_input = h5_input;
        const gugah::MotionCommand h5_command =
            gugah::Course_Update(&h5, &h5_input, &config);
        const gugah::MotionCommand h6_command =
            gugah::Course_Update(&h6, &h6_input, &config);
        assert(h6_command.mode == h5_command.mode);
        assert(h6_command.left_rpm == h5_command.left_rpm);
        assert(h6_command.right_rpm == h5_command.right_rpm);
        assert(h6.phase == h5.phase);
        assert(h6.h5_curve_mode == h5.h5_curve_mode);
        assert(h6.line_curve_factor_permille ==
               h5.line_curve_factor_permille);
        assert(h6.h5_speed_limit_millirpm ==
               h5.h5_speed_limit_millirpm);
        assert(h6.h5_commanded_accel_mm_s2 ==
               h5.h5_commanded_accel_mm_s2);
    }

    /* A wide A marker at 5200 mm is ignored by both tasks. */
    line.track_state = drivers::GRAYSCALE_TRACK_WIDE;
    line.active_mask = 0x7EU;
    int32_t count = CountsForMillimeters(5200, config);
    gugah::ChassisFeedback chassis = Chassis(5000U, count, count, 63);
    gugah::CourseInput h5_input = {
        5000U, &line, &chassis, true, 0
    };
    gugah::CourseInput h6_input = h5_input;
    gugah::MotionCommand h5_command =
        gugah::Course_Update(&h5, &h5_input, &config);
    gugah::MotionCommand h6_command =
        gugah::Course_Update(&h6, &h6_input, &config);
    assert(!h5.passed_b_or_a);
    assert(!h6.passed_b_or_a);
    assert(h6_command.left_rpm == h5_command.left_rpm);
    assert(h6_command.right_rpm == h5_command.right_rpm);

    /* Both start the same final brake, freeze the score at lap+50 mm and
     * continue through the same post-finish soft stop. */
    count = CountsForMillimeters(config.h5_brake_distance_mm, config);
    chassis = Chassis(6000U, count, count, 63);
    h5_input.now_ms = 6000U;
    h5_input.chassis = &chassis;
    h6_input = h5_input;
    h5_command = gugah::Course_Update(&h5, &h5_input, &config);
    h6_command = gugah::Course_Update(&h6, &h6_input, &config);
    assert(h5.h5_braking && h6.h5_braking);
    assert(h6_command.left_rpm == h5_command.left_rpm);
    assert(h6_command.right_rpm == h5_command.right_rpm);

    count = CountsForMillimeters(
        static_cast<int32_t>(config.lap_distance_mm) + 50, config);
    chassis = Chassis(7000U, count, count, 20);
    h5_input.now_ms = 7000U;
    h5_input.chassis = &chassis;
    h6_input = h5_input;
    h5_command = gugah::Course_Update(&h5, &h5_input, &config);
    h6_command = gugah::Course_Update(&h6, &h6_input, &config);
    assert(h5.passed_b_or_a && h6.passed_b_or_a);
    assert(h5.pass_ms == h6.pass_ms);
    assert(h6_command.left_rpm == h5_command.left_rpm);
    assert(h6_command.right_rpm == h5_command.right_rpm);

    chassis.received_ms = 8500U;
    h5_input.now_ms = 8500U;
    h5_input.line_frame_new = false;
    h6_input = h5_input;
    h5_command = gugah::Course_Update(&h5, &h5_input, &config);
    h6_command = gugah::Course_Update(&h6, &h6_input, &config);
    assert(h5_command.mode == gugah::MOTION_COMMAND_STOP);
    assert(h6_command.mode == h5_command.mode);
    assert(h6.phase == h5.phase);

    chassis = Chassis(8505U, count, count, 0);
    h5_input.now_ms = 8505U;
    h5_input.chassis = &chassis;
    h6_input = h5_input;
    (void)gugah::Course_Update(&h5, &h5_input, &config);
    (void)gugah::Course_Update(&h6, &h6_input, &config);
    assert(h5.phase == gugah::COURSE_COMPLETE);
    assert(h6.phase == h5.phase);
}

void TestH5IgnoresAThenSoftStopsAtOdometry()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H5, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::CourseInput input = {
        3000U, &line, &chassis, true, 0
    };
    chassis = Chassis(input.now_ms, 41400, 41400, 95);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(state.h5_braking);
    assert(!state.passed_b_or_a);
    assert(command.left_rpm == 95);
    assert(command.right_rpm == 95);
    assert(state.h5_commanded_accel_mm_s2 < -100);
    assert(state.h5_commanded_accel_mm_s2 > -200);

    input.now_ms = 4000U;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 55);
    assert(command.right_rpm == 55);

    line.track_state = drivers::GRAYSCALE_TRACK_WIDE;
    line.active_mask = 0x7EU;
    input.now_ms = 5000U;
    chassis = Chassis(input.now_ms, 43100, 43100, 20);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.passed_b_or_a);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);

    input.now_ms = 5008U;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.passed_b_or_a);
    assert(state.finish_confirm_frames == 0U);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);
    assert(state.h5_commanded_accel_mm_s2 == 0);

    /* 43412 counts is 6192 mm with the default calibration: 6142 + 50. */
    input.now_ms = 5017U;
    chassis = Chassis(input.now_ms, 43412, 43412, 20);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == 5017U);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 20);
    assert(command.right_rpm == 20);

    input.now_ms = 5267U;
    chassis.received_ms = input.now_ms;
    input.line_frame_new = false;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 10);
    assert(command.right_rpm == 10);

    input.now_ms = 5517U;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_STOP);
    assert(state.phase == gugah::COURSE_STOPPING);
    assert(state.pass_ms == 5017U);
}

void TestH5OdometryThresholdIsLapPlus50Mm()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H5, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    line.track_state = drivers::GRAYSCALE_TRACK_WIDE;
    line.active_mask = 0x7EU;
    gugah::CourseInput input = { 23000U, &line, &chassis, true, 0 };
    /* 43400 counts is 6190 mm, so even a wide A marker must not finish H5. */
    chassis = Chassis(input.now_ms, 43400, 43400, 20);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(!state.passed_b_or_a);

    input.now_ms = 24350U;
    chassis = Chassis(input.now_ms, 43412, 43412, 20);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(state.failure == gugah::COURSE_FAILURE_NONE);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == input.now_ms);
}

void TestH4StartsBallAndCourseTogether()
{
    gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_4;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    input.line = 0;
    input.line_frame_new = false;
    input.chassis = Chassis(100U, 1000, 2000, 0);
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;

    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.course.kind == gugah::COURSE_H4);
    assert(state.course.phase == gugah::COURSE_CRUISE);
    assert(state.ball.mode == gugah::BALL_HOLD);
    assert(state.ball.target_position_0p1mm == 0);
    assert(state.active_ball_target_0p1mm == 0);
}

void TestH7MatchesH4WithAdjustableBallTarget()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_7;

    /* H7 uses the H6-style READY adjustment and starts without gray data. */
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    input.buttons.b3_increment = true;
    for (uint8_t i = 0U; i < 30U; i++) {
        (void)gugah::HApp_Update(&state, &input, &config);
    }
    input.buttons.b3_increment = false;
    assert(state.h7_target_0p1mm == 300);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 300);

    input.line = 0;
    input.line_frame_new = false;
    input.chassis = Chassis(input.now_ms, 1000, 2000, 0);
    const gugah::BallInput ball_input = BallInput(input.now_ms, 300);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.course.kind == gugah::COURSE_H7);
    assert(state.ball.target_position_0p1mm == 300);
    assert(state.active_ball_target_0p1mm == 300);

    /* The 2 ms ball loop receives the same H4 launch/brake feedforward. */
    state.course.h4_commanded_accel_mm_s2 = 138;
    input.now_ms = 110U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(state.ball.chassis_feedforward_mdeg > 0);

    /* PASS continues holding the configured H7 target. */
    state.run_state = gugah::H_STATE_PASS;
    input.now_ms = 120U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::BallOutput hold =
        gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(hold.command_valid);
    assert(state.ball.target_position_0p1mm == 300);
}

void TestH7MatchesH4ChassisCommands()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::CourseState h4 = {};
    gugah::CourseState h7 = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 0);
    assert(gugah::Course_Start(
        &h4, gugah::COURSE_H4, &chassis, 0U, &config));
    assert(gugah::Course_Start(
        &h7, gugah::COURSE_H7, &chassis, 0U, &config));

    const uint32_t times_ms[] = { 500U, 1000U, 2000U, 2500U };
    gugah::ImuFeedback imu = {};
    for (uint8_t i = 0U; i < 4U; i++) {
        chassis.received_ms = times_ms[i];
        imu = Imu(times_ms[i], 0);
        const gugah::CourseInput input = {
            times_ms[i], 0, &chassis, false, &imu
        };
        const gugah::MotionCommand h4_command =
            gugah::Course_Update(&h4, &input, &config);
        const gugah::MotionCommand h7_command =
            gugah::Course_Update(&h7, &input, &config);
        assert(h4_command.mode == h7_command.mode);
        assert(h4_command.left_rpm == h7_command.left_rpm);
        assert(h4_command.right_rpm == h7_command.right_rpm);
        assert(h4.h4_commanded_accel_mm_s2 ==
               h7.h4_commanded_accel_mm_s2);
    }
}

void TestH5AndH6CourseAccelerationReachesBallFeedforward()
{
    const gugah::HProblem problems[] = {
        gugah::H_PROBLEM_5, gugah::H_PROBLEM_6
    };
    for (uint8_t i = 0U; i < 2U; i++) {
        gugah::HConfig config = DefaultConfig();
        gugah::HAppState state = {};
        gugah::HApp_Init(&state);
        state.selected_problem = problems[i];
        state.h6_target_0p1mm = 300;
        drivers::GrayscaleProcessedData line = ValidLine(0);
        gugah::HAppInput input = {};
        input.now_ms = 100U;
        input.line = &line;
        input.line_frame_new = true;
        input.chassis = Chassis(input.now_ms, 0, 0, 0);
        const gugah::BallInput ball_input = BallInput(input.now_ms, 0);
        input.vision = ball_input.vision;
        input.imu = ball_input.imu;
        input.dm = ball_input.dm;
        input.dm.position_mrad = -617;
        assert(gugah::HApp_Start(&state, &input, &config));
        assert(state.ball.target_position_0p1mm ==
               ((problems[i] == gugah::H_PROBLEM_6) ? 300 : 0));

        /* H5/H6 share the signed chassis acceleration used by the 2 ms
         * ball loop; only their held ball target differs. */
        state.course.h5_commanded_accel_mm_s2 = 138;
        input.now_ms = 110U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::HApp_UpdateBall2ms(&state, &input, &config);
        assert(state.ball.chassis_feedforward_mdeg > 0);

        state.course.h5_commanded_accel_mm_s2 = -138;
        input.now_ms = 120U;
        input.vision.frame.received_ms = input.now_ms;
        input.vision.frame.sequence++;
        input.imu.received_ms = input.now_ms;
        input.dm.received_ms = input.now_ms;
        (void)gugah::HApp_UpdateBall2ms(&state, &input, &config);
        assert(state.ball.chassis_feedforward_mdeg < 0);
    }
}

void TestH4AndH7ReportEightSecondTimeout()
{
    const gugah::HConfig config = DefaultConfig();
    const gugah::HProblem problems[] = {
        gugah::H_PROBLEM_4, gugah::H_PROBLEM_7
    };
    for (uint8_t i = 0U; i < 2U; i++) {
        gugah::HAppState state = {};
        gugah::HApp_Init(&state);
        state.selected_problem = problems[i];
        gugah::HAppInput input = {};
        input.now_ms = 100U;
        input.chassis = Chassis(100U, 0, 0, 0);
        const gugah::BallInput ball_input = BallInput(100U, 0);
        input.vision = ball_input.vision;
        input.imu = ball_input.imu;
        input.dm = ball_input.dm;
        assert(gugah::HApp_Start(&state, &input, &config));

        input.now_ms = 8101U;
        input.chassis.received_ms = input.now_ms;
        const gugah::HAppOutput output =
            gugah::HApp_Update(&state, &input, &config);
        assert(output.motion.mode == gugah::MOTION_COMMAND_STOP);
        assert(output.result_changed);
        assert(state.run_state == gugah::H_STATE_FAIL);
        assert(state.failure == gugah::H_FAILURE_TIMEOUT);
    }
}

void TestPassHoldRecoversTransientDeviceGap()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_4;
    state.run_state = gugah::H_STATE_PASS;
    state.active_ball_target_0p1mm = 0;
    state.ball.mode = gugah::BALL_FAILED;
    state.ball.result = gugah::BALL_RESULT_VISION_LOST;
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;

    const gugah::BallOutput recovered =
        gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(recovered.command_valid);
    assert(state.ball.mode == gugah::BALL_HOLD);
    assert(state.ball.result == gugah::BALL_RESULT_SUCCESS);
    assert(state.ball.target_position_0p1mm == 0);
}

void TestBallContinuesPastOldEndpointLimit()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    gugah::BallState state = {};
    gugah::Ball_Init(&state);
    gugah::BallInput input = BallInput(10U, 1200);
    input.dm.position_mrad = -617;
    assert(gugah::Ball_StartHold(&state, 0, &input, &config));

    input.now_ms = 20U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::BallOutput output =
        gugah::Ball_Update(&state, &input, &config);
    assert(output.command_valid);
    assert(!output.stop_chassis);
    assert(state.mode == gugah::BALL_HOLD);
    assert(state.result == gugah::BALL_RESULT_SUCCESS);
    assert(state.position_error_0p1mm < 0);
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
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 0);
    for (uint8_t i = 0U; i < 4U; i++) {
        input.buttons.b1_short = true;
        (void)gugah::HApp_Update(&state, &input, &config);
        input.buttons.b1_short = false;
    }
    assert(state.selected_problem == gugah::H_PROBLEM_6);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 0);
    input.buttons.b3_increment = true;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h6_target_0p1mm == 10);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 10);
    for (uint16_t i = 0U; i < 200U; i++) {
        (void)gugah::HApp_Update(&state, &input, &config);
    }
    assert(state.h6_target_0p1mm == 1000);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 1000);
    input.buttons.b3_increment = false;
    input.buttons.b1_short = true;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.selected_problem == gugah::H_PROBLEM_7);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 0);
    input.buttons.b1_short = false;
    input.buttons.b2_decrement = true;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h7_target_0p1mm == -10);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == -10);
    input.buttons.b2_decrement = false;
    input.buttons.b1_short = true;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.selected_problem == gugah::H_PROBLEM_CAL_ZERO);
    assert(gugah::HApp_ReadyBallTarget0p1mm(&state) == 0);
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.selected_problem == gugah::H_PROBLEM_CAL_H2_LOOP);
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.selected_problem == gugah::H_PROBLEM_CAL_GRAY);
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.selected_problem == gugah::H_PROBLEM_2);
}

void TestCalZeroButtonFlow()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_CAL_ZERO;

    gugah::HAppInput input = {};
    input.now_ms = 100U;
    const gugah::BallInput ball_input = BallInput(100U, 0);
    input.vision = ball_input.vision;
    input.imu = ball_input.imu;
    input.dm = ball_input.dm;
    input.chassis_fault = true;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(state.ball.target_position_0p1mm == 0);

    input.buttons.any_pressed = true;
    input.buttons.b2_decrement = true;
    const gugah::HAppOutput negative =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(negative.ball_zero_delta_0p1mm == -10);
    assert(!negative.save_config);

    input.buttons.b2_decrement = false;
    input.buttons.b3_increment = true;
    const gugah::HAppOutput positive =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(positive.ball_zero_delta_0p1mm == 10);

    input.buttons.b3_increment = false;
    input.buttons.b1_pressed = true;
    const gugah::HAppOutput exit =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_READY);
    assert(exit.save_config);
    assert(exit.result_changed);
    assert(!exit.timer_running);
}

void TestCalH2LoopButtonFlow()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_CAL_H2_LOOP;

    gugah::HAppInput input = {};
    input.now_ms = 100U;
    input.chassis_fault = true;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);

    input.buttons.any_pressed = true;
    input.buttons.b2_decrement = true;
    const gugah::HAppOutput negative =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(negative.h2_loop_delta_mm == -10);
    assert(!negative.save_config);

    input.buttons.b2_decrement = false;
    input.buttons.b3_increment = true;
    const gugah::HAppOutput positive =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(positive.h2_loop_delta_mm == 10);

    /* Pressing B1 alone does not exit; the short-press event arrives on
     * release and is the requested explicit confirmation. */
    input.buttons.b3_increment = false;
    input.buttons.b1_pressed = true;
    const gugah::HAppOutput pressed =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(!pressed.save_config);

    input.buttons.b1_pressed = false;
    input.buttons.b1_short = true;
    const gugah::HAppOutput exit =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_READY);
    assert(exit.save_config);
    assert(exit.result_changed);
    assert(!exit.timer_running);
}

void TestCalGrayButtonFlow()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_CAL_GRAY;

    /* Calibration must start without a valid line, chassis, vision, IMU or
     * DM because its purpose is to create the missing grayscale reference. */
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    input.chassis_fault = true;
    assert(gugah::HApp_Start(&state, &input, &config));
    assert(state.run_state == gugah::H_STATE_RUNNING);

    input.buttons.any_pressed = true;
    input.buttons.b2_decrement = true;
    gugah::HAppOutput output =
        gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(output.gray_calibrate_white);
    assert(!output.gray_calibrate_black);
    assert(output.buzzer_pulse);
    assert(!output.save_config);

    input.buttons.b2_decrement = false;
    input.buttons.b3_increment = true;
    output = gugah::HApp_Update(&state, &input, &config);
    assert(!output.gray_calibrate_white);
    assert(output.gray_calibrate_black);

    input.buttons.b3_increment = false;
    input.buttons.b1_pressed = true;
    output = gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);
    assert(!output.save_config);

    input.buttons.b1_pressed = false;
    input.buttons.b1_short = true;
    output = gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_READY);
    assert(output.motion.mode == gugah::MOTION_COMMAND_STOP);
    assert(output.save_config);
    assert(output.result_changed);
    assert(!output.timer_running);
}

} /* namespace */

int main()
{
    TestConfigRecord();
    TestVisionProtocol();
    TestVisionStateKeepsLastBall();
    TestLineControl();
    TestLineControlDerivativeFilter();
    TestCourseH2UsesCalibratedAverageOdometryWithoutTimeout();
    TestCourseRunsOncePerGrayscaleFrame();
    TestH2SearchesRightAfterLineLossAndReacquires();
    TestH5AndH6SearchRightAfterLineLossAndReacquire();
    TestBallCameraPositionCalibration();
    TestBallZeroOffsetChangesControlOrigin();
    TestBallImuBeamCompensationIsDisabled();
    TestBallControlAndFeedforward();
    TestBallRetargetIsBumpless();
    TestBallObserverUsesPositionResidual();
    TestBallEstimatorInterpolatesLowSpeedMotion();
    TestBallOverspeedUsesFastBrake();
    TestBallOuterPdHasZeroVelocityTarget();
    TestBallPredictedVelocityDoesNotCancelBreakaway();
    TestBallPidCorrectsInsideAcceptedBand();
    TestBallIntegralRemovesStaticErrorWithoutWindup();
    TestVisionAgeDoesNotChangeIntegralOrAngleLimit();
    TestBallDampingWorksAcrossFullTravel();
    TestBallDampingWorksNearTarget();
    TestH3RunsPositiveCenterNegativeSequence();
    TestH3HasNoVisionAgeTimeout();
    TestH3ReportsDmTimeoutSeparately();
    TestH3StageDeadlinesAdvanceWithoutFailure();
    TestH3ForcedCompletionKeepsNegativeHold();
    TestH3StageStartFailureIsReportedImmediately();
    TestH4TimeFreezesAtB();
    TestH4SoftLaunchAndHeadingLimit();
    TestH4SoftBrakeBeginsBeforeB();
    TestH5SoftLaunchKeepsCruiseBeforeFinish();
    TestH5CurveSpeedUsesOdometryAndRamps();
    TestH6MatchesH5ChassisStrategy();
    TestH5IgnoresAThenSoftStopsAtOdometry();
    TestH5OdometryThresholdIsLapPlus50Mm();
    TestH4StartsBallAndCourseTogether();
    TestH7MatchesH4WithAdjustableBallTarget();
    TestH7MatchesH4ChassisCommands();
    TestH5AndH6CourseAccelerationReachesBallFeedforward();
    TestH4AndH7ReportEightSecondTimeout();
    TestPassHoldRecoversTransientDeviceGap();
    TestBallContinuesPastOldEndpointLimit();
    TestH6Buttons();
    TestCalZeroButtonFlow();
    TestCalH2LoopButtonFlow();
    TestCalGrayButtonFlow();
    puts("gugaH core tests passed");
    return 0;
}
