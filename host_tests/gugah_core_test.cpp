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
    assert(config.dm_position_mrad[0] == 183);
    assert(config.dm_position_mrad[1] == -197);
    assert(config.dm_position_mrad[2] == -617);
    assert(config.dm_position_mrad[3] == -1117);
    assert(config.dm_position_mrad[4] == -1537);
    assert(config.h4_launch_ramp_rpm_s == 60U);
    assert(config.h4_stop_ramp_rpm_s == 60U);
    assert(config.h4_brake_distance_mm == 1100U);
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
    gugah::HConfigRecord record = {};
    gugah::HConfig_BuildRecord(&config, &record);
    gugah::HConfig parsed = {};
    assert(gugah::HConfig_ParseRecord(&record, &parsed));
    assert(parsed.lap_distance_mm == 6142U);

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
    assert(parsed.dm_position_mrad[0] == 183);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1537);

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
    assert(parsed.dm_position_mrad[0] == 183);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1537);

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
     * control tuning and append the conservative 60 RPM/s default. */
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
    assert(parsed.h4_launch_ramp_rpm_s == 60U);
    assert(parsed.h4_stop_ramp_rpm_s == 60U);

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
    assert(parsed.h4_brake_distance_mm == 1100U);
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
    assert(parsed.h4_brake_distance_mm == 1100U);
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

    /* Schema 17 preserves a commissioned linkage shape while translating
     * its centre knot to the newly measured -617 mrad horizontal point. */
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
    assert(parsed.dm_position_mrad[0] == 193);
    assert(parsed.dm_position_mrad[1] == -187);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[3] == -1087);
    assert(parsed.dm_position_mrad[4] == -1517);

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
    assert(parsed.h4_brake_distance_mm == 1100U);
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
    assert(parsed.h4_brake_distance_mm == 1100U);
    assert(parsed.h4_stop_distance_mm == 1700U);

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
    assert(parsed.dm_position_mrad[0] == 183);
    assert(parsed.dm_position_mrad[2] == -617);
    assert(parsed.dm_position_mrad[4] == -1537);
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
    gugah::CourseInput input = { 100U, &line, &chassis, true, 0 };
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
    assert(state.rail_compensation_mdeg == 0);
    assert(state.pid_p_mdeg == -2000);
    assert(state.pid_i_mdeg == 0);
    assert(state.beam_target_mdeg < state.rail_compensation_mdeg);
    assert(gugah::Ball_MapBeamToDm(&config, -2000) == -407);
    assert(gugah::Ball_HoldAngleMdeg(&config, -750) == 0);
    assert(gugah::Ball_HoldAngleMdeg(&config, 250) == 0);

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
    assert(state.beam_target_mdeg < -500);
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
    assert(state.beam_target_mdeg == 0);
    assert(state.imu_beam_mdeg == 1000);
    assert(state.imu_beam_error_mdeg == -1000);
    assert(state.imu_beam_compensation_mdeg == 0);
    assert(state.beam_command_mdeg == 0);
    assert(output.dm_target_mrad == -617);

    config.ball_imu_beam_kp_permille = 2000;
    config.ball_imu_beam_limit_0p1deg = 5U;
    input.now_ms = 30U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::Ball_Update(&state, &input, &config);
    assert(state.imu_beam_compensation_mdeg == 0);
    assert(state.beam_command_mdeg == 0);

}

void TestBallIntegralRemovesStaticErrorWithoutWindup()
{
    gugah::HConfig config = DefaultConfig();
    config.vision_position_invert = 0U;
    config.ball_observer_alpha_permille = 1000U;
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
    assert(state.ball.timeout_ms == 30000U);

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

void TestH3ContinuesBeyondFiveSeconds()
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

    /* Five seconds is scored performance, not a safety fault.  The sequence
     * continues and still reaches the final indefinite -50 mm hold. */
    input.now_ms = 5101U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.run_state == gugah::H_STATE_RUNNING);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 5200U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h3_stage == 1U);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 5300U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    (void)gugah::HApp_Update(&state, &input, &config);
    assert(state.h3_stage == 2U);

    state.ball.mode = gugah::BALL_HOLD;
    state.ball.result = gugah::BALL_RESULT_SUCCESS;
    input.now_ms = 5400U;
    input.vision.frame.received_ms = input.now_ms;
    input.vision.frame.sequence++;
    input.imu.received_ms = input.now_ms;
    input.dm.received_ms = input.now_ms;
    const gugah::HAppOutput pass_output =
        gugah::HApp_Update(&state, &input, &config);
    assert(pass_output.result_changed);
    assert(state.run_state == gugah::H_STATE_PASS);
    assert(state.result_time_ms == 5300U);
    assert(state.ball.mode == gugah::BALL_HOLD);
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
    assert(command.left_rpm == 30);
    assert(command.right_rpm == 30);
    assert(state.h4_commanded_accel_mm_s2 > 200);

    input.now_ms = 1000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 60);
    assert(command.right_rpm == 60);

    input.now_ms = 2000U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 120);
    assert(command.right_rpm == 120);
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
    assert(command.left_rpm == 15);
    assert(command.right_rpm == 45);
    assert(state.h4_yaw_error_mdeg == 30000);
    assert(state.h4_heading_correction_rpm == 15);
    assert(state.h4_commanded_accel_mm_s2 > 200);
}

void TestH4SoftBrakeBeginsBeforeB()
{
    gugah::HConfig config = DefaultConfig();
    config.h4_b_distance_mm = 1400U;
    config.h4_brake_distance_mm = 1000U;
    config.h4_stop_distance_mm = 1700U;
    gugah::CourseState state = {};
    gugah::ChassisFeedback chassis = Chassis(0U, 0, 0, 120);
    assert(gugah::Course_Start(
        &state, gugah::COURSE_H4, &chassis, 0U, &config));
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::ImuFeedback imu = Imu(3000U);
    gugah::CourseInput input = {
        3000U, &line, &chassis, true, &imu
    };
    chassis = Chassis(input.now_ms, 8000, 8000, 120);
    input.chassis = &chassis;
    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(state.h4_braking);
    assert(!state.passed_b_or_a);
    assert(command.mode == gugah::MOTION_COMMAND_SPEED);
    assert(command.left_rpm == 120);
    assert(command.right_rpm == 120);
    assert(state.h4_commanded_accel_mm_s2 < -200);

    input.now_ms = 3333U;
    imu.received_ms = input.now_ms;
    chassis.received_ms = input.now_ms;
    command = gugah::Course_Update(&state, &input, &config);
    assert(command.left_rpm == 101);
    assert(command.right_rpm == 101);

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
    assert(state.h4_commanded_accel_mm_s2 < -200);
    assert(state.passed_b_or_a);
    assert(state.pass_ms == 5100U);

    input.now_ms = 5434U;
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

    input.now_ms = 3000U;
    const int32_t straight_count =
        CountsForMillimeters(1100, config);
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
        CountsForMillimeters(1100, config);
    chassis = Chassis(
        input.now_ms, straight_count, straight_count, 95);
    input.chassis = &chassis;

    gugah::MotionCommand command =
        gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);

    /* The computed preparation point is about 1231 mm: 219 mm of ideal
     * braking distance plus a 50 mm settling margin before B at 1500 mm. */
    input.now_ms += 8U;
    const int32_t curve_approach_count =
        CountsForMillimeters(1240, config);
    chassis = Chassis(input.now_ms,
                      curve_approach_count,
                      curve_approach_count, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.h5_curve_mode);
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

    /* C is at 1500 + 1571 = 3071 mm; recovery starts by mileage even if
     * the line is already perfectly centred. */
    input.now_ms += 8U;
    const int32_t after_c_count =
        CountsForMillimeters(3071, config);
    chassis = Chassis(input.now_ms, after_c_count, after_c_count, 63);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);
    assert(state.h5_commanded_accel_mm_s2 > 100);
    assert(state.h5_commanded_accel_mm_s2 < 200);

    const int32_t second_straight_count =
        CountsForMillimeters(3300, config);
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

    input.now_ms += 8U;
    const int32_t before_second_curve =
        CountsForMillimeters(4290, config);
    chassis = Chassis(input.now_ms,
                      before_second_curve,
                      before_second_curve, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(!state.h5_curve_mode);

    input.now_ms += 8U;
    const int32_t second_curve_approach =
        CountsForMillimeters(4320, config);
    chassis = Chassis(input.now_ms,
                      second_curve_approach,
                      second_curve_approach, 95);
    input.chassis = &chassis;
    command = gugah::Course_Update(&state, &input, &config);
    assert(state.h5_curve_mode);
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

void TestH5CourseAccelerationReachesBallFeedforward()
{
    gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_5;
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

    /* H5's launch, curve-exit, curve-entry and final-stop paths all write
     * the same signed course acceleration field consumed by the 2 ms ball
     * loop.  Exercise both polarities at that integration boundary. */
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

void TestH4ReportsEightSecondTimeout()
{
    const gugah::HConfig config = DefaultConfig();
    gugah::HAppState state = {};
    gugah::HApp_Init(&state);
    state.selected_problem = gugah::H_PROBLEM_4;
    drivers::GrayscaleProcessedData line = ValidLine(0);
    gugah::HAppInput input = {};
    input.now_ms = 100U;
    input.line = &line;
    input.line_frame_new = true;
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

    /* A physical endpoint fault is intentionally not auto-recovered. */
    state.ball.mode = gugah::BALL_FAILED;
    state.ball.result = gugah::BALL_RESULT_ENDPOINT;
    (void)gugah::HApp_UpdateBall2ms(&state, &input, &config);
    assert(state.ball.mode == gugah::BALL_FAILED);
    assert(state.ball.result == gugah::BALL_RESULT_ENDPOINT);
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
    TestBallDampingWorksAcrossFullTravel();
    TestBallDampingWorksNearTarget();
    TestH3CentersFromArbitraryInitialPosition();
    TestH3HasNoVisionAgeTimeout();
    TestH3ReportsDmTimeoutSeparately();
    TestH3ContinuesBeyondFiveSeconds();
    TestH3StageStartFailureIsReportedImmediately();
    TestH4TimeFreezesAtB();
    TestH4SoftLaunchAndHeadingLimit();
    TestH4SoftBrakeBeginsBeforeB();
    TestH5SoftLaunchKeepsCruiseBeforeFinish();
    TestH5CurveSpeedUsesOdometryAndRamps();
    TestH5IgnoresAThenSoftStopsAtOdometry();
    TestH5OdometryThresholdIsLapPlus50Mm();
    TestH4StartsBallAndCourseTogether();
    TestH5CourseAccelerationReachesBallFeedforward();
    TestH4ReportsEightSecondTimeout();
    TestPassHoldRecoversTransientDeviceGap();
    TestH6Buttons();
    puts("gugaH core tests passed");
    return 0;
}
