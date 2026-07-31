#include "config/h_config.h"

#include <stddef.h>
#include <string.h>

namespace gugah {
namespace {

static_assert(sizeof(HConfigRecord) <= 256U,
              "HConfig record must fit one bounded FRAM burst");
static const uint16_t kSchema2PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, ball_model_roll_gain_permille));
static const uint16_t kSchema3PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, ball_pid_kp_mdeg_per_mm));
static const uint16_t kSchema4PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, ball_curve_origin_0p1mm));
static const uint16_t kSchema5PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, ball_hold_position_0p1mm));
static const uint16_t kSchema6PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h4_launch_ramp_rpm_s));
static const uint16_t kSchema9PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h4_stop_ramp_rpm_s));
static const uint16_t kSchema10PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h4_brake_distance_mm));
static const uint16_t kSchema12PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h4_heading_kp));
static const uint16_t kSchema13PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, ball_chassis_ff_permille));
static const uint16_t kSchema14PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h5_launch_ramp_rpm_s));
static const uint16_t kSchema15PayloadLength =
    static_cast<uint16_t>(offsetof(
        HConfig, h5_brake_distance_mm));

void SetMeasuredDmMapping(HConfig *config)
{
    static const int16_t positions[5] = {
        230, -150, -570, -1070, -1490
    };
    for (uint8_t i = 0U; i < 5U; i++) {
        config->dm_position_mrad[i] = positions[i];
    }
}

bool MappingValid(const HConfig *config)
{
    const bool increasing =
        config->dm_position_mrad[1] > config->dm_position_mrad[0];
    if (config->dm_position_mrad[1] == config->dm_position_mrad[0]) {
        return false;
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        if (config->beam_angle_mdeg[i] <=
            config->beam_angle_mdeg[i - 1U]) {
            return false;
        }
        if (increasing) {
            if (config->dm_position_mrad[i] <=
                config->dm_position_mrad[i - 1U]) {
                return false;
            }
        } else if (config->dm_position_mrad[i] >=
                   config->dm_position_mrad[i - 1U]) {
            return false;
        }
    }
    for (uint8_t i = 0U; i < 5U; i++) {
        if ((config->beam_angle_mdeg[i] < -15000) ||
            (config->beam_angle_mdeg[i] > 15000) ||
            (config->dm_position_mrad[i] < -12500) ||
            (config->dm_position_mrad[i] > 12500)) {
            return false;
        }
    }
    return true;
}

bool HoldTableValid(const HConfig *config)
{
    for (uint8_t i = 0U; i < 5U; i++) {
        if ((config->ball_hold_position_0p1mm[i] < -1150) ||
            (config->ball_hold_position_0p1mm[i] > 1150) ||
            (config->ball_hold_angle_mdeg[i] < -6000) ||
            (config->ball_hold_angle_mdeg[i] > 6000)) {
            return false;
        }
        if ((i != 0U) &&
            (config->ball_hold_position_0p1mm[i] <=
             config->ball_hold_position_0p1mm[i - 1U])) {
            return false;
        }
    }
    return true;
}

} /* namespace */

void HConfig_Defaults(HConfig *config)
{
    if (config == 0) {
        return;
    }
    *config = {};

    /* Commissioned values from the unchanged eight-channel sensor board. */
    const uint16_t white[drivers::GRAYSCALE_CHANNEL_COUNT] = {
        3253U, 3217U, 3189U, 3316U, 3151U, 3011U, 2802U, 3188U
    };
    const uint16_t black[drivers::GRAYSCALE_CHANNEL_COUNT] = {
        1010U, 934U, 737U, 2010U, 1548U, 1347U, 753U, 1362U
    };
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        config->grayscale.white[i] = white[i];
        config->grayscale.black[i] = black[i];
    }
    config->grayscale.threshold = 500U;
    config->grayscale.hysteresis = 300U;
    config->grayscale.position_floor = 100U;
    config->grayscale.min_line_strength = 600U;
    config->grayscale.track_mask = drivers::GRAYSCALE_ALL_CHANNEL_MASK;

    config->line_kp_milli = 40;
    config->line_kd_milli = 8;
    config->line_max_correction_rpm = 100;
    config->line_correction_slew_rpm_s = 1200U;
    config->line_lost_grace_ms = 150U;

    config->wheel_radius_um = 33050U;
    config->left_counts_per_rev = 1456U;
    config->right_counts_per_rev = 1456U;
    /*
     * Compatible commissioned values imported from the 2026-07-30 gugaPI
     * parameter export. These fields use the unchanged MotorDriver register
     * units and are pushed to the slave during Chassis_Init().
     */
    config->motor_output_invert_flags = 3U;
    config->motor_encoder_invert_flags = 1U;
    config->motor_speed_kp_q4_4 = 2U;
    config->motor_speed_ki_q4_4 = 2U;
    config->motor_speed_kd_q4_4 = 0U;
    config->motor_speed_max_duty = 60U;
    config->motor_speed_min_duty = 4U;
    config->motor_position_kp_q4_4 = 15U;
    config->motor_position_ki_q4_4 = 0U;
    config->motor_position_kd_q4_4 = 0U;
    config->motor_speed_accel_rpm_s = 1500U;
    config->motor_speed_decel_rpm_s = 2000U;
    config->motor_position_max_rpm = 40U;
    config->motor_position_tolerance_counts = 3U;
    config->cruise_rpm = 110U;
    config->approach_rpm = 55U;
    config->h4_cruise_rpm = 120U;
    config->h5_cruise_rpm = 100U;
    config->h5_approach_rpm = 50U;
    config->h6_cruise_rpm = 90U;
    config->h6_approach_rpm = 45U;
    config->lap_distance_mm = 6142U;
    config->finish_gate_mm = 5200U;
    config->approach_start_mm = 5250U;
    config->h5_finish_gate_mm = 5200U;
    config->h5_approach_start_mm = 5250U;
    config->h6_finish_gate_mm = 5200U;
    config->h6_approach_start_mm = 5250U;
    config->h4_b_distance_mm = 1500U;
    config->h4_stop_distance_mm = 1700U;
    config->sensor_to_reference_mm = 0;

    /* Legacy field reused as the measured static breakaway angle. */
    config->ball_kp_mdeg_per_0p1mm = 2300;
    /* Measured rail ends are slightly higher than the centre. */
    config->ball_kd_mdeg_per_0p1mm_s = 8;
    config->ball_ki_mdeg_per_0p1mm_s = 1;
    config->ball_pitch_gain_permille = 0;
    /* Reused as the identified rolling/static-friction angle. */
    config->ball_accel_ff_mdeg_per_mm_s2 = 0;
    config->ball_max_angle_mdeg = 6000;
    config->ball_degraded_angle_mdeg = 2500;
    config->ball_angle_slew_mdeg_s = 30000;
    config->ball_position_tolerance_0p1mm = 100;
    config->ball_velocity_tolerance_0p1mm_s = 100;
    config->ball_settle_ms = 200U;
    const int16_t angles[5] = { -8000, -4000, 0, 4000, 8000 };
    /*
     * On the competition mechanism a positive DM position lowers the
     * positive-coordinate end of the beam.  Therefore beam angle and motor
     * position have opposite signs.  Keeping this correction in the
     * five-point mechanism map leaves the ball PID and vision coordinates
     * physically meaningful.
     */
    /* 2026-08-01 linkage calibration.  The physical horizontal beam is
     * about -570 mrad and the linkage ratio is asymmetric about level. */
    const int16_t positions[5] = { 230, -150, -570, -1070, -1490 };
    for (uint8_t i = 0U; i < 5U; i++) {
        config->beam_angle_mdeg[i] = angles[i];
        config->dm_position_mrad[i] = positions[i];
    }

    /* Bench MaixCAM frames are stable around 0.32 confidence.  CRC, flags and
     * the two-frame startup lock provide the communication safety checks. */
    config->vision_min_confidence = 100U;
    config->vision_position_invert = 1U;

    /* Solid rolling sphere: a = 5/7 * g * sin(theta). */
    config->ball_model_roll_gain_permille = 714U;
    config->ball_model_response_ms = 800U;
    config->ball_model_plan_accel_0p1mm_s2 = 100U;
    config->ball_model_max_velocity_0p1mm_s = 200U;
    config->ball_observer_alpha_permille = 500U;
    config->ball_observer_beta_permille = 80U;
    config->ball_pid_kp_mdeg_per_mm = 40;
    config->ball_pid_ki_mdeg_per_mm_s = 20;
    config->ball_pid_kd_mdeg_per_mm_s = 20;
    config->ball_pid_integral_limit_mdeg = 1500;
    config->ball_curve_origin_0p1mm = 148;
    const int16_t hold_positions[5] = {
        -1000, -500, 0, 500, 1000
    };
    const int16_t hold_angles[5] = {
        -918, -518, -118, 282, 682
    };
    for (uint8_t i = 0U; i < 5U; i++) {
        config->ball_hold_position_0p1mm[i] = hold_positions[i];
        config->ball_hold_angle_mdeg[i] = hold_angles[i];
    }
    config->h4_launch_ramp_rpm_s = 60U;
    config->h4_stop_ramp_rpm_s = 60U;
    config->h4_brake_distance_mm = 1100U;
    config->h4_heading_kp = 1000;
    config->h4_heading_max_correction_rpm = 30;
    config->imu_gyro_bias_z_mdps = 0;
    config->ball_chassis_ff_permille = 1000;
    config->h5_launch_ramp_rpm_s = 60U;
    config->h5_stop_ramp_rpm_s = 60U;
    config->h5_brake_distance_mm = 5840U;
}

bool HConfig_Validate(const HConfig *config)
{
    uint8_t calibration_faults = 0U;
    if ((config == 0) ||
        (drivers::Grayscale_ValidateCalibration(
             &config->grayscale, &calibration_faults) !=
         drivers::DRIVER_OK) ||
        (calibration_faults != 0U) ||
        (config->line_kp_milli < 0) ||
        (config->line_kp_milli > 2000) ||
        (config->line_kd_milli < 0) ||
        (config->line_kd_milli > 2000) ||
        (config->line_max_correction_rpm < 1) ||
        (config->line_max_correction_rpm > 500) ||
        (config->line_correction_slew_rpm_s < 10U) ||
        (config->line_lost_grace_ms > 1000U) ||
        (config->wheel_radius_um < 10000U) ||
        (config->wheel_radius_um > 100000U) ||
        (config->left_counts_per_rev < 100U) ||
        (config->left_counts_per_rev > 100000000U) ||
        (config->right_counts_per_rev < 100U) ||
        (config->right_counts_per_rev > 100000000U) ||
        (config->motor_output_invert_flags > 3U) ||
        (config->motor_encoder_invert_flags > 3U) ||
        (config->motor_speed_kp_q4_4 == 0U) ||
        (config->motor_speed_max_duty == 0U) ||
        (config->motor_speed_max_duty > 100U) ||
        (config->motor_speed_min_duty >
         config->motor_speed_max_duty) ||
        (config->motor_speed_accel_rpm_s == 0U) ||
        (config->motor_speed_decel_rpm_s == 0U) ||
        (config->motor_position_kp_q4_4 == 0U) ||
        (config->motor_position_max_rpm == 0U) ||
        (config->motor_position_max_rpm > 1000U) ||
        (config->motor_position_tolerance_counts == 0U) ||
        (config->motor_position_tolerance_counts > 10000U) ||
        (config->cruise_rpm == 0U) ||
        (config->approach_rpm == 0U) ||
        (config->approach_rpm > config->cruise_rpm) ||
        (config->h4_cruise_rpm == 0U) ||
        (config->h5_cruise_rpm == 0U) ||
        (config->h5_approach_rpm == 0U) ||
        (config->h5_approach_rpm > config->h5_cruise_rpm) ||
        (config->h6_cruise_rpm == 0U) ||
        (config->h6_approach_rpm == 0U) ||
        (config->h6_approach_rpm > config->h6_cruise_rpm) ||
        (config->lap_distance_mm < 5000U) ||
        (config->lap_distance_mm > 8000U) ||
        (config->finish_gate_mm >= config->lap_distance_mm) ||
        (config->approach_start_mm >= config->lap_distance_mm) ||
        (config->h5_finish_gate_mm >= config->lap_distance_mm) ||
        (config->h5_approach_start_mm >= config->lap_distance_mm) ||
        (config->h6_finish_gate_mm >= config->lap_distance_mm) ||
        (config->h6_approach_start_mm >= config->lap_distance_mm) ||
        (config->h4_b_distance_mm < 1000U) ||
        (config->h4_brake_distance_mm < 100U) ||
        (config->h4_brake_distance_mm >= config->h4_b_distance_mm) ||
        (config->h4_stop_distance_mm <= config->h4_b_distance_mm) ||
        (config->ball_kp_mdeg_per_0p1mm < 0) ||
        (config->ball_kp_mdeg_per_0p1mm > 5000) ||
        (config->ball_kd_mdeg_per_0p1mm_s < 1) ||
        (config->ball_kd_mdeg_per_0p1mm_s > 1000) ||
        (config->ball_ki_mdeg_per_0p1mm_s < 0) ||
        (config->ball_ki_mdeg_per_0p1mm_s > 1000) ||
        (config->ball_pitch_gain_permille < 0) ||
        (config->ball_pitch_gain_permille > 2000) ||
        (config->ball_accel_ff_mdeg_per_mm_s2 < 0) ||
        (config->ball_accel_ff_mdeg_per_mm_s2 > 3000) ||
        (config->ball_max_angle_mdeg < 100) ||
        (config->ball_max_angle_mdeg > 15000) ||
        (config->ball_degraded_angle_mdeg < 100) ||
        (config->ball_degraded_angle_mdeg >
         config->ball_max_angle_mdeg) ||
        (config->ball_angle_slew_mdeg_s < 100) ||
        (config->ball_position_tolerance_0p1mm < 1) ||
        (config->ball_position_tolerance_0p1mm > 100) ||
        (config->ball_velocity_tolerance_0p1mm_s < 1) ||
        (config->ball_settle_ms < 10U) ||
        (config->ball_settle_ms > 2000U) ||
        (config->vision_min_confidence > 1000U) ||
        (config->vision_position_invert > 1U) ||
        (config->ball_model_roll_gain_permille < 100U) ||
        (config->ball_model_roll_gain_permille > 1500U) ||
        (config->ball_model_response_ms < 50U) ||
        (config->ball_model_response_ms > 2000U) ||
        (config->ball_model_plan_accel_0p1mm_s2 < 10U) ||
        (config->ball_model_plan_accel_0p1mm_s2 > 10000U) ||
        (config->ball_model_max_velocity_0p1mm_s < 10U) ||
        (config->ball_model_max_velocity_0p1mm_s > 2000U) ||
        (config->ball_observer_alpha_permille < 10U) ||
        (config->ball_observer_alpha_permille > 1000U) ||
        (config->ball_observer_beta_permille < 1U) ||
        (config->ball_observer_beta_permille > 1000U) ||
        (config->ball_pid_kp_mdeg_per_mm < 0) ||
        (config->ball_pid_kp_mdeg_per_mm > 1000) ||
        (config->ball_pid_ki_mdeg_per_mm_s < 0) ||
        (config->ball_pid_ki_mdeg_per_mm_s > 1000) ||
        (config->ball_pid_kd_mdeg_per_mm_s < 0) ||
        (config->ball_pid_kd_mdeg_per_mm_s > 1000) ||
        (config->ball_pid_integral_limit_mdeg < 0) ||
        (config->ball_pid_integral_limit_mdeg > 3000) ||
        (config->ball_curve_origin_0p1mm < -1000) ||
        (config->ball_curve_origin_0p1mm > 1000) ||
        (config->h4_launch_ramp_rpm_s < 10U) ||
        (config->h4_launch_ramp_rpm_s > 1000U) ||
        (config->h4_stop_ramp_rpm_s < 10U) ||
        (config->h4_stop_ramp_rpm_s > 2000U) ||
        (config->h4_heading_kp < 0) ||
        (config->h4_heading_kp > 100000) ||
        (config->h4_heading_max_correction_rpm < 0) ||
        (config->h4_heading_max_correction_rpm > 500) ||
        (config->imu_gyro_bias_z_mdps < -2000000) ||
        (config->imu_gyro_bias_z_mdps > 2000000) ||
        (config->ball_chassis_ff_permille < -2000) ||
        (config->ball_chassis_ff_permille > 2000) ||
        (config->h5_launch_ramp_rpm_s < 10U) ||
        (config->h5_launch_ramp_rpm_s > 1000U) ||
        (config->h5_stop_ramp_rpm_s < 10U) ||
        (config->h5_stop_ramp_rpm_s > 2000U) ||
        (config->h5_brake_distance_mm <= config->h5_finish_gate_mm) ||
        (config->h5_brake_distance_mm >= config->lap_distance_mm)) {
        return false;
    }
    return MappingValid(config) && HoldTableValid(config);
}

uint32_t HConfig_Crc32(const uint8_t *data, uint16_t length)
{
    if ((data == 0) && (length != 0U)) {
        return 0U;
    }
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 1U) != 0U)
                ? ((crc >> 1U) ^ 0xEDB88320UL)
                : (crc >> 1U);
        }
    }
    return ~crc;
}

void HConfig_BuildRecord(const HConfig *config, HConfigRecord *record)
{
    if ((config == 0) || (record == 0)) {
        return;
    }
    *record = {};
    record->magic = H_CONFIG_MAGIC;
    record->schema_version = H_CONFIG_SCHEMA_VERSION;
    record->payload_length = static_cast<uint16_t>(sizeof(HConfig));
    record->payload = *config;
    record->payload_crc32 = HConfig_Crc32(
        reinterpret_cast<const uint8_t *>(&record->payload),
        record->payload_length);
}

bool HConfig_ParseRecord(const HConfigRecord *record, HConfig *config)
{
    if ((record == 0) || (config == 0) ||
        (record->magic != H_CONFIG_MAGIC)) {
        return false;
    }
    const bool schema2 = record->schema_version == 2U;
    const bool schema3 = record->schema_version == 3U;
    const bool schema4 = record->schema_version == 4U;
    const bool schema5 = record->schema_version == 5U;
    const bool schema6 = record->schema_version == 6U;
    const bool schema7 = record->schema_version == 7U;
    const bool schema9 = record->schema_version == 9U;
    const bool schema10 = record->schema_version == 10U;
    const bool schema11 = record->schema_version == 11U;
    const bool schema12 = record->schema_version == 12U;
    const bool schema13 = record->schema_version == 13U;
    const bool schema14 = record->schema_version == 14U;
    const bool schema15 = record->schema_version == 15U;
    if ((record->schema_version == H_CONFIG_SCHEMA_VERSION) &&
        (record->payload_length ==
         static_cast<uint16_t>(sizeof(HConfig))) &&
        (record->payload_crc32 == HConfig_Crc32(
             reinterpret_cast<const uint8_t *>(&record->payload),
             record->payload_length))) {
        *config = record->payload;
    } else if (((record->schema_version == 2U) &&
                (record->payload_length == kSchema2PayloadLength)) ||
               ((record->schema_version == 3U) &&
                (record->payload_length == kSchema3PayloadLength)) ||
               ((record->schema_version == 4U) &&
                (record->payload_length == kSchema4PayloadLength)) ||
               ((record->schema_version == 5U) &&
                (record->payload_length == kSchema5PayloadLength)) ||
               ((record->schema_version == 6U) &&
                (record->payload_length == kSchema6PayloadLength)) ||
               ((record->schema_version == 7U) &&
                (record->payload_length == kSchema6PayloadLength)) ||
               ((record->schema_version == 8U) &&
                (record->payload_length == kSchema6PayloadLength)) ||
               ((record->schema_version == 9U) &&
                (record->payload_length == kSchema9PayloadLength)) ||
               ((record->schema_version == 10U) &&
                (record->payload_length == kSchema10PayloadLength)) ||
               ((record->schema_version == 11U) &&
                (record->payload_length == kSchema10PayloadLength)) ||
               ((record->schema_version == 12U) &&
                (record->payload_length == kSchema12PayloadLength)) ||
               ((record->schema_version == 13U) &&
                (record->payload_length == kSchema13PayloadLength)) ||
               ((record->schema_version == 14U) &&
                (record->payload_length == kSchema14PayloadLength)) ||
               ((record->schema_version == 15U) &&
                (record->payload_length == kSchema15PayloadLength))) {
        uint16_t legacy_length = schema15
            ? kSchema15PayloadLength
            : (schema14 ? kSchema14PayloadLength
            : (schema13 ? kSchema13PayloadLength
            : (schema12 ? kSchema12PayloadLength
            : ((schema10 || schema11) ? kSchema10PayloadLength
            : (schema9 ? kSchema9PayloadLength
                       : kSchema6PayloadLength)))));
        if (schema2) {
            legacy_length = kSchema2PayloadLength;
        } else if (schema3) {
            legacy_length = kSchema3PayloadLength;
        } else if (schema4) {
            legacy_length = kSchema4PayloadLength;
        } else if (schema5) {
            legacy_length = kSchema5PayloadLength;
        }
        const uint8_t *payload =
            reinterpret_cast<const uint8_t *>(&record->payload);
        uint32_t stored_crc = 0U;
        memcpy(&stored_crc, payload + legacy_length,
               sizeof(stored_crc));
        if (stored_crc != HConfig_Crc32(
                payload, legacy_length)) {
            return false;
        }
        HConfig migrated = {};
        HConfig_Defaults(&migrated);
        memcpy(&migrated, payload, legacy_length);
        *config = migrated;
    } else {
        return false;
    }
    if (schema2) {
        config->ball_kp_mdeg_per_0p1mm = 2300;
        config->ball_velocity_tolerance_0p1mm_s = 100;
        config->ball_pitch_gain_permille = 0;
        config->ball_accel_ff_mdeg_per_mm_s2 = 0;
    }
    if (schema3) {
        /* Schema 3 is preserved verbatim; the residual PID and estimator
         * tuning use their conservative newer-schema defaults. */
        config->ball_pid_kp_mdeg_per_mm = 80;
        config->ball_pid_ki_mdeg_per_mm_s = 0;
        config->ball_pid_kd_mdeg_per_mm_s = 5;
        config->ball_pid_integral_limit_mdeg = 500;
        config->ball_observer_alpha_permille = 700U;
        config->ball_observer_beta_permille = 600U;
    }
    if (schema4) {
        config->ball_curve_origin_0p1mm = 148;
    }
    if (schema2 || schema3 || schema4 || schema5 || schema6) {
        /* Schema 6 changes beta from a direct differentiated-velocity blend
         * into a position-residual observer gain.  It also applies the
         * outer PD over the full travel, so old numerical gains are unsafe. */
        config->ball_observer_alpha_permille = 500U;
        config->ball_observer_beta_permille = 80U;
        config->ball_pid_kp_mdeg_per_mm = 40;
        config->ball_pid_ki_mdeg_per_mm_s = 20;
        config->ball_pid_kd_mdeg_per_mm_s = 20;
        config->ball_pid_integral_limit_mdeg = 1500;
    }
    if (schema7 &&
        (config->ball_pid_ki_mdeg_per_mm_s == 0) &&
        (config->ball_pid_integral_limit_mdeg == 0)) {
        /* Schema 7 deliberately disabled I while commissioning the model
         * observer.  Schema 8 enables a bounded low-speed trim so existing
         * cars receive the static-error fix without losing other tuning. */
        config->ball_pid_ki_mdeg_per_mm_s = 20;
        config->ball_pid_integral_limit_mdeg = 1500;
    }
    if ((record->schema_version <= 10U) &&
        (config->h4_stop_distance_mm == 1650U) &&
        (!schema10 ||
         (config->h4_stop_ramp_rpm_s == 180U))) {
        /* The first H4 soft-stop profile was still too abrupt on the car.
         * Migrate only its exact defaults so commissioned custom values are
         * preserved.  B timing remains at 1500 mm. */
        config->h4_stop_ramp_rpm_s = 60U;
        config->h4_stop_distance_mm = 2050U;
    }
    if ((record->schema_version <= 11U) &&
        (config->h4_stop_ramp_rpm_s == 60U) &&
        (config->h4_stop_distance_mm == 2050U)) {
        /* Schema 11 began braking only after B.  Start about 400 mm before B
         * and keep the post-B hard-stop boundary close to the target. */
        config->h4_stop_distance_mm = 1700U;
    }
    const bool legacy_pid =
        ((config->ball_kp_mdeg_per_0p1mm == 50) &&
         (config->ball_kd_mdeg_per_0p1mm_s == 0) &&
         (config->ball_ki_mdeg_per_0p1mm_s == 0)) ||
        ((config->ball_kp_mdeg_per_0p1mm == 10) &&
         (config->ball_kd_mdeg_per_0p1mm_s == 3) &&
         (config->ball_ki_mdeg_per_0p1mm_s == 0)) ||
        ((config->ball_kp_mdeg_per_0p1mm == 8) &&
         (config->ball_kd_mdeg_per_0p1mm_s == 6) &&
         (config->ball_ki_mdeg_per_0p1mm_s == 0)) ||
        ((config->ball_kp_mdeg_per_0p1mm == 8) &&
         (config->ball_kd_mdeg_per_0p1mm_s == 10) &&
         (config->ball_ki_mdeg_per_0p1mm_s == 1));
    if (legacy_pid) {
        config->ball_kp_mdeg_per_0p1mm = 2300;
        config->ball_kd_mdeg_per_0p1mm_s = 8;
        config->ball_ki_mdeg_per_0p1mm_s = 1;
    }
    if (config->ball_position_tolerance_0p1mm == 70) {
        config->ball_position_tolerance_0p1mm = 100;
    }
    if (config->ball_velocity_tolerance_0p1mm_s == 30) {
        config->ball_velocity_tolerance_0p1mm_s = 100;
    }
    /* Migrate the first model-controller commissioning values in RAM. */
    if ((config->ball_kp_mdeg_per_0p1mm == 1700) ||
        (config->ball_kp_mdeg_per_0p1mm == 3000)) {
        config->ball_kp_mdeg_per_0p1mm = 2300;
    }
    if ((config->ball_kd_mdeg_per_0p1mm_s == 3) ||
        (config->ball_kd_mdeg_per_0p1mm_s == 5)) {
        config->ball_kd_mdeg_per_0p1mm_s = 8;
    }
    if ((config->ball_model_max_velocity_0p1mm_s == 80U) ||
        (config->ball_model_max_velocity_0p1mm_s == 250U)) {
        config->ball_model_max_velocity_0p1mm_s = 200U;
    }
    if ((config->ball_model_response_ms == 300U) ||
        (config->ball_model_response_ms == 220U)) {
        config->ball_model_response_ms = 800U;
    }
    if (config->ball_accel_ff_mdeg_per_mm_s2 == 650) {
        config->ball_accel_ff_mdeg_per_mm_s2 = 0;
    }
    if (config->ball_max_angle_mdeg == 8000) {
        config->ball_max_angle_mdeg = 6000;
    }
    if (config->ball_degraded_angle_mdeg == 3000) {
        config->ball_degraded_angle_mdeg = 2500;
    }
    if (config->ball_angle_slew_mdeg_s == 60000) {
        /* Undo the overly aggressive response experiment. */
        config->ball_angle_slew_mdeg_s = 30000;
    }
    if ((config->vision_min_confidence == 500U) ||
        (config->vision_min_confidence == 250U)) {
        /* Bench frames with a valid position and CRC can dip to about 170
         * confidence.  Keep the protocol flags as hard safety checks while
         * accepting these otherwise continuous measurements. */
        config->vision_min_confidence = 100U;
    }
    /*
     * Early gugaH records used an increasing DM map, which makes the actual
     * beam response run away from the ball.  Migrate that record in RAM while
     * preserving all of its commissioned grayscale, chassis and PID values.
     * A subsequent explicit `config save` persists the corrected map.
     */
    if (config->dm_position_mrad[4] > config->dm_position_mrad[0]) {
        for (uint8_t i = 0U; i < 2U; i++) {
            const int16_t old = config->dm_position_mrad[i];
            config->dm_position_mrad[i] =
                config->dm_position_mrad[4U - i];
            config->dm_position_mrad[4U - i] = old;
        }
        /* The temporary sensor-sign workaround belonged to the same faulty
         * mapping revision.  Restore the commissioned Maix coordinate sign. */
        config->vision_position_invert = 1U;
    }
    const bool old_zero_centered_map =
        (config->dm_position_mrad[0] == 1000) &&
        (config->dm_position_mrad[1] == 500) &&
        (config->dm_position_mrad[2] == 0) &&
        (config->dm_position_mrad[3] == -500) &&
        (config->dm_position_mrad[4] == -1000);
    if (old_zero_centered_map) {
        for (uint8_t i = 0U; i < 5U; i++) {
            config->dm_position_mrad[i] = static_cast<int16_t>(
                config->dm_position_mrad[i] - 500);
        }
    }
    const bool old_linear_map =
        (config->dm_position_mrad[0] == 500) &&
        (config->dm_position_mrad[1] == 0) &&
        (config->dm_position_mrad[2] == -500) &&
        (config->dm_position_mrad[3] == -1000) &&
        (config->dm_position_mrad[4] == -1500);
    if (schema2 || schema3 || schema4 || schema5 || schema6 ||
        old_linear_map) {
        SetMeasuredDmMapping(config);
    }
    return HConfig_Validate(config);
}

bool HConfig_Load(HConfig *config, HConfigReadFn read_fn)
{
    if (config == 0) {
        return false;
    }
    HConfigRecord record = {};
    if ((read_fn == 0) ||
        !read_fn(H_CONFIG_FRAM_ADDRESS,
                 reinterpret_cast<uint8_t *>(&record),
                 static_cast<uint16_t>(sizeof(record))) ||
        !HConfig_ParseRecord(&record, config)) {
        HConfig_Defaults(config);
        return false;
    }
    return true;
}

bool HConfig_Save(const HConfig *config,
                  HConfigWriteFn write_fn,
                  HConfigReadFn read_fn)
{
    if (!HConfig_Validate(config) || (write_fn == 0) || (read_fn == 0)) {
        return false;
    }
    HConfigRecord record = {};
    HConfig_BuildRecord(config, &record);
    if (!write_fn(H_CONFIG_FRAM_ADDRESS,
                  reinterpret_cast<const uint8_t *>(&record),
                  static_cast<uint16_t>(sizeof(record)))) {
        return false;
    }
    HConfigRecord verify = {};
    return read_fn(H_CONFIG_FRAM_ADDRESS,
                   reinterpret_cast<uint8_t *>(&verify),
                   static_cast<uint16_t>(sizeof(verify))) &&
           (memcmp(&record, &verify, sizeof(record)) == 0);
}

} /* namespace gugah */
