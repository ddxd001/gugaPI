#ifndef GUGAH_CONFIG_H_CONFIG_H_
#define GUGAH_CONFIG_H_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/grayscale/grayscale_processing.h"

namespace gugah {

static const uint32_t H_CONFIG_MAGIC = 0x48475547UL; /* "GUGH" */
static const uint16_t H_CONFIG_SCHEMA_VERSION = 8U;
static const uint16_t H_CONFIG_FRAM_ADDRESS = 0x0000U;

struct HConfig {
    drivers::GrayscaleCalibration grayscale;

    int32_t line_kp_milli;
    int32_t line_kd_milli;
    int16_t line_max_correction_rpm;
    uint16_t line_correction_slew_rpm_s;
    uint16_t line_lost_grace_ms;

    uint32_t wheel_radius_um;
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint8_t motor_output_invert_flags;
    uint8_t motor_encoder_invert_flags;
    uint8_t motor_speed_kp_q4_4;
    uint8_t motor_speed_ki_q4_4;
    uint8_t motor_speed_kd_q4_4;
    uint8_t motor_speed_max_duty;
    uint8_t motor_speed_min_duty;
    uint8_t motor_position_kp_q4_4;
    uint8_t motor_position_ki_q4_4;
    uint8_t motor_position_kd_q4_4;
    uint16_t motor_speed_accel_rpm_s;
    uint16_t motor_speed_decel_rpm_s;
    uint16_t motor_position_max_rpm;
    uint16_t motor_position_tolerance_counts;
    uint16_t cruise_rpm;
    uint16_t approach_rpm;
    uint16_t h4_cruise_rpm;
    uint16_t h5_cruise_rpm;
    uint16_t h5_approach_rpm;
    uint16_t h6_cruise_rpm;
    uint16_t h6_approach_rpm;
    uint16_t lap_distance_mm;
    uint16_t finish_gate_mm;
    uint16_t approach_start_mm;
    uint16_t h5_finish_gate_mm;
    uint16_t h5_approach_start_mm;
    uint16_t h6_finish_gate_mm;
    uint16_t h6_approach_start_mm;
    uint16_t h4_b_distance_mm;
    uint16_t h4_stop_distance_mm;
    int16_t sensor_to_reference_mm;

    int16_t ball_kp_mdeg_per_0p1mm;
    /* Shallow bowl curvature, mdeg of local slope per mm from centre. */
    int16_t ball_kd_mdeg_per_0p1mm_s;
    int16_t ball_ki_mdeg_per_0p1mm_s;
    int16_t ball_pitch_gain_permille;
    int16_t ball_accel_ff_mdeg_per_mm_s2;
    int16_t ball_max_angle_mdeg;
    int16_t ball_degraded_angle_mdeg;
    int32_t ball_angle_slew_mdeg_s;
    int16_t ball_position_tolerance_0p1mm;
    int16_t ball_velocity_tolerance_0p1mm_s;
    uint16_t ball_settle_ms;
    int16_t beam_angle_mdeg[5];
    int16_t dm_position_mrad[5];

    uint16_t vision_min_confidence;
    uint8_t vision_position_invert;
    uint8_t reserved;

    /* Model-based rod-ball controller.  Legacy ball_kp stores breakaway;
     * ball_kd/ball_ki and the planner fields remain for FRAM compatibility. */
    uint16_t ball_model_roll_gain_permille;
    uint16_t ball_model_response_ms;
    uint16_t ball_model_plan_accel_0p1mm_s2;
    uint16_t ball_model_max_velocity_0p1mm_s;
    uint16_t ball_observer_alpha_permille;
    uint16_t ball_observer_beta_permille;

    /* Full-travel ball position PID.  The integral path is a slow, gated
     * static-error trim rather than an unrestricted conventional integral. */
    int16_t ball_pid_kp_mdeg_per_mm;
    int16_t ball_pid_ki_mdeg_per_mm_s;
    int16_t ball_pid_kd_mdeg_per_mm_s;
    int16_t ball_pid_integral_limit_mdeg;
    /* Legacy linear-curvature origin retained for schema-5 migration. */
    int16_t ball_curve_origin_0p1mm;

    /* Measured beam angle which statically holds the ball at each position.
     * Positions are strictly increasing; angles may be non-monotonic to
     * represent a slightly warped beam. */
    int16_t ball_hold_position_0p1mm[5];
    int16_t ball_hold_angle_mdeg[5];
};

struct HConfigRecord {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t payload_length;
    HConfig payload;
    uint32_t payload_crc32;
};

typedef bool (*HConfigReadFn)(uint16_t address,
                              uint8_t *data,
                              uint16_t length);
typedef bool (*HConfigWriteFn)(uint16_t address,
                               const uint8_t *data,
                               uint16_t length);

void HConfig_Defaults(HConfig *config);
bool HConfig_Validate(const HConfig *config);
uint32_t HConfig_Crc32(const uint8_t *data, uint16_t length);
void HConfig_BuildRecord(const HConfig *config, HConfigRecord *record);
bool HConfig_ParseRecord(const HConfigRecord *record, HConfig *config);
bool HConfig_Load(HConfig *config, HConfigReadFn read_fn);
bool HConfig_Save(const HConfig *config,
                  HConfigWriteFn write_fn,
                  HConfigReadFn read_fn);

} /* namespace gugah */

#endif /* GUGAH_CONFIG_H_CONFIG_H_ */
