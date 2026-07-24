#ifndef FOC_CONFIG_H_
#define FOC_CONFIG_H_

/*
 * Board, motor, sensor, and controller parameters that affect FOC behavior.
 * Keep protocol constants and peripheral register addresses in their owning
 * modules; change motion-control values here and re-run the documented tests.
 */

/* Motor and rotor sensor. */
#define FOC_CFG_MOTOR_POLE_PAIRS                 (14U)
#define FOC_CFG_ENCODER_COUNTS_PER_REVOLUTION    (16384U)
#define FOC_CFG_ENCODER_I2C_ADDRESS              (0x40U)
#define FOC_CFG_ENCODER_FAST_PERIOD_TICKS        (20U)
#define FOC_CFG_ENCODER_FAST_TIMEOUT_TICKS       (40U)

/* PWM and control timing at a 32 MHz timer clock. */
#define FOC_CFG_CURRENT_LOOP_HZ                   (20000U)
#define FOC_CFG_PWM_PERIOD_COUNTS                 (1600U)
#define FOC_CFG_PWM_MAX_PHASE_DELTA               (200)
#define FOC_CFG_ALIGNMENT_ALPHA_VOLTAGE           (64)

/* Current-sense conversion: 20 mOhm shunts, DRV8323RS CSA at 20 V/V. */
#define FOC_CFG_ADC_REFERENCE_MV                  (3300U)
#define FOC_CFG_ADC_FULL_SCALE_COUNTS             (4095U)
#define FOC_CFG_CURRENT_SHUNT_MILLIOHMS           (20U)
#define FOC_CFG_CURRENT_CSA_GAIN                  (20U)
#define FOC_CFG_CURRENT_CALIBRATION_SAMPLES       (64U)

/* DRV8323RS conservative bring-up register payloads. */
#define FOC_CFG_DRV_GATE_DRIVE_HS                 (0x0322U)
#define FOC_CFG_DRV_GATE_DRIVE_LS                 (0x0122U)
#define FOC_CFG_DRV_OCP_CONTROL                   (0x0119U)
#define FOC_CFG_DRV_CSA_CONTROL                   (0x0283U)

/* Conservative commissioning current limits, expressed in ADC counts. */
#define FOC_CFG_IQ_REFERENCE_LIMIT_COUNTS         (124)
#define FOC_CFG_HARD_CURRENT_LIMIT_COUNTS         (298)
#define FOC_CFG_HARD_CURRENT_TRIP_SAMPLES         (3U)

/* Current PI: Q8 coefficients and joint d/q PWM-delta vector limit. */
#define FOC_CFG_CURRENT_KP_Q8                     (384)
#define FOC_CFG_CURRENT_KI_Q8                     (1)
#define FOC_CFG_CURRENT_VOLTAGE_LIMIT             (160)

/* Speed command and estimator. */
#define FOC_CFG_DEFAULT_TARGET_RPM                (25)
#define FOC_CFG_TARGET_RPM_STEP                   (25)
#define FOC_CFG_MAX_TARGET_RPM                    (400)
#define FOC_CFG_SPEED_WINDOW_TICKS                (200U)
#define FOC_CFG_SPEED_INTEGRAL_SHIFT              (6)
#define FOC_CFG_SPEED_MEASUREMENT_FILTER_SHIFT    (3U)

/* Encoder freshness and electrical-angle prediction. */
#define FOC_CFG_ENCODER_STALE_TICKS               (100U)
#define FOC_CFG_ELECTRICAL_PREDICTION_MAX_TICKS  (40U)
#define FOC_CFG_ELECTRICAL_VELOCITY_FILTER_SHIFT (3U)

/* Single-turn position controller. */
#define FOC_CFG_POSITION_STEP_DEGREES             (10)
#define FOC_CFG_POSITION_COUNTS_PER_RPM           (10)
#define FOC_CFG_POSITION_SPEED_LIMIT_RPM          (50)
#define FOC_CFG_POSITION_DIRECT_ZONE_COUNTS       (128)
#define FOC_CFG_POSITION_DIRECT_ERROR_DIVISOR     (4)
#define FOC_CFG_POSITION_DIRECT_DAMPING_PER_RPM   (2)
#define FOC_CFG_POSITION_DIRECT_IQ_LIMIT_COUNTS   (32)
#define FOC_CFG_POSITION_INTEGRAL_RELEASE_RPM     (5)

#endif /* FOC_CONFIG_H_ */
