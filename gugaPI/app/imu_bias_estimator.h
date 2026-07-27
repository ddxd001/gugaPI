#ifndef APP_IMU_BIAS_ESTIMATOR_H_
#define APP_IMU_BIAS_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>

namespace app {

static const uint32_t IMU_BIAS_WINDOW_MS = 2000U;
static const uint32_t IMU_BIAS_REQUIRED_SAMPLES = 400U;
static const uint32_t IMU_BIAS_COOLDOWN_MS = 5000U;
static const int32_t IMU_BIAS_RUNTIME_LIMIT_MDPS = 5000;

enum ImuBiasEstimatorState : uint8_t {
    IMU_BIAS_DISABLED = 0U,
    IMU_BIAS_WAITING,
    IMU_BIAS_COLLECTING,
    IMU_BIAS_COOLDOWN
};

enum ImuBiasRejectReason : uint8_t {
    IMU_BIAS_REJECT_NONE = 0U,
    IMU_BIAS_REJECT_DISABLED,
    IMU_BIAS_REJECT_SENSOR_INVALID,
    IMU_BIAS_REJECT_CHASSIS_UNAVAILABLE,
    IMU_BIAS_REJECT_FEEDBACK_STALE,
    IMU_BIAS_REJECT_TARGET_ACTIVE,
    IMU_BIAS_REJECT_WHEELS_MOVING,
    IMU_BIAS_REJECT_CONTROL_ACTIVE,
    IMU_BIAS_REJECT_ACCELERATION,
    IMU_BIAS_REJECT_GYRO_MOTION,
    IMU_BIAS_REJECT_NOISE,
    IMU_BIAS_REJECT_COOLDOWN,
    IMU_BIAS_REJECT_CONFIG_CHANGED
};

struct ImuBiasEstimatorStatus {
    ImuBiasEstimatorState state;
    ImuBiasRejectReason reject_reason;
    bool auto_enabled;
    bool manual_requested;
    bool estimate_valid;
    bool freeze_yaw;
    int32_t runtime_bias_z_mdps;
    int32_t last_mean_z_mdps;
    uint32_t last_stddev_z_mdps;
    int32_t last_adjustment_z_mdps;
    uint32_t sample_count;
    uint32_t collection_elapsed_ms;
    uint32_t last_update_ms;
    uint32_t accepted_windows;
    uint32_t rejected_windows;
};

void ImuBiasEstimator_Init(void);
void ImuBiasEstimator_SetAutoEnabled(bool enabled);
void ImuBiasEstimator_RequestCalibration(void);
void ImuBiasEstimator_ResetRuntime(void);
void ImuBiasEstimator_CommitRuntimeToFixed(void);
void ImuBiasEstimator_NotifyBlocked(ImuBiasRejectReason reason,
                                    uint32_t now_ms);
void ImuBiasEstimator_Update(const int32_t accel_mg[3],
                             const int32_t corrected_gyro_mdps[3],
                             uint32_t now_ms,
                             bool learning_allowed,
                             ImuBiasRejectReason blocked_reason);
int32_t ImuBiasEstimator_GetRuntimeBiasZMdps(void);
bool ImuBiasEstimator_ShouldFreezeYaw(void);
const ImuBiasEstimatorStatus *ImuBiasEstimator_GetStatus(void);
const char *ImuBiasEstimator_StateText(ImuBiasEstimatorState state);
const char *ImuBiasEstimator_RejectReasonText(ImuBiasRejectReason reason);

} /* namespace app */

#endif /* APP_IMU_BIAS_ESTIMATOR_H_ */
