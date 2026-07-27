#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app/imu_bias_estimator.h"

namespace {

void FeedWindow(int32_t gyro_z_mdps,
                uint32_t start_ms,
                bool alternating_noise)
{
    const int32_t accel[3] = { 0, 0, 1000 };
    for (uint32_t i = 0U; i <= app::IMU_BIAS_REQUIRED_SAMPLES; i++) {
        int32_t gyro[3] = { 0, 0, gyro_z_mdps };
        if (alternating_noise) {
            gyro[2] = ((i & 1U) == 0U) ? gyro_z_mdps : -gyro_z_mdps;
        }
        app::ImuBiasEstimator_Update(accel,
                                     gyro,
                                     start_ms + (i * 5U),
                                     true,
                                     app::IMU_BIAS_REJECT_NONE);
    }
}

} /* namespace */

int main()
{
    app::ImuBiasEstimator_Init();
    const app::ImuBiasEstimatorStatus *status =
        app::ImuBiasEstimator_GetStatus();
    assert(status->auto_enabled);
    assert(status->state == app::IMU_BIAS_WAITING);

    /* Boot acquisition applies the complete measured residual. This is the
     * sign-regression test: a -300 mdps residual must produce -300 mdps bias. */
    FeedWindow(-300, 0U, false);
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->estimate_valid);
    assert(status->runtime_bias_z_mdps == -300);
    assert(status->accepted_windows == 1U);
    assert(status->state == app::IMU_BIAS_COOLDOWN);

    /* Maintenance uses 1/8 of the remaining corrected error. */
    FeedWindow(-60, 7000U, false);
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->runtime_bias_z_mdps == -307);
    assert(status->last_adjustment_z_mdps == -7);
    assert(status->accepted_windows == 2U);

    /* A manual request deliberately uses a full correction even after the
     * automatic estimator has already converged. */
    app::ImuBiasEstimator_RequestCalibration();
    FeedWindow(-80, 10000U, false);
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->runtime_bias_z_mdps == -387);
    assert(status->last_adjustment_z_mdps == -80);

    /* Motion invalidates a partial collection and must not change the bias. */
    app::ImuBiasEstimator_RequestCalibration();
    const int32_t accel[3] = { 0, 0, 1000 };
    const int32_t quiet_gyro[3] = { 0, 0, 10 };
    app::ImuBiasEstimator_Update(accel,
                                 quiet_gyro,
                                 13000U,
                                 true,
                                 app::IMU_BIAS_REJECT_NONE);
    assert(app::ImuBiasEstimator_ShouldFreezeYaw());
    app::ImuBiasEstimator_Update(accel,
                                 quiet_gyro,
                                 13005U,
                                 false,
                                 app::IMU_BIAS_REJECT_WHEELS_MOVING);
    status = app::ImuBiasEstimator_GetStatus();
    assert(!status->freeze_yaw);
    assert(status->sample_count == 0U);
    assert(status->reject_reason == app::IMU_BIAS_REJECT_WHEELS_MOVING);
    assert(status->runtime_bias_z_mdps == -387);

    /* A noisy two-second window is rejected by the standard-deviation gate. */
    app::ImuBiasEstimator_RequestCalibration();
    FeedWindow(400, 14000U, true);
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->reject_reason == app::IMU_BIAS_REJECT_NOISE);
    assert(status->runtime_bias_z_mdps == -387);

    app::ImuBiasEstimator_CommitRuntimeToFixed();
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->runtime_bias_z_mdps == 0);
    assert(!status->estimate_valid);

    app::ImuBiasEstimator_SetAutoEnabled(false);
    status = app::ImuBiasEstimator_GetStatus();
    assert(status->state == app::IMU_BIAS_DISABLED);

    puts("imu bias estimator ok");
    return 0;
}
