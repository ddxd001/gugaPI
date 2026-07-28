#include <assert.h>
#include <stdio.h>

#include "app/heading_arc_math.h"

int main(void)
{
    using namespace app::heading_arc;

    assert(PredictErrorMdeg(90000, 30000, 60U, 500) == 87700);
    assert(PredictErrorMdeg(-90000, -30000, 60U, 500) == -87700);
    assert(PredictErrorMdeg(90000, -30000, 60U, 500) == 91800);
    assert(BrakeAngleMdeg(90000, 30000, 60U, 500) == 2300);

    const int32_t left_correction = ComputeCorrectionRpm(
        90000, 87700, 1000, 20, 60, 960, 3000);
    assert(left_correction == 60);
    const WheelCommand left = MakeWheelCommand(40, left_correction);
    assert(left.left_rpm == -20);
    assert(left.right_rpm == 100);
    assert((left.left_rpm + left.right_rpm) / 2 == 40);

    const int32_t right_correction = ComputeCorrectionRpm(
        -90000, -87700, 1000, 20, 60, 960, 3000);
    assert(right_correction == -60);
    const WheelCommand right = MakeWheelCommand(40, right_correction);
    assert(right.left_rpm == 100);
    assert(right.right_rpm == -20);
    assert((right.left_rpm + right.right_rpm) / 2 == 40);

    const WheelCommand asymmetric_left = MakeAsymmetricWheelCommand(
        90, 130, 130, 220, 120, 1000);
    assert(asymmetric_left.left_rpm == -120);
    assert(asymmetric_left.right_rpm == 220);
    const WheelCommand asymmetric_right = MakeAsymmetricWheelCommand(
        90, -130, 130, 220, 120, 1000);
    assert(asymmetric_right.left_rpm == 220);
    assert(asymmetric_right.right_rpm == -120);

    /* Partial steering preserves the old outer-wheel target while the inner
     * wheel moves smoothly toward its independent reverse endpoint. */
    const WheelCommand asymmetric_half = MakeAsymmetricWheelCommand(
        90, 65, 130, 220, 120, 1000);
    assert(asymmetric_half.left_rpm == -15);
    assert(asymmetric_half.right_rpm == 155);
    const WheelCommand asymmetric_straight = MakeAsymmetricWheelCommand(
        90, 0, 130, 220, 120, 1000);
    assert(asymmetric_straight.left_rpm == 90);
    assert(asymmetric_straight.right_rpm == 90);

    /* Outer limiting and reverse travel retain bounded, mirrored outputs. */
    const WheelCommand asymmetric_capped = MakeAsymmetricWheelCommand(
        90, 130, 130, 180, 120, 1000);
    assert(asymmetric_capped.left_rpm == -120);
    assert(asymmetric_capped.right_rpm == 180);
    const WheelCommand asymmetric_reverse = MakeAsymmetricWheelCommand(
        -90, 130, 130, 220, 120, 1000);
    assert(asymmetric_reverse.left_rpm == -220);
    assert(asymmetric_reverse.right_rpm == 120);
    const WheelCommand asymmetric_extreme = MakeAsymmetricWheelCommand(
        INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX, INT32_MAX, 1000);
    assert(asymmetric_extreme.left_rpm == -1000);
    assert(asymmetric_extreme.right_rpm == -1000);

    /* Prediction may cross the target and request a small counter-steer;
     * the minimum-turn RPM is deliberately not forced in this region. */
    const int32_t predicted_crossing =
        PredictErrorMdeg(1000, 60000, 60U, 500);
    assert(predicted_crossing == -3100);
    assert(ComputeCorrectionRpm(
        1000, predicted_crossing, 1000, 20, 60, 960, 3000) == -3);

    /* Wheel headroom has priority over the configured correction maximum. */
    assert(ComputeCorrectionRpm(
        90000, 90000, 1000, 20, 60, 50, 3000) == 50);

    assert(IsAtTarget(3000, 1500, 3000, 1500));
    assert(!IsAtTarget(3001, 1500, 3000, 1500));
    assert(!IsAtTarget(3000, 1501, 3000, 1500));

    assert(!IsRollingDistanceReached(1, 1));
    assert(IsRollingDistanceReached(0, 1));
    assert(IsRollingDistanceReached(-1, 1));
    assert(!IsRollingDistanceReached(-1, -1));
    assert(IsRollingDistanceReached(0, -1));
    assert(IsRollingDistanceReached(1, -1));

    puts("heading arc math ok");
    return 0;
}
