#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app/heading_lock_math.h"

int main(void)
{
    using app::heading_lock::ComputeCorrectionRpm;
    using app::heading_lock::DurationReached;
    using app::heading_lock::ReadyToSettle;
    using app::heading_lock::ShortestAngleDiff;
    using app::heading_lock::ShouldWake;

    assert(ShortestAngleDiff(-179000, 179000) == 2000);
    assert(ShortestAngleDiff(179000, -179000) == -2000);
    assert(ShortestAngleDiff(180000, 0) == 180000);
    assert(ShortestAngleDiff(-180000, 0) == -180000);

    assert(!ShouldWake(1999, 2000));
    assert(ShouldWake(2000, 2000));
    assert(ShouldWake(-2000, 2000));

    assert(ReadyToSettle(800, 1500, 800, 1500));
    assert(!ReadyToSettle(801, 0, 800, 1500));
    assert(!ReadyToSettle(0, -1501, 800, 1500));

    assert(!DurationReached(1249U, 1000U, 250U));
    assert(DurationReached(1250U, 1000U, 250U));
    assert(!DurationReached(3999U, 1000U, 3000U));
    assert(DurationReached(4000U, 1000U, 3000U));
    /* Unsigned subtraction keeps timers correct across the 32-bit wrap. */
    assert(DurationReached(20U, UINT32_MAX - 19U, 40U));

    /* 10 deg * 1.5 RPM/deg - 20 deg/s * 0.25 = 10 RPM. */
    assert(ComputeCorrectionRpm(10000, 20000, 1500, 250, 10, 30) == 10);
    assert(ComputeCorrectionRpm(-10000, -20000, 1500, 250, 10, 30) == -10);

    /* A same-direction sub-minimum correction must overcome friction. */
    assert(ComputeCorrectionRpm(3000, 0, 1500, 250, 10, 30) == 10);
    assert(ComputeCorrectionRpm(-3000, 0, 1500, 250, 10, 30) == -10);

    /* Opposing derivative braking is not inflated to the minimum RPM. */
    assert(ComputeCorrectionRpm(5000, 40000, 1500, 250, 10, 30) == -3);

    assert(ComputeCorrectionRpm(90000, 0, 1500, 250, 10, 30) == 30);
    assert(ComputeCorrectionRpm(-90000, 0, 1500, 250, 10, 30) == -30);

    puts("heading lock math ok");
    return 0;
}
