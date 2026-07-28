#include <assert.h>
#include <stdio.h>

#include "app/linefollow_road_handoff.h"

int main(void)
{
    using namespace app;
    using linefollow_road_handoff::ShouldHoldForPendingRoadEvent;

    assert(ShouldHoldForPendingRoadEvent(
        GRAYSCALE_ROAD_PHASE_OBSERVING));

    /* Once the event is latched, the road controller must already have
     * pre-empted LF. If it did not, ordinary invalid-track safety applies. */
    assert(!ShouldHoldForPendingRoadEvent(
        GRAYSCALE_ROAD_PHASE_LATCHED));
    assert(!ShouldHoldForPendingRoadEvent(
        GRAYSCALE_ROAD_PHASE_NORMAL));

    puts("linefollow pending-corner handoff ok");
    return 0;
}
