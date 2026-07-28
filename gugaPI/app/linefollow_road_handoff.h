#ifndef APP_LINEFOLLOW_ROAD_HANDOFF_H_
#define APP_LINEFOLLOW_ROAD_HANDOFF_H_

#include <stdint.h>

#include "app/grayscale_road.h"

namespace app {
namespace linefollow_road_handoff {

/* A road event may be published only after several exit frames, by which time
 * the instantaneous geometry can already be STRAIGHT, UNKNOWN or LOST. The
 * complete OBSERVING phase is bounded by the classifier, so keep motion alive
 * until the event can pre-empt line following. LATCHED/NORMAL loss retains the
 * ordinary safety-stop policy. */
inline bool ShouldHoldForPendingRoadEvent(GrayscaleRoadPhase phase)
{
    return phase == GRAYSCALE_ROAD_PHASE_OBSERVING;
}

} /* namespace linefollow_road_handoff */
} /* namespace app */

#endif /* APP_LINEFOLLOW_ROAD_HANDOFF_H_ */
