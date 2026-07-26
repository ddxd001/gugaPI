#ifndef APP_APP_STATE_H_
#define APP_APP_STATE_H_

#include <stdint.h>

namespace app {

enum AppMode {
    APP_MODE_IDLE = 0,
    APP_MODE_RUNNING,
    APP_MODE_FAULT,
    APP_MODE_COMPETITION_ARMED,     /* Safe idle: stationary, chassis disabled, waiting for start */
    APP_MODE_COMPETITION_RUNNING    /* Competition sequence active: ActionRunner driving */
};

struct AppState {
    AppMode mode;
    uint32_t uptime_ms;
};

} /* namespace app */

#endif /* APP_APP_STATE_H_ */