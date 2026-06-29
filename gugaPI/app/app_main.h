#ifndef APP_APP_MAIN_H_
#define APP_APP_MAIN_H_

#include "app/app_state.h"

namespace app {

void App_Init(void);
void App_Run(void);
const AppState *App_GetState(void);

} /* namespace app */

#endif /* APP_APP_MAIN_H_ */