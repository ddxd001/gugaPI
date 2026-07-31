#ifndef GUGAH_APP_H_RUNTIME_H_
#define GUGAH_APP_H_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/h_app.h"

namespace gugah {

void HRuntime_Init(void);
void HRuntime_Service(void);
void HRuntime_Update1ms(uint32_t now_ms);
void HRuntime_Update2ms(uint32_t now_ms);
void HRuntime_Update5ms(uint32_t now_ms);
void HRuntime_Update50ms(uint32_t now_ms);

HAppState *HRuntime_GetState(void);
const BallState *HRuntime_GetActiveBallState(void);
HConfig *HRuntime_GetConfig(void);
HAppInput HRuntime_GetInput(uint32_t now_ms);
bool HRuntime_Start(void);
void HRuntime_Abort(void);
void HRuntime_Select(HProblem problem);
bool HRuntime_SaveConfig(void);
void HRuntime_DefaultConfig(void);
void HRuntime_SetTelemetry(bool enabled);
bool HRuntime_GetTelemetry(void);
void HRuntime_SetVisionTrace(bool enabled);
bool HRuntime_GetVisionTrace(void);
void HRuntime_PrintStatus(void);
void HRuntime_ClearFault(void);
bool HRuntime_ChassisTest(int16_t left_rpm, int16_t right_rpm);
void HRuntime_ChassisStopTest(void);
bool HRuntime_BallHold(int16_t target_0p1mm);
bool HRuntime_BallMove(int16_t target_0p1mm);
void HRuntime_BallStopTest(void);
bool HRuntime_DmBenchTakeControl(void);
void HRuntime_DmBenchReleaseControl(void);
bool HRuntime_DmBenchHasControl(void);

} /* namespace gugah */

#endif /* GUGAH_APP_H_RUNTIME_H_ */
