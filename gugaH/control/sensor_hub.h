#ifndef GUGAH_CONTROL_SENSOR_HUB_H_
#define GUGAH_CONTROL_SENSOR_HUB_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "control/control_types.h"

namespace gugah {

bool SensorHub_Init(const HConfig *config);
void SensorHub_Update1ms(uint32_t now_ms, const HConfig *config);
void SensorHub_Update5ms(uint32_t now_ms, const HConfig *config);
void SensorHub_ServiceVision(uint32_t now_ms, const HConfig *config);
const drivers::GrayscaleProcessedData *SensorHub_GetLine(void);
bool SensorHub_TakeLineFrameReady(void);
VisionFeedback SensorHub_GetVision(uint32_t now_ms);
ImuFeedback SensorHub_GetImu(void);
bool SensorHub_IsReady(void);
uint32_t SensorHub_GetErrorCount(void);
const uint16_t *SensorHub_GetGrayscaleRaw(void);
const drivers::BallVisionParserStats *SensorHub_GetVisionStats(void);
bool SensorHub_GetLatestVisionFrame(drivers::BallVisionFrame *frame);
bool SensorHub_InjectVision(int16_t position_0p1mm,
                            uint16_t confidence,
                            uint16_t source_delay_ms,
                            uint32_t now_ms);
void SensorHub_ClearVision(void);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_SENSOR_HUB_H_ */
