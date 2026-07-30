#ifndef APP_BALL_VISION_H_
#define APP_BALL_VISION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/ball_vision/ball_vision_protocol.h"
#include "drivers/common/driver_status.h"

namespace app {

enum BallVisionState : uint8_t {
    BALL_VISION_OFFLINE = 0U,
    BALL_VISION_NO_BALL,
    BALL_VISION_FRESH,
    BALL_VISION_DEGRADED,
    BALL_VISION_STALE
};

struct BallVisionData {
    drivers::BallVisionFrame frame;
    drivers::BallVisionFrame ball_frame;
    drivers::BallVisionParserStats parser_stats;
    BallVisionState state;
    bool has_ball_sample;
    bool injected;
    uint32_t frame_age_ms;
    uint32_t ball_age_ms;
    uint32_t uart_dropped_bytes;
    uint32_t uart_errors;
    uint32_t uart_irq_count;
    drivers::DriverStatus last_status;
};

void BallVision_Init(void);
void BallVision_Update(void);
const BallVisionData *BallVision_GetData(void);
bool BallVision_IsCommunicationOnline(uint32_t now_ms);
bool BallVision_IsUsable(uint32_t now_ms);
drivers::DriverStatus BallVision_Inject(int16_t position_0p1mm,
                                        uint16_t confidence,
                                        uint32_t now_ms);
void BallVision_Clear(void);
const char *BallVision_StateText(BallVisionState state);

} /* namespace app */

#endif /* APP_BALL_VISION_H_ */
