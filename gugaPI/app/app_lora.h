#ifndef APP_APP_LORA_H_
#define APP_APP_LORA_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/lora_protocol.h"

namespace app {

struct AppLoraProtocolState {
    bool initialized;
    bool enabled;
    drivers::DriverStatus last_process_status;
    uint32_t process_error_count;
    LoraProtocolContext protocol;
};

void App_LoraProtocolInit(void);
void App_LoraProtocolRun(void);
drivers::DriverStatus App_LoraProtocolSetEnabled(bool enabled);
drivers::DriverStatus App_LoraProtocolReset(void);
drivers::DriverStatus App_LoraProtocolSend(uint8_t type,
                                           const uint8_t *payload,
                                           uint16_t length,
                                           bool acknowledgment_required,
                                           uint8_t *sequence);
bool App_LoraProtocolReadFrame(LoraProtocolFrame *frame);
const AppLoraProtocolState *App_LoraProtocolGetState(void);

} /* namespace app */

#endif /* APP_APP_LORA_H_ */

