#ifndef SERVICES_TIME_H_
#define SERVICES_TIME_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

void Time_Init(void);
void Time_Tick1ms(void);
uint32_t Time_Millis(void);
uint32_t Time_Micros(void);
bool Time_HasElapsed(uint32_t start_ms, uint32_t interval_ms);
void Time_DelayMs(uint32_t delay_ms);

} /* namespace services */

#endif /* SERVICES_TIME_H_ */
