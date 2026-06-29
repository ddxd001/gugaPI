#ifndef SERVICES_FAULT_H_
#define SERVICES_FAULT_H_

#include <stdint.h>

namespace services {

enum FaultCode {
    FAULT_NONE = 0,
    FAULT_ASSERT,
    FAULT_DRIVER_INIT,
    FAULT_DRIVER_TIMEOUT,
    FAULT_SENSOR_LOST,
    FAULT_UART_OVERFLOW,
    FAULT_UNKNOWN
};

void Fault_Init(void);
void Fault_Set(FaultCode code);
FaultCode Fault_Get(void);
void Fault_Clear(void);
void Fault_Panic(FaultCode code);

} /* namespace services */

#endif /* SERVICES_FAULT_H_ */