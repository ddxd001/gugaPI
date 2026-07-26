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

typedef void (*FaultPanicHandler)(FaultCode code);

void Fault_Init(void);
/* Latch the first non-NONE fault (later faults increment the count but do not
 * overwrite the latched code). Fault_Get returns the latched first fault. */
void Fault_Set(FaultCode code);
FaultCode Fault_Get(void);
bool Fault_HasFault(void);
uint32_t Fault_GetCount(void);
void Fault_Clear(void);
/* Register a handler invoked from Fault_Panic. Because services cannot call
 * board/app code directly, the handler (registered from main) owns the
 * observable side effects: LED blink / log. If unset, Fault_Panic just hangs. */
void Fault_SetPanicHandler(FaultPanicHandler handler);
void Fault_Panic(FaultCode code);

} /* namespace services */

#endif /* SERVICES_FAULT_H_ */