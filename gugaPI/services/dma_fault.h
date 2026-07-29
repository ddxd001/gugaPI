#ifndef SERVICES_DMA_FAULT_H_
#define SERVICES_DMA_FAULT_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

typedef void (*DmaFaultHandler)(void);

/* Registers a fixed, allocation-free DMA fault client. Duplicate handlers are
 * accepted so board drivers may be initialized more than once safely. */
bool DmaFault_RegisterHandler(DmaFaultHandler handler);
uint32_t DmaFault_GetCount(void);
void DmaFault_IrqHandler(void);

} /* namespace services */

#endif /* SERVICES_DMA_FAULT_H_ */
