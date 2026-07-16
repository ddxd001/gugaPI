#ifndef DRV8323RS_H_
#define DRV8323RS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t faultStatus1;
    uint16_t faultStatus2;
} DRV8323RS_FaultStatus;

bool DRV8323RS_initialize(void);
bool DRV8323RS_isFaultActive(void);
bool DRV8323RS_clearFaults(void);
DRV8323RS_FaultStatus DRV8323RS_readFaults(void);
uint16_t DRV8323RS_readRegister(uint8_t address);
void DRV8323RS_setEnabled(bool enabled);
void DRV8323RS_setCurrentSenseCalibration(bool enabled);

#endif
