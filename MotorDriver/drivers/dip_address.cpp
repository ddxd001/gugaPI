#include "drivers/dip_address.h"

#include "protocol/motor_registers.h"
#include "ti_msp_dl_config.h"

uint8_t DipAddress_readI2cAddress(void)
{
    uint8_t dip = 0;

    if ((DL_GPIO_readPins(GPIO_ADDR0_PORT, GPIO_ADDR0_ADDR0_PIN_PIN) & GPIO_ADDR0_ADDR0_PIN_PIN) == 0U) {
        dip |= 0x01U;
    }
    if ((DL_GPIO_readPins(GPIO_ADDR1_PORT, GPIO_ADDR1_ADDR1_PIN_PIN) & GPIO_ADDR1_ADDR1_PIN_PIN) == 0U) {
        dip |= 0x02U;
    }
    if ((DL_GPIO_readPins(GPIO_ADDR2_PORT, GPIO_ADDR2_ADDR2_PIN_PIN) & GPIO_ADDR2_ADDR2_PIN_PIN) == 0U) {
        dip |= 0x04U;
    }

    return static_cast<uint8_t>(MotorProtocol::kI2cBaseAddr + (dip & MotorProtocol::kI2cAddrMask));
}
