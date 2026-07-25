#include "board/board_nvm.h"

#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_flashctl.h>

namespace board {
namespace {

constexpr uint32_t kNvmBase = 0x0007FC00U;
constexpr uint32_t kNvmMagic = 0x49324E00U;
constexpr uint32_t kNvmMagicMask = 0xFFFFFF00U;

}  // namespace

uint8_t BoardNvm_LoadI2cAddress(void)
{
    const volatile uint32_t *ptr =
        reinterpret_cast<volatile uint32_t *>(kNvmBase);
    const uint32_t value = *ptr;

    if ((value & kNvmMagicMask) != kNvmMagic) {
        return 0U;
    }

    return static_cast<uint8_t>(value & 0xFFU);
}

bool BoardNvm_SaveI2cAddress(uint8_t address)
{
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL,
                                kNvmBase,
                                DL_FLASHCTL_REGION_SELECT_MAIN);

    DL_FLASHCTL_COMMAND_STATUS status =
        DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
                                       kNvmBase,
                                       DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return false;
    }

    DL_FlashCTL_unprotectSector(FLASHCTL,
                                kNvmBase,
                                DL_FLASHCTL_REGION_SELECT_MAIN);

    const uint32_t data = kNvmMagic | static_cast<uint32_t>(address);
    status = DL_FlashCTL_programMemoryFromRAM32(FLASHCTL, kNvmBase, &data);

    return (status == DL_FLASHCTL_COMMAND_STATUS_PASSED);
}

}  // namespace board
