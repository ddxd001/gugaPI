#include <cassert>
#include <cstdint>
#include <vector>

#include "drivers/oled/oled_ssd1306.h"

namespace {

struct Transfer {
    std::vector<uint8_t> bytes;
};

std::vector<Transfer> g_asyncTransfers;
drivers::DriverStatus g_nextPollStatus = drivers::DRIVER_OK;

drivers::I2cControllerConfig g_bus = {
    "sensor",
    reinterpret_cast<I2C_Regs *>(0x1000U),
    100U,
    reinterpret_cast<GPIO_Regs *>(0x2000U),
    1U,
    1U,
    1U,
    reinterpret_cast<GPIO_Regs *>(0x3000U),
    2U,
    2U,
    2U
};

drivers::I2cControllerDmaTxConfig g_dma = {
    reinterpret_cast<DMA_Regs *>(0x4000U),
    1U,
    50U
};

drivers::OledSsd1306Config g_config = {
    &g_bus,
    &g_dma,
    0x3CU,
    128U,
    32U
};

void CompleteTransfer(drivers::OledSsd1306Context *ctx)
{
    assert(drivers::OledSsd1306_Service(ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    assert(drivers::OledSsd1306_Service(ctx) == drivers::DRIVER_OK);
}

}  // namespace

namespace drivers {

bool I2cController_IsConfigValid(const I2cControllerConfig *config)
{
    return config != nullptr;
}

bool I2cController_IsAddressValid(uint8_t address)
{
    return (address >= I2C_CONTROLLER_MIN_7BIT_ADDRESS) &&
           (address <= I2C_CONTROLLER_MAX_7BIT_ADDRESS);
}

DriverStatus I2cController_Probe(const I2cControllerConfig *, uint8_t)
{
    return DRIVER_OK;
}

DriverStatus I2cController_Write(const I2cControllerConfig *,
                                 uint8_t,
                                 const uint8_t *,
                                 uint16_t,
                                 const uint8_t *,
                                 uint16_t)
{
    return DRIVER_OK;
}

DriverStatus I2cController_AsyncWriteStart(
    const I2cControllerConfig *,
    const I2cControllerDmaTxConfig *,
    uint8_t,
    const uint8_t *data,
    uint16_t length)
{
    Transfer transfer;
    transfer.bytes.assign(data, data + length);
    g_asyncTransfers.push_back(transfer);
    return DRIVER_OK;
}

DriverStatus I2cController_AsyncWritePoll(
    const I2cControllerConfig *,
    const I2cControllerDmaTxConfig *)
{
    const DriverStatus status = g_nextPollStatus;
    g_nextPollStatus = DRIVER_OK;
    return status;
}

void I2cController_AsyncWriteHandleInterrupt(const I2cControllerConfig *) {}

void I2cController_AsyncWriteHandleDmaFault(
    const I2cControllerConfig *, const I2cControllerDmaTxConfig *)
{
    g_nextPollStatus = DRIVER_ERROR;
}

}  // namespace drivers

int main()
{
    drivers::OledSsd1306Context ctx = {};
    assert(drivers::OledSsd1306_Init(&ctx, &g_config) == drivers::DRIVER_OK);
    assert(!drivers::OledSsd1306_HasPendingFlush(&ctx));

    g_asyncTransfers.clear();
    assert(drivers::OledSsd1306_DrawString(&ctx, 2U, 0U, "HI") ==
           drivers::DRIVER_OK);
    assert(ctx.dirty_pages == 0x04U);
    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    assert(g_asyncTransfers.size() == 1U);
    assert(g_asyncTransfers[0].bytes.size() == 9U);
    assert(g_asyncTransfers[0].bytes[0] == 0x00U);
    assert(g_asyncTransfers[0].bytes[7] == 2U);
    assert(g_asyncTransfers[0].bytes[8] == 2U);

    /* A new write during DMA remains dirty for the next flush. */
    assert(drivers::OledSsd1306_DrawString(&ctx, 0U, 0U, "A") ==
           drivers::DRIVER_OK);
    CompleteTransfer(&ctx);
    assert(g_asyncTransfers.size() == 2U);
    assert(g_asyncTransfers[0].bytes[4] == 0U);
    assert(g_asyncTransfers[0].bytes[5] == 9U);
    assert(g_asyncTransfers[1].bytes.size() == 11U);
    assert(g_asyncTransfers[1].bytes[0] == 0x40U);
    assert(ctx.dirty_pages == 0x01U);

    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    CompleteTransfer(&ctx);
    assert(!drivers::OledSsd1306_HasPendingFlush(&ctx));

    /* Non-adjacent dirty pages are coalesced into one contiguous block. */
    uint8_t frame[drivers::OLED_SSD1306_FRAME_BYTES] = {};
    frame[0] = 0x11U;
    frame[256] = 0x22U;
    g_asyncTransfers.clear();
    assert(drivers::OledSsd1306_WriteBuffer(
               &ctx, frame, sizeof(frame)) == drivers::DRIVER_OK);
    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    assert(g_asyncTransfers[0].bytes[7] == 0U);
    assert(g_asyncTransfers[0].bytes[8] == 2U);
    assert(g_asyncTransfers[0].bytes[4] == 0U);
    assert(g_asyncTransfers[0].bytes[5] == 9U);
    CompleteTransfer(&ctx);
    assert(g_asyncTransfers[1].bytes.size() == 31U);

    /* A failed command transfer restores the captured pages for retry. */
    assert(drivers::OledSsd1306_DrawString(&ctx, 3U, 0U, "ERR") ==
           drivers::DRIVER_OK);
    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    g_nextPollStatus = drivers::DRIVER_ERROR_TIMEOUT;
    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_TIMEOUT);
    assert((ctx.dirty_pages & 0x08U) != 0U);
    assert(ctx.flush_phase == drivers::OLED_FLUSH_IDLE);

    /* A shared DMA fault follows the same recoverable dirty-page path. */
    assert(drivers::OledSsd1306_Service(&ctx) ==
           drivers::DRIVER_ERROR_BUSY);
    drivers::OledSsd1306_HandleDmaFault(&ctx);
    assert(drivers::OledSsd1306_Service(&ctx) == drivers::DRIVER_ERROR);
    assert(ctx.flush_phase == drivers::OLED_FLUSH_IDLE);

    return 0;
}
