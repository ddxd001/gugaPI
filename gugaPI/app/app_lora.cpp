#include "app/app_lora.h"

#include <limits.h>

#include "board/board_lora.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kAcknowledgmentTimeoutMs = 500U;
static const uint8_t kMaximumRetries = 3U;
static const uint16_t kMaximumBytesPerRun = 64U;

AppLoraProtocolState g_state = {};

drivers::DriverStatus WriteTransport(const uint8_t *data,
                                     uint16_t length,
                                     void *user_context)
{
    (void) user_context;
    return board::Board_LoraWrite(data, length);
}

void RecordProcessStatus(drivers::DriverStatus status)
{
    if (status == drivers::DRIVER_OK) {
        return;
    }
    g_state.last_process_status = status;
    if (g_state.process_error_count != UINT32_MAX) {
        g_state.process_error_count++;
    }
}

} /* namespace */

void App_LoraProtocolInit(void)
{
    const LoraProtocolConfig config = {
        WriteTransport,
        0,
        kAcknowledgmentTimeoutMs,
        kMaximumRetries
    };
    const drivers::DriverStatus status =
        LoraProtocol_Init(&g_state.protocol, &config, 0U);
    g_state.initialized = status == drivers::DRIVER_OK;
    g_state.enabled = false;
    g_state.last_process_status = status;
    g_state.process_error_count = 0U;
}

void App_LoraProtocolRun(void)
{
    if ((!g_state.initialized) || (!g_state.enabled)) {
        return;
    }

    const uint32_t now_ms = services::Time_Millis();
    uint8_t data = 0U;
    uint16_t processed = 0U;
    while ((processed < kMaximumBytesPerRun) &&
           board::Board_LoraReadByte(&data)) {
        RecordProcessStatus(
            LoraProtocol_ProcessByte(&g_state.protocol, data, now_ms));
        processed++;
    }

    RecordProcessStatus(
        LoraProtocol_ProcessTimeouts(&g_state.protocol, now_ms));
}

drivers::DriverStatus App_LoraProtocolSetEnabled(bool enabled)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!enabled) {
        g_state.enabled = false;
        g_state.last_process_status = LoraProtocol_Reset(&g_state.protocol, 0U);
        g_state.process_error_count = 0U;
        return g_state.last_process_status;
    }
    if (!board::Board_LoraIsReady()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    drivers::DriverStatus status = board::Board_LoraClearRxBuffer();
    if (status == drivers::DRIVER_OK) {
        status = LoraProtocol_Reset(&g_state.protocol, 0U);
    }
    if (status == drivers::DRIVER_OK) {
        g_state.enabled = true;
        g_state.process_error_count = 0U;
    }
    g_state.last_process_status = status;
    return status;
}

drivers::DriverStatus App_LoraProtocolReset(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    drivers::DriverStatus status = drivers::DRIVER_OK;
    if (g_state.enabled && board::Board_LoraIsReady()) {
        status = board::Board_LoraClearRxBuffer();
    }
    if (status == drivers::DRIVER_OK) {
        status = LoraProtocol_Reset(&g_state.protocol, 0U);
    }
    g_state.last_process_status = status;
    g_state.process_error_count = 0U;
    return status;
}

drivers::DriverStatus App_LoraProtocolSend(uint8_t type,
                                           const uint8_t *payload,
                                           uint16_t length,
                                           bool acknowledgment_required,
                                           uint8_t *sequence)
{
    if ((!g_state.initialized) || (!g_state.enabled) ||
        (!board::Board_LoraIsReady())) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const drivers::DriverStatus status = LoraProtocol_Send(
        &g_state.protocol,
        type,
        payload,
        length,
        acknowledgment_required,
        services::Time_Millis(),
        sequence);
    g_state.protocol.last_tx_status = status;
    return status;
}

bool App_LoraProtocolReadFrame(LoraProtocolFrame *frame)
{
    return g_state.initialized && g_state.enabled &&
           LoraProtocol_ReadFrame(&g_state.protocol, frame);
}

const AppLoraProtocolState *App_LoraProtocolGetState(void)
{
    return &g_state;
}

} /* namespace app */

