#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include <deque>

#include "app/app_infrared_sensor.h"
#include "app/config_store.h"
#include "board/board_infrared_sensor.h"
#include "drivers/infrared_line/infrared_line_protocol.h"
#include "services/time.h"

namespace {

std::deque<uint8_t> g_rx;
uint32_t g_now_ms = 1U;
app::ConfigStoreParams g_params = {};
uint32_t g_dma_overwrites = 0U;
uint32_t g_dma_faults = 0U;

void QueueFrame(int16_t offset,
                uint16_t all_black,
                uint16_t adc1,
                uint16_t adc2,
                uint16_t adc3)
{
    uint8_t frame[drivers::INFRARED_LINE_FRAME_SIZE] = {
        0x20U, 0x03U, 0x0AU,
        static_cast<uint8_t>(static_cast<uint16_t>(offset) & 0xFFU),
        static_cast<uint8_t>(static_cast<uint16_t>(offset) >> 8U),
        static_cast<uint8_t>(all_black & 0xFFU),
        static_cast<uint8_t>(all_black >> 8U),
        static_cast<uint8_t>(adc1 & 0xFFU),
        static_cast<uint8_t>(adc1 >> 8U),
        static_cast<uint8_t>(adc2 & 0xFFU),
        static_cast<uint8_t>(adc2 >> 8U),
        static_cast<uint8_t>(adc3 & 0xFFU),
        static_cast<uint8_t>(adc3 >> 8U),
        0U, 0U
    };
    const uint16_t crc = drivers::InfraredLine_ModbusCrc16(frame, 13U);
    frame[13] = static_cast<uint8_t>(crc & 0xFFU);
    frame[14] = static_cast<uint8_t>(crc >> 8U);
    for (uint8_t byte : frame) {
        g_rx.push_back(byte);
    }
}

} /* namespace */

namespace board {

void Board_InfraredSensorServiceRx(void) {}

bool Board_InfraredSensorReadByte(uint8_t *data)
{
    if ((data == 0) || g_rx.empty()) {
        return false;
    }
    *data = g_rx.front();
    g_rx.pop_front();
    return true;
}

uint32_t Board_InfraredSensorGetDroppedCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetUartErrorCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetRxTimeoutCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetOverrunErrorCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetFramingErrorCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetParityErrorCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetNoiseErrorCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaBlockCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaProducedCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaConsumedCount(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaCurrentLag(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaMaximumLag(void) { return 0U; }
uint32_t Board_InfraredSensorGetDmaOverwriteCount(void)
{
    return g_dma_overwrites;
}
uint32_t Board_InfraredSensorGetDmaFaultCount(void) { return g_dma_faults; }
void Board_InfraredSensorClear(void) {}

} /* namespace board */

namespace services {

uint32_t Time_Millis(void) { return g_now_ms; }
uint32_t Time_Micros(void) { return g_now_ms * 1000U; }

} /* namespace services */

namespace app {

const ConfigStoreParams *ConfigStore_Get(void) { return &g_params; }

drivers::DriverStatus ConfigStore_SetInfraredCalibration(
    uint8_t invert,
    uint16_t span_raw,
    uint16_t adc_threshold,
    uint16_t adc_hysteresis)
{
    g_params.infrared_position_invert = invert;
    g_params.infrared_position_span_raw = span_raw;
    g_params.infrared_adc_threshold = adc_threshold;
    g_params.infrared_adc_hysteresis = adc_hysteresis;
    return drivers::DRIVER_OK;
}

} /* namespace app */

int main(void)
{
    app::App_InfraredSensorInit();
    assert(app::App_InfraredCalibrationBegin() == drivers::DRIVER_OK);
    assert(app::App_InfraredCalibrationCapture(app::IR_CAL_STEP_BLACK) ==
           drivers::DRIVER_OK);

    /* A real module may report all_black=0 even while its three ADC channels
     * are placed on a uniform black target. Capture must still complete. */
    for (uint16_t i = 0U; i < 64U; i++) {
        QueueFrame(0, 0U, 3200U, 3250U, 3300U);
        app::App_InfraredSensorUpdate();
        g_now_ms++;
    }

    const app::AppInfraredCalibrationStatus *status =
        app::App_InfraredCalibrationGetStatus();
    assert(status->black_ready);
    assert(status->capturing == app::IR_CAL_STEP_NONE);
    assert(status->sample_count == 64U);
    assert(status->black_level == 3200U);
    assert(status->last_status == drivers::DRIVER_OK);

    const uint32_t sequence_before_clear =
        app::App_InfraredSensorGetData()->frame.sequence;
    app::App_InfraredSensorClearStats();
    assert(app::App_InfraredSensorGetData()->frame.sequence ==
           sequence_before_clear);
    QueueFrame(0, 0U, 3000U, 3050U, 3100U);
    app::App_InfraredSensorUpdate();
    assert(app::App_InfraredSensorGetData()->frame.sequence ==
           sequence_before_clear + 1U);

    /* Isolated corrupt frames degrade diagnostics; three consecutive corrupt
     * frames fault the transport. Three valid frames recover communication,
     * without altering the monotonic frame sequence. */
    for (uint8_t i = 0U; i < 3U; i++) {
        QueueFrame(0, 0U, 3000U, 3050U, 3100U);
        g_rx.back() ^= 0x80U;
        app::App_InfraredSensorUpdate();
        g_now_ms++;
    }
    assert(app::App_InfraredSensorGetData()->communication_state ==
           app::IR_COMM_FAULT);
    for (uint8_t i = 0U; i < 3U; i++) {
        QueueFrame(0, 0U, 3000U, 3050U, 3100U);
        app::App_InfraredSensorUpdate();
        g_now_ms++;
    }
    assert(app::App_InfraredSensorGetData()->communication_state ==
           app::IR_COMM_HEALTHY);

    g_dma_overwrites++;
    app::App_InfraredSensorUpdate();
    assert(app::App_InfraredSensorGetData()->communication_state ==
           app::IR_COMM_FAULT);

    puts("infrared calibration and health ok");
    return 0;
}
