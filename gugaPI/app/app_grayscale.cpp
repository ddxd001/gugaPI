#include "app/app_grayscale.h"

#include "board/board_grayscale.h"
#include "drivers/common/driver_status.h"

namespace app {
namespace {

AppGrayscaleData g_data = { { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, false };

} /* namespace */

void App_GrayscaleInit(void)
{
    g_data.valid = false;
}

void App_GrayscaleUpdate(void)
{
    if (!board::Board_GrayscaleIsReady()) {
        return;
    }

    uint16_t raw[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
    const drivers::DriverStatus status = board::Board_GrayscaleReadAll(raw);
    if (status != drivers::DRIVER_OK) {
        return;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        g_data.raw[i] = raw[i];
    }
    g_data.valid = true;
}

const AppGrayscaleData *App_GrayscaleGetData(void)
{
    return &g_data;
}

} /* namespace app */
