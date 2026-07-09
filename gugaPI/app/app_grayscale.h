#ifndef APP_APP_GRAYSCALE_H_
#define APP_APP_GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

namespace app {

struct AppGrayscaleData {
    uint16_t raw[8];   /* 0..4095 per channel, PA15 ADC */
    bool valid;
};

void App_GrayscaleInit(void);
void App_GrayscaleUpdate(void);
const AppGrayscaleData *App_GrayscaleGetData(void);

} /* namespace app */

#endif /* APP_APP_GRAYSCALE_H_ */
