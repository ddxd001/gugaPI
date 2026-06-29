#ifndef DRIVERS_TEMPLATE_TEMPLATE_DRIVER_H_
#define DRIVERS_TEMPLATE_TEMPLATE_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace drivers {

struct TemplateDriverConfig {
    uint32_t timeout_ms;
};

struct TemplateDriverContext {
    const TemplateDriverConfig *config;
    bool initialized;
};

DriverStatus TemplateDriver_Init(TemplateDriverContext *ctx,
                                 const TemplateDriverConfig *config);
DriverStatus TemplateDriver_Deinit(TemplateDriverContext *ctx);
DriverStatus TemplateDriver_Update(TemplateDriverContext *ctx);
bool TemplateDriver_IsReady(const TemplateDriverContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_TEMPLATE_TEMPLATE_DRIVER_H_ */