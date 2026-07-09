#ifndef APP_APP_IMU_H_
#define APP_APP_IMU_H_

#include <stdbool.h>
#include <stdint.h>

namespace app {

struct AppImuData {
    int32_t accel_mg[3];    /* X, Y, Z in milli-g, bias subtracted */
    int32_t gyro_mdps[3];   /* X, Y, Z in milli-deg/s, bias subtracted */
    int32_t temp_centi_c;   /* temperature in centi-degrees Celsius */
    int32_t pitch_mdeg;     /* tilt pitch in milli-degrees, 0..360000 */
    int32_t roll_mdeg;      /* tilt roll in milli-degrees, 0..360000 */
    bool valid;
};

void App_ImuInit(void);
void App_ImuUpdate(void);
const AppImuData *App_ImuGetData(void);

} /* namespace app */

#endif /* APP_APP_IMU_H_ */
