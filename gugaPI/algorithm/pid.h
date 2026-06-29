#ifndef ALGORITHM_PID_H_
#define ALGORITHM_PID_H_

namespace algorithm {

struct PIDController {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
};

void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);
void PID_Reset(PIDController *pid);
float PID_Update(PIDController *pid,
                 float target,
                 float measured,
                 float dt_seconds);

} /* namespace algorithm */

#endif /* ALGORITHM_PID_H_ */