#include "algorithm/pid.h"

namespace algorithm {

static float Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

void PID_Reset(PIDController *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float PID_Update(PIDController *pid,
                 float target,
                 float measured,
                 float dt_seconds)
{
    if ((pid == 0) || (dt_seconds <= 0.0f)) {
        return 0.0f;
    }

    const float error = target - measured;
    pid->integral += error * dt_seconds;

    const float derivative = (error - pid->previous_error) / dt_seconds;
    const float output = (pid->kp * error) +
                         (pid->ki * pid->integral) +
                         (pid->kd * derivative);

    pid->previous_error = error;
    return Clamp(output, pid->output_min, pid->output_max);
}

} /* namespace algorithm */