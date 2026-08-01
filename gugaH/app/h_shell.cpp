#include "app/h_shell.h"

#include <stdint.h>

#include "app/h_runtime.h"
#include "board/board_can.h"
#include "control/chassis.h"
#include "control/dm_actuator.h"
#include "control/sensor_hub.h"
#include "services/shell.h"
#include "services/time.h"

namespace gugah {
namespace {

bool Equal(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0') && (*left == *right)) {
        left++;
        right++;
    }
    return (*left == '\0') && (*right == '\0');
}

bool ParseInt(const char *text, int32_t *value)
{
    if ((text == 0) || (value == 0)) {
        return false;
    }
    bool negative = false;
    if (*text == '-') {
        negative = true;
        text++;
    }
    if ((*text < '0') || (*text > '9')) {
        return false;
    }
    int64_t result = 0;
    while ((*text >= '0') && (*text <= '9')) {
        result = result * 10 + (*text - '0');
        if (result > 2147483648LL) {
            return false;
        }
        text++;
    }
    if (*text != '\0') {
        return false;
    }
    result = negative ? -result : result;
    if ((result < -2147483648LL) || (result > 2147483647LL)) {
        return false;
    }
    *value = static_cast<int32_t>(result);
    return true;
}

void Ok(bool success)
{
    services::Shell_WriteLine(success ? "OK" : "ERR");
}

void StatusCommand(int argc, const char * const argv[])
{
    (void)argc;
    (void)argv;
    HRuntime_PrintStatus();
}

void TaskCommand(int argc, const char * const argv[])
{
    int32_t value = 0;
    if ((argc != 2) || !ParseInt(argv[1], &value) ||
        (value < 2) || (value > 8)) {
        Ok(false);
        return;
    }
    HRuntime_Select(static_cast<HProblem>(value));
    Ok(static_cast<int32_t>(
           HRuntime_GetState()->selected_problem) == value);
}

void StartCommand(int argc, const char * const argv[])
{
    (void)argc;
    (void)argv;
    Ok(HRuntime_Start());
}

void StopCommand(int argc, const char * const argv[])
{
    (void)argc;
    (void)argv;
    HRuntime_Abort();
    Ok(true);
}

void FaultCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && Equal(argv[1], "clear")) {
        HRuntime_ClearFault();
        Ok(true);
        return;
    }
    Ok(false);
}

void GrayCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && Equal(argv[1], "raw")) {
        const uint16_t *raw = SensorHub_GetGrayscaleRaw();
        for (uint8_t i = 0U; i < 8U; i++) {
            if (i != 0U) {
                services::Shell_Write(",");
            }
            services::Shell_WriteInt(raw[i]);
        }
        services::Shell_Write("\r\n");
        return;
    }
    if ((argc == 2) &&
        (Equal(argv[1], "white") || Equal(argv[1], "black")) &&
        !HApp_IsRunning(HRuntime_GetState())) {
        const uint16_t *raw = SensorHub_GetGrayscaleRaw();
        HConfig *config = HRuntime_GetConfig();
        for (uint8_t i = 0U; i < 8U; i++) {
            if (Equal(argv[1], "white")) {
                config->grayscale.white[i] = raw[i];
            } else {
                config->grayscale.black[i] = raw[i];
            }
        }
        Ok(HConfig_Validate(config));
        return;
    }
    const drivers::GrayscaleProcessedData *line =
        SensorHub_GetLine();
    if ((argc == 2) && Equal(argv[1], "status") && (line != 0)) {
        services::Shell_Write("pos=");
        services::Shell_WriteInt(line->line_position);
        services::Shell_Write(" mask=");
        services::Shell_WriteInt(line->active_mask);
        services::Shell_Write(" state=");
        services::Shell_WriteInt(line->track_state);
        services::Shell_Write("\r\n");
        return;
    }
    Ok(false);
}

void ChassisCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && Equal(argv[1], "stop")) {
        HRuntime_ChassisStopTest();
        Ok(true);
        return;
    }
    if ((argc == 4) && Equal(argv[1], "test")) {
        int32_t left = 0;
        int32_t right = 0;
        Ok(ParseInt(argv[2], &left) &&
           ParseInt(argv[3], &right) &&
           (left >= -60) && (left <= 60) &&
           (right >= -60) && (right <= 60) &&
           HRuntime_ChassisTest(
               static_cast<int16_t>(left),
               static_cast<int16_t>(right)));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "status")) {
        const ChassisFeedback feedback = Chassis_GetFeedback();
        services::Shell_Write("valid=");
        services::Shell_WriteInt(feedback.valid);
        services::Shell_Write(" left_rpm=");
        services::Shell_WriteInt(feedback.left_rpm);
        services::Shell_Write(" right_rpm=");
        services::Shell_WriteInt(feedback.right_rpm);
        services::Shell_Write(" left_count=");
        services::Shell_WriteInt(feedback.left_encoder_count);
        services::Shell_Write(" right_count=");
        services::Shell_WriteInt(feedback.right_encoder_count);
        services::Shell_Write(" errors=");
        services::Shell_WriteInt(
            static_cast<int32_t>(Chassis_GetErrorCount()));
        services::Shell_Write("\r\n");
        return;
    }
    Ok(false);
}

void VisionCommand(int argc, const char * const argv[])
{
    if ((argc == 3) && Equal(argv[1], "trace") &&
        (Equal(argv[2], "on") || Equal(argv[2], "off"))) {
        HRuntime_SetVisionTrace(Equal(argv[2], "on"));
        Ok(true);
        return;
    }
    if ((argc == 2) && Equal(argv[1], "clear") &&
        !HApp_IsRunning(HRuntime_GetState())) {
        SensorHub_ClearVision();
        Ok(true);
        return;
    }
    if ((argc >= 3) && (argc <= 5) &&
        Equal(argv[1], "inject") &&
        !HApp_IsRunning(HRuntime_GetState())) {
        int32_t position = 0;
        int32_t confidence = 1000;
        int32_t delay = 0;
        const bool valid = ParseInt(argv[2], &position) &&
            ((argc < 4) || ParseInt(argv[3], &confidence)) &&
            ((argc < 5) || ParseInt(argv[4], &delay)) &&
            (position >= -1250) && (position <= 1250) &&
            (confidence >= 0) && (confidence <= 1000) &&
            (delay >= 0) && (delay <= 1000);
        Ok(valid && SensorHub_InjectVision(
            static_cast<int16_t>(position),
            static_cast<uint16_t>(confidence),
            static_cast<uint16_t>(delay),
            services::Time_Millis()));
        return;
    }
    if ((argc != 2) ||
        (!Equal(argv[1], "status") && !Equal(argv[1], "stats"))) {
        Ok(false);
        return;
    }
    const uint32_t now_ms = services::Time_Millis();
    const VisionFeedback vision = SensorHub_GetVision(now_ms);
    const drivers::BallVisionParserStats *stats =
        SensorHub_GetVisionStats();
    const int16_t physical_position =
        Ball_CalibrateCameraPosition0p1mm(
            vision.frame.position_0p1mm);
    const HConfig *config = HRuntime_GetConfig();
    const int16_t controller_position =
        (config->vision_position_invert != 0U)
        ? static_cast<int16_t>(-physical_position)
        : physical_position;
    services::Shell_Write("online=");
    services::Shell_WriteInt(vision.communication_online);
    services::Shell_Write(" usable=");
    services::Shell_WriteInt(vision.ball_usable);
    services::Shell_Write(" raw0.1mm=");
    services::Shell_WriteInt(vision.frame.position_0p1mm);
    services::Shell_Write(" real0.1mm=");
    services::Shell_WriteInt(physical_position);
    services::Shell_Write(" ctrl0.1mm=");
    services::Shell_WriteInt(controller_position);
    services::Shell_Write(" age=");
    services::Shell_WriteInt(vision.ball_age_ms);
    services::Shell_Write(" valid=");
    services::Shell_WriteInt(stats->valid_frames);
    services::Shell_Write(" crc=");
    services::Shell_WriteInt(stats->crc_errors);
    services::Shell_Write("\r\n");
}

void BallCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && Equal(argv[1], "stop")) {
        HRuntime_BallStopTest();
        Ok(true);
        return;
    }
    if ((argc == 3) &&
        (Equal(argv[1], "hold") || Equal(argv[1], "move"))) {
        int32_t millimeters = 0;
        if (!ParseInt(argv[2], &millimeters) ||
            (millimeters < -100) || (millimeters > 100)) {
            Ok(false);
            return;
        }
        const int16_t target =
            static_cast<int16_t>(millimeters * 10);
        Ok(Equal(argv[1], "hold")
            ? HRuntime_BallHold(target)
            : HRuntime_BallMove(target));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "status")) {
        const uint32_t now_ms = services::Time_Millis();
        const BallState *ball = HRuntime_GetActiveBallState();
        const VisionFeedback vision = SensorHub_GetVision(now_ms);
        const ImuFeedback imu = SensorHub_GetImu();
        const DmFeedback dm = DmActuator_GetFeedback();
        services::Shell_Write("result=");
        services::Shell_WriteInt(ball->result);
        services::Shell_Write(" vision=");
        services::Shell_WriteInt(vision.ball_usable);
        services::Shell_Write("/age=");
        services::Shell_WriteInt(
            static_cast<int32_t>(vision.ball_age_ms));
        services::Shell_Write(" imu=");
        services::Shell_WriteInt(imu.valid);
        services::Shell_Write("/age=");
        services::Shell_WriteInt(static_cast<int32_t>(
            imu.valid ? (now_ms - imu.received_ms) : 0xFFFFFFFFUL));
        services::Shell_Write(" dm_valid=");
        services::Shell_WriteInt(dm.valid);
        services::Shell_Write("/ready=");
        services::Shell_WriteInt(DmActuator_IsReady());
        services::Shell_Write("/state=");
        services::Shell_WriteInt(dm.state);
        services::Shell_Write("/age=");
        services::Shell_WriteInt(static_cast<int32_t>(
            dm.valid ? (now_ms - dm.received_ms) : 0xFFFFFFFFUL));
        services::Shell_Write(" dm_errors=");
        services::Shell_WriteInt(
            static_cast<int32_t>(DmActuator_GetErrorCount()));
        services::Shell_Write("/busy=");
        services::Shell_WriteInt(
            static_cast<int32_t>(DmActuator_GetBusyCount()));
        services::Shell_Write(" dm_target=");
        services::Shell_WriteInt(DmActuator_GetTargetPosition());
        services::Shell_Write("/ref=");
        services::Shell_WriteInt(DmActuator_GetReferencePosition());
        services::Shell_Write(" ");
        services::Shell_Write("target0.1=");
        services::Shell_WriteInt(ball->target_position_0p1mm);
        services::Shell_Write(" pos0.1=");
        services::Shell_WriteInt(ball->estimated_position_0p1mm);
        services::Shell_Write(" vel0.1s=");
        services::Shell_WriteInt(ball->estimated_velocity_0p1mm_s);
        services::Shell_Write(" vmeas0.1s=");
        services::Shell_WriteInt(ball->measured_velocity_0p1mm_s);
        services::Shell_Write(" vtarget0.1s=");
        services::Shell_WriteInt(ball->target_velocity_0p1mm_s);
        services::Shell_Write(" verr0.1s=");
        services::Shell_WriteInt(ball->velocity_error_0p1mm_s);
        services::Shell_Write(" acmd0.1s2=");
        services::Shell_WriteInt(ball->desired_acceleration_0p1mm_s2);
        services::Shell_Write(" amodel0.1s2=");
        services::Shell_WriteInt(ball->model_acceleration_0p1mm_s2);
        services::Shell_Write(" hold_mdeg=");
        services::Shell_WriteInt(ball->rail_compensation_mdeg);
        services::Shell_Write(" stiction_mdeg=");
        services::Shell_WriteInt(ball->stiction_compensation_mdeg);
        services::Shell_Write(" pid_mdeg=");
        services::Shell_WriteInt(ball->pid_correction_mdeg);
        services::Shell_Write("/");
        services::Shell_WriteInt(ball->pid_p_mdeg);
        services::Shell_Write("/");
        services::Shell_WriteInt(ball->pid_i_mdeg);
        services::Shell_Write("/");
        services::Shell_WriteInt(ball->pid_d_mdeg);
        services::Shell_Write(" chassis_ff_mdeg=");
        services::Shell_WriteInt(ball->chassis_feedforward_mdeg);
        services::Shell_Write(" beam_mdeg=");
        services::Shell_WriteInt(ball->beam_target_mdeg);
        services::Shell_Write(" imu_beam_mdeg=");
        services::Shell_WriteInt(ball->imu_beam_mdeg);
        services::Shell_Write(" imu_beam_err_mdeg=");
        services::Shell_WriteInt(ball->imu_beam_error_mdeg);
        services::Shell_Write(" imu_comp_mdeg=");
        services::Shell_WriteInt(ball->imu_beam_compensation_mdeg);
        services::Shell_Write(" beam_cmd_mdeg=");
        services::Shell_WriteInt(ball->beam_command_mdeg);
        services::Shell_Write(" dm=");
        services::Shell_WriteInt(dm.position_mrad);
        services::Shell_Write("\r\n");
        return;
    }
    Ok(false);
}

void ImuCommand(int argc, const char * const argv[])
{
    if ((argc != 2) || !Equal(argv[1], "status")) {
        Ok(false);
        return;
    }
    const uint32_t now_ms = services::Time_Millis();
    const ImuFeedback imu = SensorHub_GetImu();
    services::Shell_Write("valid=");
    services::Shell_WriteInt(imu.valid);
    services::Shell_Write(" age=");
    services::Shell_WriteInt(imu.valid
        ? static_cast<int32_t>(now_ms - imu.received_ms) : -1);
    services::Shell_Write(" accel_mg=");
    services::Shell_WriteInt(imu.accel_x_mg);
    services::Shell_Write("/");
    services::Shell_WriteInt(imu.accel_y_mg);
    services::Shell_Write("/");
    services::Shell_WriteInt(imu.accel_z_mg);
    services::Shell_Write(" gyro_mdps=");
    services::Shell_WriteInt(imu.gyro_x_mdps);
    services::Shell_Write("/");
    services::Shell_WriteInt(imu.gyro_y_mdps);
    services::Shell_Write("/");
    services::Shell_WriteInt(imu.gyro_z_mdps);
    services::Shell_Write(" temp0.01c=");
    services::Shell_WriteInt(imu.temperature_centi_c);
    services::Shell_Write(" pitch_mdeg=");
    services::Shell_WriteInt(imu.pitch_mdeg);
    services::Shell_Write(" beam_mdeg=");
    services::Shell_WriteInt(imu.beam_mdeg);
    services::Shell_Write(" yaw_mdeg=");
    services::Shell_WriteInt(imu.yaw_mdeg);
    services::Shell_Write("\r\n");
}

void DmCommand(int argc, const char * const argv[])
{
    const bool ready_state =
        HRuntime_GetState()->run_state == H_STATE_READY;
    if ((argc == 2) && Equal(argv[1], "status")) {
        const uint32_t now_ms = services::Time_Millis();
        const DmFeedback dm = DmActuator_GetFeedback();
        drivers::CanStatus can = {};
        const bool can_ok =
            board::Board_CanGetStatus(&can) == drivers::DRIVER_OK;
        services::Shell_Write("valid=");
        services::Shell_WriteInt(dm.valid);
        services::Shell_Write(" ready=");
        services::Shell_WriteInt(DmActuator_IsReady());
        services::Shell_Write(" bench=");
        services::Shell_WriteInt(HRuntime_DmBenchHasControl());
        services::Shell_Write(" enabling=");
        services::Shell_WriteInt(DmActuator_IsEnabling());
        services::Shell_Write(" state=");
        services::Shell_WriteInt(dm.state);
        services::Shell_Write(" age=");
        services::Shell_WriteInt(static_cast<int32_t>(
            dm.valid ? (now_ms - dm.received_ms) : 0xFFFFFFFFUL));
        services::Shell_Write(" pos=");
        services::Shell_WriteInt(dm.position_mrad);
        services::Shell_Write(" vel=");
        services::Shell_WriteInt(dm.velocity_mrad_s);
        services::Shell_Write(" torque=");
        services::Shell_WriteInt(dm.torque_mnm);
        services::Shell_Write(" temp=");
        services::Shell_WriteInt(dm.mos_temperature_c);
        services::Shell_Write("/");
        services::Shell_WriteInt(dm.coil_temperature_c);
        services::Shell_Write(" target/ref=");
        services::Shell_WriteInt(DmActuator_GetTargetPosition());
        services::Shell_Write("/");
        services::Shell_WriteInt(DmActuator_GetReferencePosition());
        services::Shell_Write(" tx_busy/error=");
        services::Shell_WriteInt(
            static_cast<int32_t>(DmActuator_GetBusyCount()));
        services::Shell_Write("/");
        services::Shell_WriteInt(
            static_cast<int32_t>(DmActuator_GetErrorCount()));
        services::Shell_Write(" can=");
        services::Shell_WriteInt(can_ok);
        if (can_ok) {
            services::Shell_Write("/busoff=");
            services::Shell_WriteInt(can.bus_off);
            services::Shell_Write("/tx=");
            services::Shell_WriteInt(
                static_cast<int32_t>(can.tx_request_count));
            services::Shell_Write("/rx=");
            services::Shell_WriteInt(
                static_cast<int32_t>(can.rx_count));
            services::Shell_Write("/err=");
            services::Shell_WriteInt(
                static_cast<int32_t>(can.error_event_count));
        }
        services::Shell_Write("\r\n");
        return;
    }
    if (!ready_state) {
        Ok(false);
        return;
    }
    if ((argc == 2) && Equal(argv[1], "enable")) {
        Ok(HRuntime_DmBenchTakeControl() &&
           (DmActuator_Enable() == drivers::DRIVER_OK));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "disable")) {
        Ok(HRuntime_DmBenchTakeControl() &&
           (DmActuator_Disable() == drivers::DRIVER_OK));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "clear")) {
        Ok(HRuntime_DmBenchTakeControl() &&
           (DmActuator_ClearError() == drivers::DRIVER_OK));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "hold")) {
        Ok(HRuntime_DmBenchTakeControl() &&
           (DmActuator_HoldCurrent() == drivers::DRIVER_OK));
        return;
    }
    if ((argc == 2) && Equal(argv[1], "level")) {
        const uint32_t now_ms = services::Time_Millis();
        const DmFeedback dm = DmActuator_GetFeedback();
        HConfig *config = HRuntime_GetConfig();
        const bool stopped =
            (dm.velocity_mrad_s >= -50) &&
            (dm.velocity_mrad_s <= 50);
        if (!dm.valid || (dm.state != 1U) ||
            ((now_ms - dm.received_ms) > 100U) || !stopped) {
            Ok(false);
            return;
        }
        HConfig candidate = *config;
        const int32_t offset =
            dm.position_mrad - candidate.dm_position_mrad[2];
        bool in_range = true;
        for (uint8_t i = 0U; i < 5U; i++) {
            const int32_t shifted =
                static_cast<int32_t>(candidate.dm_position_mrad[i]) +
                offset;
            if ((shifted < -12500) || (shifted > 12500)) {
                in_range = false;
                break;
            }
            candidate.dm_position_mrad[i] =
                static_cast<int16_t>(shifted);
        }
        if (in_range && HConfig_Validate(&candidate)) {
            *config = candidate;
            Ok(true);
        } else {
            Ok(false);
        }
        return;
    }
    if ((argc == 2) && Equal(argv[1], "auto")) {
        HRuntime_DmBenchReleaseControl();
        Ok(true);
        return;
    }
    if ((argc == 3) && Equal(argv[1], "position")) {
        int32_t target_mrad = 0;
        const bool valid = ParseInt(argv[2], &target_mrad) &&
            (target_mrad >= -1000) && (target_mrad <= 1000) &&
            DmActuator_IsReady();
        Ok(valid && HRuntime_DmBenchTakeControl() &&
           (DmActuator_SetPosition(target_mrad) ==
            drivers::DRIVER_OK));
        return;
    }
    Ok(false);
}

void PrintConfig(void)
{
    const HConfig *c = HRuntime_GetConfig();
    services::Shell_Write("wheel_radius_um=");
    services::Shell_WriteInt(c->wheel_radius_um);
    services::Shell_Write(" cpr_l=");
    services::Shell_WriteInt(c->left_counts_per_rev);
    services::Shell_Write(" cpr_r=");
    services::Shell_WriteInt(c->right_counts_per_rev);
    services::Shell_Write(" lap_mm=");
    services::Shell_WriteInt(c->lap_distance_mm);
    services::Shell_Write(" h2_loop_offset=");
    services::Shell_WriteInt(c->h2_loop_offset_mm);
    services::Shell_Write(" cruise=");
    services::Shell_WriteInt(c->cruise_rpm);
    services::Shell_Write(" approach=");
    services::Shell_WriteInt(c->approach_rpm);
    services::Shell_Write(" h4=");
    services::Shell_WriteInt(c->h4_cruise_rpm);
    services::Shell_Write("/ramp=");
    services::Shell_WriteInt(c->h4_launch_ramp_rpm_s);
    services::Shell_Write("/brake=");
    services::Shell_WriteInt(c->h4_stop_ramp_rpm_s);
    services::Shell_Write("/brake_at=");
    services::Shell_WriteInt(c->h4_brake_distance_mm);
    services::Shell_Write("/B=");
    services::Shell_WriteInt(c->h4_b_distance_mm);
    services::Shell_Write("/stop=");
    services::Shell_WriteInt(c->h4_stop_distance_mm);
    services::Shell_Write("/heading=");
    services::Shell_WriteInt(c->h4_heading_kp);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->h4_heading_max_correction_rpm);
    services::Shell_Write("/ball_ff=");
    services::Shell_WriteInt(c->ball_chassis_ff_permille);
    services::Shell_Write(" h5=");
    services::Shell_WriteInt(c->h5_cruise_rpm);
    services::Shell_Write("/curve=");
    services::Shell_WriteInt(c->h5_approach_rpm);
    services::Shell_Write("/ramp=");
    services::Shell_WriteInt(c->h5_launch_ramp_rpm_s);
    services::Shell_Write("/brake=");
    services::Shell_WriteInt(c->h5_stop_ramp_rpm_s);
    services::Shell_Write("/brake_at=");
    services::Shell_WriteInt(c->h5_brake_distance_mm);
    services::Shell_Write(" h6=");
    services::Shell_WriteInt(c->h6_cruise_rpm);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->h6_approach_rpm);
    services::Shell_Write(" line_kp=");
    services::Shell_WriteInt(c->line_kp_milli);
    services::Shell_Write(" line_kd=");
    services::Shell_WriteInt(c->line_kd_milli);
    services::Shell_Write(" model_roll=");
    services::Shell_WriteInt(c->ball_model_roll_gain_permille);
    services::Shell_Write(" model_tau=");
    services::Shell_WriteInt(c->ball_model_response_ms);
    services::Shell_Write(" model_plan_accel=");
    services::Shell_WriteInt(c->ball_model_plan_accel_0p1mm_s2);
    services::Shell_Write(" model_vmax=");
    services::Shell_WriteInt(c->ball_model_max_velocity_0p1mm_s);
    services::Shell_Write(" model_breakaway=");
    services::Shell_WriteInt(c->ball_kp_mdeg_per_0p1mm);
    services::Shell_Write(" model_curve=");
    services::Shell_WriteInt(c->ball_kd_mdeg_per_0p1mm_s);
    services::Shell_Write(" model_rolling_friction=");
    services::Shell_WriteInt(c->ball_accel_ff_mdeg_per_mm_s2);
    services::Shell_Write(" observer=");
    services::Shell_WriteInt(c->ball_observer_alpha_permille);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->ball_observer_beta_permille);
    services::Shell_Write(" ball_pid=");
    services::Shell_WriteInt(c->ball_pid_kp_mdeg_per_mm);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->ball_pid_ki_mdeg_per_mm_s);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->ball_pid_kd_mdeg_per_mm_s);
    services::Shell_Write(" ilim=");
    services::Shell_WriteInt(c->ball_pid_integral_limit_mdeg);
    services::Shell_Write(" vision_invert=");
    services::Shell_WriteInt(c->vision_position_invert);
    services::Shell_Write(" ball_zero0.1=");
    services::Shell_WriteInt(c->ball_zero_offset_0p1mm);
    services::Shell_Write("\r\n");
    services::Shell_Write("ball_hold=");
    for (uint8_t i = 0U; i < 5U; i++) {
        if (i != 0U) {
            services::Shell_Write(",");
        }
        services::Shell_WriteInt(c->ball_hold_position_0p1mm[i]);
        services::Shell_Write("/");
        services::Shell_WriteInt(c->ball_hold_angle_mdeg[i]);
    }
    services::Shell_Write("\r\n");
    services::Shell_Write("gray threshold=");
    services::Shell_WriteInt(c->grayscale.threshold);
    services::Shell_Write(" hysteresis=");
    services::Shell_WriteInt(c->grayscale.hysteresis);
    services::Shell_Write(" min_strength=");
    services::Shell_WriteInt(c->grayscale.min_line_strength);
    services::Shell_Write(" track_mask=");
    services::Shell_WriteInt(c->grayscale.track_mask);
    services::Shell_Write("\r\n");
    services::Shell_Write("motor speed_pid=");
    services::Shell_WriteInt(c->motor_speed_kp_q4_4);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_speed_ki_q4_4);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_speed_kd_q4_4);
    services::Shell_Write(" duty=");
    services::Shell_WriteInt(c->motor_speed_min_duty);
    services::Shell_Write("-");
    services::Shell_WriteInt(c->motor_speed_max_duty);
    services::Shell_Write(" ramp=");
    services::Shell_WriteInt(c->motor_speed_accel_rpm_s);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_speed_decel_rpm_s);
    services::Shell_Write(" invert=");
    services::Shell_WriteInt(c->motor_output_invert_flags);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_encoder_invert_flags);
    services::Shell_Write("\r\n");
    services::Shell_Write("motor position_pid=");
    services::Shell_WriteInt(c->motor_position_kp_q4_4);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_position_ki_q4_4);
    services::Shell_Write("/");
    services::Shell_WriteInt(c->motor_position_kd_q4_4);
    services::Shell_Write(" max_rpm=");
    services::Shell_WriteInt(c->motor_position_max_rpm);
    services::Shell_Write(" tolerance=");
    services::Shell_WriteInt(c->motor_position_tolerance_counts);
    services::Shell_Write("\r\n");
    for (uint8_t i = 0U; i < 5U; i++) {
        services::Shell_Write("map ");
        services::Shell_WriteInt(i);
        services::Shell_Write(" beam_mdeg=");
        services::Shell_WriteInt(c->beam_angle_mdeg[i]);
        services::Shell_Write(" dm_mrad=");
        services::Shell_WriteInt(c->dm_position_mrad[i]);
        services::Shell_Write("\r\n");
    }
    for (uint8_t i = 0U; i < 5U; i++) {
        services::Shell_Write("hold ");
        services::Shell_WriteInt(i);
        services::Shell_Write(" x0.1mm=");
        services::Shell_WriteInt(c->ball_hold_position_0p1mm[i]);
        services::Shell_Write(" angle_mdeg=");
        services::Shell_WriteInt(c->ball_hold_angle_mdeg[i]);
        services::Shell_Write("\r\n");
    }
}

bool SetConfigValue(const char *name, int32_t value)
{
    if ((value < -32768) || (value > 32767)) {
        return false;
    }
    HConfig *c = HRuntime_GetConfig();
    const HConfig old = *c;
    if (Equal(name, "cruise")) c->cruise_rpm = value;
    else if (Equal(name, "approach")) c->approach_rpm = value;
    else if (Equal(name, "h4_speed")) c->h4_cruise_rpm = value;
    else if (Equal(name, "h4_launch_ramp")) {
        c->h4_launch_ramp_rpm_s = value;
    }
    else if (Equal(name, "h4_stop_ramp")) {
        c->h4_stop_ramp_rpm_s = value;
    }
    else if (Equal(name, "h4_brake_distance")) {
        c->h4_brake_distance_mm = value;
    }
    else if (Equal(name, "h4_b_distance")) {
        c->h4_b_distance_mm = value;
    }
    else if (Equal(name, "h4_stop_distance")) {
        c->h4_stop_distance_mm = value;
    }
    else if (Equal(name, "h4_heading_kp")) {
        c->h4_heading_kp = value;
    }
    else if (Equal(name, "h4_heading_max_corr")) {
        c->h4_heading_max_correction_rpm = value;
    }
    else if (Equal(name, "imu_gyro_bias_z")) {
        c->imu_gyro_bias_z_mdps = value;
    }
    else if (Equal(name, "ball_chassis_ff")) {
        c->ball_chassis_ff_permille = value;
    }
    else if (Equal(name, "h5_speed")) c->h5_cruise_rpm = value;
    else if (Equal(name, "h5_curve_speed")) {
        c->h5_approach_rpm = value;
    }
    else if (Equal(name, "h5_launch_ramp")) {
        c->h5_launch_ramp_rpm_s = value;
    }
    else if (Equal(name, "h5_stop_ramp")) {
        c->h5_stop_ramp_rpm_s = value;
    }
    else if (Equal(name, "h5_brake_distance")) {
        c->h5_brake_distance_mm = value;
    }
    else if (Equal(name, "h6_speed")) c->h6_cruise_rpm = value;
    else if (Equal(name, "h6_approach")) c->h6_approach_rpm = value;
    else if (Equal(name, "finish_gate")) c->finish_gate_mm = value;
    else if (Equal(name, "approach_start")) c->approach_start_mm = value;
    else if (Equal(name, "h6_finish_gate")) c->h6_finish_gate_mm = value;
    else if (Equal(name, "h6_approach_start")) c->h6_approach_start_mm = value;
    else if (Equal(name, "h2_loop_offset") ||
             Equal(name, "finish_offset")) {
        c->h2_loop_offset_mm = value;
    }
    else if (Equal(name, "line_kp")) c->line_kp_milli = value;
    else if (Equal(name, "line_kd")) c->line_kd_milli = value;
    else if (Equal(name, "gray_threshold")) c->grayscale.threshold = value;
    else if (Equal(name, "gray_hysteresis")) c->grayscale.hysteresis = value;
    else if (Equal(name, "gray_position_floor")) c->grayscale.position_floor = value;
    else if (Equal(name, "gray_min_strength")) c->grayscale.min_line_strength = value;
    else if (Equal(name, "gray_track_mask")) c->grayscale.track_mask = value;
    else if (Equal(name, "speed_kp")) c->motor_speed_kp_q4_4 = value;
    else if (Equal(name, "speed_ki")) c->motor_speed_ki_q4_4 = value;
    else if (Equal(name, "speed_kd")) c->motor_speed_kd_q4_4 = value;
    else if (Equal(name, "speed_max_duty")) c->motor_speed_max_duty = value;
    else if (Equal(name, "speed_min_duty")) c->motor_speed_min_duty = value;
    else if (Equal(name, "speed_accel_rpm_s")) c->motor_speed_accel_rpm_s = value;
    else if (Equal(name, "speed_decel_rpm_s")) c->motor_speed_decel_rpm_s = value;
    else if (Equal(name, "position_kp")) c->motor_position_kp_q4_4 = value;
    else if (Equal(name, "position_ki")) c->motor_position_ki_q4_4 = value;
    else if (Equal(name, "position_kd")) c->motor_position_kd_q4_4 = value;
    else if (Equal(name, "position_max_rpm")) c->motor_position_max_rpm = value;
    else if (Equal(name, "position_tolerance_counts")) {
        c->motor_position_tolerance_counts = value;
    }
    else if (Equal(name, "motor_output_invert_flags")) {
        c->motor_output_invert_flags = value;
    }
    else if (Equal(name, "motor_encoder_invert_flags")) {
        c->motor_encoder_invert_flags = value;
    }
    else if (Equal(name, "ball_kp")) c->ball_kp_mdeg_per_0p1mm = value;
    else if (Equal(name, "ball_kd")) c->ball_kd_mdeg_per_0p1mm_s = value;
    else if (Equal(name, "ball_ki")) c->ball_ki_mdeg_per_0p1mm_s = value;
    else if (Equal(name, "model_roll")) {
        c->ball_model_roll_gain_permille = value;
    }
    else if (Equal(name, "model_tau")) c->ball_model_response_ms = value;
    else if (Equal(name, "model_plan_accel")) {
        c->ball_model_plan_accel_0p1mm_s2 = value;
    }
    else if (Equal(name, "model_vmax")) {
        c->ball_model_max_velocity_0p1mm_s = value;
    }
    else if (Equal(name, "model_friction")) {
        c->ball_accel_ff_mdeg_per_mm_s2 = value;
    }
    else if (Equal(name, "model_breakaway")) {
        c->ball_kp_mdeg_per_0p1mm = value;
    }
    else if (Equal(name, "model_curvature")) {
        c->ball_kd_mdeg_per_0p1mm_s = value;
    }
    else if (Equal(name, "model_rolling_friction")) {
        c->ball_accel_ff_mdeg_per_mm_s2 = value;
    }
    else if (Equal(name, "observer_alpha")) {
        c->ball_observer_alpha_permille = value;
    }
    else if (Equal(name, "observer_beta")) {
        c->ball_observer_beta_permille = value;
    }
    else if (Equal(name, "ball_pid_kp")) {
        c->ball_pid_kp_mdeg_per_mm = value;
    }
    else if (Equal(name, "ball_pid_ki")) {
        c->ball_pid_ki_mdeg_per_mm_s = value;
    }
    else if (Equal(name, "ball_pid_kd")) {
        c->ball_pid_kd_mdeg_per_mm_s = value;
    }
    else if (Equal(name, "ball_pid_ilim")) {
        c->ball_pid_integral_limit_mdeg = value;
    }
    else if (Equal(name, "accel_ff")) c->ball_accel_ff_mdeg_per_mm_s2 = value;
    else if (Equal(name, "max_angle")) c->ball_max_angle_mdeg = value;
    else if (Equal(name, "vision_invert")) c->vision_position_invert = value;
    else if (Equal(name, "ball_zero")) {
        c->ball_zero_offset_0p1mm = value;
    }
    else if (Equal(name, "beam0")) c->beam_angle_mdeg[0] = value;
    else if (Equal(name, "beam1")) c->beam_angle_mdeg[1] = value;
    else if (Equal(name, "beam2")) c->beam_angle_mdeg[2] = value;
    else if (Equal(name, "beam3")) c->beam_angle_mdeg[3] = value;
    else if (Equal(name, "beam4")) c->beam_angle_mdeg[4] = value;
    else if (Equal(name, "dm0")) c->dm_position_mrad[0] = value;
    else if (Equal(name, "dm1")) c->dm_position_mrad[1] = value;
    else if (Equal(name, "dm2")) c->dm_position_mrad[2] = value;
    else if (Equal(name, "dm3")) c->dm_position_mrad[3] = value;
    else if (Equal(name, "dm4")) c->dm_position_mrad[4] = value;
    else if (Equal(name, "hold_x0")) c->ball_hold_position_0p1mm[0] = value;
    else if (Equal(name, "hold_x1")) c->ball_hold_position_0p1mm[1] = value;
    else if (Equal(name, "hold_x2")) c->ball_hold_position_0p1mm[2] = value;
    else if (Equal(name, "hold_x3")) c->ball_hold_position_0p1mm[3] = value;
    else if (Equal(name, "hold_x4")) c->ball_hold_position_0p1mm[4] = value;
    else if (Equal(name, "hold_a0")) c->ball_hold_angle_mdeg[0] = value;
    else if (Equal(name, "hold_a1")) c->ball_hold_angle_mdeg[1] = value;
    else if (Equal(name, "hold_a2")) c->ball_hold_angle_mdeg[2] = value;
    else if (Equal(name, "hold_a3")) c->ball_hold_angle_mdeg[3] = value;
    else if (Equal(name, "hold_a4")) c->ball_hold_angle_mdeg[4] = value;
    else return false;
    if (!HConfig_Validate(c)) {
        *c = old;
        return false;
    }
    return true;
}

void ConfigCommand(int argc, const char * const argv[])
{
    if ((argc == 2) && Equal(argv[1], "show")) {
        PrintConfig();
        return;
    }
    if ((argc == 2) && Equal(argv[1], "save")) {
        Ok(HRuntime_SaveConfig());
        return;
    }
    if ((argc == 2) && Equal(argv[1], "defaults")) {
        HRuntime_DefaultConfig();
        Ok(true);
        return;
    }
    if ((argc == 4) && Equal(argv[1], "set") &&
        !HApp_IsRunning(HRuntime_GetState())) {
        int32_t value = 0;
        Ok(ParseInt(argv[3], &value) &&
           SetConfigValue(argv[2], value));
        return;
    }
    Ok(false);
}

void TelemetryCommand(int argc, const char * const argv[])
{
    if ((argc == 2) &&
        (Equal(argv[1], "on") || Equal(argv[1], "off"))) {
        const bool enabled = Equal(argv[1], "on");
        HRuntime_SetTelemetry(enabled);
        if (enabled) {
            services::Shell_WriteLine(
                "kind,t_ms,state,line_pos,line_mask,left_rpm,right_rpm,"
                "distance_mm,ball_0p1mm,ball_vel_0p1mm_s,"
                "ball_target_vel_0p1mm_s,ball_vel_error_0p1mm_s,"
                "error_0p1mm,"
                "beam_mdeg,imu_beam_mdeg,imu_comp_mdeg,"
                "dm_mrad,fault_count");
        }
        Ok(true);
        return;
    }
    Ok(false);
}

} /* namespace */

void HShell_Init(void)
{
    (void)services::Shell_Register("status", "system/task status", StatusCommand);
    (void)services::Shell_Register(
        "task", "task 2..8 (7=CAL_ZERO 8=CAL_H2_LOOP)", TaskCommand);
    (void)services::Shell_Register("start", "start selected task", StartCommand);
    (void)services::Shell_Register("stop", "abort and stop", StopCommand);
    (void)services::Shell_Register("fault", "fault clear", FaultCommand);
    (void)services::Shell_Register(
        "gray", "gray raw|status|white|black", GrayCommand);
    (void)services::Shell_Register(
        "chassis", "chassis status|test L R|stop", ChassisCommand);
    (void)services::Shell_Register(
        "vision", "vision status|stats|trace on|off|inject P [C D]|clear",
        VisionCommand);
    (void)services::Shell_Register("imu", "imu status", ImuCommand);
    (void)services::Shell_Register(
        "ball", "ball status|hold mm|move mm|stop", BallCommand);
    (void)services::Shell_Register(
        "dm", "dm status|enable|hold|level|position mrad|clear|disable",
        DmCommand);
    (void)services::Shell_Register(
        "config", "config show|set N V|save|defaults", ConfigCommand);
    (void)services::Shell_Register(
        "telem", "telem on|off", TelemetryCommand);
}

} /* namespace gugah */
