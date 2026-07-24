#ifndef MOTOR_OPEN_LOOP_H_
#define MOTOR_OPEN_LOOP_H_

#include <stdbool.h>
#include <stdint.h>

#include "foc_config.h"

/*
 * The file name is retained so the existing CCS managed build keeps working,
 * but this interface now implements encoder-sensored FOC rather than the old
 * six-step open-loop commutator.
 */

#define MOTOR_FOC_DEFAULT_TARGET_RPM       FOC_CFG_DEFAULT_TARGET_RPM
#define MOTOR_FOC_TARGET_RPM_STEP          FOC_CFG_TARGET_RPM_STEP
#define MOTOR_FOC_MAX_TARGET_RPM           FOC_CFG_MAX_TARGET_RPM
#define MOTOR_FOC_POSITION_STEP_DEGREES    FOC_CFG_POSITION_STEP_DEGREES

typedef enum {
    MOTOR_FOC_MODE_SPEED = 0,
    MOTOR_FOC_MODE_POSITION,
} MOTOR_FOC_ControlMode;

typedef enum {
    MOTOR_FOC_STATE_STOPPED = 0,
    MOTOR_FOC_STATE_ALIGNING,
    MOTOR_FOC_STATE_RUNNING,
    MOTOR_FOC_STATE_FAULT,
} MOTOR_FOC_State;

typedef enum {
    MOTOR_FOC_STOP_NONE = 0,
    MOTOR_FOC_STOP_COMMAND,
    MOTOR_FOC_STOP_DRIVER_FAULT,
    MOTOR_FOC_STOP_ENCODER_LOST,
    MOTOR_FOC_STOP_OVERCURRENT,
} MOTOR_FOC_StopReason;

typedef struct {
    MOTOR_FOC_State state;
    MOTOR_FOC_StopReason stopReason;
    MOTOR_FOC_ControlMode controlMode;
    int32_t targetRpm;
    int32_t measuredRpm;
    int32_t positionErrorCounts;
    int32_t iqReferenceMilliamps;
    int32_t idMilliamps;
    int32_t iqMilliamps;
    uint16_t dutyApermille;
    uint16_t dutyBpermille;
    uint16_t dutyCpermille;
    uint16_t encoderRaw;
    uint16_t targetPositionRaw;
    uint16_t electricalAngleSample;
    uint16_t electricalAnglePredicted;
    int32_t electricalVelocityQ16;
} MOTOR_FOC_Status;

void MOTOR_FOC_initialize(void);
void MOTOR_FOC_enableAdcTrigger(void);
bool MOTOR_FOC_beginAlignment(void);
bool MOTOR_FOC_finishAlignment(
    uint16_t encoderRaw, uint32_t currentSampleSequence);
void MOTOR_FOC_stop(void);
void MOTOR_FOC_emergencyStop(MOTOR_FOC_StopReason reason);

void MOTOR_FOC_onEncoderSample(
    uint16_t encoderRaw, bool valid, uint32_t currentSampleSequence);
void MOTOR_FOC_currentLoopISR(int16_t phaseACounts, int16_t phaseBCounts,
    int16_t phaseCCounts, uint32_t currentSampleSequence);

bool MOTOR_FOC_isRunning(void);
bool MOTOR_FOC_isActive(void);
MOTOR_FOC_State MOTOR_FOC_getState(void);
MOTOR_FOC_StopReason MOTOR_FOC_getStopReason(void);
void MOTOR_FOC_clearStopReason(void);

void MOTOR_FOC_adjustTargetRpm(int32_t deltaRpm);
void MOTOR_FOC_reverse(void);
bool MOTOR_FOC_enterPositionMode(void);
void MOTOR_FOC_enterSpeedMode(void);
void MOTOR_FOC_adjustTargetPositionDegrees(int32_t deltaDegrees);
MOTOR_FOC_ControlMode MOTOR_FOC_getControlMode(void);
void MOTOR_FOC_getStatus(MOTOR_FOC_Status *status);
int32_t MOTOR_FOC_getTargetRpm(void);
int32_t MOTOR_FOC_getMeasuredRpm(void);
int32_t MOTOR_FOC_getIqReferenceMilliamps(void);
int32_t MOTOR_FOC_getIdMilliamps(void);
int32_t MOTOR_FOC_getIqMilliamps(void);
uint16_t MOTOR_FOC_getElectricalOffset(void);
uint16_t MOTOR_FOC_getEncoderRaw(void);
uint16_t MOTOR_FOC_getDutyApermille(void);
uint16_t MOTOR_FOC_getDutyBpermille(void);
uint16_t MOTOR_FOC_getDutyCpermille(void);

#endif
