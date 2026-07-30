#ifndef APP_ACTION_H_
#define APP_ACTION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

static const uint8_t ACTION_MAX_INSTRS = 54U;

/* Instruction-table interpreter: each instruction runs an action until a
 * condition (or safety timeout), then jumps to on_success / on_timeout. This
 * replaces the old linear timed-action sequencer with condition-driven
 * branching (follow-until-line-lost, turn-until-heading-reached, branch on
 * line/button, loop). */

enum ActionOp {
    ACT_OP_NONE = 0,
    ACT_OP_DRIVE = 1,    /* drive straight (heading hold) until cond */
    ACT_OP_TURN = 2,     /* turn to relative angle, until heading_reached */
    ACT_OP_FOLLOW = 3,   /* line-follow until cond */
    ACT_OP_WAIT = 4,     /* wait until cond (usually timeout) */
    ACT_OP_STOP = 5,     /* stop chassis+heading+linefollow, immediate */
    ACT_OP_BRANCH = 6,   /* no motion; branch on condition */
    ACT_OP_END = 7,      /* finish the sequence (success) */
    ACT_OP_DRIVE_MM = 8, /* encoder distance + heading hold */
    ACT_OP_LED_ON = 9,   /* p1: 0=LED2+3, 2=LED2, 3=LED3 */
    ACT_OP_LED_OFF = 10,
    ACT_OP_LED_TOGGLE = 11,
    ACT_OP_BUZZER_ON = 12, /* p1=0; p2=auto-off ms */
    ACT_OP_BUZZER_OFF = 13,
    ACT_OP_BUZZER_TOGGLE = 14,
    ACT_OP_CONDITION = 15, /* generic immediate/wait comparison */
    ACT_OP_DRIVE_IF = 16,  /* heading hold until generic comparison */
    ACT_OP_FOLLOW_IF = 17, /* line follow until generic comparison */
    ACT_OP_LOOP = 18,      /* counted loop: success=body, timeout=done */
    ACT_OP_ROAD_NAV = 19,  /* follow through next junction by fixed route */
    ACT_OP_DM_POSITION = 20,
    ACT_OP_DM_SPEED = 21,
    ACT_OP_DM_DISABLE = 22,
    ACT_OP_BALL_HOLD = 23,
    ACT_OP_BALL_MOVE = 24,
    ACT_OP_BALL_DISABLE = 25,
    ACT_OP_TRACK_COURSE = 26
};

enum ActionCond {
    ACT_COND_TIMEOUT = 0,        /* complete when param2 ms elapsed */
    ACT_COND_HEADING_REACHED,   /* heading returned to IDLE (turn done) */
    ACT_COND_LINE_DETECTED,     /* grayscale sees the line */
    ACT_COND_LINE_LOST,         /* grayscale lost the line */
    ACT_COND_BUTTON,            /* button 1 pressed */
    ACT_COND_IMMEDIATE,         /* always true (instant) */
    ACT_COND_DISTANCE_REACHED,  /* encoder distance mode returned to idle */
    ACT_COND_DM_ABSOLUTE,
    ACT_COND_DM_RELATIVE
};

/* Generic comparison sources. Values fit in the upper five bits of the
 * persisted condition spec; the lower three bits hold ActionCompareOp. */
enum ActionConditionSource : uint8_t {
    ACT_SOURCE_CONSTANT = 0U,
    ACT_SOURCE_BUTTON1_LEVEL,
    ACT_SOURCE_BUTTON1_PRESSED,
    ACT_SOURCE_BUTTON2_LEVEL,
    ACT_SOURCE_BUTTON2_PRESSED,
    ACT_SOURCE_BUTTON3_LEVEL,
    ACT_SOURCE_BUTTON3_PRESSED,
    ACT_SOURCE_LINE_DETECTED,
    ACT_SOURCE_LINE_POSITION_MPOS,
    ACT_SOURCE_LINE_CONFIDENCE,
    ACT_SOURCE_ROAD_TYPE,
    ACT_SOURCE_ROAD_EVENT_TYPE,
    ACT_SOURCE_ROAD_EVENT_PATHS,
    ACT_SOURCE_IMU_VALID,
    ACT_SOURCE_IMU_YAW_MDEG,
    ACT_SOURCE_IMU_PITCH_MDEG,
    ACT_SOURCE_IMU_ROLL_MDEG,
    ACT_SOURCE_IMU_GYRO_Z_MDPS,
    ACT_SOURCE_CHASSIS_FEEDBACK_VALID,
    ACT_SOURCE_LEFT_ACTUAL_RPM,
    ACT_SOURCE_RIGHT_ACTUAL_RPM,
    ACT_SOURCE_LEFT_DISTANCE_MM,
    ACT_SOURCE_RIGHT_DISTANCE_MM,
    ACT_SOURCE_AVERAGE_DISTANCE_MM,
    ACT_SOURCE_CHASSIS_INITIALIZED,
    ACT_SOURCE_HEADING_MODE,
    ACT_SOURCE_LINEFOLLOW_MODE,
    ACT_SOURCE_APP_MODE,
    ACT_SOURCE_COUNT
};

enum ActionCompareOp : uint8_t {
    ACT_COMPARE_EQ = 0U,
    ACT_COMPARE_NE,
    ACT_COMPARE_LT,
    ACT_COMPARE_LE,
    ACT_COMPARE_GT,
    ACT_COMPARE_GE,
    ACT_COMPARE_CONTAINS,
    ACT_COMPARE_NOT_CONTAINS
};

enum ActionConditionMode : uint8_t {
    ACT_CONDITION_IMMEDIATE = 0U,
    ACT_CONDITION_WAIT
};

struct ActionConditionConfig {
    ActionConditionSource source;
    ActionCompareOp compare;
    int32_t value;
    ActionConditionMode mode;
    uint16_t timeout_ms;
    uint16_t stable_ms;
};

/* on_success / on_timeout target. ACT_NEXT in on_success = next instruction;
 * ACT_NEXT in on_timeout = abort (fail). A real index 0..count-1 = goto. */
static const uint8_t ACT_NEXT = 0xFFU;

struct Instr {
    ActionOp op;
    int32_t param1;       /* motion parameter or LED target */
    int32_t param2;       /* timeout/duration, max rpm, or auto-off ms */
    ActionCond until;     /* success condition */
    int32_t condition_value; /* generic comparison threshold; otherwise 0 */
    uint8_t on_success;   /* ACT_NEXT or index */
    uint8_t on_timeout;   /* ACT_NEXT(=abort) or index */
};

enum ActionRunResult {
    ACT_RUN_IDLE = 0,
    ACT_RUN_RUNNING,
    ACT_RUN_SUCCESS,
    ACT_RUN_ABORTED,
    ACT_RUN_CANCELLED,
    ACT_RUN_FAULT,
    ACT_RUN_TIMEOUT,
    ACT_RUN_INVALID
};

enum ActionFailureReason {
    ACT_FAIL_NONE = 0,
    ACT_FAIL_START,
    ACT_FAIL_INSTR_TIMEOUT,
    ACT_FAIL_SEQUENCE_TIMEOUT,
    ACT_FAIL_FAULT,
    ACT_FAIL_CANCELLED,
    ACT_FAIL_INVALID,
    ACT_FAIL_CONDITION_FALSE,
    ACT_FAIL_CONDITION_TIMEOUT,
    ACT_FAIL_CONDITION_UNAVAILABLE,
    ACT_FAIL_ROUTE_UNAVAILABLE,
    ACT_FAIL_ROUTE_REACQUIRE_FAILED,
    ACT_FAIL_BALL_VISION_LOST,
    ACT_FAIL_BALL_ENDPOINT,
    ACT_FAIL_BALL_CONTROL,
    ACT_FAIL_COURSE
};

enum ActionValidationField {
    ACT_VALID_FIELD_NONE = 0,
    ACT_VALID_FIELD_TABLE,
    ACT_VALID_FIELD_OP,
    ACT_VALID_FIELD_PARAM1,
    ACT_VALID_FIELD_PARAM2,
    ACT_VALID_FIELD_CONDITION,
    ACT_VALID_FIELD_ON_SUCCESS,
    ACT_VALID_FIELD_ON_TIMEOUT
};

enum ActionValidationReason {
    ACT_VALID_OK = 0,
    ACT_VALID_EMPTY,
    ACT_VALID_TOO_MANY,
    ACT_VALID_UNKNOWN_OP,
    ACT_VALID_OUT_OF_RANGE,
    ACT_VALID_MUST_BE_ZERO,
    ACT_VALID_MUST_BE_NONZERO,
    ACT_VALID_WRONG_CONDITION,
    ACT_VALID_BAD_TARGET
};

struct ActionValidationResult {
    bool valid;
    uint8_t index;
    ActionValidationField field;
    ActionValidationReason reason;
};

struct ActionRunnerState {
    Instr instrs[ACTION_MAX_INSTRS];
    uint8_t count;
    uint8_t current;
    bool running;
    bool last_success;    /* last instr result (for status) */
    uint32_t seq_start_ms;
    uint32_t instr_start_ms;
    bool instr_started;
    drivers::DriverStatus last_status;
    ActionRunResult result;
    ActionFailureReason failure_reason;
    uint8_t failure_index;
};

void ActionRunner_Init(void);
drivers::DriverStatus ActionRunner_Clear(void);
/* Append an instruction. op as text ("drive"/"turn"/...), until as text
 * ("timeout"/"heading_reached"/"line_detected"/"line_lost"/"button"/
 * "immediate"/"distance_reached"). on_success/on_timeout: index, or
 * "next"/"abort" (=ACT_NEXT). */
drivers::DriverStatus ActionRunner_AddInstr(ActionOp op,
                                            int32_t param1,
                                            int32_t param2,
                                            ActionCond until,
                                            uint8_t on_success,
                                            uint8_t on_timeout);
drivers::DriverStatus ActionRunner_AddRoadNav(
    int32_t route,
    int32_t rpm,
    int32_t timeout_ms,
    uint8_t on_success,
    uint8_t on_failure);
drivers::DriverStatus ActionRunner_AddTrackCourse(
    int32_t cruise_rpm,
    int32_t approach_rpm,
    int32_t lap_distance_mm,
    uint8_t on_success,
    uint8_t on_failure);
drivers::DriverStatus ActionRunner_AddDmPosition(
    bool relative,
    int32_t target_mrad,
    int32_t max_velocity_mrad_s,
    int32_t timeout_ms,
    uint8_t on_success,
    uint8_t on_failure);
drivers::DriverStatus ActionRunner_AddCompareInstr(
    ActionOp op,
    int32_t param1,
    const ActionConditionConfig *condition,
    uint8_t on_success,
    uint8_t on_failure);
drivers::DriverStatus ActionRunner_Validate(ActionValidationResult *result);
drivers::DriverStatus ActionRunner_ValidateCompetition(
    ActionValidationResult *result);
drivers::DriverStatus ActionRunner_Start(void);
drivers::DriverStatus ActionRunner_Cancel(void);
void ActionRunner_Update(void);
const ActionRunnerState *ActionRunner_GetState(void);

bool ActionCondition_IsCompareOp(ActionOp op);
bool ActionCondition_Decode(const Instr *instr,
                            ActionConditionConfig *condition);
const char *ActionCondition_SourceText(ActionConditionSource source);
const char *ActionCondition_CompareText(ActionCompareOp compare);

} /* namespace app */

#endif /* APP_ACTION_H_ */
