
#include "../Submodules/Arduino-PID-Library/PID_v1.h"
#include "../Submodules/PID_AutoTune_v0/PID_AutoTune_v0.h"
#include <stdbool.h>
#include <stdint.h>

#include "driver/mcpwm.h"

#define initial_tuning false

#define find_stall_speed false

enum MtrDirection {
    Stationary,
    Forward,
    Reverse
};

struct MtrConfig {
};

class PID_Knocker {
public:
    PID_Knocker(float startStall, float stopStall, uint32_t tolerance);
    float Knock(uint32_t currentTick, float PidOutput, int32_t PosSetpoint, uint32_t PosMeasured, float Velocity);

private:
    uint32_t Tick, StartKnockTick, PositionTolerance;
    float StallSpeedStatic;
    float StallSpeedMoving;
};


class PosContMot {
public:
    static PosContMot PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin, uint8_t direction, uint16_t offset);
    static PosContMot PosContMotCreate(uint8_t unit, uint8_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin, uint8_t direction, uint16_t offset);
    ~PosContMot();

    bool begin();
    void SetPos(int32_t Position, uint32_t Tick);

private:
    PosContMot(uint16_t _GPIOa, uint16_t _GPIOb, uint16_t _FeedbackPin, int direction, uint16_t offset);
    void MoveMotor(MtrDirection NextDirection, float PidMag);
    float FindStallInDirection(MtrDirection SearchDirection);
    PID_Knocker *TheOneWhoKnocks;
    mcpwm_config_t McPwm_Config;
    mcpwm_unit_t this_Unit;
    mcpwm_timer_t this_Timer;
    uint16_t GPIOa, GPIOb, PositionFeedbackPin, ZeroOffset;
    uint8_t PID_Loop_Offset;
    float previous_Duty;
    MtrDirection current_Direction;
    bool chanel_A_State;
    bool chanel_B_State;
    float input, output, setpoint;
    float input_vel, output_vel, setpoint_vel;
    uint8_t direction_Setting;
    PID* positionPID;
    PID* velocityPID;
    PID_ATune* aTune;
    bool tuning = initial_tuning;
    bool findingStall = find_stall_speed;
    float stallSpeed = 61;

    //  static bool pwm_timmer_init();
};

