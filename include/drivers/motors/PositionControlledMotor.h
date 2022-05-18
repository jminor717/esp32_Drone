
#include <stdint.h>
#include <stdbool.h>

#include "driver/mcpwm.h"

enum MtrDirection
{
    Stationary,
    Forward,
    Reverse
};

class PosContMot
{
public:
    static PosContMot PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin);
    static PosContMot PosContMotCreate(uint8_t unit, uint8_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin);
    ~PosContMot();

    bool begin();
    void SetPos(int32_t Position, uint32_t Tick);

private:
    PosContMot(uint16_t _GPIOa, uint16_t _GPIOb, uint16_t _FeedbackPin);
    mcpwm_config_t McPwm_Config;
    mcpwm_unit_t this_Unit;
    mcpwm_timer_t this_Timer;
    uint16_t GPIOa, GPIOb, PositionFeedbackPin;
    uint8_t PID_Loop_Offset;
    int16_t previous_Duty;
    MtrDirection current_Direction;
    bool chanel_A_State;
    bool chanel_B_State;
    //  static bool pwm_timmer_init();
};