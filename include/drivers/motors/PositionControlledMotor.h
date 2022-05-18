
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
    static PosContMot PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint8_t GPIOa, uint8_t GPIOb);
    static PosContMot PosContMotCreate(uint8_t unit, uint8_t timer, uint8_t GPIOa, uint8_t GPIOb);
    ~PosContMot();

    bool begin();
    void SetPos(int32_t Position);

private:
    PosContMot();
    mcpwm_config_t McPwm_Config;
    mcpwm_unit_t this_Unit;
    mcpwm_timer_t this_Timer;
    uint8_t GPIOa, GPIOb;
    int16_t previous_Duty;
    MtrDirection current_Direction;
    bool chanel_A_State;
    bool chanel_B_State;
    //  static bool pwm_timmer_init();
};