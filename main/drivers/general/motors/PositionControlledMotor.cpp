#include "drivers/motors/PositionControlledMotor.h"
#include "../Submodules/FastPID/src/FastPID.h"
#include <cstdlib>
//#include "motor_ctrl_timer.h"

#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 15 // Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16 // Set GPIO 16 as PWM0B

float Kp = 0.1, Ki = 0.5, Kd = 0.5, Hz = 50;
int output_bits = 12;
bool output_signed = true;

//-2^(n-1)  to  (2^(n-1))-1
float Divisor = ((2 ^ (output_bits - output_signed)) / 100.0);

/**
 * @brief the minimum duty cycle below which the motor will stop spining
 *
 */
const int16_t hysteresis = int(2.0 * Divisor);
/**
 * @brief the minimum duty cycle below which the motor will not start spining
 *
 */
const int16_t MinimumDutyCycle = int(5.0 * Divisor);

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

PosContMot PosContMot::PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint8_t GPIOa, uint8_t GPIOb)
{
    PosContMot ctrl = PosContMot();
    ctrl.this_Timer = timer;
    ctrl.this_Unit = unit;
    ctrl.GPIOa = GPIOa;
    ctrl.GPIOb = GPIOb;
    return ctrl;
}

PosContMot PosContMot::PosContMotCreate(uint8_t unit, uint8_t timer, uint8_t GPIOa, uint8_t GPIOb)
{
    PosContMot ctrl = PosContMot();
    ctrl.this_Timer = (mcpwm_timer_t)timer;
    ctrl.this_Unit = (mcpwm_unit_t)unit;
    ctrl.GPIOa = GPIOa;
    ctrl.GPIOb = GPIOb;
    return ctrl;
}

PosContMot::PosContMot()
{
    McPwm_Config.frequency = 1000; // frequency = 500Hz,
    McPwm_Config.cmpr_a = 0;       // duty cycle of PWMxA = 0
    McPwm_Config.cmpr_b = 0;       // duty cycle of PWMxb = 0
    McPwm_Config.counter_mode = MCPWM_UP_COUNTER;
    McPwm_Config.duty_mode = MCPWM_DUTY_MODE_0;
}

PosContMot::~PosContMot()
{
}

bool PosContMot::begin()
{

    // 1. mcpwm gpio initialization
    mcpwm_gpio_init(this_Unit, (mcpwm_io_signals_t)(0 + (2 * (int)this_Timer)), GPIOa);
    mcpwm_gpio_init(this_Unit, (mcpwm_io_signals_t)(1 + (2 * (int)this_Timer)), GPIOb);

    // 2. initial mcpwm configuration
    mcpwm_init(this_Unit, this_Timer, &McPwm_Config); // Configure PWM0A & PWM0B with above settings
    return true;
}

bool SameSign(int x, int y)
{
    return (x >= 0) ^ (y < 0);
}

void PosContMot::SetPos(int32_t Position)
{
    // todo: get current Pos and run PID
    int16_t PidOut = Position;//- 30000; //(Position - ((UINT16_MAX - 2) / 2)) >> 4;
    int16_t PidMag = abs(PidOut);
    MtrDirection NextDirection;

    // apply hysteresis logic to PID output value
    if (previous_Duty == 0)
    {
        // we were in a stoped state
        if (PidMag < MinimumDutyCycle)
        {
            PidOut = 0;
            NextDirection = Stationary;
        }
    }
    else
    {
        if (PidMag < hysteresis)
        {
            PidOut = 0;
            NextDirection = Stationary;
        }
    }

    // determine what dirrection the PID is comanding the motor in
    if (PidOut >= 1)
    {
        NextDirection = Forward;
    }
    else if (PidOut <= -1)
    {
        NextDirection = Reverse;
    }

    // when we change we need to set the PWM signal for the previous direction low
    // if (NextDirection != current_Direction)
    // {
    //     switch (current_Direction)
    //     {
    //     case Forward:
    //         mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
    //         break;
    //     case Reverse:
    //         mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
    //         break;
    //     default:
    //         mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
    //         mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
    //         break;
    //     }
    // }

    // set the duty cycle for the current dirrection
    switch (NextDirection)
    {
    case Forward:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_A, ((float)PidMag / 327.6));
        if (NextDirection != current_Direction)
            mcpwm_set_duty_type(this_Unit, this_Timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
        break;
    case Reverse:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_B, (((float)PidMag) / 327.6));
        if (NextDirection != current_Direction)
            mcpwm_set_duty_type(this_Unit, this_Timer, MCPWM_GEN_B, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
        break;
    default:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
        break;
    }

    previous_Duty = PidOut;
    current_Direction = NextDirection;
}