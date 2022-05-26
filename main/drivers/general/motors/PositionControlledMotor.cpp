#include "drivers/motors/PositionControlledMotor.h"
#include "../Submodules/FastPID/src/FastPID.h"
#include <cstdlib>
//#include "motor_ctrl_timer.h"

#include "soc/mcpwm_periph.h"
#include "../common/stabilizer_types.h"
extern "C"
{
#include "adc_esp32.h"
}

#define DEBUG_MODULE "PosCont"
#include "debug_cf.h"

float Kp = 5, Ki = -1, Kd = 0.001, Hz = 25;
int output_bits = 12;
bool output_signed = true;

//-2^(n-1)  to  (2^(n-1))-1
float Divisor = 0.024414; // (100.0 / (2 ^ output_bits));

/**
 * @brief the minimum duty cycle below which the motor will stop spining
 *
 */
const int16_t hysteresis = int(12 / Divisor);
/**
 * @brief the minimum duty cycle below which the motor will not start spining
 *
 */
const int16_t MinimumDutyCycle = int(16 / Divisor);

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

PosContMot PosContMot::PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin)
{
    PosContMot ctrl = PosContMot(GPIOa, GPIOb, FeedbackPin);
    ctrl.this_Timer = timer;
    ctrl.this_Unit = unit;
    ctrl.PID_Loop_Offset = ((int)ctrl.this_Timer) + ((int)ctrl.this_Unit * (int)MCPWM_TIMER_MAX);
    return ctrl;
}

PosContMot PosContMot::PosContMotCreate(uint8_t unit, uint8_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin)
{
    PosContMot ctrl = PosContMot(GPIOa, GPIOb, FeedbackPin);
    ctrl.this_Timer = (mcpwm_timer_t)timer;
    ctrl.this_Unit = (mcpwm_unit_t)unit;
    ctrl.PID_Loop_Offset = (((int)ctrl.this_Timer) + ((int)ctrl.this_Unit * (int)MCPWM_TIMER_MAX)) * (SERVO_RATE / 5);
    return ctrl;
}

PosContMot::PosContMot(uint16_t _GPIOa, uint16_t _GPIOb, uint16_t _FeedbackPin)
{
    GPIOa = _GPIOa;
    GPIOb = _GPIOb;
    PositionFeedbackPin = _FeedbackPin;
    previous_Duty = 0;
    McPwm_Config.frequency = 15000; // frequency = 500Hz,
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

void PosContMot::SetPos(int32_t Position, uint32_t Tick)
{
    int16_t PidOut = previous_Duty;
    if (RATE_DO_EXECUTE_WITH_OFFSET(SERVO_RATE, Tick, PID_Loop_Offset))
    {
        uint32_t feedback = analogReadRaw(PositionFeedbackPin);
        PidOut = myPID.step(Position >> 4, feedback);
        // if (PositionFeedbackPin == 1)
        // {
        //     DEBUG_PRINTI("%d,%d    %d  __ %f", feedback, Position >> 4, PidOut, ((float)abs(PidOut)) * Divisor);
        // }
    }

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

    // determine what direction the PID is commanding the motor in
    if (PidOut >= 1)
    {
        NextDirection = Reverse;
    }
    else if (PidOut <= -1)
    {
        NextDirection = Forward;
        PidMag *= 2; // TODO figure out why the PID is operating with a range from -2048 to 4095
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
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_A, ((float)PidMag * Divisor));
        if (NextDirection != current_Direction)
            mcpwm_set_duty_type(this_Unit, this_Timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
        break;
    case Reverse:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_B, (((float)PidMag) * Divisor));
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