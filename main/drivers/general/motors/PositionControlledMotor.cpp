#include "drivers/motors/PositionControlledMotor.h"
#include "../Submodules/Arduino-PID-Library/PID_v1.h"
#include <cstdlib>

//#include "motor_ctrl_timer.h"

#include "../common/stabilizer_types.h"
#include "soc/mcpwm_periph.h"

extern "C" {
#include "adc_esp32.h"
}

#define DEBUG_MODULE "PosCont"
#include "debug_cf.h"

const int16_t Min_Angle = 900, Max_Angle = 3300;

double kpd = 0.5, kid = 0.1, kdd = 0.01; // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID3(&input, &output, &setpoint, kpd, kid, kdd, P_ON_E, DIRECT); // if motor will only run at full speed try ‘REVERSE’ instead of ‘DIRECT’

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
    McPwm_Config.cmpr_a = 0; // duty cycle of PWMxA = 0
    McPwm_Config.cmpr_b = 0; // duty cycle of PWMxb = 0
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
    myPID3.SetSampleTime(SERVO_RATE);
    myPID3.SetOutputLimits(-100, 100);
    myPID3.SetMode(AUTOMATIC);

    return true;
}

bool SameSign(int x, int y)
{
    return (x >= 0) ^ (y < 0);
}
/**
 * @brief will iterate the PID loop of the specifed motor and attempt to drive the motor to the specified position
 * @remarks this should be called on every run of the main control loop
 *
 * @param Position
 * @param Tick tick variable from the main control loop to determine how often PID should be run
 */
void PosContMot::SetPos(int32_t Position, uint32_t Tick)
{
    // int16_t PidOut = previous_Duty;
    uint32_t feedback = analogReadRaw(PositionFeedbackPin);
    if (RATE_DO_EXECUTE_WITH_OFFSET(SERVO_RATE, Tick, PID_Loop_Offset)) {
        Position = Position >> 4;
        Position += 2047;
        //feedback -= 2047;
        if (Position < Min_Angle) {
            Position = Min_Angle;
        } else if (Position > Max_Angle) {
            Position = Max_Angle;
        }

        setpoint = Position;
        input = feedback;
        myPID3.Compute();
        // if (PositionFeedbackPin == 10)
        //     DEBUG_PRINTI("%d,%d::%d        %.2f", feedback, Position, feedback - Position, output);
    } else {
        return;
    }

    float PidMag = abs(output);
    MtrDirection NextDirection;

    // apply hysteresis logic to PID output value
    if (previous_Duty == 0) {
        // we were in a stoped state
        if (PidMag < 16.0) {
            output = 0;
            NextDirection = Stationary;
        }
    } else {
        if (PidMag < 20.0) {
            output = 0;
            NextDirection = Stationary;
        }
    }

    // determine what direction the PID is commanding the motor in
    if (output >= 1) {
        NextDirection = Reverse;
    } else if (output <= -1) {
        NextDirection = Forward;
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
    switch (NextDirection) {
    case Forward:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
        // mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_A, ((float)PidMag * Divisor));
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_A, PidMag);
        if (NextDirection != current_Direction)
            mcpwm_set_duty_type(this_Unit, this_Timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
        break;
    case Reverse:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
        // mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_B, (((float)PidMag) * Divisor));
        mcpwm_set_duty(this_Unit, this_Timer, MCPWM_GEN_B, PidMag);
        if (NextDirection != current_Direction)
            mcpwm_set_duty_type(this_Unit, this_Timer, MCPWM_GEN_B, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
        break;
    default:
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_A);
        mcpwm_set_signal_low(this_Unit, this_Timer, MCPWM_GEN_B);
        break;
    }

    previous_Duty = output;
    current_Direction = NextDirection;
}