#include "drivers/motors/PositionControlledMotor.h"
#include "../Submodules/Arduino-PID-Library/PID_v1.h"
#include "../Submodules/PID_AutoTune_v0/PID_AutoTune_v0.h"
#include <cstdlib>

//#include "motor_ctrl_timer.h"

#include "../common/stabilizer_types.h"
#include "soc/mcpwm_periph.h"

extern "C" {
#include "adc_esp32.h"
}

#define DEBUG_MODULE "PosCont"
#include "debug_cf.h"

// const int16_t Min_Angle = 900, Max_Angle = 3300;
const int16_t Min_Angle = 300, Max_Angle = 3700;

// float kpd = 1.2, kid = 2, kdd = 0.01; // modify for optimal performance
// float kpd = 0.5, kid = 0.1, kdd = 0.01;
// float kpd = 2, kid = 1, kdd = 0.05;
float kpd = 2, kid = 1, kdd = 0.05;
#ifdef velocityCtrl
float kpv = 2, kiv = 1, kdv = 0.05;
#endif
int ProportionOn = P_ON_E; // P_ON_E   P_ON_M
float aTuneStep = 80, aTuneNoise = 4, aTuneStartValue = 0;
unsigned int aTuneLookBack = 20;

PosContMot PosContMot::PosContMotCreate(mcpwm_unit_t unit, mcpwm_timer_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin, uint8_t direction, uint16_t offset)
{
    PosContMot ctrl = PosContMot(GPIOa, GPIOb, FeedbackPin, direction, offset);
    ctrl.this_Timer = timer;
    ctrl.this_Unit = unit;
    ctrl.PID_Loop_Offset = (((int)ctrl.this_Timer) + ((int)ctrl.this_Unit * 2));
    return ctrl;
}

PosContMot PosContMot::PosContMotCreate(uint8_t unit, uint8_t timer, uint16_t GPIOa, uint16_t GPIOb, uint16_t FeedbackPin, uint8_t direction, uint16_t offset)
{
    PosContMot ctrl = PosContMot(GPIOa, GPIOb, FeedbackPin, direction, offset);
    ctrl.this_Timer = (mcpwm_timer_t)timer;
    ctrl.this_Unit = (mcpwm_unit_t)unit;
    ctrl.PID_Loop_Offset = (((int)ctrl.this_Timer) + ((int)ctrl.this_Unit * 2));
    return ctrl;
}

PosContMot::PosContMot(uint16_t _GPIOa, uint16_t _GPIOb, uint16_t _FeedbackPin, int direction, uint16_t offset)
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
    ZeroOffset = offset;
    input = 0;
    output = 0;
    setpoint = 0;
    input_vel = 0;
    output_vel = 0;
    setpoint_vel = 0;
    direction_Setting = direction;
    positionPID = new PID(&input, &output, &setpoint, kpd, kid, kdd, ProportionOn, DIRECT); // if motor will only run at full speed try ‘REVERSE’ instead of ‘DIRECT’
#ifdef velocityCtrl
    velocityPID = new PID(&input_vel, &output_vel, &setpoint_vel, kpv, kiv, kdv, P_ON_M, DIRECT);
#endif
    aTune = new PID_ATune(&input, &output);
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
    positionPID->SetSampleTime(1000 / SERVO_RATE);
    positionPID->SetOutputLimits(-1000, 1000);
    positionPID->SetMode(AUTOMATIC);
#ifdef velocityCtrl
    velocityPID->SetSampleTime(1000 / SERVO_RATE);
    velocityPID->SetOutputLimits(-100, 100);
    velocityPID->SetMode(AUTOMATIC);
#endif
    if (tuning) {
        output = aTuneStartValue;
        aTune->SetControlType(0);
        aTune->SetNoiseBand(aTuneNoise);
        aTune->SetOutputStep(aTuneStep);
        aTune->SetLookbackSec((int)aTuneLookBack);
        aTune->SetSetpoint(ZeroOffset);
    }

    return true;
}

bool SameSign(int x, int y)
{
    return (x >= 0) ^ (y < 0);
}

int16_t absInt(int16_t N)
{

    // If the number is less than
    // zero, then multiply by (-1)
    if (N < 0) {
        N = (-1) * N;
    }
    return N;
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
    Position = Position >> 4;
    Position += ZeroOffset;
    bool currentAngleHigh = feedback > Max_Angle;
    bool currentAngleLow = feedback < Min_Angle;
    // if (tuning) {
    //     int val = aTune->Runtime();
    //     if (val != 0) {
    //         tuning = false;
    //     }
    //     if (!tuning) { // we're done, set the tuning parameters
    //         kpd = aTune->GetKp();
    //         kid = aTune->GetKi();
    //         kdd = aTune->GetKd();
    //         positionPID->SetTunings(kpd, kid, kdd);
    //         aTune->Cancel();
    //         tuning = false;
    //         DEBUG_PRINTI("kp:%f, ki:%f, kd:%f        %d,%d", kpd, kid, kdd, (int)this_Unit, (int)this_Timer);
    //     }
    // } else
    if (RATE_DO_EXECUTE_WITH_OFFSET(SERVO_RATE, Tick, PID_Loop_Offset)) {
        // feedback -= 2047;
        input_vel = (feedback - input);
        input = feedback;

        if (Position < Min_Angle) {
            Position = Min_Angle;
        } else if (Position > Max_Angle) {
            Position = Max_Angle;
        }
        setpoint = Position;

        positionPID->Compute();
#ifdef velocityCtrl
        setpoint_vel = output;
        velocityPID->Compute();
#endif
    } else if (!(currentAngleHigh || currentAngleLow)) {
        return;
    }

    MtrDirection NextDirection;
#ifdef velocityCtrl
    float PidMag = abs(output_vel);
#else
    float PidMag = abs(output);
#endif
    if ((absInt(feedback - Position) <= 5)) {
        NextDirection = Stationary;
    } else {
        // apply hysteresis logic to PID output value
        if (previous_Duty == 0) {
            // we were in a stoped state
            if (PidMag < 20) {
                output = 0;
                NextDirection = Stationary;
            }
        } else {
            if (PidMag < 30) {
                output = 0;
                NextDirection = Stationary;
            } else if (PidMag < 50.0) {
                // PidMag = 50;
            }
        }

        // determine what direction the PID is commanding the motor in
        if (output >= 1) {
            if (direction_Setting == DIRECT) {
                NextDirection = Forward;
            } else {
                NextDirection = Reverse;
            }
        } else if (output <= -1) {
            if (direction_Setting == DIRECT) {
                NextDirection = Reverse;
            } else {
                NextDirection = Forward;
            }
        }

        if ((currentAngleHigh && NextDirection == Reverse) || (currentAngleLow && NextDirection == Forward)) {
            NextDirection = Stationary;
        }

        if (PositionFeedbackPin == 1) { // || PositionFeedbackPin == 10
            if (RATE_DO_EXECUTE_WITH_OFFSET(25, Tick, PID_Loop_Offset))
                DEBUG_PRINTI("%d,%d::%d        %.2f,%.2f: %.2f        %d,%d", feedback, Position, feedback - Position, output, input_vel, output_vel, (int)this_Unit, (int)this_Timer);
        }
    }

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