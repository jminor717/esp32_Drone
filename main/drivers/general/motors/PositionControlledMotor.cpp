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

struct {
    float StartingDuty;
    float StoppingDuty;
} myStallSpeed;

// #define velocityCtrl

// const int16_t Min_Angle = 900, Max_Angle = 3300;
const int16_t Min_Angle = 500, Max_Angle = 3500;

// float kpd = 1.2, kid = 2, kdd = 0.01; // modify for optimal performance
// float kpd = 0.5, kid = 0.1, kdd = 0.01;
// float kpd = 1.5, kid = 1, kdd = 0.08;
// float kpd = 2, kid = 1, kdd = 0;

// float kpd = 0.2, kid = 0.2, kdd = 0.0001; // 50
float kpd = 0.3, kid = 5, kdd = 0.006; // 50
// float kpd = 0.1, kid = 0.8, kdd = 0.005; // 50

#ifdef velocityCtrl
// float kpv = 0.2, kiv = 0.1, kdv = 0.05; // P_ON_E
// float kpv = 2, kiv = 0.2, kdv = 0.0001; // P_ON_M
float kpv = 10, kiv = 0, kdv = 0; // P_ON_M
#endif
int ProportionOn = P_ON_M; // P_ON_E   P_ON_M
float aTuneStep = 80, aTuneNoise = 5, aTuneStartValue = 0;
unsigned int aTuneLookBack = 20;

const uint32_t inputVoltage_cv = 1200 * 1200, motorResistance_cOhm = 5000, averagePowerLimit_cw = 100, motorMaxHeatEnergy_cj = 1000;
const float Const1 = (inputVoltage_cv / 100) / (float)motorResistance_cOhm;
const float MaxStaticDuty = averagePowerLimit_cw / Const1;
const float deltaT = 1 / SERVO_RATE;
float motorHeatEnergy_cj = 0;

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
    TheOneWhoKnocks = new PID_Knocker(63.98, 56.56, 32); // tolerance = 32 ~ 2.5 degrees
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
    velocityPID = new PID(&input_vel, &output_vel, &setpoint_vel, kpv, kiv, kdv, P_ON_E, DIRECT); // P_ON_M P_ON_E
#endif
    aTune = new PID_ATune(&input, &output);
}

PosContMot::~PosContMot()
{
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool PosContMot::begin()
{
    // 1. mcpwm gpio initialization
    mcpwm_gpio_init(this_Unit, (mcpwm_io_signals_t)(0 + (2 * (int)this_Timer)), GPIOa);
    mcpwm_gpio_init(this_Unit, (mcpwm_io_signals_t)(1 + (2 * (int)this_Timer)), GPIOb);

    // 2. initial mcpwm configuration
    mcpwm_init(this_Unit, this_Timer, &McPwm_Config); // Configure PWM0A & PWM0B with above settings
    positionPID->SetSampleTime(1000 / SERVO_RATE);
    positionPID->SetOutputLimits(-100, 100);
    positionPID->SetMode(AUTOMATIC);
#ifdef velocityCtrl
    velocityPID->SetSampleTime(1000 / SERVO_RATE);
    velocityPID->SetOutputLimits(-100, 100);
    velocityPID->SetMode(AUTOMATIC);
#endif
    if (tuning) {
        output = aTuneStartValue;
        aTune->SetControlType(1);
        aTune->SetNoiseBand(aTuneNoise);
        aTune->SetOutputStep(aTuneStep);
        aTune->SetLookbackSec((int)aTuneLookBack);
        aTune->SetSetpoint(2048); // ZeroOffset
    }

    if (findingStall) {
        float Stall = FindStallInDirection(Forward);
        if (Stall > 0.03)
            DEBUG_PRINTI("================\n                                 %d   %.2f, %.2f\n=============================================================================", Forward, myStallSpeed.StartingDuty, myStallSpeed.StoppingDuty);
        vTaskDelay(1000);
        Stall += FindStallInDirection(Reverse);
        if (Stall > 0.03)
            DEBUG_PRINTI("================\n                                 %d   %.2f, %.2f\n=============================================================================", Reverse, myStallSpeed.StartingDuty, myStallSpeed.StoppingDuty);
        vTaskDelay(1000);
        Stall += FindStallInDirection(Forward);
        if (Stall > 0.03)
            DEBUG_PRINTI("================\n                                 %d   %.2f, %.2f\n=============================================================================", Forward, myStallSpeed.StartingDuty, myStallSpeed.StoppingDuty);
        vTaskDelay(1000);
        Stall += FindStallInDirection(Reverse);
        if (Stall > 0.03)
            DEBUG_PRINTI("================\n                                 %d   %.2f, %.2f\n=============================================================================", Reverse, myStallSpeed.StartingDuty, myStallSpeed.StoppingDuty);
        stallSpeed = Stall / 4.0;
        DEBUG_PRINTI("Found Stall speed %0.2f", stallSpeed);
        findingStall = false;
    }

    return true;
}

uint8_t window = 8;
float k = 2.0 / (window + 1); //  == 1 / 8

// motor 0 59.28 ||       63.98, 56.56
// motor 1 53.75 ||       58.53, 55.47
float PosContMot::FindStallInDirection(MtrDirection SearchDirection)
{
    if (!(PositionFeedbackPin == 10 || PositionFeedbackPin == 1))
        return 0.0;

    bool searching = true;
    float DutyCycle = 30;

    float RollingVel = 0;
    uint32_t i = 0;
    uint32_t feedback = analogReadRaw(PositionFeedbackPin);
    input = feedback;
    while (searching) {
        DutyCycle += 0.01;
        i++;
        feedback = analogReadRaw(PositionFeedbackPin);
        float RawVel = (feedback - input);
        input = feedback;
        // exponential moving average
        RollingVel = k * RawVel + (1 - k) * RollingVel;
        if (abs(RollingVel) >= 1) {
            searching = false;
        }
        // if (RollingVel > 0.1) // RATE_DO_EXECUTE(25, i) ||
        //     DEBUG_PRINTI("%.2f:: %d::   %.2f,%.2f", DutyCycle, feedback, RawVel, RollingVel);
        MoveMotor(SearchDirection, DutyCycle);
        vTaskDelay(1);
    }
    myStallSpeed.StartingDuty = DutyCycle;
    searching = true;
    DutyCycle += 2;
    MoveMotor(SearchDirection, DutyCycle);
    vTaskDelay(100);
    feedback = analogReadRaw(PositionFeedbackPin);
    input = feedback;
    i = 0;
    while (searching) {
        DutyCycle -= 0.01;
        i++;
        feedback = analogReadRaw(PositionFeedbackPin);
        float RawVel = (feedback - input);
        input = feedback;
        // exponential moving average
        RollingVel = k * RawVel + (1 - k) * RollingVel;
        if (abs(RollingVel) <= 0.1 && i > 10) {
            searching = false;
        }
        // if (RATE_DO_EXECUTE(25, i) || RollingVel > 0.1) //
        //     DEBUG_PRINTI("%.2f:: %d::   %.2f,%.2f", DutyCycle, feedback, RawVel, RollingVel);
        MoveMotor(SearchDirection, DutyCycle);
        vTaskDelay(1);
    }
    myStallSpeed.StoppingDuty = DutyCycle;

    MoveMotor(Stationary, 0.0);
    vTaskDelay(100);
    return DutyCycle;
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
    if (findingStall) {
        return;
    }

    // int16_t PidOut = previous_Duty;
    uint32_t feedback = analogReadRaw(PositionFeedbackPin);
    Position = Position >> 4;
    Position += ZeroOffset;
    bool currentAngleHigh = feedback > Max_Angle;
    bool currentAngleLow = feedback < Min_Angle;
#if initial_tuning
    if (tuning) {
        int val = aTune->Runtime(Tick);
        if (val != 0) {
            tuning = false;
        }
        if (!tuning) { // we're done, set the tuning parameters
            kpd = aTune->GetKp();
            kid = aTune->GetKi();
            kdd = aTune->GetKd();
            positionPID->SetTunings(kpd, kid, kdd);
            aTune->Cancel();
            tuning = false;
            DEBUG_PRINTI("kp:%f, ki:%f, kd:%f        %d,%d", kpd, kid, kdd, (int)this_Unit, (int)this_Timer);
        }
    } else
#endif
        if (RATE_DO_EXECUTE_WITH_OFFSET(SERVO_RATE, Tick, PID_Loop_Offset)) {

        // input_vel = (feedback - input);
        input_vel = k * (feedback - input) + (1 - k) * input_vel;
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

#ifdef velocityCtrl
    output = output_vel;
#endif
    float PidMag = 0;
    MtrDirection NextDirection = Stationary;
    if ((absInt(feedback - Position) <= 15)) {
        NextDirection = Stationary;
        if (input_vel < 0.3) {
            // call Initialize to avoid integral windup if we aren't fully at the setpoint
            output = 0;
            positionPID->Initialize();
        }
    } else {
        // float PIDcap = output;
        output = TheOneWhoKnocks->Knock(Tick, output, Position, feedback, input_vel);
        PidMag = abs(output);

        // if (PositionFeedbackPin == 1) { //|| PositionFeedbackPin == 10
        //     if (RATE_DO_EXECUTE_WITH_OFFSET(25, Tick, PID_Loop_Offset)) {
        //         DEBUG_PRINTI("B %d,%d::%d        %.2f,%.2f: %.2f        %d,%d   ,%d", feedback, Position, feedback - Position, output, PIDcap, input_vel, (int)this_Unit, (int)this_Timer, absInt(Position - feedback) - (absInt((int16_t)PIDcap) * 5));
        //     }
        // }

        // determine what direction the PID is commanding the motor in
        if (PidMag < stallSpeed + 1) {
            output = 0;
            NextDirection = Stationary;
        } else {
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
        }

        if (currentAngleHigh && NextDirection == Reverse) {
            if (direction_Setting == DIRECT) {
                NextDirection = Reverse;
            } else {
                NextDirection = Forward;
            }
        }
        if (currentAngleLow && NextDirection == Forward) {
            if (direction_Setting == DIRECT) {
                NextDirection = Forward;
            } else {
                NextDirection = Reverse;
            }
        }
    }

    // Const1 2.88
    //  motorHeatEnergy_cj += (((inputVoltage_cv * (PidMag / 100)) / motorResistance_cOhm) - averagePowerLimit_cw) * deltaT;
    //  motorHeatEnergy_cj += ((PidMag * Const1) - averagePowerLimit_cw) * deltaT;
    //  if(motorHeatEnergy_cj < 0){
    //      motorHeatEnergy_cj = 0;
    //  }
    //  if (motorHeatEnergy_cj > motorMaxHeatEnergy_cj && PidMag > MaxStaticDuty)
    //  {
    //      PidMag = MaxStaticDuty;
    //  }

    // set the duty cycle for the current direction

    MoveMotor(NextDirection, PidMag);
    // previous_Duty = output;
}

void PosContMot::MoveMotor(MtrDirection NextDirection, float PidMag)
{
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
    current_Direction = NextDirection;
}

PID_Knocker::PID_Knocker(float startStall, float stopStall, uint32_t tolerance)
{
    StallSpeedStatic = startStall;
    StallSpeedMoving = stopStall;
    PositionTolerance = tolerance;
    StartKnockTick = 0;
}

float PID_Knocker::Knock(uint32_t currentTick, float PidInput, int32_t PosSetpoint, uint32_t PosMeasured, float Velocity)
{

    // int16_t KnockWeight = absInt(PosSetpoint - PosMeasured) - (absInt((int16_t)PidInput) * 5);
    // if (Velocity > 1 && KnockWeight > 30 && (currentTick - StartKnockTick) > 150) {
    //     DEBUG_PRINTI("Z %d,%d        %.2f", currentTick, StartKnockTick, PidInput);
    //     StartKnockTick = currentTick;
    // }

    // if ((currentTick - StartKnockTick) <= 5) {
    //     PidInput = 50;
    //     DEBUG_PRINTI("A %d,%d::%d    %.2f", PosMeasured, PosSetpoint, KnockWeight, PidInput);
    // }

    float PidOutput = 0;
    if (PidInput > 0) {
        PidOutput = map(PidInput, 0, 100, StallSpeedMoving, 100);
    } else if (PidInput < 0) {
        PidOutput = map(PidInput, -100, 0, -100, -StallSpeedMoving);
    }

    // if (RATE_DO_EXECUTE(25, currentTick))
    // DEBUG_PRINTI("A %d,%d::%d     %.2f, %.2f", PosMeasured, PosSetpoint, KnockWeight, PidInput, PidOutput);

    return PidOutput;
}