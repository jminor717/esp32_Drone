/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include <string.h>

#include "power_distribution.h"

#include "log.h"
#include "motors.h"
#include "num.h"
#include "param.h"
#include "platform.h"
#include <string.h>

#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"

static bool motorSetEnable = false;

static struct
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motorPower;

static struct
{
    int32_t s1;
    int32_t s2;
    int32_t s3;
    int32_t s4;
} ServoPosition;

static struct
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} motorPowerSet;

uint32_t LocalTick = 0;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
    // platformConfigGetMotorMapping()
    MotorPerifDef mtr[4];
    mtr[0].drvType = M1Type;
    mtr[1].drvType = M2Type;
    mtr[2].drvType = M3Type;
    mtr[3].drvType = M4Type;
    motorsInit(mtr);
}

bool powerDistributionTest(void)
{
    bool pass = true;

    pass &= motorsTest();

    return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
    motorPower.m1 = 0;
    motorPower.m2 = 0;
    motorPower.m3 = 0;
    motorPower.m4 = 0;
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t* control, const setpoint_t* setpoint)
{
#if defined(QUAD_FORMATION_X)
    int16_t r = control->roll * 0.5f;
    int16_t p = control->pitch * 0.5f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 = limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + r + p - control->yaw);
#elif defined(QUAD_FORMATION_TRI)
    // measured 160g thrust with one prop and 380g with 2 seprately powered contra rotating props, at 2.4 volts
    float thrustScaled = control->thrust * 1.19; // 0.84;
    int16_t r = control->roll * 2;
    int16_t p = control->pitch;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(thrustScaled * 0.90 - p - control->yaw); // lower propeller spins slightly slower than the upper prop
    motorPower.m3 = limitThrust(thrustScaled * 1.10 - p + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + r + p - control->yaw);
#elif defined(QUAD_FORMATION_NORMAL)
    motorPower.m1 = limitThrust(control->thrust + control->pitch + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll - control->yaw);
    motorPower.m3 = limitThrust(control->thrust - control->pitch + control->yaw);
    motorPower.m4 = limitThrust(control->thrust + control->roll - control->yaw);
#elif defined(TWO_PROP_PLANE)
    motorPower.m1 = limitThrust(control->thrust + control->yaw);
    motorPower.m2 = limitThrust(control->thrust);
    motorPower.m3 = limitThrust(control->thrust);
    motorPower.m4 = limitThrust(control->thrust - control->yaw);
    int16_t pitchStab = 0;
    if (setpoint->mode.pitch == modeAngularVelocity) {
        pitchStab = control->pitch;
    }

    ServoPosition.s1 = setpoint->attitudeRate.roll + (setpoint->attitudeRate.pitch); // - pitchStab
    // ServoPosition.s2 = -setpoint->attitudeRate.yaw;
    // ServoPosition.s3 = setpoint->attitudeRate.yaw;
    ServoPosition.s4 = setpoint->attitudeRate.roll - (setpoint->attitudeRate.pitch); //- pitchStab
#else
#error Motor layout not defined!
#endif

    if (motorSetEnable) {
        motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
        motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
        motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
        motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
    } else {
        if (motorPower.m1 < idleThrust) {
            motorPower.m1 = idleThrust;
        }
        if (motorPower.m2 < idleThrust) {
            motorPower.m2 = idleThrust;
        }
        if (motorPower.m3 < idleThrust) {
            motorPower.m3 = idleThrust;
        }
        if (motorPower.m4 < idleThrust) {
            motorPower.m4 = idleThrust;
        }

        motorsSetRatio(MOTOR_M1, motorPower.m1);
        motorsSetRatio(MOTOR_M2, motorPower.m2);
        motorsSetRatio(MOTOR_M3, motorPower.m3);
        motorsSetRatio(MOTOR_M4, motorPower.m4);
        servoSetPosition(SERVO_S1, ServoPosition.s1, LocalTick);
        servoSetPosition(SERVO_S2, ServoPosition.s2, LocalTick);
        servoSetPosition(SERVO_S3, ServoPosition.s3, LocalTick);
        servoSetPosition(SERVO_S4, ServoPosition.s4, LocalTick);
    }
    LocalTick++;
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
