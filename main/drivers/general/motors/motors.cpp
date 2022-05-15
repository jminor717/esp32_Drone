/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"
#include "cfassert.h"
#include "../Submodules/DShotRMT/src/DShotRMT.h"
#include "drivers/motors/PwmMotors.h"

static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef **motorMap; /* Current map configuration */

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

const uint16_t testsound[NBR_OF_MOTORS] = {NOTE_A4, NOTE_A5, NOTE_F5, NOTE_D5};

static bool isInit = false;

// ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
//     {
//         .channel = MOT_PWM_CH1,
//         .duty = 0,
//         .gpio_num = MOTOR1_GPIO,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .timer_sel = LEDC_TIMER_0,
//     },
//     {
//         .channel = MOT_PWM_CH2,
//         .duty = 0,
//         .gpio_num = MOTOR2_GPIO,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .timer_sel = LEDC_TIMER_0,
//     },
//     {
//         .channel = MOT_PWM_CH3,
//         .duty = 0,
//         .gpio_num = MOTOR3_GPIO,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .timer_sel = LEDC_TIMER_0,
//     },
//     {
//         .channel = MOT_PWM_CH4,
//         .duty = 0,
//         .gpio_num = MOTOR4_GPIO,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .timer_sel = LEDC_TIMER_0,
//     },
// };

//LEDC chanels seem to be a bit wonky on the s3 some pins 
// PWMSpeedControll pwm_ctrl1 = PWMSpeedControll::PWMControll(MOTOR1_GPIO, 2);
// PWMSpeedControll pwm_ctrl2 = PWMSpeedControll::PWMControll(MOTOR2_GPIO, 3);
// PWMSpeedControll pwm_ctrl3 = PWMSpeedControll::PWMControll(MOTOR3_GPIO, 4);
// PWMSpeedControll pwm_ctrl4 = PWMSpeedControll::PWMControll(MOTOR4_GPIO, 5);

// PWMSpeedControll PwmMotors[NBR_OF_MOTORS] = {pwm_ctrl1, pwm_ctrl2, pwm_ctrl3, pwm_ctrl4};

DShotRMT dshot_01((gpio_num_t)MOTOR1_GPIO, RMT_CHANNEL_0);
DShotRMT dshot_02((gpio_num_t)MOTOR2_GPIO, RMT_CHANNEL_1);
DShotRMT dshot_03((gpio_num_t)MOTOR3_GPIO, RMT_CHANNEL_2);
DShotRMT dshot_04((gpio_num_t)MOTOR4_GPIO, RMT_CHANNEL_3);

DShotRMT DShotMotors[NBR_OF_MOTORS] = {dshot_01, dshot_02, dshot_03, dshot_04};

// Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    int i;

    if (isInit)
    {
        // First to init will configure it
        return;
    }

    motorMap = motorMapSelect;

    for (i = 0; i < NBR_OF_MOTORS; i++)
    {
        //   DShotMotors[i].begin(DSHOT300, false);
    }
    dshot_01.begin(DSHOT300, false);
    dshot_02.begin(DSHOT300, false);
    dshot_03.begin(DSHOT300, false);
    dshot_04.begin(DSHOT300, false);
    DShotMotors[0] = dshot_01;
    DShotMotors[1] = dshot_02;
    DShotMotors[2] = dshot_03;
    DShotMotors[3] = dshot_04;

    // pwm_ctrl1.begin();
    // pwm_ctrl2.begin();
    // pwm_ctrl3.begin();
    // pwm_ctrl4.begin();
    // PwmMotors[0] = pwm_ctrl1;
    // PwmMotors[1] = pwm_ctrl2;
    // PwmMotors[2] = pwm_ctrl3;
    // PwmMotors[3] = pwm_ctrl4;
    isInit = true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
        //!   ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
    {
        // if (motorMap[i]->drvType == BRUSHED)
        // {
#ifdef ACTIVATE_STARTUP_SOUND
        motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(MOTORS[i], false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
        motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsSetRatio(MOTORS[i], 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
        // }
    }

    return isInit;
}

// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    // if (ithrust >0){
    //     DEBUG_PRINTI("set motor %d to %d", id, ithrust);
    // }
    if (isInit)
    {
        uint16_t ratio;

        //! ASSERT(id < NBR_OF_MOTORS);

        ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED

        if (motorMap[id]->drvType == BRUSHED)
        {
            float thrust = ((float)ithrust / 65536.0f) * 40; // Modify according to actual weight
            float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
            float supply_voltage = pmGetBatteryVoltage();
            float percentage = volts / supply_voltage;
            percentage = percentage > 1.0f ? 1.0f : percentage;
            ratio = percentage * UINT16_MAX;
            motor_ratios[id] = ratio;
        }

#endif

        motor_ratios[id] = ratio;
//        PwmMotors[id].SetRatio(map(ratio, 0, UINT16_MAX, 0, UINT8_MAX));
        DShotMotors[id].send_dshot_value(map(ratio, 0, UINT16_MAX, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX), NO_TELEMETRIC);
#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    }
}

int motorsGetRatio(uint32_t id)
{
    int ratio = 0;
    //! ASSERT(id < NBR_OF_MOTORS);
    //  ratio = motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    return ratio;
    return 0;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    //     uint32_t freq_hz = 15000;
    //     ASSERT(id < NBR_OF_MOTORS);
    //     if (ratio != 0) {
    //         ratio = (uint16_t)(0.05*(1<<16));
    //     }

    //     if (enable) {
    //         freq_hz = frequency;
    //     }

    //     ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,freq_hz);
    //     ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
    //     ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    // motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    // motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    // motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    // motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    // vTaskDelay(M2T(duration_msec));
    // motorsBeep(MOTOR_M1, false, frequency, 0);
    // motorsBeep(MOTOR_M2, false, frequency, 0);
    // motorsBeep(MOTOR_M3, false, frequency, 0);
    // motorsBeep(MOTOR_M4, false, frequency, 0);
    return;
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
    // int i = 0;
    // uint16_t note;     // Note in hz
    // uint16_t duration; // Duration in ms

    // do
    // {
    //   note = notes[i++];
    //   duration = notes[i++];
    //   motorsPlayTone(note, duration);
    // } while (duration != 0);
    return;
}
// LOG_GROUP_START(pwm)
// LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
// LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
// LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
// LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
// LOG_GROUP_STOP(pwm)
