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
 * Motors.h - Motor driver header file
 *
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

#include "driver/ledc.h"

#include "config.h"
#include "stm32_legacy.h"
#include "config/pin_config.h"

/******** Defines ********/
// CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.

#define MOTORS_TIM_BEEP_CLK_FREQ  4000000

// Compensate thrust depending on battery voltage so it will produce about the same
// amount of thrust independent of the battery voltage. Based on thrust measurement.

//#define ENABLE_THRUST_BAT_COMPENSATED


#define NBR_OF_MOTORS 4
// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#define MOTOR1_GPIO CONFIG_MOTOR01_PIN
#define MOTOR2_GPIO CONFIG_MOTOR02_PIN
#define MOTOR3_GPIO CONFIG_MOTOR03_PIN
#define MOTOR4_GPIO CONFIG_MOTOR04_PIN


#define MOT_PWM_CH1  4      // Motor M1 pwmchannel
#define MOT_PWM_CH2  5      // Motor M2 pwmchannel
#define MOT_PWM_CH3  6      // Motor M3 pwmchannel
#define MOT_PWM_CH4  7      // Motor M4 pwmchannel     

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// Sound defines
#define NOTE_C4    262
#define NOTE_DES4  277
#define NOTE_D4    294
#define NOTE_ES4   311
#define NOTE_E4    330
#define NOTE_F4    349
#define NOTE_GES4  370
#define NOTE_G4    392
#define NOTE_AS4   415
#define NOTE_A4    440
#define NOTE_B4    466
#define NOTE_H4    493
#define NOTE_C5    523
#define NOTE_DES5  554
#define NOTE_D5    587
#define NOTE_ES5   622
#define NOTE_E5    659
#define NOTE_F5    698
#define NOTE_GES5  740
#define NOTE_G5    783
#define NOTE_AS5   830
#define NOTE_A5    880
#define NOTE_B5    932
#define NOTE_H5    987
#define NOTE_C6    1046
#define NOTE_DES6  1108
#define NOTE_D6    1174
#define NOTE_ES6   1244
#define NOTE_E6    1318
#define NOTE_F6    1396
#define NOTE_GES6  1479
#define NOTE_G6    1567
#define NOTE_AS6   1661
#define NOTE_A6    1760
#define NOTE_B6    1864
#define NOTE_H6    1975
#define NOTE_C7    2093
#define NOTE_DES7  2217
#define NOTE_D7    2349
#define NOTE_ES7   2489
#define NOTE_E7    2637
#define NOTE_F7    2793
#define NOTE_GES7  2959
#define NOTE_G7    3135
#define NOTE_AS7   3322
#define NOTE_A7    3520
#define NOTE_H7    3729
#define NOTE_B7    3951

// Sound duration defines
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0

typedef enum {
    BRUSHED,
    BRUSHLESS
} motorsDrvType;

typedef struct {
    motorsDrvType drvType;

} MotorPerifDef;

/**
 * Motor mapping configurations
 */
//extern const MotorPerifDef* motorMapNoMotors[NBR_OF_MOTORS];
extern const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBoltBrushless[NBR_OF_MOTORS];

/**
 * Test sound tones
 */
extern const uint16_t testsound[NBR_OF_MOTORS];
/*** Public interface ***/

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

/**
 * Initialisation. Will set all motors ratio to 0%
 */
EXTERNC void motorsInit(const MotorPerifDef **motorMapSelect);

/**
 * DeInitialisation. Reset to default
 */
EXTERNC void motorsDeInit(const MotorPerifDef **motorMapSelect);

/**
 * Test of the motor modules. The test will spin each motor very short in
 * the sequence M1 to M4.
 */
EXTERNC bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
EXTERNC void motorsSetRatio(uint32_t id, uint16_t ratio);

/**
 * Get the PWM ratio of the motor 'id'. Return -1 if wrong ID.
 */
EXTERNC int motorsGetRatio(uint32_t id);

/**
 * FreeRTOS Task to test the Motors driver
 */
EXTERNC void motorsTestTask(void *params);

/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
EXTERNC void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#undef EXTERNC

#endif /* __MOTORS_H__ */

