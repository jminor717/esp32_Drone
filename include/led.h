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
 * led.h - LED functions header file
 */
#ifndef __LED_H__
#define __LED_H__

#include "config/pin_config.h"
#include <stdbool.h>

#define LED_GPIO_BLUE  0
#define LED_GPIO_GREEN 1
#define LED_GPIO_RED   2


#define LINK_LED         LED_GREEN
#define CHG_LED          LED_RED
#define LOWBAT_LED       LED_RED
#define LINK_DOWN_LED    LED_BLUE
#define SYS_LED          LED_BLUE
#define ERR_LED1         LED_RED
#define ERR_LED2         LED_RED

#define LED_NUM 3

typedef enum {
    LED_BLUE = 0,
    LED_RED,
    LED_GREEN,
} led_t;

void ledInit();
bool ledTest();

// Clear all configured LEDs
void ledClearAll(void);

// Set all configured LEDs
void ledSetAll(void);

// Procedures to set the status of the LEDs
void ledSet(led_t led, uint8_t value);

void SetLedRaw(uint8_t r, uint8_t g, uint8_t b);

void ledTask(void* param);

//Legacy functions
#define ledSetRed(VALUE) ledSet(LED_RED, VALUE)
#define ledSetGreen(VALUE) ledSet(LED_GREEN, VALUE)

#endif
