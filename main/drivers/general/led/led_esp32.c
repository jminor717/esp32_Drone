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
 * led.c - LED handing functions
 */
#include <stdbool.h>

/*FreeRtos includes*/
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "led_strip.h"
#include "stm32_legacy.h"


static unsigned int led_pin[] = {
    [LED_BLUE] = LED_GPIO_BLUE,
    [LED_RED]   = LED_GPIO_RED,
    [LED_GREEN] = LED_GPIO_GREEN,
};

static bool isInit = false;
static uint8_t s_led_state = 0;
static led_strip_t* pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color*/
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    } else {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(RMT_CHANNEL_4, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

//Initialize the green led pin as output
void ledInit()
{
    int i;

    if (isInit) {
        return;
    }

    isInit = true;
}

bool ledTest(void)
{
    ledSet(LED_GREEN, 1);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 1);
    vTaskDelay(M2T(250));
    // LED test end
    ledClearAll();
    ledSet(LED_BLUE, 1);

    return isInit;
}

void ledClearAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn on the LED:s
        ledSet(i, 1);
    }
}
void ledSet(led_t led, bool value)
{

}


