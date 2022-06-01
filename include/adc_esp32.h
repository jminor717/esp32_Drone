/**
 * ESP-Drone Firmware
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
 * adc.h - Analog Digital Conversion header file
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "config.h"

/*
32 ADC1_CHANNEL_4
33 ADC1_CHANNEL_5
34 ADC1_CHANNEL_6
35 ADC1_CHANNEL_7
36 ADC1_CHANNEL_0
37 ADC1_CHANNEL_1
38 ADC1_CHANNEL_2
39 ADC1_CHANNEL_3

0 ADC2_CHANNEL_1
2 ADC2_CHANNEL_2
4 ADC2_CHANNEL_0
12 ADC2_CHANNEL_5
13 ADC2_CHANNEL_4
14 ADC2_CHANNEL_6
15 ADC2_CHANNEL_3
25 ADC2_CHANNEL_8
26 ADC2_CHANNEL_9
27 ADC2_CHANNEL_7
*/

/******** Types ********/

typedef struct __attribute__((packed)) {
    uint16_t vref;
    uint16_t val;
} AdcPair;

typedef struct __attribute__((packed)) {
    AdcPair vbat;
} AdcGroup;

typedef struct
{
    uint16_t vbat;
    uint16_t vbatVref;
} AdcDeciGroup;

/*** Public interface ***/

/**
 * Initialize analog to digital converter. Configures gyro and vref channels.
 */
void adcInit(void);

bool adcTest(void);

/**
 * @brief Converts a 12 bit ADC value to battery voltage
 * @param vbat  12 bit adc value
 * @param vref  12 bit adc value of the internal voltage
 *              reference, 1.2V
 * @return The voltage in a float value
 */
float adcConvertToVoltageFloat(uint16_t v, uint16_t vref);

/**
 * Starts converting ADC samples by activating the DMA.
 */
void adcDmaStart(void);

/**
 * Stop converting ADC samples.
 */
void adcDmaStop(void);

/**
 * ADC interrupt handler
 */
void adcInterruptHandler(void);

/**
 * ADC task
 */
void adcTask(void* param);

float analogReadVoltage(uint32_t pin); //should in deck_analog.c

uint32_t analogReadRaw(uint32_t pin);

#endif /* ADC_H_ */
