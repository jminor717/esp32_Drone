/**
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
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/adc_channel.h"

#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_11; //11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 30  //Multisampling

// ESP32
// const int8_t ADC_GPIO_TO_CHANNEL[] = {ADC2_CHANNEL_1, -1, ADC2_CHANNEL_2, -1, ADC2_CHANNEL_0, -1, -1, -1, -1, -1, -1, -1,
//                                       ADC2_CHANNEL_5, ADC2_CHANNEL_4, ADC2_CHANNEL_6, ADC2_CHANNEL_3, -1, -1, -1, -1, -1, -1, -1, -1, -1,
//                                       ADC2_CHANNEL_8, ADC2_CHANNEL_9, ADC2_CHANNEL_7, -1, -1, -1, -1,
//                                       ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3};

// const adc_unit_t GPIO_TO_ADC_UNIT[] = {ADC_UNIT_2, 8, ADC_UNIT_2, 8, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8,
//                                        ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8, 8, 8,
//                                        ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8,
//                                        ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1};

// ESP32 S3
const int8_t ADC_GPIO_TO_CHANNEL[] = {-1, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_8, ADC1_CHANNEL_9, -1,
                                      ADC2_CHANNEL_5, ADC2_CHANNEL_4, ADC2_CHANNEL_6, ADC2_CHANNEL_3, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      ADC2_CHANNEL_8, ADC2_CHANNEL_9, ADC2_CHANNEL_7, -1, -1, -1, -1,
                                      ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3};

const adc_unit_t GPIO_TO_ADC_UNIT[] = {ADC_UNIT_2, 8, ADC_UNIT_2, 8, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8,
                                       ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8, 8, 8,
                                       ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8,
                                       ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1};

static void checkEfuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

float analogReadVoltage(uint32_t pin)
{
    adc_unit_t adcUnit = GPIO_TO_ADC_UNIT[pin];
    if (adcUnit == ADC_UNIT_MAX)
    {
        return 0;
    }
    int8_t adcChannel = ADC_GPIO_TO_CHANNEL[pin];
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (adcUnit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)adcChannel);
        }
        else if (adcUnit == ADC_UNIT_2)
        {
            int raw;
            adc2_get_raw((adc2_channel_t)adcChannel, width, &raw);
            adc_reading += raw;
        }
        else
        {
            return 0;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage / 1000.0;
}

void adcInit(void)
{
    if (isInit)
    {
        return;
    }

    checkEfuse();
    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(width);
        adc1_config_channel_atten(ADC_CHANNEL_0, atten);
        adc1_config_channel_atten(ADC_CHANNEL_1, atten);
        adc1_config_channel_atten(ADC_CHANNEL_2, atten);
        adc1_config_channel_atten(ADC_CHANNEL_3, atten);
        adc1_config_channel_atten(ADC_CHANNEL_4, atten);
        adc1_config_channel_atten(ADC_CHANNEL_5, atten);
        adc1_config_channel_atten(ADC_CHANNEL_6, atten);
        adc1_config_channel_atten(ADC_CHANNEL_7, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)ADC_CHANNEL_7, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
