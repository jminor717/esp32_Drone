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
#include "config/pin_config.h"
#include <string.h>
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12; // SOC_ADC_DIGI_MAX_BITWIDTH

static const adc_atten_t atten = ADC_ATTEN_DB_11; // 11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 30  // Multisampling
#define SAMPLE_RATE 1000

#if CONFIG_IDF_TARGET_ESP32
const int8_t ADC_GPIO_TO_CHANNEL[] = {ADC2_CHANNEL_1, -1, ADC2_CHANNEL_2, -1, ADC2_CHANNEL_0, -1, -1, -1, -1, -1, -1, -1,
                                      ADC2_CHANNEL_5, ADC2_CHANNEL_4, ADC2_CHANNEL_6, ADC2_CHANNEL_3, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      ADC2_CHANNEL_8, ADC2_CHANNEL_9, ADC2_CHANNEL_7, -1, -1, -1, -1,
                                      ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3};

const adc_unit_t GPIO_TO_ADC_UNIT[] = {ADC_UNIT_2, 8, ADC_UNIT_2, 8, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8,
                                       ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8, 8, 8,
                                       ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8,
                                       ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1};
#elif CONFIG_IDF_TARGET_ESP32S3
const int8_t ADC_GPIO_TO_CHANNEL[] = {-1, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_8, ADC1_CHANNEL_9,
                                      ADC2_CHANNEL_0, ADC2_CHANNEL_1, ADC2_CHANNEL_2, ADC2_CHANNEL_3, ADC2_CHANNEL_4, ADC2_CHANNEL_5, ADC2_CHANNEL_6, ADC2_CHANNEL_7, ADC2_CHANNEL_8, ADC2_CHANNEL_9,
                                      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

const adc_unit_t GPIO_TO_ADC_UNIT[] = {8, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1,
                                       ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2,
                                       8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
                                       8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
#endif

#define TIMES NO_OF_SAMPLES
#define ActiveChanels 5

#if CONFIG_IDF_TARGET_ESP32
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 1                  // For ESP32, this should always be set to 1
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1 // ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
#define ADC_RESULT_BYTE 4
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_ALTER_UNIT // ESP32C3 only supports alter mode
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_RESULT_BYTE 4
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#endif

static void checkEfuse(void)
{
    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
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

#if !CONFIG_IDF_TARGET_ESP32
static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2)
        return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
        return false;

    return true;
}
#endif

uint32_t analogReadRaw(uint32_t pin)
{
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);
    uint32_t ret_num = 0;

    esp_err_t ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
    {
        if (ret == ESP_ERR_INVALID_STATE)
        {
            /**
             * @note 1
             * Issue:
             * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
             * fast for the task to handle. In this condition, some conversion results lost.
             *
             * Reason:
             * When this error occurs, you will usually see the task watchdog timeout issue also.
             * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
             * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
             * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
             *
             * Solution:
             * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
             */
        }

        DEBUG_PRINTI("ret is %x, ret_num is %d", ret, ret_num);
        for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE)
        {
            adc_digi_output_data_t *p = (void *)&result[i];
            DEBUG_PRINTI("Unit: %d,_Channel: %d, Value: %x", p->type2.unit + 1, p->type2.channel, p->type2.data);
            if (check_valid_data(p))
            {
                DEBUG_PRINTI("Unit: %d,_Channel: %d, Value: %x", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
            else
            {
                // abort();
                DEBUG_PRINTI("Invalid data [%d_%d_%x]", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
        }
        // See `note 1`
        // vTaskDelay(1);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        /**
         * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
         * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
         */
        DEBUG_PRINTW("No data, increase timeout or reduce conv_num_each_intr");
        vTaskDelay(1000);
    }

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
            // int raw;
            // adc2_get_raw((adc2_channel_t)adcChannel, width, &raw);
            // adc_reading += raw;
        }
        else
        {
            return 0;
        }
    }
    return adc_reading / NO_OF_SAMPLES;
}

float analogReadVoltage(uint32_t pin)
{

    uint32_t adc_reading = analogReadRaw(pin);
    // Convert adc_reading to voltage in mV
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
    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        uint16_t adc1_chan_mask = 0;
        adc_channel_t channel[ActiveChanels] = {
            ADC_GPIO_TO_CHANNEL[CONFIG_VBat_PIN],
            ADC_GPIO_TO_CHANNEL[M1_POSITION_PIN],
            ADC_GPIO_TO_CHANNEL[M2_POSITION_PIN],
            ADC_GPIO_TO_CHANNEL[M3_POSITION_PIN],
            ADC_GPIO_TO_CHANNEL[M4_POSITION_PIN],
        };
        for (size_t i = 0; i < ActiveChanels; i++)
        {
            adc1_chan_mask = adc1_chan_mask | BIT((int)channel[i]);
        }

        adc_digi_init_config_t adc_dma_config = {
            .max_store_buf_size = 1024,
            .conv_num_each_intr = TIMES,
            .adc1_chan_mask = adc1_chan_mask,
            .adc2_chan_mask = 0,
        };
        ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

        adc_digi_configuration_t dig_cfg = {
            .conv_limit_en = ADC_CONV_LIMIT_EN,
            .conv_limit_num = 250,
            .sample_freq_hz = SAMPLE_RATE,
            .conv_mode = ADC_CONV_MODE,
            .format = ADC_OUTPUT_TYPE,
        };

        adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
        dig_cfg.pattern_num = ActiveChanels;
        for (int i = 0; i < ActiveChanels; i++)
        {
            uint8_t ch = channel[i];
            adc_pattern[i].atten = atten;
            adc_pattern[i].channel = ch;
            adc_pattern[i].unit = ADC_UNIT_1;
            adc_pattern[i].bit_width = width;

            DEBUG_PRINTI("adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
            DEBUG_PRINTI("adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
            DEBUG_PRINTI("adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
        }
        dig_cfg.adc_pattern = adc_pattern;
        ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));

        adc_digi_start();
        // adc1_config_width(width);
        // adc1_config_channel_atten(ADC_CHANNEL_0, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_1, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_2, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_3, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_4, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_5, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_6, atten);
        // adc1_config_channel_atten(ADC_CHANNEL_7, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)ADC_CHANNEL_7, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
