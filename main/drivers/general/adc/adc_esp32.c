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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/adc_channel.h"

#include "adc_esp32.h"
#include "config.h"
#include "config/pin_config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#include <string.h>

#define DEBUG_MODULE "ADC"
#include "debug_cf.h"
#include "static_mem.h"

static bool isInit;

static esp_adc_cal_characteristics_t* adc_chars;

static const adc_bits_width_t width = SOC_ADC_DIGI_MAX_BITWIDTH; // SOC_ADC_DIGI_MAX_BITWIDTH

static const adc_atten_t atten = ADC_ATTEN_DB_11; // 11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 4 // Multisampling
#define SAMPLE_RATE 10000

#if CONFIG_IDF_TARGET_ESP32
const int8_t ADC_GPIO_TO_CHANNEL[] = {
    ADC2_CHANNEL_1, -1, ADC2_CHANNEL_2, -1, ADC2_CHANNEL_0, -1, -1, -1, -1, -1, -1, -1,
    ADC2_CHANNEL_5, ADC2_CHANNEL_4, ADC2_CHANNEL_6, ADC2_CHANNEL_3, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    ADC2_CHANNEL_8, ADC2_CHANNEL_9, ADC2_CHANNEL_7, -1, -1, -1, -1,
    ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3
};
const adc_unit_t GPIO_TO_ADC_UNIT[] = {
    ADC_UNIT_2, 8, ADC_UNIT_2, 8, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8,
    ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, 8, 8, 8, 8,
    ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1
};
#elif CONFIG_IDF_TARGET_ESP32S3
const int8_t ADC_GPIO_TO_CHANNEL[] = {
    -1, ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7, ADC1_CHANNEL_8, ADC1_CHANNEL_9,
    ADC2_CHANNEL_0, ADC2_CHANNEL_1, ADC2_CHANNEL_2, ADC2_CHANNEL_3, ADC2_CHANNEL_4, ADC2_CHANNEL_5, ADC2_CHANNEL_6, ADC2_CHANNEL_7, ADC2_CHANNEL_8, ADC2_CHANNEL_9,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};
const adc_unit_t GPIO_TO_ADC_UNIT[] = {
    8, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1, ADC_UNIT_1,
    ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2, ADC_UNIT_2,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8
};
#endif

#define ActiveChanels 5

#if CONFIG_IDF_TARGET_ESP32
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 1 // For ESP32, this should always be set to 1
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

static uint16_t adc1_chan_mask = BIT(0) | BIT(1) | BIT(3) | BIT(7) | BIT(9);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channelsInUse[5] = { ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_3, ADC1_CHANNEL_7, ADC1_CHANNEL_9 };
static uint8_t numChanels = sizeof(channelsInUse) / sizeof(adc_channel_t);
uint16_t AveragedAdcData[20] = { 0 };
// ADC_GPIO_TO_CHANNEL[CONFIG_VBat_PIN],
// ADC_GPIO_TO_CHANNEL[M1_POSITION_PIN],
// ADC_GPIO_TO_CHANNEL[M2_POSITION_PIN],
// ADC_GPIO_TO_CHANNEL[M3_POSITION_PIN],
// ADC_GPIO_TO_CHANNEL[M4_POSITION_PIN],

void adcTask(void* param);
STATIC_MEM_TASK_ALLOC(adcTask, ADC_TASK_STACKSIZE);

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t* channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 5100,
        .conv_num_each_intr = NO_OF_SAMPLES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 250,
        .sample_freq_hz = SAMPLE_RATE,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = { 0 };
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = 0;
        uint8_t ch = channel[i];
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        DEBUG_PRINTI("adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        DEBUG_PRINTI("adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        DEBUG_PRINTI("adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

#if !CONFIG_IDF_TARGET_ESP32
static bool check_valid_data(const adc_digi_output_data_t* data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2)
        return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
        return false;

    return true;
}
#endif

void continuous_adc_Test()
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[NO_OF_SAMPLES] = { 0 };
    memset(result, 0xcc, NO_OF_SAMPLES);

    while (1) {
        // todo Make this a task and pass data between this and the analog reads
        ret = adc_digi_read_bytes(result, NO_OF_SAMPLES * ADC_RESULT_BYTE, &ret_num, ADC_MAX_DELAY);
        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
            if (ret == ESP_ERR_INVALID_STATE) {
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

            ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
            for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
                adc_digi_output_data_t* p = (void*)&result[i];
#if CONFIG_IDF_TARGET_ESP32
                DEBUG_PRINTI("Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
#else
                if (ADC_CONV_MODE == ADC_CONV_BOTH_UNIT || ADC_CONV_MODE == ADC_CONV_ALTER_UNIT || ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_1) {
                    if (check_valid_data(p)) {
                        DEBUG_PRINTI("Unit: %d,_Channel: %d, Value: %x", p->type2.unit + 1, p->type2.channel, p->type2.data);
                    } else {
                        // abort();
                        DEBUG_PRINTI("Invalid data [%d_%d_%x]", p->type2.unit + 1, p->type2.channel, p->type2.data);
                    }
                }
#if CONFIG_IDF_TARGET_ESP32S2
                else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_2) {
                    DEBUG_PRINTI("Unit: %d, Channel: %d, Value: %x", 2, p->type1.channel, p->type1.data);
                } else if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_1) {
                    DEBUG_PRINTI("Unit: %d, Channel: %d, Value: %x", 1, p->type1.channel, p->type1.data);
                }
#endif //#if CONFIG_IDF_TARGET_ESP32S2
#endif
            }
            // See `note 1`
            vTaskDelay(1);
        } else if (ret == ESP_ERR_TIMEOUT) {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            DEBUG_PRINTW("No data, increase timeout or reduce conv_num_each_intr");
            vTaskDelay(1000);
        }
    }
}

static void checkEfuse(void)
{
    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

uint32_t analogReadRaw(uint32_t pin)
{
    // adc_unit_t adcUnit = GPIO_TO_ADC_UNIT[pin];
    // if (adcUnit == ADC_UNIT_MAX) {
    //     return 0;
    // }
    // int8_t adcChannel = ADC_GPIO_TO_CHANNEL[pin];
    // uint32_t adc_reading = 0;
    // for (int i = 0; i < NO_OF_SAMPLES; i++) {
    //     if (adcUnit == ADC_UNIT_1) {
    //         adc_reading += adc1_get_raw((adc1_channel_t)adcChannel);
    //     } else if (adcUnit == ADC_UNIT_2) {
    //         // int raw;
    //         // adc2_get_raw((adc2_channel_t)adcChannel, width, &raw);
    //         // adc_reading += raw;
    //     } else {
    //         return 0;
    //     }
    // }

    // return adc_reading / NO_OF_SAMPLES;

    return AveragedAdcData[ADC_GPIO_TO_CHANNEL[pin]];
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
    if (isInit) {
        return;
    }

    checkEfuse();
    // Configure ADC
    if (unit == ADC_UNIT_1) {
        //continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channelsInUse, sizeof(channelsInUse) / sizeof(adc_channel_t));
        //adc_digi_start();

        //continuous_adc_Test();

        // adc1_config_width(width);
        for (int i = 0; i < numChanels; i++) {
            uint8_t ch = channelsInUse[i];
            adc1_config_channel_atten((adc1_channel_t)ch, atten);
        }
    } else {
        adc2_config_channel_atten((adc2_channel_t)ADC_CHANNEL_7, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    STATIC_MEM_TASK_CREATE(adcTask, adcTask, ADC_TASK_NAME, NULL, ADC_TASK_PRI);
    isInit = true;
}

void adcTask(void* param)
{
    while (1) {
        vTaskDelay(1);
        for (int i = 0; i < numChanels; i++) {
            uint8_t ch = channelsInUse[i];
            uint16_t reading = adc1_get_raw((adc1_channel_t)ch);

            // exponential moving average
            uint8_t window = 15;
            float k = 2.0 / (window + 1);
            AveragedAdcData[ch] = k * reading + (1 - k) * AveragedAdcData[ch];
        }
    }
}

bool adcTest(void)
{
    return isInit;
}
