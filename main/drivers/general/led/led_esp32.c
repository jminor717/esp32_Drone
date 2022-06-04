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
#include <string.h>

/*FreeRtos includes*/
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "RGB_LED"
#include "debug_cf.h"

// #include "led_strip.h"

static bool isInit = false;
uint8_t* LedDataBuffer;

#define OneCode 0b1100
#define ZeroCode 0b1000
#define codeLength 4
#define BaudSize 24
#define transactionSize 12 // bytes
#define BlankingSize 256
#define HalfBlankingSize 128
#define I2S_NUM (0)

#define SpiHostMosiOnly SPI3_HOST

spi_bus_config_t buscfg = {
    .miso_io_num = -1,
    .mosi_io_num = CONFIG_LED_DOUT,
    .sclk_io_num = -1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = BlankingSize,
    .flags = 0,
    .intr_flags = ESP_INTR_FLAG_IRAM
};

spi_device_handle_t LEDspi;
spi_device_interface_config_t LEDcfg = {
    .clock_speed_hz = 3333 * 1000, // Clock out at 1 MHz
    .mode = 3, // SPI mode 0
    .spics_io_num = -1, // COM13909_SS,  // CS pin
    .queue_size = 4, // We want to be able to queue 7 transactions at a time
    .address_bits = 0, // 16,
    .dummy_bits = 0 // 8
    //.pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
};

void Master_Transaction(spi_device_handle_t spi, uint8_t* dat, uint32_t nextSize)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = nextSize * 8; // Len is in bytes, transaction length is in bits.
    t.tx_buffer = dat; // Data
    ret = spi_device_queue_trans(spi, &t, 1); // Transmit!

    assert(ret == ESP_OK);
}

void ledInit()
{
    if (isInit) {
        return;
    }

    // LedDataBuffer = (uint8_t*)heap_caps_malloc(BlankingSize, MALLOC_CAP_DMA);
    // memset(LedDataBuffer, 0, BlankingSize);

    // esp_err_t ret;
    // ret = spi_bus_initialize(SpiHostMosiOnly, &buscfg, SPI_DMA_CH_AUTO);
    // ESP_ERROR_CHECK(ret);

    // ret = spi_bus_add_device(SpiHostMosiOnly, &LEDcfg, &LEDspi);
    // ESP_ERROR_CHECK(ret);

    // i2s_config_t i2s_config = {
    //     .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    //     .sample_rate = 36000,
    //     .bits_per_sample = I2S_BITS_PER_SAMPLE_8BIT,
    //     .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    //     .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    //     .dma_buf_count = 3,
    //     .dma_buf_len = 60,
    //     .use_apll = true,
    //     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //     .fixed_mclk = 3333 * 1000,
    // };
    // i2s_pin_config_t pin_config = {
    //     .mck_io_num = -1,
    //     .bck_io_num = -1,
    //     .ws_io_num = -1,
    //     .data_out_num = CONFIG_LED_DOUT,
    //     .data_in_num = -1 // Not used
    // };
    // i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    // i2s_set_pin(I2S_NUM, &pin_config);
    // i2s_set_clk(I2S_NUM, 36000 * 4, 8, 2);

    // const uart_config_t uart_config = {
    //     .baud_rate = 3333 * 1000,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_APB,
    // };
    // // We won't use a buffer for sending data.
    // uart_driver_install(UART_NUM_2, 1024 * 2, 256, 0, NULL, 0);
    // uart_param_config(UART_NUM_2, &uart_config);
    // uart_set_pin(UART_NUM_2, CONFIG_LED_DOUT, -1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    isInit = true;
    DEBUG_PRINTI("Led's Init");
}

bool ledTest(void)
{
    ledSet(LED_GREEN, 255);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 255);
    vTaskDelay(M2T(250));
    // LED test end
    ledClearAll();
    ledSet(LED_BLUE, 255);

    return isInit;
}

void ledClearAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        // Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        // Turn on the LED:s
        ledSet(i, 255);
    }
}

void ledSet(led_t led, uint8_t value)
{

    switch (led) {
    case LED_GREEN:
        SetLedRaw(0, value, 0);
        break;
    case LED_RED:
        SetLedRaw(value, 0, 0);
        break;
    case LED_BLUE:
        SetLedRaw(0, 0, value);
        break;
    default:
        break;
    }
}

void EncodeColor(uint32_t* bits, uint8_t colorValue)
{
    for (size_t i = 0; i < 8; i++) {
        if ((colorValue >> i) & 0x1)
            *bits |= OneCode << (i * codeLength);
        else
            *bits |= ZeroCode << (i * codeLength);
    }
}

void reverse(uint8_t arr[], uint8_t n)
{
    uint8_t aux[n];

    for (uint8_t i = 0; i < n; i++) {
        aux[n - 1 - i] = arr[i];
    }

    for (uint8_t i = 0; i < n; i++) {
        arr[i] = aux[i];
    }
}

void SetLedRaw(uint8_t r, uint8_t g, uint8_t b)
{
    // TODO Fuck
    // uint32_t rBit = 0, gBit = 0, bBit = 0;
    // EncodeColor(&rBit, r);
    // EncodeColor(&gBit, g);
    // EncodeColor(&bBit, b);

    // assuming nothing else touches this data we should only need to clear this section of memory,
    // TODO: optimize this to use less memory,
    // uint8_t data[transactionSize] = {0};
    // size_t i2s_bytes_write = 0;
    // memset(LedDataBuffer + HalfBlankingSize, 0, transactionSize);
    // memcpy(data + (codeLength * 2) + (0), &gBit, codeLength);
    // memcpy(data + codeLength + (0), &rBit, codeLength);
    // memcpy(data + (0), &bBit, codeLength);
    // DEBUG_PRINTI("set led to #%x%x%x raw R: 0x%x, G: 0x%x, B: 0x%x", r, g, b, rBit, gBit, bBit);
    //  DEBUG_PRINTI("%x%x%x %x%x%x %x%x%x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
    //  DEBUG_PRINTI("%x%x%x %x%x%x %x%x%x", data[8], data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);

    //Master_Transaction(LEDspi, LedDataBuffer, BlankingSize);
    //i2s_write(I2S_NUM, data, transactionSize, &i2s_bytes_write, 10);
    // uart_write_bytes_with_break(UART_NUM_2, (const char*)data, transactionSize, 80);
}
