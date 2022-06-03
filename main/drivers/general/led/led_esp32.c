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
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "RGB_LED"
#include "debug_cf.h"

// #include "led_strip.h"

static bool isInit = false;

#define SpiHostMosiOnly SPI3_HOST

spi_bus_config_t buscfg = {
    .miso_io_num = -1,
    .mosi_io_num = CONFIG_LED_DOUT,
    .sclk_io_num = -1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 256,
    .flags = 0,
    .intr_flags = ESP_INTR_FLAG_IRAM
};

spi_device_handle_t LEDspi;
spi_device_interface_config_t LEDcfg = {
    .clock_speed_hz = 2500 * 1000, // Clock out at 1 MHz
    .mode = 1, // SPI mode 0
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
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);
}

void ledInit()
{
    if (isInit) {
        return;
    }

    esp_err_t ret;
    ret = spi_bus_initialize(SpiHostMosiOnly, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SpiHostMosiOnly, &LEDcfg, &LEDspi);
    ESP_ERROR_CHECK(ret);

    isInit = true;
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
        //Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++) {
        //Turn on the LED:s
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

#define OneCode 110
#define ZeroCode 100
#define BaudSize 24 //bytes
#define transactionSize 9 //bytes
void EncodeColor(uint32_t* bits, uint8_t colorValue)
{
    for (size_t i = 0; i < 8; i++) {
        if ((colorValue >> i) & 0x1)
            *bits |= OneCode << (i * 3);
        else
            *bits |= ZeroCode << (i * 3);
    }
}

void SetLedRaw(uint8_t r, uint8_t g, uint8_t b)
{
    // uint32_t BaudData = g << 16 | r << 8 | b;
    uint32_t rBit = 0, gBit = 0, bBit = 0;
    EncodeColor(&rBit, r);
    EncodeColor(&gBit, g);
    EncodeColor(&bBit, b);
    uint8_t data[transactionSize] = { 0 };
    memcpy(data + 6, &gBit, 3);
    memcpy(data + 3, &rBit, 3);
    memcpy(data, &bBit, 3);
    DEBUG_PRINTI("set led to #%x%x%x raw R:%x, G:%x, B:%x", r, g, b, rBit, gBit, bBit);

    Master_Transaction(LEDspi, data, transactionSize);
}
