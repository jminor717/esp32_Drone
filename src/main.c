/*
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
 */

//#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"

#include "wifi_esp32.h"

#include "platform.h"
#include "system.h"
#define DEBUG_MODULE "APP_MAIN"
//#include "debug_cf.h"

#define SENSOR_INCLUDED_MPU9250_LPS25H

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void spi_sent_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;               // no need to send anything
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = len * 8;       // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;       // Data
    // t.user = (void *)1;                         // D/C needs to be set to 1
    t.addr = 0b1001101001010101;
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

uint8_t data2[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

void app_main()
{

    esp_log_level_set("*", ESP_LOG_VERBOSE);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());

    fflush(stdout);

    // esp_restart();

    /*
     * Initialize the platform and Launch the system task
     * app_main will initialize and start everything
     */

    /* initialize nvs flash prepare for Wi-Fi */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // wifiInit();

    /*Initialize the platform.*/
    if (platformInit() == false)
    {
        printf("platformInit failed");
        while (1)
            ; // if  firmware is running on the wrong hardware, Halt
    }

    gpio_set_direction(STM32_SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(STM32_SPI_SCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(STM32_SPI_MISO, GPIO_MODE_INPUT);

    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = STM32_SPI_MISO,
        .mosi_io_num = STM32_SPI_MOSI,
        .sclk_io_num = STM32_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096};
    spi_device_interface_config_t devcfg = {
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz = 26 * 1000 * 1000, // Clock out at 26 MHz
#else
        .clock_speed_hz = 500 * 1000, // Clock out at 1 MHz
#endif
        .mode = 0,          // SPI mode 0
        .spics_io_num = -1, // CS pin
        .queue_size = 7,    // We want to be able to queue 7 transactions at a time
        .address_bits = 16,
        .dummy_bits = 8
        //.pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(BUSS_1_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(BUSS_1_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    printf("prepairing to Launch system");

    fflush(stdout);

    // uint32_t i = 0;
    // while (true)
    // {
    //     uint8_t *dat = (uint8_t *)malloc(sizeof(uint8_t) * 5);
    //     dat[0] = 0xff;
    //     dat[1] = (++i & 0xff00) >> 8;
    //     dat[2] = i & 0x00ff;
    //     dat[3] = 0b01010101;
    //     dat[4] = 0b10101010;
    //     vTaskDelay(1);
    //     spi_sent_data(spi, dat, 5);
    //     free(dat);
    // }

    /*launch the system task */
    systemLaunch();
}