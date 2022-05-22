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
extern "C"
{
//#define SENSOR_INCLUDED_MPU6050_HMC5883L_MS5611

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "config/pin_config.h"

#include <../Common/Data_type.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/spi_common.h"

#include "wifi_esp32.h"

#include "platform.h"
#include "system.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"
}

#include "radiolink.h"



extern "C"
{
    void app_main();
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    // esp_pm_config_esp32 cfg;

    // esp_pm_configure();

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    DEBUG_PRINTI("This is %s chip with %d CPU cores, WiFi%s%s, silicon revision %d,  %dMB %s flash ",
                 CONFIG_IDF_TARGET,
                 chip_info.cores,
                 (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
                 (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
                 chip_info.revision,
                 spi_flash_get_chip_size() / (1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    DEBUG_PRINTI("Free heap: %d", esp_get_free_heap_size());

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
        DEBUG_PRINTW("platformInit failed");
        while (1)
            ; // if  firmware is running on the wrong hardware, 
    }

    DEBUG_PRINTI("prepairing to Launch system \n");
    /*launch the system task */

    systemLaunch();

    for (;;)
    {
        vTaskDelay(1000);
    }
}