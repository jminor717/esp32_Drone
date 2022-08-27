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
 * Controller\*,components\*
 */

//#include <stdlib.h>
extern "C" {
//#define SENSOR_INCLUDED_MPU6050_HMC5883L_MS5611

#include <stdio.h>
#include <string.h>

#include "config/pin_config.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "drivers\WIFI\WifiServer.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "nmea.h"

#include "gpgga.h"
#include "gpgll.h"
#include "gpgsa.h"
#include "gpgsv.h"
#include "gprmc.h"
#include "gptxt.h"
#include "gpvtg.h"

#include "platform.h"
#include "system.h"
// #include "wifi_esp32.h"
// #include <../Common/Data_type.h>

#include "param.h"

#include "../common/GPSCalc.h"

#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"
}

#include "radiolink.h"
#define DebugTasks defined(CONFIG_FREERTOS_USE_TRACE_FACILITY) && defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)

#define Print_Params true

#if DebugTasks
void ProfileTaskStats(char* pcWriteBuffer);
#endif

#if Print_Params
void PrintParams(char* pcWriteBuffer, size_t BufferSize);
#endif

extern "C" {
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

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // wifiInit();

    /*Initialize the platform.*/
    if (platformInit() == false) {
        DEBUG_PRINTW("platformInit failed");
        while (1)
            ; // if  firmware is running on the wrong hardware,
    }

    DEBUG_PRINTI("prepairing to Launch system \n");

    struct coordinate self = { .lat = 0.01, .lon = 0.001, .elv = 0.1 };
    struct coordinate target = { .lat = 0.02, .lon = 0.002, .elv = 0.2 };
    struct PointingVector vector = CalculateOrientationToTarget(self, target, true);

    DEBUG_PRINTI("Distance: %0.3f km, Azimuth: %0.4f deg, Altitude: %0.4f deg", vector.Distance, vector.Azimuth, vector.Altitude);

    /*launch the system task */
    systemLaunch();

#if Print_Params
    vTaskDelay(5000);
    // Start_OTA_Wifi_Server();
    char* pcWriteBuffer;
    pcWriteBuffer = (char*)malloc(50000);
    PrintParams(pcWriteBuffer, 50000);
    DEBUG_PRINTI("%s", pcWriteBuffer);
    free(pcWriteBuffer);
#endif

    for (;;) {
        vTaskDelay(1000);
#if DebugTasks
        char* pcWriteBuffer;
        pcWriteBuffer = (char*)malloc(900);
        ProfileTaskStats(pcWriteBuffer);
        uint32_t freeHeap = esp_get_free_heap_size();
        DEBUG_PRINTI("%d\n%s", freeHeap, pcWriteBuffer);
        free(pcWriteBuffer);
#endif
    }
}

// This example demonstrates how a human readable table of run time stats
// information is generated from raw data provided by uxTaskGetSystemState().
// The human readable table is written to pcWriteBuffer
void ProfileTaskStats(char* pcWriteBuffer)
{
    TaskStatus_t* pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Make sure the write buffer does not contain a string.
    *pcWriteBuffer = 0x00;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        // For percentage calculations.
        ulTotalRunTime /= 100UL;

        // Avoid divide by zero errors.
        if (ulTotalRunTime > 0) {
            // For each populated position in the pxTaskStatusArray array,
            // format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++) {
                // What percentage of the total run time has the task used?
                // This will always be rounded down to the nearest integer.
                // ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

                if (ulStatsAsPercentage > 0UL) {
                    sprintf(pcWriteBuffer, "%-15s\t%d\t%-10d\t%d%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].usStackHighWaterMark, pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
                } else {
                    // If the percentage is zero here then the task has
                    // consumed less than 1% of the total run time.
                    sprintf(pcWriteBuffer, "%-15s\t%d\t%-10d\t<1%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].usStackHighWaterMark, pxTaskStatusArray[x].ulRunTimeCounter);
                }

                pcWriteBuffer += strlen((char*)pcWriteBuffer);
            }
        }

        // The array is no longer needed, free the memory it consumes.
        vPortFree(pxTaskStatusArray);
    }
}

void PrintParams(char* pcWriteBuffer, size_t BufferSize)
{
    int numParams = getParamsLen();
    size_t totalLen = 0;
    char* group = (char*)malloc(20);

    // Make sure the write buffer does not contain a string.
    *pcWriteBuffer = 0x00;
    sprintf(pcWriteBuffer, "num p: %d\r\n", numParams);
    pcWriteBuffer += strlen((char*)pcWriteBuffer);

    param_s* paramsToPrint = getParamsArr();
    for (uint16_t i = 0; i < numParams; i++) {
        paramVarId_t VarId = {
            .id = 1,
            .ptr = i
        };
        if (paramsToPrint[i].type & PARAM_GROUP) {
            // memset(group, 0 , 20);
            group = paramsToPrint[i].name;
            // sprintf(pcWriteBuffer, "i: %-3d, n:%-16s\r\n", i, paramsToPrint[i].name);
        }else if (paramsToPrint[i].type == PARAM_FLOAT) {
            sprintf(pcWriteBuffer, "i: %-3d, n:%-16s, g:%-16s, type:F,%-3d  v:%0.4f\r\n", i, paramsToPrint[i].name, group, paramsToPrint[i].type, paramGetFloat(VarId));
        } else {
            sprintf(pcWriteBuffer, "i: %-3d, n:%-16s, g:%-16s, type:i,%-3d  v:%d\r\n", i, paramsToPrint[i].name, group, paramsToPrint[i].type, paramGetInt(VarId));
        }
        size_t localLen = strlen((char*)pcWriteBuffer);
        pcWriteBuffer += localLen;
        totalLen += localLen;
        if (totalLen > BufferSize - 100) {
            break;
        }
    }
}