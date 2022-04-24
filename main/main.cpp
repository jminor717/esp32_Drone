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
}

#include <Arduino.h>
#include <SPI.h>

#define RH_PLATFORM 1
#include <..\Common\Submodules\RadioHead\RH_RF69.h>

//#include "debug_cf.h"
#define GPIO_HANDSHAKE 2
#define SENSOR_INCLUDED_MPU9250_LPS25H

extern "C"
{
    void app_main();
}

RH_RF69 rf69(COM13909_SS, 46);

void app_main()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    // esp_pm_config_esp32 cfg;

    // esp_pm_configure();

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

    printf("prepairing to Launch system \n");

    // gpio_set_level(_slaveSelectPin, 0);

    fflush(stdout);
    /*launch the system task */

    // spi_bus_config_t buscfg;
    // memset(&buscfg, 0, sizeof(buscfg));

    // buscfg.miso_io_num = SPI_DEV_MISO;
    // buscfg.mosi_io_num = SPI_DEV_MOSI;
    // buscfg.sclk_io_num = SPI_DEV_SCK;
    // buscfg.quadwp_io_num = -1;
    // buscfg.quadhd_io_num = -1;
    // buscfg.max_transfer_sz = 4096;
    // buscfg.flags = 0;
    // buscfg.intr_flags = ESP_INTR_FLAG_IRAM;

    // ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    // ESP_ERROR_CHECK(ret);

    // int16_t RF69_Good = RH_RF69_init(SPI3_HOST);
    // if (RF69_Good != -1)
    // {
    //     printf("RF69 setup failed %d\n", RF69_Good);
    //     fflush(stdout);
    // }
    // else
    // {
    //     printf("RF69 setup succeeded\n");
    //     fflush(stdout);
    systemLaunch();
    // }

    // setFrequency(915.0);

    // setTxPower(10, true);

    // For compat with RFM69 Struct_send
    // setModemConfig(GFSK_Rb250Fd250);
    // setPreambleLength(3);
    // uint8_t syncwords[] = {0x2d, 0x64};
    // setSyncWords(syncwords, sizeof(syncwords));
    // setEncryptionKey((uint8_t *)"thisIsEncryptKey");

    vTaskDelay(50);

    SPI.begin(SPI_DEV_SCK, SPI_DEV_MISO, SPI_DEV_MOSI, COM13909_SS);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);

    // RH_RF69(COM13909_SS, 46, 1);
    if (!rf69.init())
    {
        printf("RF69 setup failed\n");
        fflush(stdout);
    }
    rf69.setTxPower(-2, true);

    int32_t lastTick = xTaskGetTickCount();
    int32_t currentTick = xTaskGetTickCount();
    for (;;)
    {
        // printf("_");
        // fflush(stdout);
        if (rf69.available())
        {
            // Should be a message for us now
            uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = {0};
            uint8_t len = sizeof(buf);
            if (rf69.recv(buf, &len))
            {
                CRTPPacket cmd;
                // memcpy(&cmd, wifiIn.data, sizeof(CRTPPacket));
                uint8_t CRTP_len = buf[0];
                memcpy(&cmd, buf, CRTP_len);
                uint8_t checkSum = calculate_cksum(buf, CRTP_len);

                if (cmd.data[0] == ControllerType)
                {
                    struct RawControllsPackett_s DataOut;
                    memcpy(&DataOut, cmd.data + 1, sizeof(RawControllsPackett_s));

                    // log_v("lx:%d, ly:%d, rx:%d, ry:%d, r2:%d, l2:%d, " PRINTF_BINARY_PATTERN_INT32 "", DataOut.Lx, DataOut.Ly, DataOut.Rx, DataOut.Ry, DataOut.R2, DataOut.L2, PRINTF_BYTE_TO_BINARY_INT32(DataOut.ButtonCount.ButtonCount));
                    printf("X:%d, O:%d, △:%d, ▢:%d, ←:%d, →:%d, ↑:%d, ↓:%d, R1:%d, R3:%d, L1:%d, L3:%d ____ lx:%d, ly:%d, rx:%d, ry:%d, r2:%d, l2:%d",
                           DataOut.ButtonCount.XCount, DataOut.ButtonCount.OCount, DataOut.ButtonCount.TriangleCount, DataOut.ButtonCount.SquareCount,
                           DataOut.ButtonCount.LeftCount, DataOut.ButtonCount.RightCount, DataOut.ButtonCount.UpCount, DataOut.ButtonCount.DownCount,
                           DataOut.ButtonCount.R1Count, DataOut.ButtonCount.R3Count, DataOut.ButtonCount.L1Count, DataOut.ButtonCount.R3Count,
                           DataOut.Lx, DataOut.Ly, DataOut.Rx, DataOut.Ry, DataOut.R2, DataOut.L2);
                }
                else
                {
                    printf("got request: ");
                    printf("%d, %d, %d, %d    ", buf[0], buf[1], buf[2], buf[3]);
                    printf((char *)buf);
                    printf("  RSSI: ");
                    printf("%d", rf69.lastRssi());
                    printf("\n");
                }
                // memcpy(cmd.data + 1, &ContorlData, sizeof(RawControllsPackett_s));

                //      RH_RF69::printBuffer("request: ", buf, len);

                fflush(stdout);

                // Send a reply
                uint8_t data[] = "And hello back to you";
                rf69.send(data, sizeof(data));
                rf69.waitPacketSent();
                // Serial.println("Sent a reply");
            }
            else
            {
                printf(".");
                fflush(stdout);
            }
        }

        vTaskDelay(1);
        // //handleInterrupt();
        // uint8_t len = 5;
        // uint8_t buf[5] = {0};
        // // memset(buf, 0, len);
        // recv(buf, &len);

        // // micros(); // update overflow
        // currentTick = xTaskGetTickCount();
        // printf("%d ||       ", currentTick - lastTick);
        // for (int i = 0; i < len; ++i)
        // {
        //     printf("%d, ", buf[i]);
        // }
        // printf("\n");
        // fflush(stdout);
        // setModeIdle();
        // setModeRx();
        // uint8_t data[] = "And hello back to you too";
        // rf69.send(data, sizeof(data));
        // rf69.waitPacketSent(30);
        // printf("_");
        // fflush(stdout);
        lastTick = currentTick;
    }
}