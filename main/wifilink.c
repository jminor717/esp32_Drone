/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2012 BitCraze AB
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
 * wifilink.c: ESP32 Wi-Fi implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "wifilink.h"
#include "wifi_esp32.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "queuemonitor.h"
#include "freertos/semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "WIFILINK"
#include "debug_cf.h"
#include "static_mem.h"

#define WIFI_ACTIVITY_TIMEOUT_MS (1000)

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static uint8_t sendBuffer[64];

static UDPPacket wifiIn;
static CRTPPacket p;
static uint32_t lastPacketTick;

static int wifilinkSendPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);
static void wifilinkTask(void *param);

STATIC_MEM_TASK_ALLOC(wifilinkTask, USBLINK_TASK_STACKSIZE);

static bool wifilinkIsConnected(void)
{
    return (xTaskGetTickCount() - lastPacketTick) < M2T(WIFI_ACTIVITY_TIMEOUT_MS);
}

static struct crtpLinkOperations wifilinkOp = {
    .setEnable = wifilinkSetEnable,
    .sendPacket = wifilinkSendPacket,
    .receivePacket = wifilinkReceiveCRTPPacket,
    .isConnected = wifilinkIsConnected,
};

void wifilinkInit()
{
    if (isInit)
    {
        return;
    }

    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    STATIC_MEM_TASK_CREATE(wifilinkTask, wifilinkTask, WIFILINK_TASK_NAME, NULL, WIFILINK_TASK_PRI);

    isInit = true;
}

static void wifilinkTask(void *param)
{
    while (1)
    {
        /* command step - receive  03 Fetch a wifi packet off the queue */
        wifiGetDataBlocking(&wifiIn);
        lastPacketTick = xTaskGetTickCount();

        /* command step - receive  04 copy CRTP part from packet, the size not contain head */
        p.size = wifiIn.size - 1;
        // memcpy(&p.raw, wifiIn.data, wifiIn.size);
        memcpy(&p, wifiIn.data, sizeof(CRTPPacket));

        // uint8_t cksum = p.data[1];
        // p.data[1] = 0;
        // uint8_t cksum2 = calculate_cksum(p.data, CRTP_MAX_DATA_SIZE);
        // if (cksum == cksum2)
        // {
        /* command step - receive 05 send to crtpPacketDelivery queue */
        //?DEBUG_PRINTI("xQueueSend wifilinkTask");
        xQueueSend(crtpPacketDelivery, &p, 0);
        // }
        // else
        // {
        //     DEBUG_PRINTI("CRTP packet cksum unmatched c1%d, c2%d", cksum, cksum2);
        // }
    }
}

static int wifilinkReceiveCRTPPacket(CRTPPacket *p)
{
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
    {
        //! ledseqRun(&seq_linkUp);
        return 0;
    }

    return -1;
}

static int wifilinkSendPacket(CRTPPacket *p)
{
    int dataSize;

    ASSERT(p->size < SYSLINK_MTU);

    sendBuffer[0] = p->header;

    if (p->size <= CRTP_MAX_DATA_SIZE)
    {
        memcpy(&sendBuffer[1], p->data, p->size);
    }

    dataSize = p->size + 1;

    /*ledseqRun(&seq_linkDown);*/

    return wifiSendData(dataSize, sendBuffer);
}

static int wifilinkSetEnable(bool enable)
{
    return 0;
}

bool wifilinkTest()
{
    return isInit;
}

struct crtpLinkOperations *wifilinkGetLink()
{
    return &wifilinkOp;
}
