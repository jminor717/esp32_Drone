/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 * radiolink.c - Radio link layer
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>


/*FreeRtos includes*/
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"


#include "config.h"
#include "configblock.h"
#include "crtp.h"
#include "led.h"
#include "ledseq.h"
#include "log.h"
#include "queuemonitor.h"
#include "radiolink.h"
#include "syslink.h"


#define DEBUG_MODULE "RADIOLINK"
#include "debug_cf.h"
#include "static_mem.h"

#include <Arduino.h>
#include <SPI.h>

#define RH_PLATFORM 14
#include <../Common/Submodules/RadioHead/RH_RF69.h>

#include "stm32_legacy.h"

#define RADIOLINK_TX_QUEUE_SIZE (1)
#define RADIOLINK_CRTP_QUEUE_SIZE (5)
#define RADIO_ACTIVITY_TIMEOUT_MS (1000)

#define RADIOLINK_P2P_QUEUE_SIZE (5)

static xQueueHandle txQueue;
STATIC_MEM_QUEUE_ALLOC(txQueue, RADIOLINK_TX_QUEUE_SIZE, sizeof(CRTPPacket));

static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, RADIOLINK_CRTP_QUEUE_SIZE, sizeof(CRTPPacket));

STATIC_MEM_TASK_ALLOC(radioLinkTask, LEDSEQCMD_TASK_STACKSIZE); // todo test lowering the stack size

STATIC_MEM_TASK_ALLOC(radioISRTask, ADC_TASK_STACKSIZE);

/* Allow ISR to comunicate with main task */
static xSemaphoreHandle RF69InteruptMutex;
// static StaticSemaphore_t RF69InteruptMutexBuffer;

static xSemaphoreHandle RF69TXMutex;
// static StaticSemaphore_t RF69TXMutexBuffer;

static xSemaphoreHandle RF69RXMutex;
// static StaticSemaphore_t RF69RXMutexBuffer;

static bool isInit;

static int radioLinkSendCRTPPacket(CRTPPacket* p);
static int radioLinkSetEnable(bool enable);
static int radioLinkReceiveCRTPPacket(CRTPPacket* p);
static void radioLinkTask(void* param);
static void radioISRTask(void* param);

// Local RSSI variable used to enable logging of RSSI values from Radio
static uint8_t rssi;
static uint32_t lastPacketTick;

static volatile P2PCallback p2p_callback;

RH_RF69 rf69(COM13909_SS, COM13909_INT0);

static bool radioLinkIsConnected(void)
{
    return (xTaskGetTickCount() - lastPacketTick) < M2T(RADIO_ACTIVITY_TIMEOUT_MS);
}

static int radioLinkreset(void)
{
    return (int)(xTaskGetTickCount() - lastPacketTick);
}

/**
 * @brief
 * override RH_RF69 implamentation of this ISR so that we can handle all of the spi operations outside of the ISR
 * @param arg pointer to ISR arguments
 */
void RH_INTERRUPT_ATTR RH_RF69::isr0(void* arg)
{
    // uint32_t gpio_num = (uint32_t)arg;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(RF69InteruptMutex, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static struct crtpLinkOperations radioLinkOp = {
    .setEnable = radioLinkSetEnable,
    .sendPacket = radioLinkSendCRTPPacket,
    .receivePacket = radioLinkReceiveCRTPPacket,
    .isConnected = radioLinkIsConnected,
    .reset = radioLinkreset,
};

void radioLinkInit(void)
{
    if (isInit)
        return;

    txQueue = STATIC_MEM_QUEUE_CREATE(txQueue);
    DEBUG_QUEUE_MONITOR_REGISTER(txQueue);

    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    RF69InteruptMutex = xSemaphoreCreateBinary();
    RF69RXMutex = xSemaphoreCreateBinary();
    RF69TXMutex = xSemaphoreCreateBinary();

    // RF69RXMutex = xSemaphoreCreateMutexStatic(&RF69RXMutexBuffer);
    // RF69TXMutex = xSemaphoreCreateMutexStatic(&RF69TXMutexBuffer);

    SPI.begin(SPI_DEV_SCK, SPI_DEV_MISO, SPI_DEV_MOSI, -1);

    if (!rf69.init()) {
        DEBUG_PRINTW("RF69 setup failed\n");
    }
    rf69.setTxPower(-2, true);

    STATIC_MEM_TASK_CREATE(radioLinkTask, radioLinkTask, RADIO_TASK_NAME, NULL, RADIOLINK_TASK_PRI);
    STATIC_MEM_TASK_CREATE(radioISRTask, radioISRTask, "RADIOLINK_ISR", NULL, 4);

    //!   radioLinkSetChannel(configblockGetRadioChannel());
    //   radioLinkSetDatarate(configblockGetRadioSpeed());
    //   radioLinkSetAddress(configblockGetRadioAddress());

    isInit = true;
}

typedef enum {
    RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
    RHModeSleep, ///< Transport hardware is in low power sleep mode (if supported)
    RHModeIdle, ///< Transport is idle.
    RHModeTx, ///< Transport is in the process of transmitting a message.
    RHModeRx, ///< Transport is in the process of receiving a message.
    RHModeCad ///< Transport is in the process of detecting channel activity (if supported)
} RHMode;

/**
 * @brief high priority task to handle inturupts and read the spi buffer after a packet is received
 * 
 * @param param task params pointer
 */
static void radioISRTask(void* param)
{
    while (1) {
        //its not a huge problem to run this every second but without this there are some unresolved issues that lead to this indefinitely blocking at startup
        xSemaphoreTake(RF69InteruptMutex, 1000);
        uint8_t mode = rf69.handleInterrupt();

        if (mode == (uint8_t)RHModeRx) {
            xSemaphoreGive(RF69RXMutex);
        } else if (mode == (uint8_t)RHModeTx) {
            xSemaphoreGive(RF69TXMutex);
        }
    }
}

/**
 * @brief medium priority task to parse received packets and send a responce
 *
 * @remark seperated from radioISRTask to keep time spent in a high priority task to a minimum
 *
 * @param param task params pointer
 */
static void radioLinkTask(void* param)
{
    DEBUG_PRINTI("RF69 setup Passed");
    CRTPPacket cmd;

    while (1) {
        // set from radioISRTask when a new packet has been read from the radio module, needs a finite timeout to avoid indefinitely blocking at startup
        xSemaphoreTake(RF69RXMutex, 1500);
        if (rf69.available()) {
            lastPacketTick = xTaskGetTickCount();
            uint8_t buf[RH_RF69_MAX_MESSAGE_LEN] = { 0 };
            uint8_t len = rf69.recv(buf, 0);
            if (len) {
                rssi = rf69.lastRssi();
                memcpy(&cmd, buf, len);
                uint8_t cksum = buf[len - 1];
                // remove cksum, do not belong to CRTP
                uint8_t cksumCalk = calculate_cksum(buf, len - 1);

                // check packet
                if (cksum == cksumCalk && len < 64) {
                    xQueueSend(crtpPacketDelivery, &cmd, 0);

                    // if (cmd.data[0] == ControllerType)
                    // {
                    //     struct RawControlsPacket_s DataOut;
                    //     memcpy(&DataOut, cmd.data + 1, sizeof(RawControlsPacket_s));

                    // TODO: tailor responce based of of current state and cmd inputs, eg acknowledge specific button presses
                    // }
                    // else
                    // {
                    //     DEBUG_PRINTI("got request: %d, %d, %d, %d   %s   RSSI: %d", buf[0], buf[1], buf[2], buf[3], (char *)buf, rf69.lastRssi());
                    // }
                    uint8_t data[] = "ReSp";
                    rf69.send(data, sizeof(data));
                } else {
                    DEBUG_PRINTD("udp packet cksum unmatched c1:%d, c2:%d", cksum, cksumCalk);
                    uint8_t data[12];
                    snprintf((char*)data, sizeof(data), "Rs:%d,%d", cksum, cksumCalk);
                    rf69.send(data, sizeof(data));
                }
                // wait for the reply to be sent
                xSemaphoreTake(RF69TXMutex, portMAX_DELAY);
            } else {
                DEBUG_PRINTI(".");
            }
            rf69.setModeRx();
        }
    }
}

bool radioLinkTest(void)
{
    return isInit; // syslinkTest();
}

static int radioLinkReceiveCRTPPacket(CRTPPacket* p)
{
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
        return 0;
    }

    return -1;
}

/**
 * @brief currently has no affect,
 * TODO implament packet sending in radioLinkTask
 *
 * @param p
 * @return int 1 success, 0 failure
 */
static int radioLinkSendCRTPPacket(CRTPPacket* p)
{
    static SyslinkPacket slp;

    // ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

    slp.type = SYSLINK_RADIO_RAW;
    slp.length = p->size + 1;
    memcpy(slp.data, &p->header, p->size + 1);

    if (xQueueSend(txQueue, &slp, M2T(100)) == pdTRUE) {
        return true;
    }

    return false;
}

struct crtpLinkOperations* radioLinkGetLink()
{
    return &radioLinkOp;
}

static int radioLinkSetEnable(bool enable)
{
    return 0;
}

void radioLinkSetChannel(uint8_t channel)
{
    //   SyslinkPacket slp;

    //   slp.type = SYSLINK_RADIO_CHANNEL;
    //   slp.length = 1;
    //   slp.data[0] = channel;
    //   syslinkSendPacket(&slp);
}

void radioLinkSetDatarate(uint8_t datarate)
{
    //   SyslinkPacket slp;

    //   slp.type = SYSLINK_RADIO_DATARATE;
    //   slp.length = 1;
    //   slp.data[0] = datarate;
    //   syslinkSendPacket(&slp);
}

void radioLinkSetAddress(uint64_t address)
{
    //   SyslinkPacket slp;

    //   slp.type = SYSLINK_RADIO_ADDRESS;
    //   slp.length = 5;
    //   memcpy(&slp.data[0], &address, 5);
    //   syslinkSendPacket(&slp);
}

void radioLinkSetPowerDbm(int8_t powerDbm)
{
    //   SyslinkPacket slp;

    //   slp.type = SYSLINK_RADIO_POWER;
    //   slp.length = 1;
    //   slp.data[0] = powerDbm;
    //   syslinkSendPacket(&slp);
}

//!
// void radioLinkSyslinkDispatch(SyslinkPacket *slp)
// {
//     static SyslinkPacket txPacket;

//     if (slp->type == SYSLINK_RADIO_RAW || slp->type == SYSLINK_RADIO_RAW_BROADCAST)
//     {
//         lastPacketTick = xTaskGetTickCount();
//     }

//     if (slp->type == SYSLINK_RADIO_RAW)
//     {
//         slp->length--; // Decrease to get CRTP size.
//         xQueueSend(crtpPacketDelivery, &slp->length, 0);
//         ledseqRun(&seq_linkUp);
//         // If a radio packet is received, one can be sent
//         if (xQueueReceive(txQueue, &txPacket, 0) == pdTRUE)
//         {
//             ledseqRun(&seq_linkDown);
//             syslinkSendPacket(&txPacket);
//         }
//     }
//     else if (slp->type == SYSLINK_RADIO_RAW_BROADCAST)
//     {
//         slp->length--; // Decrease to get CRTP size.
//         xQueueSend(crtpPacketDelivery, &slp->length, 0);
//         ledseqRun(&seq_linkUp);
//         // no ack for broadcasts
//     }
//     else if (slp->type == SYSLINK_RADIO_RSSI)
//     {
//         // Extract RSSI sample sent from radio
//         memcpy(&rssi, slp->data, sizeof(uint8_t)); // rssi will not change on disconnect
//     }
//     else if (slp->type == SYSLINK_RADIO_P2P_BROADCAST)
//     {
//         ledseqRun(&seq_linkUp);
//         P2PPacket p2pp;
//         p2pp.port = slp->data[0];
//         p2pp.rssi = slp->data[1];
//         memcpy(&p2pp.data[0], &slp->data[2], slp->length - 2);
//         p2pp.size = slp->length;
//         if (p2p_callback)
//             p2p_callback(&p2pp);
//     }

//     isConnected = radioLinkIsConnected();
// }

// LOG_GROUP_START(radio)
// LOG_ADD(LOG_UINT8, rssi, &rssi)
// LOG_ADD(LOG_UINT8, isConnected, &isConnected)
// LOG_GROUP_STOP(radio)
