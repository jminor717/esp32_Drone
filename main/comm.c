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
 * comm.c - High level communication module
 */

#include <stdbool.h>
#define DEBUG_MODULE "COMM"
#include "comm.h"
#include "config.h"

#include "crtp.h"
//#include "console.h"
#include "crtpservice.h"
#include "debug_cf.h"
#include "log.h"
#include "param.h"

#if (COMMS_MODE == WIFI_COMMS_MODE)
#include "wifi_esp32.h"
#include "wifilink.h"
#else
#include "radiolink.h"
#endif
#include "crtp_localization_service.h"
#include "platformservice.h"


static bool isInit;

void commInit(void)
{
    if (isInit) {
        return;
    }

    /* These functions  are moved to be initialized early so
     * that DEBUG_PRINTD can be used early */

#if (COMMS_MODE == WIFI_COMMS_MODE)
    wifilinkInit();
#else
    radioLinkInit();
#endif

    crtpInit();
    // consoleInit();

#if (COMMS_MODE == WIFI_COMMS_MODE)
    crtpSetLink(wifilinkGetLink());
#else
    crtpSetLink(radioLinkGetLink());
#endif

    crtpserviceInit();
    // platformserviceInit();
    // logInit();
    // paramInit();
    // locSrvInit();

    // setup CRTP communication channel
    // TODO: check for USB first and prefer USB over radio
    // if (usbTest())
    //   crtpSetLink(usbGetLink);
    // else if (radioLinkTest())
    //     crtpSetLink(radioLinkGetLink());
    isInit = true;
}

bool commTest(void)
{
    bool pass = isInit;
#if (COMMS_MODE == WIFI_COMMS_MODE)
    pass &= wifilinkTest();
    DEBUG_PRINTI("wifilinkTest = %d ", pass);
#endif
    pass &= crtpTest();
    DEBUG_PRINTI("crtpTest = %d ", pass);
    // pass &= crtpserviceTest();
    // DEBUG_PRINTI("crtpserviceTest = %d ", pass);
    // pass &= platformserviceTest();
    // DEBUG_PRINTI("platformserviceTest = %d ", pass);
    // pass &= consoleTest();
    // DEBUG_PRINTI("consoleTest = %d ", pass);
    // pass &= paramTest();
    // DEBUG_PRINTI("paramTest = %d ", pass);

    return pass;
}
