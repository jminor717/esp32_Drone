/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 *
 */
#include <stdbool.h>
#include <stddef.h>

#include "crtp_commander.h"

#include "cfassert.h"
#include "commander.h"
#include "crtp.h"
#define DEBUG_MODULE "CRPTDECODE"
#include "debug_cf.h"

static bool isInit;

static void commanderCrtpCB(CRTPPacket* pk);

void crtpCommanderInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
  isInit = true;
}

typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);

/* ---===== 2 - Decoding functions =====--- */

/* notifySetpointsStop meta-command. See commander.h function
 * commanderNotifySetpointsStop() for description and motivation.
 */
struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));
void notifySetpointsStopDecoder(const void *data, size_t datalen)
{
  ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
  const struct notifySetpointsStopPacket *values = data;
  commanderNotifySetpointsStop(values->remainValidMillisecs);
}

 /* ---===== packetDecoders array =====--- */
const static metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};

/* Decoder switch */
static void commanderCrtpCB(CRTPPacket* pk)
{
  static setpoint_t setpoint;
  DEBUG_PRINTI("commanderCrtpCB got type p%d, c%d, d%d", pk->port, pk->channel, pk->data[0]);

  if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
    crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
    switch (pk->channel) {
    case SET_SETPOINT_CHANNEL:
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      break;
    case META_COMMAND_CHANNEL: {
        uint8_t metaCmd = pk->data[0];
        if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL)) {
          metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
        }
      }
      break;
    default:
      /* Do nothing */
      break;
    }
  }
}
