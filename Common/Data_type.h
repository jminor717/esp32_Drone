#include <stdint.h>
#include <string.h>

#define CRTP_MAX_DATA_SIZE 30

static uint8_t calculate_cksum(uint8_t *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++)
    {
        cksum += *(c++);
    }

    return cksum;
}

typedef struct _CRTPPacket
{
    uint8_t size; //< Size of data
    union
    {
        struct
        {
            union
            {
                uint8_t header; //< Header selecting channel and port
                struct
                {
#ifndef CRTP_HEADER_COMPAT
                    uint8_t channel : 2; //< Selected channel within port
                    uint8_t reserved : 2;
                    uint8_t port : 4; //< Selected port
#else
                    uint8_t channel : 2;
                    uint8_t port : 4;
                    uint8_t reserved : 2;
#endif
                };
            };
            uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
        };
        uint8_t raw[CRTP_MAX_DATA_SIZE + 1]; //< The full packet "raw"
    };
} __attribute__((packed)) CRTPPacket;

//data[0] type
//data[1] checksum for data array

typedef enum
{
    CRTP_PORT_CONSOLE = 0x00,
    CRTP_PORT_PARAM = 0x02,
    CRTP_PORT_SETPOINT = 0x03, //raw setpoint comand for direct rol pitch yaw thrust control
    CRTP_PORT_MEM = 0x04,
    CRTP_PORT_LOG = 0x05,
    CRTP_PORT_LOCALIZATION = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07, //setpoint comands defined in packetType_e
    CRTP_PORT_SETPOINT_HL = 0x08,      // high level comands defined in TrajectoryCommand_e
    CRTP_PORT_PLATFORM = 0x0D,
    CRTP_PORT_LINK = 0x0F,
} CRTPPort;

/* ---===== 1 - packetType_e enum =====--- */
enum packet_type
{
    stopType = 0,
    velocityWorldType = 1,
    zDistanceType = 2,
    cppmEmuType = 3,
    altHoldType = 4,
    hoverType = 5,
    fullStateType = 6,
    positionType = 7,
};
typedef enum packet_type packet_type;

/* velocityDecoder  velocityWorldType
 * Set the Crazyflie velocity in the world coordinate system
 */
struct velocityPacket_s
{
    float vx;      // m in the world frame of reference
    float vy;      // ...
    float vz;      // ...
    float yawrate; // deg/s
} __attribute__((packed));

/* zDistanceDecoder  zDistanceType
 * Set the Crazyflie absolute height and roll/pitch angles
 */
struct zDistancePacket_s
{
    float roll;      // deg
    float pitch;     // ...
    float yawrate;   // deg/s
    float zDistance; // m in the world frame of reference
} __attribute__((packed));

/* altHoldDecoder  altHoldType
 * Set the Crazyflie vertical velocity and roll/pitch angle
 */
typedef struct altHoldPacket_s
{
    uint8_t checksum;
    uint8_t type;
    float roll;      // rad
    float pitch;     // ...
    float yawrate;   // deg/s
    float zVelocity; // m/s in the world frame of reference
} __attribute__((packed));
typedef struct altHoldPacket_s altHoldPacket_s;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void altHoldPacket_Decode(altHoldPacket_s *packet, const uint8_t *data)
{
    packet->checksum = data[1];
    packet->type = data[0];
    memcpy(&packet->roll, &data[2], sizeof(packet->roll));
    memcpy(&packet->pitch, &data[6], sizeof(packet->pitch));
    memcpy(&packet->yawrate, &data[10], sizeof(packet->yawrate));
    memcpy(&packet->zVelocity, &data[14], sizeof(packet->zVelocity));
}
static void altHoldPacket_Encode(altHoldPacket_s *packet, uint8_t *data, packet_type type)
{
    data[0] = type;
    memcpy(&data[2], &packet->roll, sizeof(packet->roll));
    memcpy(&data[6], &packet->pitch, sizeof(packet->pitch));
    memcpy(&data[10], &packet->yawrate, sizeof(packet->yawrate));
    memcpy(&data[14], &packet->zVelocity, sizeof(packet->zVelocity));
    data[1] = calculate_cksum(data, CRTP_MAX_DATA_SIZE);
}
#pragma GCC diagnostic pop

/* hoverDecoder  hoverType
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */
struct hoverPacket_s
{
    float vx;        // m/s in the body frame of reference
    float vy;        // ...
    float yawrate;   // deg/s
    float zDistance; // m in the world frame of reference
} __attribute__((packed));

/**
 * CRTP commander rpyt packet format
 */
struct CommanderCrtpLegacyValues
{
    float roll;  // deg
    float pitch; // deg
    float yaw;   // deg
    uint16_t thrust;
} __attribute__((packed));

// trajectory command (first byte of crtp packet)
enum TrajectoryCommand_e
{
    COMMAND_SET_GROUP_MASK = 0,
    COMMAND_TAKEOFF = 1, // Deprecated, use COMMAND_TAKEOFF_2
    COMMAND_LAND = 2,    // Deprecated, use COMMAND_LAND_2
    COMMAND_STOP = 3,
    COMMAND_GO_TO = 4,
    COMMAND_START_TRAJECTORY = 5,
    COMMAND_DEFINE_TRAJECTORY = 6,
    COMMAND_TAKEOFF_2 = 7,
    COMMAND_LAND_2 = 8,
    COMMAND_TAKEOFF_WITH_VELOCITY = 9,
    COMMAND_LAND_WITH_VELOCITY = 10,
};

enum crtpSetpointGenericChannel
{
    SET_SETPOINT_CHANNEL = 0,
    META_COMMAND_CHANNEL = 1,
};

/* Channel 1 of the generic commander port is used for "meta-commands"
 * that alter the behavior of the commander itself, e.g. mode switching.
 * Although we use the generic commander port due to increasing pressure on the
 * 4-bit space of ports numbers, meta-commands that are unrelated to
 * streaming generic setpoint control modes are permitted.
 *
 * The packet format for meta-commands is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * TYPE is an 8-bit value. The remainder of the data depends on the command.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the metaCommand_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the metaCommandDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

/* ---===== 1 - metaCommand_e enum =====--- */
enum metaCommand_e
{
    metaNotifySetpointsStop = 0,
    nMetaCommands, //total number of meta comands, add new comands before this line
};