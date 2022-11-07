#ifndef _DATA_TYPES_H_
#define _DATA_TYPES_H_

#include <stdint.h>
#include <string.h>

#define CRTP_MAX_DATA_SIZE 30

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static uint8_t calculate_cksum(uint8_t* data, size_t len)
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
#pragma GCC diagnostic pop

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
                    uint8_t channel : 2; //< Selected channel within port
                    uint8_t reserved : 2;
                    uint8_t port : 4; //< Selected port
                };
            };
            uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
        };
        uint8_t raw[CRTP_MAX_DATA_SIZE + 1]; //< The full packet "raw"
    };
} __attribute__((packed)) CRTPPacket;

// data[0] type
// data[1] checksum for data array

typedef enum
{
    CRTP_PORT_CONSOLE = 0x00,
    CRTP_PORT_PARAM = 0x02,
    CRTP_PORT_SETPOINT = 0x03, // raw setpoint comand for direct rol pitch yaw thrust control
    CRTP_PORT_MEM = 0x04,
    CRTP_PORT_LOG = 0x05,
    CRTP_PORT_LOCALIZATION = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07, // setpoint comands defined in packetType_e
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
    ControllerType = 8,
    DO_NOT_USE_STRUCT_LENGTH = 9,
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
struct altHoldPacket_s
{
    uint8_t type;    // placeholder, not directly needed for the alt hold function but needs to be stored in the data array
    float roll;      // rad
    float pitch;     // ...
    float yawrate;   // deg/s
    float zVelocity; // m/s in the world frame of reference
};

/* hoverDecoder  hoverType
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */
struct hoverPacket_s
{
    uint8_t type;    // placeholder, not directly needed for the alt hold function but needs to be stored in the data array
    float vx;        // m/s in the body frame of reference
    float vy;        // ...
    float yawrate;   // deg/s
    float zDistance; // m in the world frame of reference
};

struct _BtnCTX
{
    union
    {
        struct
        {
            uint8_t XCount : 3;
            uint8_t OCount : 3;
            uint8_t TriangleCount : 3;
            uint8_t SquareCount : 3;
            uint8_t UpCount : 3;
            uint8_t DownCount : 3;
            uint8_t LeftCount : 3;
            uint8_t RightCount : 3;
            uint8_t R1Count : 2;
            uint8_t L1Count : 2;
            uint8_t R3Count : 2;
            uint8_t L3Count : 2;
        };
    };
};// __attribute__((packed));
typedef struct _BtnCTX BtnCTX;

struct RawControlsPacket_s
{
    int8_t Rx;
    int8_t Ry;
    uint8_t R2;
    int8_t Lx;
    int8_t Ly;
    uint8_t L2;
    BtnCTX ButtonCount;
};

struct RawControllsPackettCOMPACT_s
{
    union
    {
        struct
        {
            uint8_t X : 1;
            uint8_t O : 1;
            uint8_t Triangle : 1;
            uint8_t Square : 1;
            uint8_t Up : 1;
            uint8_t Down : 1;
            uint8_t Left : 1;
            uint8_t Right : 1;
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t R3 : 1;
            uint8_t L3 : 1;
            uint8_t Reserved : 4;
        };
    };
    union
    {
        struct
        {
            uint8_t Rx : 1;
            uint8_t Ry : 1;
            uint8_t R2 : 1;
            uint8_t Lx : 1;
            uint8_t Ly : 1;
            uint8_t L2 : 1;
            uint8_t Reserved2 : 2;
        };
    };
    uint8_t data[];
};

typedef struct altHoldPacket_s altHoldPacket_s;
typedef struct hoverPacket_s hoverPacket_s;
typedef struct RawControlsPacket_s RawControlsPacket_s;
typedef struct RawControllsPackettCOMPACT_s RawControllsPackettCOMPACT_s;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void altHoldPacket_Decode_Min(altHoldPacket_s *packet, const uint8_t *data)
{
    packet->type = data[0]; // note to self all the folowing options work the same (assuming you actually cast to int16 and not int8)
    packet->roll = ((int16_t)(data[1] + (data[2] << 8))) / 62.5;
    packet->pitch = ((int16_t)((data[4] << 8) + data[3])) / 62.5;
    packet->yawrate = ((int16_t)((data[6] << 8) | data[5])) / 62.5;
    packet->zVelocity = (data[7] / 510.0) - 0.1;
}
static uint8_t altHoldPacket_Encode_Min(int16_t r, int16_t p, int16_t y, uint8_t t, uint8_t *data)
{
    data[0] = altHoldType;
    data[1] = r & 0xFF;
    data[2] = (r >> 8) & 0xFF;
    data[3] = p & 0xFF;
    data[4] = (p >> 8) & 0xFF;
    data[5] = y & 0xFF;
    data[6] = (y >> 8) & 0xFF;
    data[7] = t;
    return 8;
}

static void hoverPacket_Decode_Min(hoverPacket_s *packet, const uint8_t *data)
{
    packet->type = data[0];
    packet->vx = ((int16_t)(data[1] + (data[2] << 8))) / 62.5;
    packet->vy = ((int16_t)((data[4] << 8) + data[3])) / 62.5;
    packet->yawrate = ((int16_t)((data[6] << 8) | data[5])) / 62.5;
    packet->zDistance = (data[7] / 50.0);
}

static uint8_t hoverPacket_Encode_Min(int16_t vx, int16_t vy, int16_t y, uint8_t z, uint8_t *data)
{
    data[0] = hoverType;
    data[1] = vx & 0xFF;
    data[2] = (vx >> 8) & 0xFF;
    data[3] = vy & 0xFF;
    data[4] = (vy >> 8) & 0xFF;
    data[5] = y & 0xFF;
    data[6] = (y >> 8) & 0xFF;
    data[7] = z;
    return 8;
}

#pragma GCC diagnostic pop

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
    metaStartOTAWifi,
    nMetaCommands, // total number of meta comands, add new comands before this line
};

enum STM32_SPI_CMD
{
    defaultTransmition = 0,
    configServos = 0x11
};

typedef struct _SPI_ESP_PACKET_HEADER
{
    uint8_t PacketCRC;
    uint8_t nextSpiSize; //< Size of data
    uint8_t altCmd;
    union
    {
        uint8_t RX_Contents; //< Header selecting channel and port
        struct
        {
            uint8_t motorSpeeed : 1;
            uint8_t servoAngle : 1;
            uint8_t radioSendData : 1;
            uint8_t reserved : 4;
            uint8_t specialData : 1; //?
        };
    };

    union
    {
        uint8_t TX_Request; //< Header selecting channel and port
        struct
        {
            uint8_t motorSpeeed_TX : 1;
            uint8_t servoAngle_TX : 1;
            uint8_t radioSendData_TX : 1;
            uint8_t reserved_TX : 5;
        };
    };
} __attribute__((packed)) SPI_ESP_PACKET_HEADER;

enum MotorPacket
{
    motor1RPM_a,
    motor1RPM_b,
    motor2RPM_a,
    motor2RPM_b,
    motor3RPM_a,
    motor3RPM_b,
    motor4RPM_a,
    motor4RPM_b
};

enum ServoPacket
{
    servo1Angle_a,
    servo1Angle_b,
    servo2Angle_a,
    servo2Angle_b,
    servo3Angle_a,
    servo3Angle_b,
    servo4Angle_a,
    servo4Angle_b
};

// 16 bit spi buffer
enum registers
{
    // output

    // inputs
    motor1Setpoint_a,
    motor1Setpoint_b,
    motor2Setpoint_a,
    motor2Setpoint_b,
    motor3Setpoint_a,
    motor3Setpoint_b,
    motor4Setpoint_a,
    motor4Setpoint_b,
    servo1Setpoint_a,
    servo1Setpoint_b,
    servo2Setpoint_a,
    servo2Setpoint_b,
    servo3Setpoint_a,
    servo3Setpoint_b,
    servo4Setpoint_a,
    servo4Setpoint_b,
    // cfg
    motor1CFG,
    motor2CFG,
    motor3CFG,
    motor4CFG,
    servo1CFG,
    servo2CFG,
    servo3CFG,
    servo4CFG,

};

#endif