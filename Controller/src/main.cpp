#define WIFI_COMMS_MODE 0
#define RF69_COMMS_MODE 1

#define COMMS_MODE RF69_COMMS_MODE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <Arduino.h>
#include "../Submodules/PS4-esp32/src/PS4Controller.h"
#include <../Common/Data_type.h>
#if (COMMS_MODE == WIFI_COMMS_MODE)
#include <WiFi.h>
#else
#include <SPI.h>
//#define RH_PLATFORM 14
#include <../Common/Submodules/RadioHead/RH_RF69.h>
#endif

//#include <atomic>
#include "esp32-hal-log.h"

#define WIFIssid "ESP-DRONE_BCDDC2D254AD"
#define WIFIpassword "12345678"
#define UDP_SERVER_PORT 2390
#define UDP_RECEIVE_PORT 2390

#define WIFI_TRANSMIT_RATE_Us 7000 // 40ms = 25hz

#define MIN_THRUST 1000
#define MAX_THRUST 60000

#define MIN_DEGREE 2.0
#define MAX_DEGREE 2.0

#define MIN_JOYCON_ANGLE 20

/*
low level send and receive interface
    wifi -> esp32 wifi, mimic existing drone implamentation
    long range radio -> combine TX and RX loops on both controller and Drone to help minimize likelihood of colission
high level independent command and setpoint transmition lops to allow commands to be sent more frequently than setpoints
    low level will run at the rate of the fastest high level and try to combine high level packets to minimize the time that TX is active (only for long range radio)

cd  \repos\ESPDrone\Controller
mklink /D "Common" "..\Common"
*/

static xSemaphoreHandle PacketReadySendSem;

int64_t now = esp_timer_get_time();
int64_t NextAvalableTransmit = esp_timer_get_time() + WIFI_TRANSMIT_RATE_Us;

bool Flying = false;
uint8_t type = stopType;

void handleControlUpdate();
void SendDataToDrone(CRTPPacket cmd, uint8_t len);

#if (COMMS_MODE == WIFI_COMMS_MODE)
IPAddress DroneAddress = IPAddress();
WiFiUDP udp = WiFiUDP();
// AsyncUDP udp;

void handleWifiConnected(system_event_t *event)
{
    // CRTPPacket cmd;
    // memset(&cmd, 0, sizeof(CRTPPacket));
    // cmd.channel = SET_SETPOINT_CHANNEL;
    // cmd.port = CRTP_PORT_SETPOINT_GENERIC;

    // altHoldPacket_Encode_Min(0, 0, 0, 0, cmd.data, stopType);

    // uint8_t cmdLength = sizeof(cmd);
    // uint8_t *buffer = (uint8_t *)calloc(1, cmdLength + 1);
    // memcpy(buffer, (const uint8_t *)&cmd, cmdLength);
    // buffer[cmdLength] = calculate_cksum(buffer, cmdLength);

    // AsyncUDPMessage msg;
    // msg.write(buffer, cmdLength + 1);
    // udp.broadcastTo(msg, UDP_SERVER_PORT);

    // free(buffer);
}

void handleWifiDropped(system_event_t *event)
{
    delay(5000);
    WiFi.begin(WIFIssid, WIFIpassword);
}

#else
#define MAIN_SPI_SCK 25
#define MAIN_SPI_MISO 26
#define MAIN_SPI_MOSI 27
#define MAIN_SPI_SS 13

RH_RF69 rf69(MAIN_SPI_SS, 39);
#endif

CRTPPacket cmd_Data;
uint8_t cmd_len;
bool has_cmd = false;

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    // "60:5b:b4:d9:7b:14"; 58:a0:23:dd:a1:84  4C:11:AE:DF:8B:F4  03:03:03:03:03:03
    // char mac[] = "60:5b:b4:d9:7b:14";
    char mac[] = "4C:11:AE:DF:8B:F4";
    PS4.begin(mac);
    Serial.println("Ready.");
    Serial.println(mac);
    PS4.attach(&handleControlUpdate);

    PacketReadySendSem = xSemaphoreCreateBinary();

#if (COMMS_MODE == WIFI_COMMS_MODE)
    DroneAddress.fromString("192.168.43.42");
    WiFi.begin(WIFIssid, WIFIpassword);
    // WiFi.setTxPower(WIFI_POWER_19_5dBm);
    udp.begin(UDP_SERVER_PORT);

    WiFi.onEvent((WiFiEventSysCb)handleWifiConnected, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent((WiFiEventSysCb)handleWifiDropped, SYSTEM_EVENT_STA_DISCONNECTED);
#else
    SPI.begin(MAIN_SPI_SCK, MAIN_SPI_MISO, MAIN_SPI_MOSI, MAIN_SPI_SS);
    if (!rf69.init())
        Serial.println("init failed");

    // if (!rf69.setFrequency(915.0))
    //     Serial.println("setFrequency failed");

    rf69.setTxPower(-2, true);

    // // For compat with RFM69 Struct_send
    // rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
    // rf69.setPreambleLength(3);
    // uint8_t syncwords[] = {0x2d, 0x64};
    // rf69.setSyncWords(syncwords, sizeof(syncwords));
    // rf69.setEncryptionKey((uint8_t *)"thisIsEncryptKey");
#endif

    log_v("Verbose");
    log_d("Debug");
    log_i("Info");
    log_w("Warning");
    log_e("Error");
    now = esp_timer_get_time();
}

uint16_t val = 0;

void handleControlUpdate()
{
    // Below has all accessible outputs from the controller
    if (PS4.isConnected())
    {
        now = esp_timer_get_time();
        val++;
        // if (PS4.Share())
        //     Serial.println("Share Button");
        // if (PS4.Options())
        //     Serial.println("Options Button");

        // if (PS4.PSButton())
        //     Serial.println("PS Button");
        // if (PS4.Touchpad())
        //     Serial.println("Touch Pad Button");

        // if (PS4.L2())
        //     Serial.printf("L2 button at %d\n", PS4.L2Value());
        // if (PS4.R2())
        //     Serial.printf("R2 button at %d\n", PS4.R2Value());

        // ps4_sensor_t sense = PS4.SensorData();
        // Serial.printf("% 06.3f: % 06.3f: % 06.3f    %05d  %05d  %05d\n", sense.accelerometer.x / 8192.0, sense.accelerometer.y / 8192.0, sense.accelerometer.z / 8192.0, sense.gyroscope.x, sense.gyroscope.y, sense.gyroscope.z);

        // if (PS4.Charging())
        //     Serial.println("The controller is charging");
        // if (PS4.Audio())
        //     Serial.println("The controller has headphones attached");
        // if (PS4.Mic())
        //     Serial.println("The controller has a mic attached");

        // Serial.printf("Battery Level : %d\n", PS4.Battery());
        if (now > NextAvalableTransmit)
        {
            val = 0;
            NextAvalableTransmit = now + WIFI_TRANSMIT_RATE_Us;
            /*

            [38]	active low	finger 1 id
            [39 - 41]	finger 1 coordinates
            [42]	active low	finger 2 id
            [43 - 45]	finger 2 coordinates
            */

            // log_v("    %d -- %d,%d,%d,%d,%d,%d,%d    %d -- %d,%d,%d,%d,%d,%d,%d    %d -- %d,%d,%d,%d,%d,%d,%d    %d -- %d,%d,%d,%d,%d,%d,%d",
            //       PS4.data.latestPacket[37], PS4.data.latestPacket[38], PS4.data.latestPacket[39], PS4.data.latestPacket[40], PS4.data.latestPacket[41], PS4.data.latestPacket[42], PS4.data.latestPacket[43], PS4.data.latestPacket[44], PS4.data.latestPacket[45],
            //       PS4.data.latestPacket[46], PS4.data.latestPacket[47], PS4.data.latestPacket[48], PS4.data.latestPacket[49], PS4.data.latestPacket[50], PS4.data.latestPacket[51], PS4.data.latestPacket[52], PS4.data.latestPacket[53], PS4.data.latestPacket[54],
            //       PS4.data.latestPacket[55], PS4.data.latestPacket[56], PS4.data.latestPacket[57], PS4.data.latestPacket[58], PS4.data.latestPacket[59], PS4.data.latestPacket[60], PS4.data.latestPacket[61], PS4.data.latestPacket[62], PS4.data.latestPacket[63],
            //       PS4.data.latestPacket[64], PS4.data.latestPacket[65], PS4.data.latestPacket[66], PS4.data.latestPacket[67], PS4.data.latestPacket[68], PS4.data.latestPacket[69], PS4.data.latestPacket[70], PS4.data.latestPacket[71], PS4.data.latestPacket[72]);

            CRTPPacket cmd;
            memset(&cmd, 0, sizeof(CRTPPacket));
            cmd.channel = SET_SETPOINT_CHANNEL;
            cmd.port = CRTP_PORT_SETPOINT_GENERIC;

            struct RawControllsPackett_s ContorlData;
            ContorlData.Rx = PS4.RStickX();
            ContorlData.Ry = PS4.RStickY();
            ContorlData.Lx = PS4.LStickX();
            ContorlData.Ly = PS4.LStickY();
            ContorlData.R2 = PS4.R2Value();
            ContorlData.L2 = PS4.L2Value();
            ContorlData.ButtonCount.RightCount = PS4.Right() || PS4.UpRight() || PS4.DownRight();
            ContorlData.ButtonCount.LeftCount = PS4.Left() || PS4.UpLeft() || PS4.DownLeft();
            ContorlData.ButtonCount.UpCount = PS4.Up() || PS4.UpLeft() || PS4.UpRight();
            ContorlData.ButtonCount.DownCount = PS4.Down() || PS4.DownLeft() || PS4.DownRight();
            ContorlData.ButtonCount.SquareCount = PS4.Square();
            ContorlData.ButtonCount.XCount = PS4.Cross();
            ContorlData.ButtonCount.OCount = PS4.Circle();
            ContorlData.ButtonCount.TriangleCount = PS4.Triangle();
            ContorlData.ButtonCount.L1Count = PS4.L1();
            ContorlData.ButtonCount.L3Count = PS4.L3();
            ContorlData.ButtonCount.R1Count = PS4.R1();
            ContorlData.ButtonCount.R3Count = PS4.R3();
            cmd.data[0] = ControllerType;
            memcpy(cmd.data + 1, &ContorlData, sizeof(RawControllsPackett_s));

            struct RawControllsPackett_s DataOut;
            memcpy(&DataOut, cmd.data + 1, sizeof(RawControllsPackett_s));

            // log_v("X:%d, O:%d, △:%d, ▢:%d, ←:%d, →:%d, ↑:%d, ↓:%d, R1:%d, R3:%d, L1:%d, L3:%d ____ lx:%d, ly:%d, rx:%d, ry:%d, r2:%d, l2:%d",
            //       DataOut.ButtonCount.XCount, DataOut.ButtonCount.OCount, DataOut.ButtonCount.TriangleCount, DataOut.ButtonCount.SquareCount,
            //       DataOut.ButtonCount.LeftCount, DataOut.ButtonCount.RightCount, DataOut.ButtonCount.UpCount, DataOut.ButtonCount.DownCount,
            //       DataOut.ButtonCount.R1Count, DataOut.ButtonCount.L3Count, DataOut.ButtonCount.L1Count, DataOut.ButtonCount.R3Count,
            //       DataOut.Lx, DataOut.Ly, DataOut.Rx, DataOut.Ry, DataOut.R2, DataOut.L2);

            // SendDataToDrone(cmd, sizeof(RawControllsPackett_s) + 1);
            cmd_Data = cmd;
            cmd_len = sizeof(RawControllsPackett_s) + 1;
            has_cmd = true;
            // uint8_t R2 = 0;
            // int16_t rx = 0, ry = 0, lx = 0;
            // if (PS4.R2())
            //     R2 = PS4.R2Value(); // 0 - 255
            // val = PS4.LStickX();
            // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
            //     lx = val;
            // val = PS4.RStickX();
            // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
            //     rx = val;
            // val = PS4.RStickY();
            // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
            //     ry = val;
            // if (PS4.Cross())
            // {
            //     Flying = false;
            //     type = stopType;
            // }
            // if (PS4.Right())
            //     type = hoverType;
            // // if (PS4.Down())
            // // if (PS4.Up())
            // if (PS4.Left())
            //     type = altHoldType;

            // uint8_t len = 0;
            // switch (type)
            // {
            // case hoverType:
            //     len = hoverPacket_Encode_Min(rx, -ry, lx, R2, cmd.data);
            //     break;
            // case altHoldType:
            //     len = altHoldPacket_Encode_Min((rx * 3), (-ry * 3), (lx * 3), R2, cmd.data);
            //     struct altHoldPacket_s values;
            //     altHoldPacket_Decode_Min(&values, cmd.data);
            //     // log_v("r%f,p%f,y%f,v%f", values.roll, values.pitch, values.yawrate, values.zVelocity);
            //     break;
            // default:
            //     len = 1;
            //     cmd.data[0] = stopType;
            //     break;
            // }

            // if (PS4.R2())
            // {
            //     if (!Flying)
            //         type = altHoldType; // defaultcmd type and take off type
            //     Flying = true;
            // }

            // log_v("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            //       cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5], cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9]);

            // log_v("%d,%d,   %d,%d,   %d,%d",
            //       rx, (rx * 3), ry, (-ry * 3), lx, (lx * 3));

            // SendDataToDrone(cmd, len);

            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(PacketReadySendSem, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

char packetBuffer[255];
void loop()
{
    while (true)
    {
        // int packetSize = udp.parsePacket();
        // if (packetSize)
        // {
        //     int len = udp.read(packetBuffer, 255);
        //     if (len > 0)
        //     {
        //         packetBuffer[len] = 0;
        //     }
        //     float VBat = 0;
        //     uint32_t batteryLowTime = 0;

        //     // cprt packet adds one extra byte for header
        //     memcpy(&VBat, &packetBuffer[3], sizeof(VBat));
        //     memcpy(&batteryLowTime, &packetBuffer[7], sizeof(batteryLowTime));

        //     // char *buf;
        //     // buf = (char *)malloc(300); // 3 * 64  with som extra
        //     // for (size_t i = 0; i < len; i++)
        //     // {
        //     //     sprintf(buf + i * 3, "%02X,", packetBuffer[i]);
        //     // }
        //     // free(buf);

        //     Serial.printf("VBat: %f    batteryLowTime: %d", VBat, batteryLowTime);
        //     Serial.println();
        //     // Serial.write(buf);
        //     // Serial.println();
        // }
        // else
        // {
        //     delay(1);
        // }
        xSemaphoreTake(PacketReadySendSem, 1000);
        if (has_cmd)
        {
            has_cmd = false;
            SendDataToDrone(cmd_Data, cmd_len);
        }
    }
}

const uint16_t maxCnt = 100;
uint16_t respCount = 0;
int16_t RssiCnt[maxCnt] = {0};
int64_t last = 1;
/**
 * cmd: cprt data to send to drone
 * len: length of the cprt data not including
 */
void SendDataToDrone(CRTPPacket cmd, uint8_t len)
{
    len = len + 2; // add one byte to account for cprt header and another for checksum
    cmd.size = len + 3;
    uint8_t *buffer = (uint8_t *)calloc(1, len + 1);
    memcpy(buffer, (const uint8_t *)&cmd, len);
    buffer[len] = calculate_cksum(buffer, len);

    // log_v("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    //       buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9],
    //       buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19],
    //       buffer[20], buffer[21], buffer[22], buffer[23], buffer[24], buffer[25], buffer[26], buffer[27], buffer[28], buffer[29], buffer[30]);
#if (COMMS_MODE == WIFI_COMMS_MODE)
    udp.beginPacket(DroneAddress, UDP_SERVER_PORT);
    udp.write(buffer, len + 1);
    udp.endPacket();
    // AsyncUDPMessage msg;
    // msg.write(buffer, len + 1);
    // udp.broadcastTo(msg, UDP_SERVER_PORT);
#else
    // log_v("%d, %d", buffer[len], len);
    //  Send a message to rf69_server
    rf69.send(buffer, len + 1);

    rf69.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len2 = sizeof(buf);

    if (rf69.waitAvailableTimeout(50))
    {
        // Should be a reply message for us now
        uint8_t lenOut = rf69.recv(buf, len2);
        if (lenOut)
        {

            RssiCnt[respCount] = rf69.lastRssi();
            respCount++;
            if (respCount >= maxCnt)
            {
                respCount = 0;
                int32_t sum = 0;
                for (size_t i = 0; i < maxCnt; i++)
                {
                    sum += RssiCnt[i];
                }
                now = esp_timer_get_time();
                Serial.print("transmit frequency: ");
                Serial.print(maxCnt / ((now - last) / 1000000.0), DEC);
                Serial.print("   with an avg RSSI of ");
                Serial.print(rf69.lastRssi(), DEC);
                Serial.print("   ");
                Serial.println(sum / (float)maxCnt, DEC);
                // log_v("received %d messages in %d with an avg RSSI of %d", , RssiCnt[1], sum / (float)maxCnt);
                last = esp_timer_get_time();
            }
            if (lenOut > 5)
            {
                buf[lenOut + 1] = 0;
                Serial.println((char *)buf);
            }
        }
        else
        {
            Serial.println("recv failed");
        }
    }
    else
    {
        Serial.print(".");
    }
#endif

    free(buffer);
}