#include <Arduino.h>
#include "../Submodules/PS4-esp32/src/PS4Controller.h"
#include <WiFi.h>
#include <../Common/Data_type.h>
//#include <atomic>
#include "esp32-hal-log.h"

#define WIFIssid "ESP-DRONE_BCDDC2D254AD"
#define WIFIpassword "12345678"
#define UDP_SERVER_PORT 2390
#define UDP_RECEIVE_PORT 2390

#define WIFI_TRANSMIT_RATE_Us 40000 // 40ms = 25hz

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


*/
IPAddress DroneAddress = IPAddress();
WiFiUDP udp = WiFiUDP();
// AsyncUDP udp;

int64_t now = esp_timer_get_time();
int64_t NextAvalableTransmit = esp_timer_get_time() + WIFI_TRANSMIT_RATE_Us;
// std::atomic<bool> inTransmit(false);
bool inTransmit = false;
bool Flying = false;
uint8_t type = stopType;

void handleControlUpdate();
void SendDataToDrone(CRTPPacket cmd, uint8_t len);

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

    DroneAddress.fromString("192.168.43.42");
    WiFi.begin(WIFIssid, WIFIpassword);
    // WiFi.setTxPower(WIFI_POWER_19_5dBm);
    udp.begin(UDP_SERVER_PORT);

    WiFi.onEvent((WiFiEventSysCb)handleWifiConnected, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent((WiFiEventSysCb)handleWifiDropped, SYSTEM_EVENT_STA_DISCONNECTED);

    log_v("Verbose");
    log_d("Debug");
    log_i("Info");
    log_w("Warning");
    log_e("Error");
    now = esp_timer_get_time();
    map();
}

void handleControlUpdate()
{
    int8_t val = 0;
    // Below has all accessible outputs from the controller
    if (PS4.isConnected())
    {
        now = esp_timer_get_time();
        if (PS4.Right())
            Serial.println("Right Button");
        if (PS4.Down())
            Serial.println("Down Button");
        if (PS4.Up())
            Serial.println("Up Button");
        if (PS4.Left())
            Serial.println("Left Button");

        if (PS4.Square())
            Serial.println("Square Button");
        if (PS4.Cross())
            Serial.println("Cross Button");
        if (PS4.Circle())
            Serial.println("Circle Button");
        if (PS4.Triangle())
            Serial.println("Triangle Button");

        if (PS4.UpRight())
            Serial.println("Up Right");
        if (PS4.DownRight())
            Serial.println("Down Right");
        if (PS4.UpLeft())
            Serial.println("Up Left");
        if (PS4.DownLeft())
            Serial.println("Down Left");

        if (PS4.L1())
            Serial.println("L1 Button");
        if (PS4.R1())
            Serial.println("R1 Button");

        if (PS4.Share())
            Serial.println("Share Button");
        if (PS4.Options())
            Serial.println("Options Button");
        if (PS4.L3())
            Serial.println("L3 Button");
        if (PS4.R3())
            Serial.println("R3 Button");

        if (PS4.PSButton())
            Serial.println("PS Button");
        if (PS4.Touchpad())
            Serial.println("Touch Pad Button");

        // if (PS4.L2())
        //     Serial.printf("L2 button at %d\n", PS4.L2Value());
        // if (PS4.R2())
        //     Serial.printf("R2 button at %d\n", PS4.R2Value());

        // val = PS4.LStickX();
        // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
        //     Serial.printf("Left Stick x at %d\n", val);
        // val = PS4.LStickY();
        // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
        //     Serial.printf("Left Stick y at %d\n", val);
        // val = PS4.RStickX();
        // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
        //     Serial.printf("Right Stick x at %d\n", val);
        // val = PS4.RStickY();
        // if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
        //     Serial.printf("Right Stick y at %d\n", val);
        // ps4_sensor_t sense = PS4.SensorData();
        // Serial.printf("% 06.3f: % 06.3f: % 06.3f    %05d  %05d  %05d\n", sense.accelerometer.x / 8192.0, sense.accelerometer.y / 8192.0, sense.accelerometer.z / 8192.0, sense.gyroscope.x, sense.gyroscope.y, sense.gyroscope.z);

        if (PS4.Charging())
            Serial.println("The controller is charging");
        if (PS4.Audio())
            Serial.println("The controller has headphones attached");
        if (PS4.Mic())
            Serial.println("The controller has a mic attached");

        // Serial.printf("Battery Level : %d\n", PS4.Battery());
        if (now > NextAvalableTransmit && !inTransmit)
        {
            inTransmit = true; // atomic block around buffer and udp operations
            NextAvalableTransmit = now + WIFI_TRANSMIT_RATE_Us;

            uint8_t R2 = 0;
            int16_t rx = 0, ry = 0, lx = 0;
            if (PS4.R2())
                R2 = PS4.R2Value(); // 0 - 255
            val = PS4.LStickX();
            if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
                lx = val;
            val = PS4.RStickX();
            if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
                rx = val;
            val = PS4.RStickY();
            if (val > MIN_JOYCON_ANGLE || val < -MIN_JOYCON_ANGLE)
                ry = val;
            if (PS4.Cross())
            {
                Flying = false;
                type = stopType;
            }
            if (PS4.Right())
                type = hoverType;
            // if (PS4.Down())
            // if (PS4.Up())
            if (PS4.Left())
                type = altHoldType;

            CRTPPacket cmd;
            memset(&cmd, 0, sizeof(CRTPPacket));
            cmd.channel = SET_SETPOINT_CHANNEL;
            cmd.port = CRTP_PORT_SETPOINT_GENERIC;

            uint8_t len = 0;
            switch (type)
            {
            case hoverType:
                len = hoverPacket_Encode_Min(rx, -ry, lx, R2, cmd.data);
                break;
            case altHoldType:
                len = altHoldPacket_Encode_Min((rx * 3), (-ry * 3), (lx * 3), R2, cmd.data);
                struct altHoldPacket_s values;
                altHoldPacket_Decode_Min(&values, cmd.data);
                log_v("r%f,p%f,y%f,v%f", values.roll, values.pitch, values.yawrate, values.zVelocity);
                break;
            default:
                len = 1;
                cmd.data[0] = stopType;
                break;
            }

            if (PS4.R2())
            {
                if (!Flying)
                    type = altHoldType; // defaultcmd type and take off type
                Flying = true;
            }

            // log_v("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            //       cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4], cmd.data[5], cmd.data[6], cmd.data[7], cmd.data[8], cmd.data[9]);

            // log_v("%d,%d,   %d,%d,   %d,%d",
            //       rx, (rx * 3), ry, (-ry * 3), lx, (lx * 3));

            SendDataToDrone(cmd, len);
            inTransmit = false;
        }
    }
}

char packetBuffer[255];
void loop()
{
    while (true)
    {
        int packetSize = udp.parsePacket();
        if (packetSize)
        {
            int len = udp.read(packetBuffer, 255);
            if (len > 0)
            {
                packetBuffer[len] = 0;
            }
            float VBat = 0;
            uint32_t batteryLowTime = 0;

            // cprt packet adds one extra byte for header
            memcpy(&VBat, &packetBuffer[3], sizeof(VBat));
            memcpy(&batteryLowTime, &packetBuffer[7], sizeof(batteryLowTime));

            // char *buf;
            // buf = (char *)malloc(300); // 3 * 64  with som extra
            // for (size_t i = 0; i < len; i++)
            // {
            //     sprintf(buf + i * 3, "%02X,", packetBuffer[i]);
            // }
            // free(buf);

            Serial.printf("VBat: %f    batteryLowTime: %d", VBat, batteryLowTime);
            Serial.println();
            // Serial.write(buf);
            // Serial.println();
        }
        else
        {
            delay(50);
        }
    }
}

/**
 * cmd: cprt data to send to drone
 * len: length of the cprt data not including
 */
void SendDataToDrone(CRTPPacket cmd, uint8_t len)
{
    len = len + 2; // add one byte to account for cprt header and another for checksum
    uint8_t *buffer = (uint8_t *)calloc(1, len + 1);
    memcpy(buffer, (const uint8_t *)&cmd, len);
    buffer[len] = calculate_cksum(buffer, len);

    // log_v("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    //       buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9],
    //       buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19],
    //       buffer[20], buffer[21], buffer[22], buffer[23], buffer[24], buffer[25], buffer[26], buffer[27], buffer[28], buffer[29], buffer[30]);

    udp.beginPacket(DroneAddress, UDP_SERVER_PORT);
    udp.write(buffer, len + 1);
    udp.endPacket();
    // AsyncUDPMessage msg;
    // msg.write(buffer, len + 1);
    // udp.broadcastTo(msg, UDP_SERVER_PORT);

    free(buffer);
}