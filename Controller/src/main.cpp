#include <Arduino.h>
#include "../Submodules/PS4-esp32/src/PS4Controller.h"
#include <WiFi.h>
//#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include <../Common/Data_type.h>
#include <atomic>
#include "esp32-hal-log.h"

#define WIFIssid "ESP-DRONE_BCDDC2D254AD"
#define WIFIpassword "12345678"
#define UDP_SERVER_PORT 2390
#define UDP_RECEIVE_PORT 2391

#define WIFI_TRANSMIT_RATE_Us 40000

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
//WiFiUDP udp = WiFiUDP();
AsyncUDP udp2;

int64_t now = esp_timer_get_time();
int64_t NextAvalableTransmit = esp_timer_get_time() + WIFI_TRANSMIT_RATE_Us;
std::atomic<bool> inTransmit(false);

void handleControlUpdate();

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    //"60:5b:b4:d9:7b:14"; 58:a0:23:dd:a1:84  4C:11:AE:DF:8B:F4  03:03:03:03:03:03
    //char mac[] = "60:5b:b4:d9:7b:14";
    char mac[] = "4C:11:AE:DF:8B:F4";
    PS4.begin(mac);
    Serial.println("Ready.");
    Serial.println(mac);
    PS4.attach(&handleControlUpdate);

    DroneAddress.fromString("192.168.43.42");
    WiFi.begin(WIFIssid, WIFIpassword);
    //WiFi.setTxPower(WIFI_POWER_19_5dBm);
    // udp.begin(UDP_SERVER_PORT);

    if (udp2.listen(UDP_RECEIVE_PORT))
    {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp2.onPacket([](AsyncUDPPacket packet)
                      {
                          Serial.print("UDP Packet Type: ");
                          Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                                                 : "Unicast");
                          Serial.print(", From: ");
                          Serial.print(packet.remoteIP());
                          Serial.print(":");
                          Serial.print(packet.remotePort());
                          Serial.print(", To: ");
                          Serial.print(packet.localIP());
                          Serial.print(":");
                          Serial.print(packet.localPort());
                          Serial.print(", Length: ");
                          Serial.print(packet.length());
                          Serial.print(", Data: ");
                          Serial.write(packet.data(), packet.length());
                          Serial.println();
                          //reply to the client
                          //packet.printf("Got %u bytes of data", packet.length());
                      });
    }

    log_v("Verbose");
    log_d("Debug");
    log_i("Info");
    log_w("Warning");
    log_e("Error");
    now = esp_timer_get_time();
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
        //Serial.printf("% 06.3f: % 06.3f: % 06.3f    %05d  %05d  %05d\n", sense.accelerometer.x / 8192.0, sense.accelerometer.y / 8192.0, sense.accelerometer.z / 8192.0, sense.gyroscope.x, sense.gyroscope.y, sense.gyroscope.z);

        if (PS4.Charging())
            Serial.println("The controller is charging");
        if (PS4.Audio())
            Serial.println("The controller has headphones attached");
        if (PS4.Mic())
            Serial.println("The controller has a mic attached");

        //Serial.printf("Battery Level : %d\n", PS4.Battery());
        if (now > NextAvalableTransmit && !inTransmit)
        {
            uint8_t R2 = 0;
            int8_t rx = 0, ry = 0, lx = 0;
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

            inTransmit = true;
            struct altHoldPacket_s values;
            values.roll = rx / 62.5;               //map(rx, -125, 125, -(MIN_DEGREE), MAX_DEGREE) / 1000.0;
            values.pitch = ry / 62.5;              // map(ry, -125, 125, -(MIN_DEGREE), MAX_DEGREE) / 1000.0;
            values.yawrate = lx / 62.5;            // map(lx, -125, 125, -(MIN_DEGREE), MAX_DEGREE) / 1000.0;
            values.zVelocity = (R2 / 510.0) - 0.1; //map(R2, 0, 125, MIN_THRUST, MAX_THRUST);
            // if (values.thrust == MIN_THRUST)
            // {
            //     values.thrust = 0;
            // }
            CRTPPacket cmd;
            memset(&cmd, 0, sizeof(CRTPPacket));
            cmd.channel = SET_SETPOINT_CHANNEL;
            cmd.port = CRTP_PORT_SETPOINT_GENERIC;
            packet_type type = altHoldType;
            if (!PS4.R2())
            {
                type = stopType;
            }
            //memcpy(cmd.data + 1, (const uint8_t *)&values, sizeof(altHoldPacket_s));
            
            (&values, cmd.data, type);

            uint8_t *buffer = (uint8_t *)calloc(1, sizeof(CRTPPacket));
            memcpy(buffer, (const uint8_t *)&cmd, sizeof(CRTPPacket));
            log_v("r%f,p%f,y%f,v%f, c2%d", values.roll, values.pitch, values.yawrate, values.zVelocity, cmd.data[1]);

            // log_v("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            //       buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9],
            //       buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19],
            //       buffer[20], buffer[21], buffer[22], buffer[23], buffer[24], buffer[25], buffer[26], buffer[27], buffer[28], buffer[29], buffer[30]);
            //  udp.beginPacket(DroneAddress, UDP_SERVER_PORT);

            //  udp.write(buffer, sizeof(CRTPPacket));

            //  udp.endPacket();
            AsyncUDPMessage msg;
            msg.write(buffer, sizeof(CRTPPacket));
            udp2.broadcastTo(msg, UDP_SERVER_PORT);

            free(buffer);
            NextAvalableTransmit = now + WIFI_TRANSMIT_RATE_Us;
            inTransmit = false;
        }
    }
}

void loop()
{
    while (true)
    {
        delay(portMAX_DELAY);
    }
}
