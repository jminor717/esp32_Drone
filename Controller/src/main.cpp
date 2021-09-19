#include <Arduino.h>
#include "../Submodules/PS4-esp32/src/PS4Controller.h"

/*
low level send and receive interface
    wifi -> esp32 wifi, mimic existing drone implamentation
    long range radio -> combine TX and RX loops on both controller and Drone to help minimize likelihood of colission
high level independent command and setpoint transmition lops to allow commands to be sent more frequently than setpoints
    low level will run at the rate of the fastest high level and try to combine high level packets to minimize the time that TX is active
    

*/



void handleControlUpdate();

void setup()
{
    Serial.begin(115200);
    //char mac[] = "60:5b:b4:d9:7b:14";
    char mac[] = "03:03:03:03:03:03";
    PS4.begin(mac);
    Serial.println("Ready.");
    Serial.println(mac);
    PS4.attach(&handleControlUpdate);
}

void handleControlUpdate()
{
    int8_t val = 0;
    // Below has all accessible outputs from the controller
    if (PS4.isConnected())
    {
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
        if (PS4.L2())
        {
            Serial.printf("L2 button at %d\n", PS4.L2Value());
        }
        if (PS4.R2())
        {
            Serial.printf("R2 button at %d\n", PS4.R2Value());
        }
        val = PS4.LStickX();
        if (val > 10 || val < -10)
        {
            Serial.printf("Left Stick x at %d\n", val);
        }
        val = PS4.LStickY();
        if (val > 10 || val < -10)
        {
            Serial.printf("Left Stick y at %d\n", val);
        }
        val = PS4.RStickX();
        if (val > 10 || val < -10)
        {
            Serial.printf("Right Stick x at %d\n", val);
        }
        val = PS4.RStickY();
        if (val > 10 || val < -10)
        {
            Serial.printf("Right Stick y at %d\n", val);
        }
        // ps4_sensor_t sense = PS4.SensorData();
        //Serial.printf("% 06.3f: % 06.3f: % 06.3f    %05d  %05d  %05d\n", sense.accelerometer.x / 8192.0, sense.accelerometer.y / 8192.0, sense.accelerometer.z / 8192.0, sense.gyroscope.x, sense.gyroscope.y, sense.gyroscope.z);

        if (PS4.Charging())
            Serial.println("The controller is charging");
        if (PS4.Audio())
            Serial.println("The controller has headphones attached");
        if (PS4.Mic())
            Serial.println("The controller has a mic attached");

        //Serial.printf("Battery Level : %d\n", PS4.Battery());
    }
}

void loop()
{
    while (true)
    {
        delay(portMAX_DELAY);
    }
}
