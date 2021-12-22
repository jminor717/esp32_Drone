
#include <stdint.h>

// 16 bit spi buffer
enum registers
{
    // output
    motor1RPM_a,
    motor1RPM_b,
    motor2RPM_a,
    motor2RPM_b,
    motor3RPM_a,
    motor3RPM_b,
    motor4RPM_a,
    motor4RPM_b,
    servo1Angle_a,
    servo1Angle_b,
    servo2Angle_a,
    servo2Angle_b,
    servo3Angle_a,
    servo3Angle_b,
    servo4Angle_a,
    servo4Angle_b,
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