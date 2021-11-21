#include "stdint.h"

typedef enum
{
    user = 5000,
    system = 0
} LastUpdateKeepalive;

typedef struct
{
    uint16_t pin;
    float maxRate;
    LastUpdateKeepalive LastUpdateSource;
    float desiredAngle;
    float currentAngle;
} Servo;

//  create task that will set servo to move at a particular rate so that we allways know the current angle with out active feedback from the servo
//      task will be suspended once the target angle is reached
//      when a new angle is set resume the task
//          may be useful to keep the task alive longer depending on where the task came from as some source may send repeated requests and pausing and resuming the task may be inefficient
//  method to get info for a specific servo