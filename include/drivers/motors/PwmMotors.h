

#include <stdint.h>
#include <stdbool.h>

#include "driver/ledc.h"

class PWMSpeedControll
{
public:
    static PWMSpeedControll PWMControll(gpio_num_t gpio, ledc_channel_t channel);
    static PWMSpeedControll PWMControll(uint8_t pin, uint8_t channel);
    ~PWMSpeedControll();

    bool begin();
    void SetRatio(uint16_t throttle_value);

    static bool isTimerInit;
private:
    PWMSpeedControll();
    ledc_channel_config_t Ledc_Config;
};