
#include <stdint.h>
#include <stdbool.h>

class PosContMot
{
public:
    static PosContMot PosContMot(gpio_num_t gpio, ledc_channel_t channel);
    static PosContMot PosContMot(uint8_t pin, uint8_t channel);
    ~PosContMot();

    bool begin();
    void SetPos(uint16_t throttle_value);

    static bool isTimerInit;

private:
    PosContMot();
    ledc_channel_config_t Ledc_Config;
    // static bool pwm_timmer_init();
};