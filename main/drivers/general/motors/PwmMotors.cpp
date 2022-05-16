#include "drivers/motors/PwmMotors.h"
#include "driver/ledc.h"
#define MOTORS_PWM_BITS 8

bool PWMSpeedControll::isTimerInit = false;

PWMSpeedControll PWMSpeedControll::PWMControll(gpio_num_t gpio, ledc_channel_t channel)
{
    PWMSpeedControll ctrl = PWMSpeedControll();
    ctrl.Ledc_Config.gpio_num = gpio;
    ctrl.Ledc_Config.channel = channel;
    return ctrl;
}

PWMSpeedControll PWMSpeedControll::PWMControll(uint8_t pin, uint8_t channel)
{
    PWMSpeedControll ctrl = PWMSpeedControll();
    ctrl.Ledc_Config.gpio_num = pin;
    ctrl.Ledc_Config.channel = (ledc_channel_t)channel;
    return ctrl;
}

PWMSpeedControll::PWMSpeedControll()
{
    Ledc_Config.duty = 0;
    Ledc_Config.speed_mode = LEDC_LOW_SPEED_MODE;
    Ledc_Config.timer_sel = LEDC_TIMER_0;
}

PWMSpeedControll::~PWMSpeedControll()
{
    ledc_stop(Ledc_Config.speed_mode, Ledc_Config.channel, 0);
}

bool PWMSpeedControll::begin()
{
    ledc_channel_config(&Ledc_Config);
    if (PWMSpeedControll::isTimerInit)
    {
        // First to init will configure it
        return true;
    }

    /*
     * Prepare and set configuration of timers
     * that will be used by MOTORS Controller
     */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,                    // timer mode
        .duty_resolution = (ledc_timer_bit_t)MOTORS_PWM_BITS, // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,                            // timer index
        .freq_hz = 15000,                                     // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK,                             // Auto select the source clock
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK)
    {
        PWMSpeedControll::isTimerInit = true;
        return true;
    }

    return false;
}

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void PWMSpeedControll::SetRatio(uint16_t throttle_value)
{
    ledc_set_duty(Ledc_Config.speed_mode, Ledc_Config.channel, throttle_value);
    ledc_update_duty(Ledc_Config.speed_mode, Ledc_Config.channel);
}

