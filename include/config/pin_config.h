// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// 20, 24, 28, 29, 30, 31 are not exposed on the qfn package
// 0, 2, 5, 12, 15 can affect boot sequence if used improperly
// 6, 7, 8, 9, 10, 11 are connected to internal spi flash
// 32, 33 are connected to intrenal crystal and cannot be used
// 34, 35, 36, 39 are input only
// 1, 3, 5, 14, 15 output pwm durring boot/reboot, do not tie between boards

#define CONFIG_LED_PIN_BLUE 0
#define CONFIG_LED_PIN_GREEN 1
#define CONFIG_LED_PIN_RED 2

#define CONFIG_MOTOR01_PIN 12
#define CONFIG_MOTOR02_PIN 13
#define CONFIG_MOTOR03_PIN 14
#define CONFIG_MOTOR04_PIN 16

#define CONFIG_MPU_PIN_INT 4

#define CONFIG_ADC1_PIN 35

#define CONFIG_I2C0_PIN_SCL 17
#define CONFIG_I2C0_PIN_SDA 18

#define CONFIG_I2C1_PIN_SCL 19
#define CONFIG_I2C1_PIN_SDA 21

//
#define CONFIG_PITCH_CALIB 0
#define CONFIG_ROLL_CALIB 0