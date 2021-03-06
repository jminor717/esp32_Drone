// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
// 20, 24, 28, 29, 30, 31 are not exposed on the qfn package
// 0, 2, 5, 12, 15, 14 can affect boot sequence if used improperly (Strapping Pins)
// 6, 7, 8, 9, 10, 11 are connected to internal spi flash
// 32, 33 are connected to intrenal crystal and cannot be used
// 34, 35, 36, 39 are input only
// 1, 3, 5, 14, 15 output pwm durring boot/reboot

/**  https://www.esp32.com/viewtopic.php?t=5840
 * There are three "power domains" on the chip, i.e. three separate power input sections.
 * Domain VDD3P3_CPU in on pin 37 (NOT GPIO 37) supplies power to GPIO5; GPIO18; GPIO23; GPIO19; GPIO22; GPIO3; GPIO1; and GPIO21.
 * This domain can SOURCE up to 40mA and SINK up to 28mA on STRENGTH 3
 *
 * Domain VDD3P3_RTC in on pin 19 (NOT GPIO 19) supplies power to GPIO36; GPIO37; GPIO38; GPIO39 ; GPIO34; GPIO35; GPIO32; GPIO33; GPIO25; GPIO26; GPIO27;GPIO14; GPIO12; GPIO13; GPIO15; GPIO2; GPIO0; and GPIO4.
 * Remember GPIO34..GPIO39 are inputs only, so cannot be used to source or sink current meaningfully.
 * This domain can SOURCE 40mA and SINK up to 28mA on STRENGTH 3
 *
 * Domain VDD_SDIO in on pin 26 (NOT GPIO 26) is a special bidirectional power pin which can be driven by 3.3V externally, route VDD3P3_RTC via an internal network, or supply 1.8V via a LDO regulator.
 * The LDO regulator can only supply 40mA. If using power through VDD3P3_RTC, the internal network will cause a voltage drop; the higher the current drawn, the more voltage will be lost in the chip and more heat generated.
 * Domain VDD_SDIO supplies power to GPIO16; GPIO17; GPIO9; GPIO10; GPIO11; GPIO6; GPIO7; and GPIO8
 * This domain can SOURCE up to 20mA and SINK up to 28mA on STRENGTH 3 (Remember the 40mA total limit at 1.8V)
 *
 * The sum of all the I/O current may not exceed 1200mA
 *
 * And as a bonus hard-to-find figure, the internal pull-up and pull-down resistors are 45kOhms each.
 */

/* esp32 s3
    strapping pins 0, 45, 46, 3
    Flash pins 4, 5, 6, 7 // Wroom-2, free on Wroom-1
free
    15, 16, 17, 18, 8, (19, 20 USB JTag)
    46, 9, 10, 11, 12, 13, 14, 21, 47, 48, 45
    38, 39, 40, 41, 42, 2, 1
total  25 - 2
*/
#include "config/dev_config.h"
#include "hal/spi_types.h"

// battery voltage monitor
#define CONFIG_VBat_PIN 8
#define VBat_VOLTAGE_DIVIDER_RATIO 1.193 // 0.838 //measured

#ifdef V2_BOARD

// Servos
#define M1A_PIN 46
#define M1B_PIN 9
#define M1_POSITION_PIN 10

#define M2A_PIN 21
#define M2B_PIN 47
#define M2_POSITION_PIN 4

#define M3A_PIN 36
#define M3B_PIN 37
#define M3_POSITION_PIN 2

#define M4A_PIN 41
#define M4B_PIN 42
#define M4_POSITION_PIN 1

#define CONFIG_MOTOR01_PIN 5
#define CONFIG_MOTOR02_PIN 6
#define CONFIG_MOTOR03_PIN 7
#define CONFIG_MOTOR04_PIN 15

// SPI
#define SPI_DEV_SCK 13
#define SPI_DEV_MISO 11
#define SPI_DEV_MOSI 12

// COM-13909
#define COM13909_SS 14
#define COM13909_INT0 18

// sensor pins, i2c shared with vl53l1x
#define CONFIG_MPU_PIN_INT 38
#define CONFIG_I2C0_PIN_SCL 40
#define CONFIG_I2C0_PIN_SDA 39

#ifndef SENSOR_AND_DECK_ON_ONE_BUS
    //  vl53l1x
    #define CONFIG_I2C1_PIN_SCL 23
    #define CONFIG_I2C1_PIN_SDA 22
#endif

// GPS
#define GPS_RX 19
#define GPS_TX 20

#define CONFIG_LED_DOUT 48
#define EXTRA_PWM_OUT 35
#endif

#define CONFIG_PITCH_CALIB 0
#define CONFIG_ROLL_CALIB 0

#define BUSS_1_HOST HSPI_HOST
#define DMA_CHAN 2

//

//

//

// V1
#ifdef V1_BOARD
#define CONFIG_DCDC_PGOOD_PIN 35

// motor outputs
#define CONFIG_MOTOR01_PIN 13
#define CONFIG_MOTOR02_PIN 4
#define CONFIG_MOTOR03_PIN 12
#define CONFIG_MOTOR04_PIN 27

// sensor pins
#define CONFIG_MPU_PIN_INT 18
#define CONFIG_I2C0_PIN_SCL 21
#define CONFIG_I2C0_PIN_SDA 19
//#define CONFIG_MPU_Fsync 22
//#define CONFIG_MPU_DRDY 23

// vl53l1x
#define CONFIG_I2C1_PIN_SCL 26
#define CONFIG_I2C1_PIN_SDA 25
//#define CONFIG_VL53_GPIO1 26
//#define CONFIG_VL53_XSHUT 25

// misc
#define CONFIG_LED_DOUT 15
#endif

//

//

//

//

#ifdef DontDefMeBro
#define M4_POSITION_PIN 1
#define M3_POSITION_PIN 2
// 43 TX
// 44 RX
#define M4B_PIN 42
#define M4A_PIN 41
#define CONFIG_I2C0_PIN_SCL 40
#define CONFIG_I2C0_PIN_SDA 39
#define CONFIG_MPU_PIN_INT 38
#define M3B_PIN 37
#define M3A_PIN 36
#define EXTRA_PWM_OUT 35
// 0 Strapping
// 45 Strapping
#define CONFIG_LED_DOUT 48
#define M2B_PIN 47
#define M2A_PIN 21
// 20 USB GPS_RX 19
// 19 USB GPS_TX 20

//

#define M1_POSITION_PIN 10
#define CONFIG_MOTOR01_PIN 5
#define CONFIG_MOTOR02_PIN 6
#define CONFIG_MOTOR03_PIN 7
#define CONFIG_MOTOR04_PIN 15
// 16
// 17
#define COM13909_INT0 18
#define CONFIG_VBat_PIN 8
// 3 Strapping
#define M1A_PIN 46 // 46 Strapping  only used to determine logging level during boot
#define M1B_PIN 9
#define M2_POSITION_PIN 10
#define SPI_DEV_MISO 11
#define SPI_DEV_MOSI 12
#define SPI_DEV_SCK 13
#define COM13909_SS 14
#endif