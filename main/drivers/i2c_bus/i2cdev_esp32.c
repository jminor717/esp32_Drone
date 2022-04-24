/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * i2cdev.c - Functions to write to I2C devices
 */
#define DEBUG_MODULE "I2CDEV"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "stm32_legacy.h"
#include "i2cdev.h"
#include "i2c_drv.h"
#include "config/nvicconf.h"
#include "debug_cf.h"

// TODO rework to use static i2c_cmd_handle_t that avoids using i2c_cmd_link_delete possibly also look at polling to improve reliability and crashes from ISRs
//maybe also reduce use of mutexes
/**
 0x4037e58b: vPortClearInterruptMaskFromISR at C:/Users/jacob/esp/esp-idf/components/freertos/port/xtensa/include/freertos/portmacro.h:571
 (inlined by) xQueueGenericSendFromISR at C:/Users/jacob/esp/esp-idf/components/freertos/queue.c:1198

0x40377765: i2c_isr_handler_default at C:/Users/jacob/esp/esp-idf/components/driver/i2c.c:506

0x40376c56: _xt_lowint1 at C:/Users/jacob/esp/esp-idf/components/freertos/port/xtensa/xtensa_vectors.S:1111

0x40384993: multi_heap_internal_lock at C:/Users/jacob/esp/esp-idf/components/heap/multi_heap.c:152
 (inlined by) multi_heap_free_impl at C:/Users/jacob/esp/esp-idf/components/heap/multi_heap.c:218

0x40375e9e: heap_caps_free at C:/Users/jacob/esp/esp-idf/components/heap/heap_caps.c:305

0x40385299: free at C:/Users/jacob/esp/esp-idf/components/newlib/heap.c:39

0x420198f7: i2c_cmd_link_delete at C:/Users/jacob/esp/esp-idf/components/driver/i2c.c:1071

0x42012765: i2cdevReadReg8 at c:\repos\espdrone\build/../main/drivers/i2c_bus/i2cdev_esp32.c:102

0x4200ec67: sensorsTask at c:\repos\espdrone\build/../main/sensors_mpu6050_hm5883L_ms5611.c:307 (discriminator 8)

0x40380d51: vPortTaskWrapper at C:/Users/jacob/esp/esp-idf/components/freertos/port/xtensa/port.c:131

 */
uint8_t StaticCmdbuffer[1000] = {0};

int i2cdevInit(I2C_Dev *dev)
{
    i2cdrvInit(dev);
    return true;
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, I2CDEV_NO_MEM_ADDR, len, data);
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data)
{
    uint8_t byte;
    bool status;

    status = i2cdevReadReg8(dev, devAddress, memAddress, 1, &byte);
    *data = byte & (1 << bitNum);

    return status;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
    }

    return status;
}

bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)50) == pdFALSE) {
        return false;
    }

    memset(StaticCmdbuffer, 0, sizeof(uint8_t) * 1000);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(StaticCmdbuffer, 1000);
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete_static(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Read_ %d bytes from register 0x%X, data: %s", port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }

}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)50) == pdFALSE) {
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Read_ %d bytes from register 0x%X, data: %s", port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }

}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data)
{
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        byte &= ~(mask);                  // zero all important bits in existing byte
        byte |= data;                     // combine data with existing byte
        status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
    }

    return status;
}


bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)50) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Write %d bytes to register 0x%X, data: %s",
                      port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)50) == pdFALSE) {
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);
#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Write %d bytes to register 0x%X, data: %s",
                      port, devAddr, length, regAddr, str);
    }

#endif

#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }
}
