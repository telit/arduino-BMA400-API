/**
 * Copyright (C) 2021 Telit Communications S.p.A Italy. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Arduino.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */
#include <bma400.h>
#ifdef __cplusplus
}
#endif

/*! Read write length varies based on user requirement */
#define READ_WRITE_LENGTH  UINT8_C(46)

/* Variable to store the device address */
static uint8_t s_dev_addr;

/*!
 * @brief I2C read function map to Arduino platform
 */
BMA400_INTF_RET_TYPE bma400_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();

    Wire.requestFrom((int16_t)dev_addr, len);
    while (Wire.available()) {
        for (uint16_t i = 0; i < len; i ++) {
            reg_data[i] = Wire.read();
        }
    }
    return BMA400_INTF_RET_SUCCESS;
}

/*!
 * @brief I2C write function map to Arduino platform
 */
BMA400_INTF_RET_TYPE bma400_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (uint16_t i = 0; i < len; i++ ) {
      Wire.write(reg_data[i]);
    }
    Wire.endTransmission();
    return BMA400_INTF_RET_SUCCESS;
}

/*!
 * @brief SPI read function map to Arduino platform
 */
BMA400_INTF_RET_TYPE bma400_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return BMA400_E_COM_FAIL;
}

/*!
 * @brief SPI write function map to Arduino platform
 */
BMA400_INTF_RET_TYPE bma400_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return BMA400_E_COM_FAIL;
}

/*!
 * @brief Delay function map to Arduino platform
 */
void bma400_delay_us(uint32_t period, void *intf_ptr)
{
  delayMicroseconds(period);
}

void bma400_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:

            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.print("] : Null pointer\r\n");
            break;
        case BMA400_E_COM_FAIL:
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.print("] : Communication failure\r\n");
            break;
        case BMA400_E_INVALID_CONFIG:
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.print("] : Invalid configuration\r\n");
            break;
        case BMA400_E_DEV_NOT_FOUND:
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.print("] : Device not found\r\n");
            break;
        default:
            Serial.print("Error [");
            Serial.print(rslt);
            Serial.print("] : Unknown error code\r\n");
            break;
    }
}

int8_t bma400_interface_init(struct bma400_dev *bma400, uint8_t intf)
{
    int8_t rslt = BMA400_OK;

    if (bma400 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMA400_I2C_INTF)
        {
            Wire.begin();
            s_dev_addr = BMA400_I2C_ADDRESS_SDO_LOW;
            bma400->read = bma400_i2c_read;
            bma400->write = bma400_i2c_write;
            bma400->intf = BMA400_I2C_INTF;
        }
        /* Bus configuration : SPI */
        else if (intf == BMA400_SPI_INTF)
        {
            rslt = BMA400_E_NULL_PTR;
        }

        bma400->intf_ptr = &s_dev_addr;
        bma400->delay_us = bma400_delay_us;
        bma400->read_write_len = READ_WRITE_LENGTH;

        delay(200);
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }
    return rslt;
}
