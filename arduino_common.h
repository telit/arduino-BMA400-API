/**
 * Copyright (C) 2021 Telit Communications S.p.A Italy. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __COMMON_BMA400_H__
#define __COMMON_BMA400_H__

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <bma400.h>
#ifdef __cplusplus
}
#endif

int8_t bma400_interface_init(struct bma400_dev *bme, uint8_t intf);

int8_t bma400_read_i2c(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

int8_t bma400_write_i2c(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);

int8_t bma400_read_spi(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

int8_t bma400_write_spi(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

void bma400_delay_ms(uint32_t period_ms);

void bma400_check_rslt(const char api_name[], int8_t rslt);

#endif /*__COMMON_BMA400_H__*/