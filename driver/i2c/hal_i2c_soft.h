#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include "stdint.h"
#include "stdbool.h"

void hal_i2c_init(void);

bool hal_i2c_msg_write(uint16_t addr, uint8_t cmd, uint8_t *data, uint8_t len);
bool hal_i2c_msg_read(uint16_t addr, uint8_t *data, uint8_t len);

#endif

