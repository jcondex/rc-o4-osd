#pragma once

#include <stdint.h>
#include "hardware/i2c.h"

inline bool i2c_write_reg_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t value, uint32_t timeout_us = 3000) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(i2c, addr, buf, 2, false, timeout_us) == 2;
}

inline bool i2c_read_regs(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_us = 3000) {
    int wr = i2c_write_timeout_us(i2c, addr, &reg, 1, true, timeout_us);
    if (wr != 1) return false;
    return i2c_read_timeout_us(i2c, addr, buf, len, false, timeout_us) == int(len);
}
