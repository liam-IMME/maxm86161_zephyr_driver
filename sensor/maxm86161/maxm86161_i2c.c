/* maxm86161_i2c.c
 *
 * Zephyr I²C transport layer for the MAXM86161 PPG sensor.
 * Pulls bus info from DT_INST(0, maxim_maxm86161).
 */

#include <zephyr/device.h>
//#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include "maxm86161.h"

/* Instantiate the I²C spec from devicetree (instance 0) */
static const struct i2c_dt_spec maxm86161_i2c = I2C_DT_SPEC_INST_GET(0);

/* Write one byte to a register */
int32_t maxm86161_i2c_write_to_register(uint8_t reg, uint8_t val)
{
    if (!device_is_ready(maxm86161_i2c.bus)) {
        return -ENODEV;
    }
    return i2c_reg_write_byte_dt(&maxm86161_i2c, reg, val);
}

/* Read one byte from a register */
int32_t maxm86161_i2c_read_from_register(uint8_t reg)
{
    uint8_t val;
    int ret;

    if (!device_is_ready(maxm86161_i2c.bus)) {
        return -ENODEV;
    }
    ret = i2c_reg_read_byte_dt(&maxm86161_i2c, reg, &val);
    return ret < 0 ? ret : val;
}

/* Burst-write N bytes starting at register “reg” */
int32_t maxm86161_i2c_block_write(uint8_t reg, uint8_t len, uint8_t const *buf)
{
    if (!device_is_ready(maxm86161_i2c.bus)) {
        return -ENODEV;
    }
    return i2c_burst_write_dt(&maxm86161_i2c, reg, buf, len);
}

/* Burst-read N bytes starting at register “reg” */
int32_t maxm86161_i2c_block_read(uint8_t reg, uint16_t len, uint8_t *buf)
{
    if (!device_is_ready(maxm86161_i2c.bus)) {
        return -ENODEV;
    }
    return i2c_burst_read_dt(&maxm86161_i2c, reg, buf, len);
}
