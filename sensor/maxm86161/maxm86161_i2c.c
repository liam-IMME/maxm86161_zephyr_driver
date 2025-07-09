/* maxm86161_i2c.c
 *
 * Zephyr I²C transport layer for the MAXM86161 PPG sensor.
 * Now uses device instance passed from driver.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include "maxm86161.h"

/* Global device pointer - set during driver initialization */
static const struct device *maxm86161_dev = NULL;

void maxm86161_set_device(const struct device *dev)
{
    maxm86161_dev = dev;
}

static const struct i2c_dt_spec *get_i2c_spec(void)
{
    if (!maxm86161_dev) {
        return NULL;
    }
    const struct maxm86161_config *cfg = maxm86161_dev->config;
    return &cfg->i2c;
}

/* Write one byte to a register */
int32_t maxm86161_i2c_write_to_register(uint8_t reg, uint8_t val)
{
    const struct i2c_dt_spec *spec = get_i2c_spec();
    if (!spec || !device_is_ready(spec->bus)) {
        return -ENODEV;
    }
    return i2c_reg_write_byte_dt(spec, reg, val);
}

/* Read one byte from a register */
int32_t maxm86161_i2c_read_from_register(uint8_t reg)
{
    uint8_t val;
    int ret;
    const struct i2c_dt_spec *spec = get_i2c_spec();

    if (!spec || !device_is_ready(spec->bus)) {
        return -ENODEV;
    }
    ret = i2c_reg_read_byte_dt(spec, reg, &val);
    return ret < 0 ? ret : val;
}

/* Burst-write N bytes starting at register "reg" */
int32_t maxm86161_i2c_block_write(uint8_t reg, uint8_t len, uint8_t const *buf)
{
    const struct i2c_dt_spec *spec = get_i2c_spec();
    if (!spec || !device_is_ready(spec->bus)) {
        return -ENODEV;
    }
    return i2c_burst_write_dt(spec, reg, buf, len);
}

/* Burst-read N bytes starting at register "reg" */
int32_t maxm86161_i2c_block_read(uint8_t reg, uint16_t len, uint8_t *buf)
{
    const struct i2c_dt_spec *spec = get_i2c_spec();
    if (!spec || !device_is_ready(spec->bus)) {
        return -ENODEV;
    }
    return i2c_burst_read_dt(spec, reg, buf, len);
}
