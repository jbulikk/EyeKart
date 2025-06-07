#ifndef ICM20948_H
#define ICM20948_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#define ICM20948_ADDR 0x68

int icm_init(const struct device *i2c_dev);
int icm_read_all_data(const struct device *i2c_dev,
                      int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz,
                      int16_t *temp);
void icm_read_whoami(const struct device *i2c_dev);

#endif