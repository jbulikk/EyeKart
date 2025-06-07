#include "ICM20948.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define REG_BANK_SEL  0x7F
#define WHO_AM_I_REG  0x00
#define PWR_MGMT_1    0x06
#define PWR_MGMT_2    0x07
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG  0x14
#define ACCEL_XOUT_H  0x2D
#define GYRO_XOUT_H   0x33
#define TEMP_OUT_H    0x39

static int icm_write_reg(const struct device *i2c_dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write(i2c_dev, buf, 2, ICM20948_ADDR);
}

static int icm_read_reg(const struct device *i2c_dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_write_read(i2c_dev, ICM20948_ADDR, &reg, 1, buf, len);
}

static int icm_read_word(const struct device *i2c_dev, uint8_t reg, int16_t *val) {
    uint8_t data[2];
    int ret = icm_read_reg(i2c_dev, reg, data, 2);
    if (ret == 0) {
        *val = (int16_t)((data[0] << 8) | data[1]);
    }
    return ret;
}

static void icm_select_bank(const struct device *i2c_dev, uint8_t bank) {
    icm_write_reg(i2c_dev, REG_BANK_SEL, bank << 4);
}

static void icm_reset(const struct device *i2c_dev) {
    icm_select_bank(i2c_dev, 0);
    icm_write_reg(i2c_dev, PWR_MGMT_1, 0x80);
    k_msleep(100);
}

static void icm_wake(const struct device *i2c_dev) {
    icm_select_bank(i2c_dev, 0);
    icm_write_reg(i2c_dev, PWR_MGMT_1, 0x01);
    icm_write_reg(i2c_dev, PWR_MGMT_2, 0x00);
}

static void icm_config_sensors(const struct device *i2c_dev) {
    icm_select_bank(i2c_dev, 2);
    icm_write_reg(i2c_dev, GYRO_CONFIG_1, 0x06);
    icm_write_reg(i2c_dev, ACCEL_CONFIG,  0x06);
}

int icm_init(const struct device *i2c_dev) {
    icm_reset(i2c_dev);
    icm_wake(i2c_dev);
    icm_config_sensors(i2c_dev);
    return 0;
}

int icm_read_all_data(const struct device *i2c_dev,
                      int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz,
                      int16_t *temp) {
    icm_select_bank(i2c_dev, 0);
    if (icm_read_word(i2c_dev, ACCEL_XOUT_H + 0, ax) != 0 ||
        icm_read_word(i2c_dev, ACCEL_XOUT_H + 2, ay) != 0 ||
        icm_read_word(i2c_dev, ACCEL_XOUT_H + 4, az) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 0, gx) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 2, gy) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 4, gz) != 0 ||
        icm_read_word(i2c_dev, TEMP_OUT_H, temp) != 0) {
        return -EIO;
    }
    return 0;
}

void icm_read_whoami(const struct device *i2c_dev) {
    uint8_t reg_addr = WHO_AM_I_REG;
    uint8_t who_am_i = 0;
    int ret = i2c_write_read(i2c_dev, ICM20948_ADDR, &reg_addr, 1, &who_am_i, 1);
    if (ret != 0) {
        printk("WHO_AM_I read error: %d\n", ret);
    } else {
        printk("WHO_AM_I: 0x%02X\n", who_am_i);
    }
}