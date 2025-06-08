#include "ICM20948.h"
#include <math.h>
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

const static float acc_sensitivity = 4096;
const static float gyro_sensitivity = 16.4;
float sampling_time_sec;
float measurement_time;
float last_measurement_time;
float alpha = 0.998;

double get_time_seconds() {
    printk("%f", (k_uptime_get() / 1000.0f));
    return k_uptime_get() / 1000.0f;
}

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
    icm_write_reg(i2c_dev, ACCEL_CONFIG,  0x02);
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

int icm_read_acc_mag_temp(const struct device *i2c_dev, ImuData *imu) {
    measurement_time = get_time_seconds();
    sampling_time_sec = measurement_time - last_measurement_time;
    last_measurement_time = measurement_time;
    icm_select_bank(i2c_dev, 0);

    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t raw_temp;

    if (icm_read_word(i2c_dev, ACCEL_XOUT_H + 0, &acc_x) != 0 ||
        icm_read_word(i2c_dev, ACCEL_XOUT_H + 2, &acc_y) != 0 ||
        icm_read_word(i2c_dev, ACCEL_XOUT_H + 4, &acc_z) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 0, &gyro_x) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 2, &gyro_y) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 4, &gyro_z) != 0 ||
        icm_read_word(i2c_dev, TEMP_OUT_H, &raw_temp) != 0) 
    {
        return -EIO;
    }

    imu->accelerometer_raw.x = acc_x;
    imu->accelerometer_raw.y = acc_y;
    imu->accelerometer_raw.z = acc_z;

    imu->gyroscope_raw.x = gyro_x;
    imu->gyroscope_raw.y = gyro_y;
    imu->gyroscope_raw.z = gyro_z;

    imu->accelerometer_scaled.x = (float)imu->accelerometer_raw.x / acc_sensitivity;
	imu->accelerometer_scaled.y = (float)imu->accelerometer_raw.y / acc_sensitivity;
	imu->accelerometer_scaled.z = (float)imu->accelerometer_raw.z / acc_sensitivity;

    imu->gyroscope_scaled.x = (float)imu->gyroscope_raw.x / gyro_sensitivity;
    imu->gyroscope_scaled.y = (float)imu->gyroscope_raw.y / gyro_sensitivity;
    imu->gyroscope_scaled.z = (float)imu->gyroscope_raw.z / gyro_sensitivity;

    imu->pitch_acc = atan2(imu->accelerometer_scaled.y, imu->accelerometer_scaled.z) * 180.0/M_PI;
    imu->pitch_gyro = imu->gyroscope_scaled.x;
    imu->pitch_complementary = alpha * (imu->pitch_complementary + imu->pitch_gyro * sampling_time_sec) + (1.0 - alpha) * imu->pitch_acc;
   
    imu->roll_acc = atan2(imu->accelerometer_scaled.x, imu->accelerometer_scaled.z) * 180.0/M_PI;
    imu->roll_gyro = imu->gyroscope_scaled.y;
    imu->roll_complementary = alpha * (imu->roll_complementary + imu->roll_gyro * sampling_time_sec) + (1.0 - alpha) * imu->roll_acc;

    imu->temp = raw_temp;
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