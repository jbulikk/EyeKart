#include "ICM20948.h"
#include "madgwick.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define REG_BANK_SEL 0x7F
#define WHO_AM_I_REG 0x00
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define GYRO_CONFIG_1 0x01
#define ACCEL_CONFIG 0x14
#define ACCEL_XOUT_H 0x2D
#define GYRO_XOUT_H 0x33
#define TEMP_OUT_H 0x39

// Magnetometer (AK09916) registers (ICM20948 pass-through)
#define MAG_I2C_ADDR 0x0C
#define MAG_WIA2 0x01
#define MAG_ST1 0x10
#define MAG_HXL 0x11
#define MAG_ST2 0x18
#define MAG_CNTL2 0x31

const static float acc_sensitivity = 4096;
const static float gyro_sensitivity = 16.4;
static float sampling_time_sec = 0.0f;
static float measurement_time = 0.0f;
static float last_measurement_time = 0.0f;
static float alpha = 0.8f;

// Gyro calibration offsets
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

double get_time_seconds() {
    // printk("%f", (k_uptime_get() / 1000.0f));
    return k_uptime_get() / 1000.0f;
}

static int icm_write_reg(const struct device *i2c_dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
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
    icm_write_reg(i2c_dev, ACCEL_CONFIG, 0x02);
}

// --- Magnetometer read helper ---
static int ak09916_read_mag(const struct device *i2c_dev, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t buf[6];
    uint8_t st1;
    // Read status 1 register to check for new data
    int ret = i2c_write_read(i2c_dev, MAG_I2C_ADDR, (uint8_t[]){MAG_ST1}, 1, &st1, 1);
    if (ret != 0 || !(st1 & 0x01))
        return -EIO;
    // Read 6 bytes of magnetometer data
    ret = i2c_write_read(i2c_dev, MAG_I2C_ADDR, (uint8_t[]){MAG_HXL}, 1, buf, 6);
    if (ret != 0)
        return ret;
    *mx = (int16_t)(buf[1] << 8 | buf[0]);
    *my = (int16_t)(buf[3] << 8 | buf[2]);
    *mz = (int16_t)(buf[5] << 8 | buf[4]);
    return 0;
}

// Magnetometer (AK09916) initialization for ICM20948
static void ak09916_init(const struct device *i2c_dev) {
    // Enable I2C master
    icm_select_bank(i2c_dev, 0);
    icm_write_reg(i2c_dev, 0x03, 0x00); // USER_CTRL: I2C_MST_EN
    icm_write_reg(i2c_dev, 0x0F, 0x02); // BYPASS_EN: Enable bypass mode for AK09916
    // Set I2C master clock
    icm_select_bank(i2c_dev, 3);
    icm_write_reg(i2c_dev, 0x01, 0x07); // I2C_MST_CTRL: 345.6 kHz
    // Set AK09916 to continuous measurement mode 1  AK09916 address (write)
    icm_write_reg(i2c_dev, 0x04, 0x31); // I2C_SLV0_REG: CNTL2
    icm_write_reg(i2c_dev, 0x06, 0x01); // I2C_SLV0_DO: 0x01 (continuous mode)
    icm_write_reg(i2c_dev, 0x05, 0x81); // I2C_SLV0_CTRL: enable, 1 byte
    k_msleep(100);
    // Set up I2C_SLV0 to read 6 bytes from HXL (0x11) of AK09916
    icm_write_reg(i2c_dev, 0x03, 0x8C); // I2C_SLV0_ADDR: AK09916 address (read)
    icm_write_reg(i2c_dev, 0x04, 0x11); // I2C_SLV0_REG: HXL
    icm_write_reg(i2c_dev, 0x05, 0x86); // I2C_SLV0_CTRL: enable, 6 bytes
}

int icm_init(const struct device *i2c_dev) {
    icm_reset(i2c_dev);
    icm_wake(i2c_dev);
    icm_config_sensors(i2c_dev);

    ak09916_init(i2c_dev);
    return 0;
}

int icm_read_all_data(const struct device *i2c_dev,
                      int16_t *ax,
                      int16_t *ay,
                      int16_t *az,
                      int16_t *gx,
                      int16_t *gy,
                      int16_t *gz,
                      int16_t *temp) {
    icm_select_bank(i2c_dev, 0);
    if (icm_read_word(i2c_dev, ACCEL_XOUT_H + 0, ax) != 0 || icm_read_word(i2c_dev, ACCEL_XOUT_H + 2, ay) != 0 ||
        icm_read_word(i2c_dev, ACCEL_XOUT_H + 4, az) != 0 || icm_read_word(i2c_dev, GYRO_XOUT_H + 0, gx) != 0 ||
        icm_read_word(i2c_dev, GYRO_XOUT_H + 2, gy) != 0 || icm_read_word(i2c_dev, GYRO_XOUT_H + 4, gz) != 0 ||
        icm_read_word(i2c_dev, TEMP_OUT_H, temp) != 0) {
        return -EIO;
    }
    return 0;
}

void icm_calibrate_gyro(const struct device *i2c_dev, int samples) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gx, gy, gz;
    printk("Calibrating gyro... keep device still\n");
    for (int i = 0; i < samples; ++i) {
        icm_select_bank(i2c_dev, 0);
        icm_read_word(i2c_dev, GYRO_XOUT_H + 0, &gx);
        icm_read_word(i2c_dev, GYRO_XOUT_H + 2, &gy);
        icm_read_word(i2c_dev, GYRO_XOUT_H + 4, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        k_msleep(2);
    }
    gyro_offset_x = (float)sum_x / samples;
    gyro_offset_y = (float)sum_y / samples;
    gyro_offset_z = (float)sum_z / samples;
    printk(
        "Gyro offsets: x=%.2f, y=%.2f, z=%.2f\n", (double)gyro_offset_x, (double)gyro_offset_y, (double)gyro_offset_z);
}

// --- Madgwick filter implementation (minimal, single file) ---
// 9DOF version, based on open-source MadgwickAHRSupdate
// Beta parameter controls filter responsiveness (try 0.1-0.4)
#define MADGWICK_BETA 0.15f

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
        icm_read_word(i2c_dev, GYRO_XOUT_H + 4, &gyro_z) != 0 || icm_read_word(i2c_dev, TEMP_OUT_H, &raw_temp) != 0) {
        return -EIO;
    }

    // Subtract gyro offsets
    gyro_x -= (int16_t)gyro_offset_x;
    gyro_y -= (int16_t)gyro_offset_y;
    gyro_z -= (int16_t)gyro_offset_z;

    // printk("acc_x=%d, acc_y=%d, acc_z=%d, gyro_x=%d, gyro_y=%d, gyro_z=%d\n",
    //     acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

    imu->accelerometer_raw.x = acc_x;
    imu->accelerometer_raw.y = acc_y;
    imu->accelerometer_raw.z = acc_z; // multiply by 1.16

    imu->gyroscope_raw.x = gyro_x;
    imu->gyroscope_raw.y = gyro_y;
    imu->gyroscope_raw.z = gyro_z;

    imu->accelerometer_scaled.x = (float)imu->accelerometer_raw.x / acc_sensitivity;
    imu->accelerometer_scaled.y = (float)imu->accelerometer_raw.y / acc_sensitivity;
    imu->accelerometer_scaled.z = (float)imu->accelerometer_raw.z / acc_sensitivity;

    imu->gyroscope_scaled.x = (float)imu->gyroscope_raw.x / gyro_sensitivity;
    imu->gyroscope_scaled.y = (float)imu->gyroscope_raw.y / gyro_sensitivity;
    imu->gyroscope_scaled.z = (float)imu->gyroscope_raw.z / gyro_sensitivity;

    imu->pitch_acc = atan2(imu->accelerometer_scaled.y, imu->accelerometer_scaled.z) * 180.0 / M_PI;
    imu->pitch_gyro = imu->gyroscope_scaled.x;
    imu->pitch_complementary =
        alpha * (imu->pitch_complementary + imu->pitch_gyro * sampling_time_sec) + (1.0f - alpha) * imu->pitch_acc;

    imu->roll_acc = atan2(imu->accelerometer_scaled.x, imu->accelerometer_scaled.z) * 180.0 / M_PI;
    imu->roll_gyro = imu->gyroscope_scaled.y;
    imu->roll_complementary =
        alpha * (imu->roll_complementary + imu->roll_gyro * sampling_time_sec) + (1.0f - alpha) * imu->roll_acc;

    // Yaw calculation (accel + gyro, no magnetometer)
    imu->yaw_acc = atan2(imu->accelerometer_scaled.x, imu->accelerometer_scaled.y) * 180.0 / M_PI;
    imu->yaw_gyro = imu->gyroscope_scaled.z;
    imu->yaw_complementary =
        alpha * (imu->yaw_complementary + imu->yaw_gyro * sampling_time_sec) + (1.0f - alpha) * imu->yaw_acc;

    // printf("acc_x=%f, acc_y=%f, acc_z=%f, gyro_x=%f, gyro_y=%f, gyro_z=%f\n",
    //     imu->accelerometer_scaled.x, imu->accelerometer_scaled.y, imu->accelerometer_scaled.z,
    //     imu->gyroscope_scaled.x, imu->gyroscope_scaled.y, imu->gyroscope_scaled.z);

    // printf("pitch=%f, roll=%f\n", imu->pitch_complementary, imu->roll_complementary);

    imu->temp = raw_temp;

    // --- Magnetometer readout (added, does not replace existing logic) ---
    int16_t mag_x = 0, mag_y = 0, mag_z = 0;
    ak09916_read_mag(i2c_dev, &mag_x, &mag_y, &mag_z);
    imu->magnetometer_raw.x = mag_x;
    imu->magnetometer_raw.y = mag_y;
    imu->magnetometer_raw.z = mag_z;
    imu->magnetometer_scaled.x = (float)mag_x * 0.15f; // AK09916: 0.15 uT/LSB
    imu->magnetometer_scaled.y = (float)mag_y * 0.15f;
    imu->magnetometer_scaled.z = (float)mag_z * 0.15f;

    // --- Madgwick 9DoF fusion (added, does not replace existing logic) ---
    static MadgwickState madgwick = {1, 0, 0, 0};

    char mess[128];
    snprintf(mess,
             sizeof(mess),
             "Madgwick input: gx=%.3f, gy=%.6f, gz=%.3f, ax=%.3f, ay=%.3f, az=%.3f, mx=%.3f, my=%.3f, mz=%.3f, dt=%.3f",
             imu->gyroscope_scaled.x * (M_PI / 180.0f),
             imu->gyroscope_scaled.y * (M_PI / 180.0f),
             imu->gyroscope_scaled.z * (M_PI / 180.0f),
             imu->accelerometer_scaled.x,
             imu->accelerometer_scaled.y,
             imu->accelerometer_scaled.z,
             imu->magnetometer_scaled.x,
             imu->magnetometer_scaled.y,
             imu->magnetometer_scaled.z,
             sampling_time_sec);

    printk("%s\n", mess);

    madgwick_update(&madgwick,
                    imu->gyroscope_scaled.x * (M_PI / 180.0f),
                    imu->gyroscope_scaled.y * (M_PI / 180.0f),
                    imu->gyroscope_scaled.z * (M_PI / 180.0f),
                    imu->accelerometer_scaled.x,
                    imu->accelerometer_scaled.y,
                    imu->accelerometer_scaled.z,
                    imu->magnetometer_scaled.y,  // Swap X/Y for mag
                    -imu->magnetometer_scaled.x, // Invert X for mag
                    imu->magnetometer_scaled.z,
                    sampling_time_sec);
    // Convert quaternion to Euler angles (degrees)
    float q0 = madgwick.q0, q1 = madgwick.q1, q2 = madgwick.q2, q3 = madgwick.q3;
    imu->fused_roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    imu->fused_pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    imu->fused_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

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

void ak_read_whoami(const struct device *i2c_dev) {
    uint8_t reg_addr = MAG_WIA2;
    uint8_t who_am_i = 0;
    int ret = i2c_write_read(i2c_dev, MAG_I2C_ADDR, &reg_addr, 1, &who_am_i, 1);
    if (ret != 0) {
        printk("WHO_AM_I read error: %d\n", ret);
    } else {
        printk("WHO_AM_I: 0x%02X\n", who_am_i);
    }
}
