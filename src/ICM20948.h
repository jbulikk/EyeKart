#ifndef ICM20948_H
#define ICM20948_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include "madgwick.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ICM20948_ADDR 0x68


typedef struct {
    int16_t x;  
    int16_t y;
    int16_t z;
} AxisData;

typedef struct {
    float x;  
    float y;
    float z;
} AxisDataFloat;

typedef struct {
    AxisData accelerometer_raw;
    AxisData gyroscope_raw;
    AxisData magnetometer_raw;
    AxisDataFloat accelerometer_scaled;
    AxisDataFloat gyroscope_scaled;
    AxisDataFloat magnetometer_scaled;
    float pitch_acc;
    float roll_acc;
    float yaw_acc;
    float pitch_gyro;
    float roll_gyro;
    float yaw_gyro;
    float pitch_complementary;
    float roll_complementary;
    float yaw_complementary;
    float fused_pitch;
    float fused_roll;
    float fused_yaw;
    int16_t temp;
} ImuData;

double get_time_seconds();

void madgwick_update(MadgwickState *state, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

int icm_init(const struct device *i2c_dev);
int icm_read_all_data(const struct device *i2c_dev,
                      int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz,
                      int16_t *temp);
void icm_read_whoami(const struct device *i2c_dev);
void ak_read_whoami(const struct device *i2c_dev);

int icm_read_acc_mag_temp(const struct device *i2c_dev, ImuData *imu);
int icm_read_data_and_calculate_angle(const struct device *i2c_dev, ImuData *imu);
void icm_calibrate_gyro(const struct device *i2c_dev, int samples);

#endif