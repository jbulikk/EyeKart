#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include "ICM20948.h"

#define I2C_DEV_LABEL "I2C_0"

int main(void) {
    printk("Start of main\n");

    const struct device *i2c_dev = device_get_binding(I2C_DEV_LABEL);
    if (!i2c_dev) {
        printk("I2C device not found: %s\n", I2C_DEV_LABEL);
        return 0;
    }

    icm_read_whoami(i2c_dev);
    ImuData imu = {0};

    printk("Initializing ICM-20948...\n");
    icm_init(i2c_dev);

    printf("Start loop...\n");

    while (1) {
        // if (icm_read_data_and_calculate_angle(i2c_dev, &imu) == 0) {
        //     // printk("Pitch: acc=%d°, gyro=%d°, comp=%d° | Roll: acc=%d°, gyro=%d°, comp=%d°\n",
        //     //        imu.pitch_acc, imu.pitch_gyro, imu.pitch_complementary,
        //     //        imu.roll_acc, imu.roll_gyro, imu.roll_complementary);
        // } else {
        //     printk("Read error!\n");
        // }

        icm_read_acc_mag_temp(i2c_dev, &imu);

        k_sleep(K_SECONDS(0.01));
    }
}
