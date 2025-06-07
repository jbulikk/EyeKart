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

    printk("Initializing ICM-20948...\n");
    icm_init(i2c_dev);

    while (1) {
        int16_t ax, ay, az, gx, gy, gz, temp;
        if (icm_read_all_data(i2c_dev, &ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
            printk("ACC [mg]: X=%d Y=%d Z=%d | ", ax, ay, az);
            printk("GYRO [mdps]: X=%d Y=%d Z=%d | TEMP [raw]=%d\n", gx, gy, gz, temp);
        } else {
            printk("Failed to read sensor data\n");
        }
        k_msleep(500);
    }
}
