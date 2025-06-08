#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include "ICM20948.h"
#include "bt_central.h"
#include "led_control.h"

#define I2C_DEV_LABEL "I2C_0"

static ImuData imu;

static void led_timer_callback(struct k_timer *timer_id)
{
	led_toggle();
	// bt_central_send("Hello from timer");
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);

// BT CENTRAL
static void on_connected(void) {
    printk("Connected to peripheral.\n");
}

static void send_angle_callback(struct k_timer *timer_id)
{
    char message[16];
    snprintf(message, sizeof(message), "%.2f", imu.pitch_complementary); 
    
    printk("Sending %s\n", message);
    bt_central_send(message);  // Send the whole string, not a single char
}
K_TIMER_DEFINE(send_angle_timer, send_angle_callback, NULL);


int main(void) {
    led_init();
	k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));

    printk("Start of main\n");
    bt_central_init(on_connected);
	k_timer_start(&send_angle_timer, K_MSEC(300), K_MSEC(300));

    const struct device *i2c_dev = device_get_binding(I2C_DEV_LABEL);
    if (!i2c_dev) {
        printk("I2C device not found: %s\n", I2C_DEV_LABEL);
        return 0;
    }

    icm_read_whoami(i2c_dev);
    

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
