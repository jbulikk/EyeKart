/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

 #include <zephyr/kernel.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/pwm.h>
 #include <zephyr/drivers/i2c.h>
#include "ICM_20948_C.h"



#define I2C_DEV_LABEL "I2C_0"
#define REG_BANK_SEL  0x7F
#define BANK_0        0x00
#define WHO_AM_I_REG  0x00
#define ICM20948_ADDR 0x69

void i2c_scanner(void);
void read_icm_whoami(void);

int main(void)
{
	printk("Start of main\n");

	const struct device *i2c_dev = device_get_binding(I2C_DEV_LABEL); // nazwa zależy od platformy
	if (!i2c_dev) {
 		printk("Nie można znaleźć\n");
        printk(I2C_DEV_LABEL);
    	return;
	}

    k_sleep(K_SECONDS(10));
	while(1){
        i2c_scanner();
        k_sleep(K_SECONDS(2));
	}
}

void icm_set_bank(const struct device *i2c_dev, uint8_t addr, uint8_t bank) {
    uint8_t reg[2] = { 0x7F, bank << 4 }; // rejestr 0x7F, wartość BANK << 4
    i2c_write(i2c_dev, reg, 2, addr);
}

void icm_init(const struct device *i2c_dev, uint8_t addr) {
    // Przełącz na BANK 0
    icm_set_bank(i2c_dev, addr, 0);
    k_sleep(K_MSEC(100));

    // Wyłącz tryb sleep
    uint8_t pwr_mgmt_1[2] = { 0x10, 0x01 }; // Reset device
    i2c_write(i2c_dev, pwr_mgmt_1, 2, addr);
    k_sleep(K_MSEC(100));

    pwr_mgmt_1[1] = 0x01; // Clock source = auto
    i2c_write(i2c_dev, pwr_mgmt_1, 2, addr);
    k_sleep(K_MSEC(100));

    // Włącz akcelerometr i żyroskop
    uint8_t pwr_mgmt_2[2] = { 0x11, 0x00 }; // Włącz wszystko
    i2c_write(i2c_dev, pwr_mgmt_2, 2, addr);
}

void icm_read_accel_gyro(const struct device *i2c_dev, uint8_t addr) {
    // BANK 2 dla danych
    icm_set_bank(i2c_dev, addr, 2);

    uint8_t reg = 0x2D; // ACCEL_XOUT_H
    uint8_t data[12] = {0};

    // Odczytaj 12 bajtów: 6 dla akcelerometru, 6 dla żyroskopu
    i2c_write_read(i2c_dev, addr, &reg, 1, data, 12);

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    int16_t gx = (data[6] << 8) | data[7];
    int16_t gy = (data[8] << 8) | data[9];
    int16_t gz = (data[10] << 8) | data[11];

    printk("Accel: X=%d Y=%d Z=%d\n", ax, ay, az);
    printk("Gyro:  X=%d Y=%d Z=%d\n", gx, gy, gz);
}

void read_icm_whoami(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    uint8_t reg_addr = WHO_AM_I_REG;
    uint8_t who_am_i = 0;

    int ret = i2c_write_read(i2c_dev, ICM20948_ADDR, &reg_addr, 1, &who_am_i, 1);
    if (ret != 0) {
        printk("Błąd odczytu WHO_AM_I: %d\n", ret);
    } else {
        printk("WHO_AM_I: 0x%02X\n", who_am_i);
    }
}

void i2c_scanner(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    printk("Skanowanie I2C...\n");

    while(true)
    {
        for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
            uint8_t dummy = 0;
            int ret = i2c_write(i2c_dev, &dummy, 1, addr);
            if (ret == 0) {
                printk("Znaleziono urządzenie na adresie 0x%02X\n", addr);
            }
        }
        printk("koniec skanu\n");
        k_sleep(K_SECONDS(3));
    }
}
















// int main(void)
// {
// 	printk("Start of main\n");

// 	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
//     if (!device_is_ready(i2c_dev)) {
//         printk("I2C device not ready!\n");
//         return 0;
//     }

//     // Ustawienie banku 0 (opcjonalnie, zależnie od stanu początkowego)
//     uint8_t set_bank_cmd[2] = { REG_BANK_SEL, BANK_0 };
//     if (i2c_write(i2c_dev, set_bank_cmd, sizeof(set_bank_cmd), ICM20948_ADDR) != 0) {
//         printk("Failed to select bank 0!\n");
//         return 0;
//     }

//     // Odczyt WHO_AM_I
//     uint8_t reg = REG_WHO_AM_I;
//     uint8_t who_am_i = 0;

//     if (i2c_write_read(i2c_dev, ICM20948_ADDR, &reg, 1, &who_am_i, 1) != 0) {
//         printk("Failed to read WHO_AM_I!\n");
//         return 0;
//     }

//     printk("WHO_AM_I: 0x%02X\n", who_am_i);

// }

 


// Timer example
static void led_timer_callback(struct k_timer *timer_id)
{
	led_toggle();
	// bt_central_send("Hello from timer");
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);


// BT PERIPHERAL
static void on_receive(const char *data) {
    printk("Peripheral received: %s\n", data);
}

// BT CENTRAL
static void on_connected(void) {
    printk("Connected to peripheral, sending data...\n");
    // bt_central_send("Hello Peripheral!");
	// printk("Data 1 sent\n");
    // bt_central_send("DZIALA");
	// printk("Data 2 sent\n");
}
