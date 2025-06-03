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

 #define SERVO_YAW_NODE DT_ALIAS(servo_yaw)
 #define SERVO_PITCH_NODE DT_ALIAS(servo_pitch)
 
 #define STEP PWM_USEC(10)
 
 typedef struct servo_t
 {
	 struct pwm_dt_spec pwm;
	 uint32_t min_pulse;
	 uint32_t max_pulse;
 } servo_t;
 
 static const servo_t servo_yaw = {
		 .pwm = PWM_DT_SPEC_GET(SERVO_YAW_NODE),
		 .min_pulse = DT_PROP(SERVO_YAW_NODE, min_pulse),
		 .max_pulse = DT_PROP(SERVO_YAW_NODE, max_pulse),
 };
 
 static const servo_t servo_pitch =
	 {
		 .pwm = PWM_DT_SPEC_GET(SERVO_PITCH_NODE),
		 .min_pulse = DT_PROP(SERVO_PITCH_NODE, min_pulse),
		 .max_pulse = DT_PROP(SERVO_PITCH_NODE, max_pulse),
 };
 
 enum direction {
	 DOWN,
	 UP,
 };

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

int main(void)
{
	printk("Start of main\n");
    led_init();
	// bt_peripheral_init(on_receive);
	bt_central_init(on_connected);


	// Start the LED toggle timer to fire every 1 second
	k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));

	 uint32_t pulse_width = servo_yaw.min_pulse;
	 enum direction dir = UP;
	 int ret;
 
	 printk("Servomotor control\n");
 
	 if (!pwm_is_ready_dt(&servo_yaw.pwm))
	 {
		 printk("Error: PWM device %s is not ready\n", servo_yaw.pwm.dev->name);
		 return 0;
	 };

	int i = 0;
	char buf[32];
	while (1) {
		snprintf(buf, sizeof(buf), "ABC %d", i);
		printk("Sending: %s\n", buf);
		bt_central_send(buf);
		i++;
		k_sleep(K_MSEC(5000));
	}

	//  while (1)
	//  {
	// 	 ret = pwm_set_pulse_dt(&servo_yaw.pwm, pulse_width);
	// 	 if (ret < 0)
	// 	 {
	// 		 printk("Error %d: failed to set pulse width\n", ret);
	// 		 return 0;
	// 	 }
 
	// 	 if (dir == DOWN)
	// 	 {
	// 		 if (pulse_width <= servo_yaw.min_pulse)
	// 		 {
	// 			 dir = UP;
	// 			 pulse_width = servo_yaw.min_pulse;
	// 		 }
	// 		 else
	// 		 {
	// 			 pulse_width -= STEP;
	// 		 }
	// 	 }
	// 	 else
	// 	 {
	// 		 pulse_width += STEP;
 
	// 		 if (pulse_width >= servo_yaw.max_pulse)
	// 		 {
	// 			 dir = DOWN;
	// 			 pulse_width = servo_yaw.max_pulse;
	// 		 }
	// 	 }
	// 	 printf("PWM pulse width [us]: %d\n", pulse_width);
	// 	 k_sleep(K_MSEC(10));
	//  }
	 return 0;
 }
 