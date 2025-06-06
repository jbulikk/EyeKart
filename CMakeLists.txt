# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(servo_motor)

target_include_directories(app PRIVATE
  src
  src/icm20948
)

target_sources(app PRIVATE src/main.c src/led_control.c src/bt_central.c src/bt_peripheral.c src/icm20948/ICM_20948_C.c)
