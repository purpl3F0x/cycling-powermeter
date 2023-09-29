/**
 * @file calibration_settings.h
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <inttypes.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>


struct imu_cal_data
{
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
};

struct adc_cal_data
{
  double slope;
  int    offset;
};

extern struct imu_cal_data imu_cal_values;
extern struct adc_cal_data adc_cal_values;


extern struct settings_handler my_conf;