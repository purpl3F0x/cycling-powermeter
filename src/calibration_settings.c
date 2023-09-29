/**
 * @file calibration_settings.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "calibration_settings.h"


static int cal_settings_set(const char* name, size_t len, settings_read_cb read_cb, void* cb_arg)
{
  const char* next;
  int         rc;

  if (settings_name_steq(name, "imu", &next) && !next)
  {
    if (len != sizeof(imu_cal_values))
    {
      return -EINVAL;
    }

    rc = read_cb(cb_arg, &imu_cal_values, sizeof(imu_cal_values));
    if (rc >= 0)
    {
      return 0;
    }

    return rc;
  }

  else if (settings_name_steq(name, "adc", &next) && !next)
  {
    if (len != sizeof(adc_cal_values))
    {
      return -EINVAL;
    }

    rc = read_cb(cb_arg, &adc_cal_values, sizeof(adc_cal_values));
    if (rc >= 0)
    {
      return 0;
    }

    return rc;
  }


  return -ENOENT;
}


struct imu_cal_data imu_cal_values = {
  .acc_x  = 0.0,
  .acc_y  = 0.0,
  .acc_z  = 0.0,
  .gyro_x = 0.0,
  .gyro_y = 0.0,
  .gyro_z = 0.0,

};

struct adc_cal_data adc_cal_values = {
  .slope  = 0.0,
  .offset = 0,
};


struct settings_handler my_conf = {
  .name  = "calibration",
  .h_set = cal_settings_set,
};
