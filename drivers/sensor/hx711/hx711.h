/*
 * Copyright (c) 2020 George Gkinis
 * Copyright (c) 2021 Jan Gnip
 * Copyright (c) 2023 Stavros Avramidis
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_HX711_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_HX711_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

  enum hx711_gain
  {
    HX711_GAIN_128X = 1,
    HX711_GAIN_32X,
    HX711_GAIN_64X,
  };

  enum hx711_rate
  {
    HX711_RATE_10HZ,
    HX711_RATE_80HZ,
  };


  int hx711_read(const struct device* dev, int32_t* const reading);

  void hx711_set_offset(const struct device* dev, const int32_t offset);

  int avia_hx711_tare(const struct device* dev, const uint8_t readings);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_HX711_H_ */