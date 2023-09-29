/**
 * @file custom_ble_service.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "custom_ble_service.h"

#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(custom_service);

struct custom_service_data
{
  int32_t force;
  float   gyro_x, gyro_y, gyro_z;
  int16_t angle;
  float   tangential_velocity;
  int16_t power;
};

static struct custom_service_data adv_data = {
  .gyro_x = 0.0,
  .gyro_y = 0.0,
  .gyro_z = 0.0,
  .angle  = 0,
};

static unsigned connnected;


BT_GATT_SERVICE_DEFINE(custom_ble_service,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERV),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHRC,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              NULL,
                                              NULL,
                                              NULL),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static int custom_service_init(void)
{
  connnected = 0;
  return 0;
}

// bool custom_srvc_is_connected() {
//   return true;
// }

void advertise_data(const int32_t force,
                    const float   gyro_x,
                    const float   gyro_y,
                    const float   gyro_z,
                    const int16_t angle,
                    const float   tangential_velocity,
                    const int16_t power)
{
  adv_data.force               = force;
  adv_data.gyro_x              = gyro_x;
  adv_data.gyro_y              = gyro_y;
  adv_data.gyro_z              = gyro_z;
  adv_data.angle               = angle;
  adv_data.tangential_velocity = tangential_velocity;
  adv_data.power               = power;

  bt_gatt_notify(NULL, &custom_ble_service.attrs[1], &adv_data, sizeof(adv_data));
}

SYS_INIT(custom_service_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);