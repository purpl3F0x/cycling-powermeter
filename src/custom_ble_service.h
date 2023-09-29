/**
 * @file custom_ble_service.h
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <stdint.h>

#include <zephyr/bluetooth/uuid.h>

// 1ef880a7-48fe-4517-8bfa-7063ffb80970


#define BT_UUID_CUSTOM_SERV_VAL                                                                    \
  BT_UUID_128_ENCODE(0x1ef880a7, 0x48fe, 0x4517, 0x8bfa, 0x7063ffb80970)


#define BT_UUID_CUSTOM_CHRC_VAL                                                                    \
  BT_UUID_128_ENCODE(0x1ef880a7, 0x48fe, 0x4517, 0x8bfa, 0x7063ffb80971)


#define BT_UUID_CUSTOM_SERV BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERV_VAL)
#define BT_UUID_CUSTOM_CHRC BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHRC_VAL)

// bool custom_srvc_is_connected();

void advertise_data(const int32_t force,
                    const float   gyro_x,
                    const float   gyro_y,
                    const float   gyro_z,
                    const int16_t angle,
                    const float   tangential_velocity,
                    const int16_t power);
