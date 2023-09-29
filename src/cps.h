/**
 * @file cps.h
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

#ifndef BT_UUID_GATT_CPS_CPCP
#define BT_UUID_GATT_CPS_CPCP_VAL 0x2a66
#define BT_UUID_GATT_CPS_CPCP     BT_UUID_DECLARE_16(BT_UUID_GATT_CPS_CPCP_VAL)
#define BT_UUID_GATT_CPS_CPM_VAL  0x2a63
#define BT_UUID_GATT_CPS_CPM      BT_UUID_DECLARE_16(BT_UUID_GATT_CPS_CPM_VAL)
#define BT_UUID_CPS_VAL           0x1818
#define BT_UUID_CPS               BT_UUID_DECLARE_16(BT_UUID_CPS_VAL)
#define BT_UUID_GATT_CPS_CPF_VAL  0x2a65
#define BT_UUID_GATT_CPS_CPF      BT_UUID_DECLARE_16(BT_UUID_GATT_CPS_CPF_VAL)

#endif

#define CRANK_LENGHT_TO_MM(x) (x / 2.0)

#define CPS_CPF_PEDAL_POWER_BALANCE           (1 << 0)
#define CPS_CPF_ACCUMULATED_TORQUE            (1 << 1)
#define CPS_CPF_WHEEL_REVOLUTION_DATA         (1 << 2)
#define CPS_CPF_CRANK_REVOLUTION_DATA         (1 << 3)
#define CPS_CPF_EXTREME_FORCE_MAGNITUDES      (1 << 4)
#define CPS_CPF_EXTREME_TOQUE_MAGNITUDES      (1 << 5)
#define CPS_CPF_EXTREME_ANGLES                (1 << 6)
#define CPS_CPF_TOP_DEAD_SPOT_ANGLE           (1 << 7)
#define CPS_CPF_ACCUMULATED_ENERGY            (1 << 8)
#define CPS_CPF_OFFSET_COMPENSATION_INDICATOR (1 << 9)
#define CPS_CPF_POWER_MEASURMETN_MASKING      (1 << 10)
#define CPS_CPF_MULTIPLE_LOCATIONS_SUPPORTED  (1 << 11)
#define CPS_CPF_CRANK_LENGTH_ADJUSTMENT       (1 << 12)
#define CPS_CPF_CHAIN_LENGTH_ADJUSTMENT       (1 << 13)
#define CPS_CPF_CHAIN_WEIGHT_ADJUSTMENT       (1 << 14)
#define CPS_CPF_SPAN_LENGTH_ADJUSTMENT        (1 << 15)
#define CPS_CPF_SENSOR_MEASURMENT_FORCE       (0 << 16)
#define CPS_CPF_SENSOR_MEASURMENT_TORQUE      (1 << 16)
#define CPS_CPF_INSTANTANEOUS_DIRECTION       (1 << 17)
#define CPS_CPF_FACTORY_CALIBRATION_DATE      (1 << 18)
#define CPS_CPF_ENCHANCED_OFFSET_COMPENSATION (1 << 19)


enum cps_sensor_location
{
  OTHER        = 0,
  TOP_OF_SHOE  = 1,
  IN_SHOE      = 2,
  HIP          = 3,
  FRONT_WHEEL  = 4,
  LEFT_CRANK   = 5,
  RIGHT_CRANK  = 6,
  LEFT_PEDAL   = 7,
  RIGHT_PEDAL  = 8,
  FRONTHUB     = 9,
  REAR_DROPOUT = 10,
  CHAINSTAY    = 11,
  REAR_WHEEL   = 12,
  REAR_HUB     = 13,
  CHEST        = 14,
  SPIDER       = 15,
  CHAINRING    = 16,
};

enum cps_ctrl_opcode
{
  SET_CUMULATIVE_VALUE                                  = 1,
  UPDATE_SENSOR_LOCATION                                = 2,
  REQUEST_SUPPORTED_SENSOR_LOCATIONS                    = 3,
  SET_CRANK_LENGTH                                      = 4,
  REQUEST_CRANK_LENGTH                                  = 5,
  SET_CHAIN_LENGTH                                      = 6,
  REQUEST_CHAIN_LENGTH                                  = 7,
  SET_CHAIN_WEIGHT                                      = 8,
  REQUEST_CHAIN_WEIGHT                                  = 9,
  SET_SPAN_LENGTH                                       = 10,
  REQUEST_SPAN_LENGTH                                   = 11,
  START_OFFSET_COMPENSATION                             = 12,
  MASK_CYCLING_POWER_MEASUREMENT_CHARACTERISTIC_CONTENT = 13,
  REQUEST_SAMPLING_RATE                                 = 14,
  REQUEST_FACTORY_CALIBRATION_DATE                      = 15,
  START_ENHANCED_OFFSET_COMPENSATION                    = 16,
  RESPONSE_CODE                                         = 32,
};


enum cps_ctrl_responce
{
  SUCCESS               = 1,
  OP_CODE_NOT_SUPPORTED = 2,
  INVALID_PARAMTER      = 3,
  OPERATION_FAILED      = 4,
};


/**
 * @brief Set the power n revolution object
 *
 * @param power 
 * @param revolutions 
 * @param rev_time 
 * @return int 
 */
int set_power_n_revolution(const uint16_t power, const uint16_t revolutions, const uint16_t last_rev_time);

/**
 * @brief Set the power vector inst force object
 *
 * @param power force in Newtons
 * @return int
 */
int set_power_vector_inst_force(const uint16_t force);


void set_calibration_callback(int (*calibration_callback)(void));
