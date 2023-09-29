/**
 * @file cps.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cps.h"


#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define LOG_LEVEL CONFIG_BT_CPS_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cps);


struct cps_data
{
  uint32_t features;
  uint32_t power;
  uint32_t crank_revolutions;
  uint8_t  location;
  uint16_t crank_length;

  int (*calibration_callback)(void);
};

static struct cps_data         data;
struct bt_gatt_indicate_params ctrl_ind_params;
static uint8_t                 indicate_buf[16];

static ssize_t read_power_feature(struct bt_conn*            conn,
                                  const struct bt_gatt_attr* attr,
                                  void*                      buf,
                                  uint16_t                   len,
                                  uint16_t                   offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &data.features, sizeof(data.features));
}


static ssize_t on_read_sensor_location(struct bt_conn*            conn,
                                       const struct bt_gatt_attr* attr,
                                       void*                      buf,
                                       uint16_t                   len,
                                       uint16_t                   offset)
{
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &data.location, sizeof(data.location));
}


static void cps_measurment_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
  ARG_UNUSED(attr);

  bool is_enabled = (value == BT_GATT_CCC_NOTIFY);

  LOG_INF("CTS Measurement Notifications %s", is_enabled ? "enabled" : "disabled");
}

static void cps_power_vector_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
  ARG_UNUSED(attr);

  bool is_enabled = (value == BT_GATT_CCC_NOTIFY);

  LOG_INF("CTS Power Vector Notifications %s", is_enabled ? "enabled" : "disabled");
}


static ssize_t on_write_ctrl_point(struct bt_conn*            conn,
                                   const struct bt_gatt_attr* attr,
                                   const void*                buf,
                                   uint16_t                   len,
                                   uint16_t                   offset,
                                   uint8_t                    flags)
{
  const uint8_t* buffer  = (uint8_t*)buf;
  const uint8_t  op_code = buffer[0];

  LOG_INF("Received OpCode %d", op_code);

  indicate_buf[0] = RESPONSE_CODE;
  indicate_buf[1] = op_code;

  switch (op_code)
  {
    case REQUEST_CRANK_LENGTH:
      LOG_INF(" CPS CTRL: Request for crank lenght");

      indicate_buf[2] = SUCCESS;

      sys_put_le16(data.crank_length, &indicate_buf[3]);
      ctrl_ind_params.len = 5;

      bt_gatt_indicate(NULL, &ctrl_ind_params);

      break;

    case START_OFFSET_COMPENSATION:
    case START_ENHANCED_OFFSET_COMPENSATION:
      LOG_INF(" CPS CTRL: Request for Zero Offset Compensation");
      if (data.calibration_callback != NULL)
      {
        int ret = data.calibration_callback();
        ARG_UNUSED(ret);
        indicate_buf[2]     = SUCCESS;
        ctrl_ind_params.len = 3;
        bt_gatt_indicate(NULL, &ctrl_ind_params);

        break;
      }

    default:
      LOG_WRN(" CPS CTRL: Not Supported Op Code: %d", op_code);

      indicate_buf[2]     = OP_CODE_NOT_SUPPORTED;
      ctrl_ind_params.len = 3;
      bt_gatt_indicate(NULL, &ctrl_ind_params);

      break;
  }

  return len;
}


static void cps_ctrl_point_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
  ARG_UNUSED(attr);

  bool is_enabled = (value == BT_GATT_CCC_INDICATE);

  LOG_INF("CTS Control Point Indications %s", is_enabled ? "enabled" : "disabled");
}


BT_GATT_SERVICE_DEFINE(
  cps,
  BT_GATT_PRIMARY_SERVICE(BT_UUID_CPS),

  BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPF,
                         BT_GATT_CHRC_READ,
                         BT_GATT_PERM_READ,
                         read_power_feature,
                         NULL,
                         NULL),

  BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPM,
                         BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ,
                         NULL,
                         NULL,
                         NULL),
  BT_GATT_CCC(cps_measurment_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

  BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION,
                         BT_GATT_CHRC_READ,
                         BT_GATT_PERM_READ,
                         on_read_sensor_location,
                         NULL,
                         NULL),

  BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPCP,
                         BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
                         BT_GATT_PERM_WRITE,
                         NULL,
                         on_write_ctrl_point,
                         NULL),
  BT_GATT_CCC(cps_ctrl_point_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

  BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPV,
                         BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ,
                         NULL,
                         NULL,
                         NULL),
  BT_GATT_CCC(cps_power_vector_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );


static int cps_init(void)
{
  data.features = CPS_CPF_CRANK_REVOLUTION_DATA | CPS_CPF_OFFSET_COMPENSATION_INDICATOR |
                  CPS_CPF_ENCHANCED_OFFSET_COMPENSATION;
  data.power             = 0;
  data.crank_revolutions = 0;
  data.location          = LEFT_CRANK;
  data.crank_length      = 345; // 172.5mm * 2

  // Initialize Control Point Indication struct
  ctrl_ind_params.attr    = &cps.attrs[9];
  ctrl_ind_params.func    = NULL;
  ctrl_ind_params.destroy = NULL;
  ctrl_ind_params.data    = &indicate_buf;
  ctrl_ind_params.len     = 0;

  return 0;
}


int set_power_n_revolution(const uint16_t power,
                           const uint16_t revolutions,
                           const uint16_t last_rev_time)
{
  static uint8_t buffer[8] = { (1 << 5) };

  sys_put_le16(power, buffer + 2);
  sys_put_le16(revolutions, buffer + 4);
  sys_put_le16(last_rev_time, buffer + 6);

  return bt_gatt_notify(NULL, &cps.attrs[3], &buffer, sizeof(buffer));
}


int set_power_vector_inst_force(const uint16_t force)
{
  static uint8_t buffer[3] = { (1 << 2), 0, 0 };

  sys_put_le16(force, buffer + 1);

  return bt_gatt_notify(NULL, &cps.attrs[11], &buffer, sizeof(buffer));
}


void set_calibration_callback(int (*calibration_callback)(void))
{
  data.calibration_callback = calibration_callback;
}


SYS_INIT(cps_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);