/**
 * @file main.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "battery_meas.h"
#include "calibration_settings.h"
#include "cps.h"
#include "custom_ble_service.h"

#include <sensor/hx711/hx711.h>

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>


// Zephyr
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/settings/settings.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>

// NCS
#include <bluetooth/services/nus.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define CRANK_LENTH 0.1725
#define M_PI        3.14159265358979323846


/*
 * Devices
 */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static const struct device* imu_dev = DEVICE_DT_GET_ONE(st_lsm6ds3tr);

static const struct device* hx711_dev = DEVICE_DT_GET_ONE(avia_hx711);

static struct sensor_trigger imu_trig = {
  .type = SENSOR_TRIG_DATA_READY,
  .chan = SENSOR_CHAN_GYRO_XYZ,
};

// Mutex for taking priority over main thread
K_MUTEX_DEFINE(main_thread_mutex);

static struct sensor_value gyro_x, gyro_y, gyro_z;
static int32_t             force;
static double              angular_velocity = 0.0;
static double              pseudo_angle     = 0.0;
static double              tangential_velocity;
static double              power;
static int16_t             power_uint;
static uint32_t            last_rev_time_ticks = 0;
static uint16_t            last_rev_time       = 0;
static uint16_t            crank_revolutions;


static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
  BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                BT_UUID_16_ENCODE(BT_UUID_CPS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_DIS_VAL), ),
};


static void connected(struct bt_conn* conn, uint8_t err)
{
  if (err)
    LOG_ERR("Connection failed (err 0x%02x)\n", err);
  else
    LOG_INF("Connected\n");
}


static void disconnected(struct bt_conn* conn, uint8_t reason)
{
  LOG_INF("Disconnected (reason 0x%02x)\n", reason);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
  .connected    = connected,
  .disconnected = disconnected,
};


static void bt_ready(void)
{
  int err;

  LOG_INF("Bluetooth initialized\n");

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err)
  {
    LOG_ERR("Advertising failed to start (err %d)\n", err);
    return;
  }

  LOG_INF("Advertising successfully started\n");
}


static void settings_ready(void)
{
  if (settings_register(&my_conf))
  {
    LOG_ERR("Error registeing my_conf settings");
    return;
  }

  if (settings_load())
  {
    LOG_ERR("Eror loading settings");
    return;
  }

  LOG_INF("IMU Calibration:\n\t\tACCEL: X= %4.6f, Y= %4.6f, Z= %4.6f\n\t\tGYRO:  X= %4.6f, Y= "
          "%4.6f, Z= %4.6f\n",
          imu_cal_values.acc_x,
          imu_cal_values.acc_y,
          imu_cal_values.acc_z,
          imu_cal_values.gyro_x,
          imu_cal_values.gyro_y,
          imu_cal_values.gyro_z);

  LOG_INF("ADC Calibration: Offset %7d, Slope: %lf\n", adc_cal_values.offset, adc_cal_values.slope);
}


static int calibration_routine(void)
{

  int                 err = 0;
  double              x = 0, y = 0, z = 0;
  struct sensor_value gyro_x, gyro_y, gyro_z;

  k_mutex_lock(&main_thread_mutex, K_FOREVER);

  LOG_INF("Starting Calibration Routine");
  LOG_INF("Gyro Calibration ...");

  for (unsigned i = 0; i < 256; i++)
  {
    gpio_pin_set_dt(&led_green, 1);
    sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

    x += sensor_value_to_double(&gyro_x);
    y += sensor_value_to_double(&gyro_y);
    z += sensor_value_to_double(&gyro_z);

    k_sleep(K_MSEC(10));
    gpio_pin_set_dt(&led_green, 0);
    k_sleep(K_MSEC(90));
  }

  imu_cal_values.gyro_x = x / 256.0f;
  imu_cal_values.gyro_y = y / 256.0;
  imu_cal_values.gyro_z = z / 256.0;

  gpio_pin_set_dt(&led_green, 1);
  gpio_pin_set_dt(&led_red, 1);

  LOG_INF("Gyro Calibration Avg: X= %4.6f, Y= %4.6f, Z= %4.6f",
          imu_cal_values.gyro_x,
          imu_cal_values.gyro_y,
          imu_cal_values.gyro_z);

  LOG_INF("ADC Calibration...");

  adc_cal_values.offset = avia_hx711_tare(hx711_dev, 100);

  gpio_pin_set_dt(&led_green, 0);
  gpio_pin_set_dt(&led_red, 0);

  LOG_INF("ADC Avg: %7d", adc_cal_values.offset);


  // Save settings to Flash
  settings_save_one("calibration/imu", &imu_cal_values, sizeof(imu_cal_values));
  settings_save_one("calibration/adc", &adc_cal_values, sizeof(adc_cal_values));

  // TODO: Save ADC settings

  k_mutex_unlock(&main_thread_mutex);

  return err;
}


void imu_data_ready_handler(const struct device* dev, const struct sensor_trigger* trigger)
{
  sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
  sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);

  sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

  angular_velocity = (sensor_value_to_double(&gyro_z) - imu_cal_values.gyro_z);
  angular_velocity = -angular_velocity; // TODO: fix in hardware

  pseudo_angle += round(angular_velocity) / 104.0; // TODO: shouldn't be hardcoded
  if (pseudo_angle > 360.0 || pseudo_angle < -360.0)
  {
    last_rev_time_ticks = k_cycle_get_32();
    crank_revolutions += 1;
    pseudo_angle = fmod(pseudo_angle, 360);
  }

  // LOG_INF("Angle:\t\t %3.3f", pseudo_angle);
}


static void sys_poweroff(void)
{
  LOG_WRN("***** Powering System Off *****");
  (void)irq_lock();

  // Make sure battery pin is disabled
  battery_meas_stop();

  // Put ADC to sleep
  // pm_device_state_set(hx711_dev, PM_DEVICE_ACTION_TURN_OFF, NULL, NULL);
  nrf_gpio_cfg_output(2);
  nrf_gpio_pin_set(2);

  // Enable wake-up interupt
  imu_trig.type = SENSOR_TRIG_MOTION;
  sensor_trigger_set(imu_dev, &imu_trig, NULL);

  nrf_gpio_cfg_input(11, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_sense_set(11, NRF_GPIO_PIN_SENSE_HIGH);

  nrf_power_system_off(NRF_POWER);

  CODE_UNREACHABLE;
}


int main(void)
{
  int err;

  LOG_INF("Starting, build time: " __DATE__ " " __TIME__ "\n");


  // IMU init
  if (device_is_ready(imu_dev) == false)
  {
    LOG_ERR("imu sensor: device not ready.\n");
    return 0;
  }

  // ADC front-end init
  if (device_is_ready(hx711_dev) == false)
  {
    LOG_ERR("HX711 sensor: device not ready.\n");
    return 0;
  }


  // Settings init
  err = settings_subsys_init();
  if (err)
  {
    LOG_ERR("Settings init failed (err %d)", err);
  }
  settings_ready();
  hx711_set_offset(hx711_dev, adc_cal_values.offset);

  // BT init
  err = bt_enable(NULL);
  if (err)
  {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 0;
  }
  bt_ready();

  // Calibration routine will be triggered by CPS control point
  // Will prevent xecution of the mian thread
  set_calibration_callback(calibration_routine);

  err = sensor_trigger_set(imu_dev, &imu_trig, imu_data_ready_handler);
  if (err)
  {
    LOG_ERR("Cannot enable IMU trigger");
    return 0;
  }

  LOG_INF("Successful initialisation\n");

  while (1)
  {
    // Race condition with calibration request
    k_mutex_lock(&main_thread_mutex, K_FOREVER);


    // TODO: Auto zero power meter
    err = hx711_read(hx711_dev, &force);
    if (err != 0)
    {
      LOG_ERR("Cannot take measurement: %d  ", err);
      continue;
    };


    force = (double)force / -1767.1;
    // LOG_INF("Force:\t\t%10d", force);

    gpio_pin_set_dt(&led_red, 1);
    gpio_pin_set_dt(&led_blue, 1);

    /*
     * Power calculation
     * v = ω * R
     * P = 2 * F * v   (x2: measuring only left leg so half the power)
     */
    tangential_velocity = round(angular_velocity * M_PI * 100.0 * CRANK_LENTH / 180.0) / 100.0;

    power = 2.0 * force * tangential_velocity;

    power = fabs(power);

    power_uint = (uint16_t)power;

    // LOG_INF("V:\t\t%10.2lf", tangential_velocity);
    // LOG_INF("Power:\t%10.2lf\n", power);

    /*
     * BLE Advertisments
     */
    last_rev_time = (uint16_t)(last_rev_time_ticks / (CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1024u));

    set_power_n_revolution(power_uint, crank_revolutions, last_rev_time);
    // set_power_vector_inst_force((int16_t)force);

    /* Get gyro value */
    // sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    advertise_data(force,
                   sensor_value_to_double(&gyro_x) - imu_cal_values.gyro_x,
                   sensor_value_to_double(&gyro_y) - imu_cal_values.gyro_y,
                   angular_velocity,
                   pseudo_angle * 10,
                   tangential_velocity,
                   power_uint);


    gpio_pin_set_dt(&led_red, 0);
    gpio_pin_set_dt(&led_blue, 0);

    k_mutex_unlock(&main_thread_mutex);

    // Turn off system off, if crank hasn't rotated for 5'
    if (k_cycle_get_32() - last_rev_time_ticks > (5UL * 60UL * CONFIG_SYS_CLOCK_TICKS_PER_SEC))
    {
      sys_poweroff();
    }
  }

  return 0;
}
