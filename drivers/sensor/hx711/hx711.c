/**
 * @file hx711.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-24
 *
 * @copyright Copyright (c) 2023
 * @copyright (c) 2020 George Gkinis
 * @copyright (c) 2021 Jan Gnip
 * 
 */

#define DT_DRV_COMPAT avia_hx711

#include "hx711.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>

struct hx711_data
{
  struct gpio_callback dout_gpio_cb;
  struct k_sem         dout_sem;

  const struct hx711_config* cfg;

  int32_t reading;

  int32_t         offset;
  double          slope;
  int             sample_fetch_timeout;
  char            gain;
  enum hx711_rate rate;
};

struct hx711_config
{
  int instance;

  struct gpio_dt_spec dout;
  struct gpio_dt_spec sck;
  struct gpio_dt_spec rate;
};


LOG_MODULE_REGISTER(HX711, CONFIG_SENSOR_LOG_LEVEL);

static void hx711_gpio_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
  struct hx711_data*         data = CONTAINER_OF(cb, struct hx711_data, dout_gpio_cb);
  const struct hx711_config* cfg  = data->cfg;

  gpio_pin_interrupt_configure_dt(&cfg->dout, GPIO_INT_DISABLE);

  /* Signal thread that data is now ready */
  k_sem_give(&data->dout_sem);
}


/**
 * @brief Send a pulse on the SCK pin.
 *
 * @param dev Pointer to the hx711 device structure
 * @return int Pointer to the hx711 device structure
 */
static int hx711_cycle(const struct device* dev)
{
  const struct hx711_config* cfg = dev->config;

  /* SCK set HIGH */
  gpio_pin_set_dt(&cfg->sck, true);
  /* From the datasheet page 5 : PD_SCK high time = PD_SCK low time = 1μs (TYP) */
  k_busy_wait(1);

  /* SCK set LOW */
  gpio_pin_set_dt(&cfg->sck, false);
  /* From the datasheet page 5 : PD_SCK high time = PD_SCK low time = 1μs (TYP) */
  k_busy_wait(1);

  /* Return DOUT pin state */
  return gpio_pin_get_dt(&cfg->dout);
}


static int hx711_sample_fetch(const struct device* dev)
{

  int      ret   = 0;
  uint32_t count = 0;
  int      i;

  struct hx711_data*         data = dev->data;
  const struct hx711_config* cfg  = dev->config;

  ret = pm_device_runtime_get(dev);
  if (ret < 0)
  {
    return ret;
  }

  if (k_sem_take(&data->dout_sem, K_MSEC(data->sample_fetch_timeout)))
  {
    LOG_ERR("Weight data not ready within %d ms", data->sample_fetch_timeout);
    gpio_pin_interrupt_configure_dt(&cfg->dout, GPIO_INT_EDGE_TO_INACTIVE);
    return -EIO;
  }

  /* Clock data out.
   * HX711 is a 24bit ADC.
   * Clock out 24 bits.
   */

  unsigned key = irq_lock();
  for (i = 0; i < 24; i++)
  {
    count = count << 1;
    if (hx711_cycle(dev))
    {
      count++;
    }
  }

  /* set GAIN for next read */
  for (i = 0; i < data->gain; i++)
  {
    hx711_cycle(dev);
  }
  irq_unlock(key);


  /* Add padding to 24bit value to make a 32bit one. */
  count ^= 0x800000;

  data->reading = count;

  LOG_DBG("Raw reading : %d", data->reading);

  ret = gpio_pin_interrupt_configure_dt(&cfg->dout, GPIO_INT_EDGE_TO_INACTIVE);
  if (ret != 0)
  {
    LOG_ERR("Failed to set dout GPIO interrupt");
    return ret;
  }

  return pm_device_runtime_put(dev);
}


int hx711_read(const struct device* dev, int32_t* const reading)
{
  const struct hx711_data* data = dev->data;
  int                      err;

  err = hx711_sample_fetch(dev);
  if (err == 0)
  {
    *reading = (data->reading - data->offset);
  }

  return err;
}

void hx711_set_offset(const struct device* dev, const int offset)
{
  struct hx711_data* const data = dev->data;

  data->offset = offset;
}


/**
 * @brief Initialise HX711.
 *
 * @param dev Pointer to the hx711 device structure
 *
 * @retval 0 on success
 * @retval -EINVAL if an invalid argument is given
 *
 */
static int hx711_init(const struct device* dev)
{
  LOG_DBG("Initialising HX711");

  int                        ret  = 0;
  struct hx711_data*         data = dev->data;
  const struct hx711_config* cfg  = dev->config;

  /* enable device runtime power management */
  ret = pm_device_runtime_enable(dev);
  if ((ret < 0) && (ret != -ENOSYS))
  {
    return ret;
  }

  LOG_DBG("HX711 Instance: %d", cfg->instance);
  LOG_DBG("SCK GPIO port : %s", cfg->sck.port->name);
  LOG_DBG("SCK Pin : %d", cfg->sck.pin);
  LOG_DBG("DOUT GPIO port : %s", cfg->dout.port->name);
  LOG_DBG("DOUT Pin : %d", cfg->dout.pin);

  if (cfg->rate.port != NULL)
  {
    LOG_DBG("RATE GPIO port : %s", cfg->rate.port->name);
    LOG_DBG("RATE Pin : %d", cfg->rate.pin);
  }

  LOG_DBG("Gain : %d", data->gain);

  /* Configure SCK as output, LOW */
  LOG_DBG("SCK pin controller name is %s", cfg->sck.port->name);

  ret = gpio_pin_configure_dt(&cfg->sck, GPIO_OUTPUT_INACTIVE | cfg->sck.dt_flags);
  if (ret != 0)
  {
    return ret;
  }

  if (cfg->rate.port != NULL)
  {
    /* Configure RATE as output, LOW */
    LOG_DBG("RATE pin controller name is %s", cfg->rate.port->name);
    ret = gpio_pin_configure_dt(&cfg->rate, GPIO_OUTPUT_INACTIVE | cfg->rate.dt_flags);
    if (ret != 0)
    {
      return ret;
    }

    ret = gpio_pin_set_dt(&cfg->rate, data->rate);
    if (ret != 0)
    {
      return ret;
    }
  }

  /* Configure DOUT as input */
  LOG_DBG("DOUT pin controller name is %s", cfg->dout.port->name);
  ret = gpio_pin_configure_dt(&cfg->dout, GPIO_INPUT | cfg->dout.dt_flags);
  if (ret != 0)
  {
    return ret;
  }
  LOG_DBG("Set DOUT pin : %d", cfg->dout.pin);

  /* Pointer to cfg is needed within hx711_gpio_callback. */
  data->cfg = cfg;

  k_sem_init(&data->dout_sem, 1, 1);
  gpio_init_callback(&data->dout_gpio_cb, hx711_gpio_callback, BIT(cfg->dout.pin));

  if (gpio_add_callback(cfg->dout.port, &data->dout_gpio_cb) < 0)
  {
    LOG_DBG("Failed to set GPIO callback");
    return -EIO;
  }

  /* Get a reading to set GAIN */
  ret = hx711_sample_fetch(dev);

  return ret;
}

/**
 * @brief Zero the HX711.
 *
 * @param dev Pointer to the hx711 device structure
 * @param readings Number of readings to get average offset.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The offset value
 *
 */
int avia_hx711_tare(const struct device* dev, const uint8_t readings)
{
  int32_t            avg  = 0;
  struct hx711_data* data = dev->data;

  for (int i = 0; i < readings; i++)
  {
    hx711_sample_fetch(dev);
    avg += data->reading;
  }

  LOG_DBG("Average before division : %d", avg);
  avg = avg / readings;
  LOG_DBG("Average after division : %d", avg);
  data->offset = avg;

  LOG_DBG("Offset set to %d", data->offset);

  return data->offset;
}


#ifdef CONFIG_PM_DEVICE
/**
 * @brief Set the Device Power Management State.
 *
 * @param dev - The device structure.
 * @param action - power management state
 * @retval 0 on success
 * @retval -ENOTSUP if an unsupported action is given
 *
 */
static int hx711_pm_action(const struct device* dev, enum pm_device_action action)
{
  int                        ret;
  const struct hx711_config* cfg = dev->config;

  switch (action)
  {
    case PM_DEVICE_ACTION_RESUME:
      ret = gpio_pin_set_dt(&cfg->sck, 0);
      if (ret < 0)
      {
        return ret;
      }
      /* Fetch a sample to set GAIN again.
       * GAIN is set to 128 channel A after RESET
       */
      LOG_DBG("Setting GAIN. Ignore the next measurement.");
      hx711_sample_fetch(dev);
      break;
    case PM_DEVICE_ACTION_TURN_OFF:
    case PM_DEVICE_ACTION_SUSPEND:
      return gpio_pin_set_dt(&cfg->sck, 1);
    default:
      return -ENOTSUP;
  }
  return 0;
}
#endif /* CONFIG_PM_DEVICE */


#define HX711_INIT(index)                                                                          \
  static struct hx711_data hx711_data_##index = {                                                  \
    .reading              = 0,                                                                     \
    .sample_fetch_timeout = DT_INST_PROP(index, sample_fetch_timeout_ms),                          \
    .gain                 = DT_INST_PROP(index, gain),                                             \
    .rate                 = DT_INST_PROP_OR(index, rate_hz, HX711_RATE_10HZ),                      \
    .slope                = 1.0,                                                                   \
  };                                                                                               \
  static const struct hx711_config hx711_config_##index = {                                        \
    .instance = index,                                                                             \
    .dout     = GPIO_DT_SPEC_INST_GET(index, dout_gpios),                                          \
    .sck      = GPIO_DT_SPEC_INST_GET(index, sck_gpios),                                           \
    .rate     = GPIO_DT_SPEC_INST_GET_OR(index, rate_gpios, {})                                    \
  };                                                                                               \
                                                                                                   \
  PM_DEVICE_DT_INST_DEFINE(index, hx711_pm_action);                                                \
                                                                                                   \
  DEVICE_DT_INST_DEFINE(index,                                                                     \
                        hx711_init,                                                                \
                        PM_DEVICE_DT_INST_GET(index),                                              \
                        &hx711_data_##index,                                                       \
                        &hx711_config_##index,                                                     \
                        POST_KERNEL,                                                               \
                        CONFIG_SENSOR_INIT_PRIORITY,                                               \
                        NULL);

DT_INST_FOREACH_STATUS_OKAY(HX711_INIT)