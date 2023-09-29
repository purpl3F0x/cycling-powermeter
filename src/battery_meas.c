/**
 * @file battery_meas.c
 * @author Stavros Avramidis (stavros9899@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2023
 * @copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * @copyright (c) 2019-2020 Nordic Semiconductor ASA
 * 
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BATTERY, CONFIG_ADC_LOG_LEVEL);


#define VBATT       DT_PATH(vbatt)
#define ZEPHYR_USER DT_PATH(zephyr_user)

#define BATTERY_WORK_INTERVAL 5000UL

#ifdef CONFIG_BOARD_THINGY52_NRF52832
/* This board uses a divider that reduces max voltage to
 * reference voltage (600 mV).
 */
#define BATTERY_ADC_GAIN ADC_GAIN_1
#else
/* Other boards may use dividers that only reduce battery voltage to
 * the maximum supported by the hardware (3.6 V)
 */
#define BATTERY_ADC_GAIN ADC_GAIN_1_6
#endif


struct battery_level_point
{
  /** Remaining life at #lvl_mV. */
  uint8_t lvl_pph;
  /** Battery voltage at #lvl_pph remaining life. */
  uint16_t lvl_mV;
};


struct io_channel_config
{
  uint8_t channel;
};


struct divider_config
{
  struct io_channel_config io_channel;
#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  struct gpio_dt_spec power_gpios;
#endif

/* output_ohm is used as a flag value: if it is nonzero then
 * the battery is measured through a voltage divider;
 * otherwise it is assumed to be directly connected to Vdd.
 */
#if (DT_PROP(VBATT, output_ohms) != 0)
  uint32_t output_ohm;
  uint32_t full_ohm;
#endif
};

static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
	.io_channel = {
		DT_IO_CHANNELS_INPUT(VBATT),
	},
#if DT_NODE_HAS_PROP(VBATT, power_gpios)
	.power_gpios = GPIO_DT_SPEC_GET(VBATT, power_gpios),
#endif
#if (DT_PROP(VBATT, output_ohms) != 0)
	.output_ohm = DT_PROP(VBATT, output_ohms),
	.full_ohm = DT_PROP(VBATT, full_ohms),
#endif

#else  /* /vbatt exists */
	.io_channel = {
		DT_IO_CHANNELS_INPUT(ZEPHYR_USER),
	},
#endif /* /vbatt exists */
};


struct divider_data
{
  const struct device*   adc;
  struct adc_channel_cfg adc_cfg;
  struct adc_sequence    adc_seq;
  int16_t                raw;
};
static struct divider_data divider_data = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
  .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(VBATT)),
#else
  .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER)),
#endif
};


static struct k_work_delayable battery_lvl_read;
static struct k_poll_signal    async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event     async_evt =
  K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &async_sig);


static int divider_setup(void)
{
  const struct divider_config*    cfg  = &divider_config;
  const struct io_channel_config* iocp = &cfg->io_channel;
#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  const struct gpio_dt_spec* gcp = &cfg->power_gpios;
#endif
  struct divider_data*    ddp  = &divider_data;
  struct adc_sequence*    asp  = &ddp->adc_seq;
  struct adc_channel_cfg* accp = &ddp->adc_cfg;
  int                     rc;

  if (!device_is_ready(ddp->adc))
  {
    LOG_ERR("ADC device is not ready %s", ddp->adc->name);
    return -ENOENT;
  }

#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  if (!device_is_ready(gcp->port))
  {
    LOG_ERR("%s: device not ready", gcp->port->name);
    return -ENOENT;
  }
  rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
  if (rc != 0)
  {
    LOG_ERR("Failed to control feed %s.%u: %d", gcp->port->name, gcp->pin, rc);
    return rc;
  }
#endif

  *asp = (struct adc_sequence){
    .channels     = BIT(0),
    .buffer       = &ddp->raw,
    .buffer_size  = sizeof(ddp->raw),
    .oversampling = 8,
    .calibrate    = true,
  };

#ifdef CONFIG_ADC_NRFX_SAADC
  *accp = (struct adc_channel_cfg){
    .gain             = BATTERY_ADC_GAIN,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
  };

#if (DT_PROP(VBATT, output_ohms) != 0)
  accp->input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + iocp->channel;
#else
  accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
#endif

  asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

  rc = adc_channel_setup(ddp->adc, accp);
  LOG_INF("Setup AIN%u got %d", iocp->channel, rc);

  return rc;
}

#if DT_NODE_HAS_PROP(VBATT, power_gpios)
static inline int battery_power_pin_enable(const bool enable)
{
  static const struct gpio_dt_spec* gcp = &divider_config.power_gpios;

  return gpio_pin_set_dt(gcp, enable);
}
#endif


static inline uint8_t battery_level_percent(unsigned int batt_mV)
{
  static const struct battery_level_point look_up[] = {
    { 100, 4000 }, { 80, 3850 }, { 50, 3660 }, { 30, 3550 }, { 10, 3225 }, { 5, 3225 }, { 0, 2800 },
  };

  struct battery_level_point const *pa, *pb = look_up;

  if (batt_mV >= pb->lvl_mV)
  {
    /* Measured voltage above highest point, cap at maximum. */
    return pb->lvl_pph;
  }
  /* Go down to the last point at or below the measured voltage. */
  while ((pb->lvl_pph > 0) && (batt_mV < pb->lvl_mV))
  {
    ++pb;
  }
  if (batt_mV < pb->lvl_mV)
  {
    /* Below lowest point, cap at minimum */
    return pb->lvl_pph;
  }

  /* Linear interpolation between below and above points. */
  pa = pb - 1;

  return pb->lvl_pph +
         ((pa->lvl_pph - pb->lvl_pph) * (batt_mV - pb->lvl_mV) / (pa->lvl_mV - pb->lvl_mV));
}


static void battery_sample(struct k_work* work)
{
  int rc = -ENOENT;
  ARG_UNUSED(work);

  struct divider_data*         ddp = &divider_data;
  const struct divider_config* dcp = &divider_config;
  struct adc_sequence*         sp  = &ddp->adc_seq;

#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  battery_power_pin_enable(0);
#endif

  rc = adc_read_async(ddp->adc, sp, &async_sig);
  if (rc == 0)
  {
    rc = k_poll(&async_evt, 1, K_NO_WAIT);
  }

#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  battery_power_pin_enable(1);
#endif

  if (rc)
  {
    LOG_WRN("Battery level poll failed");
    goto exit;
  }


  sp->calibrate = false;
  if (rc == 0)
  {
    int32_t val = ddp->raw;

    adc_raw_to_millivolts(adc_ref_internal(ddp->adc), ddp->adc_cfg.gain, sp->resolution, &val);

#if (DT_PROP(VBATT, output_ohms) != 0)
    rc = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
    LOG_INF("raw %u ~ %u mV => %d mV", ddp->raw, val, rc);
#else
    rc = val;
    LOG_INF("raw %u ~ %u mV", ddp->raw, val);
#endif
    const uint8_t lvl = battery_level_percent(rc);
    bt_bas_set_battery_level(lvl);

    LOG_INF("Battery lvl(%d mV): %d %%\n", rc, lvl);
  }

exit:
  k_work_reschedule(&battery_lvl_read, K_MSEC(BATTERY_WORK_INTERVAL));
}


static int battery_meas_init(void)
{
  int rc = divider_setup();

  LOG_INF("Battery setup: %d", rc);

  k_work_init_delayable(&battery_lvl_read, battery_sample);
  k_work_reschedule(&battery_lvl_read, K_MSEC(BATTERY_WORK_INTERVAL));

  return rc;
}
SYS_INIT(battery_meas_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);


void battery_meas_stop(void)
{
  (void)k_work_cancel_delayable(&battery_lvl_read);

#if DT_NODE_HAS_PROP(VBATT, power_gpios)
  battery_power_pin_enable(1);
#endif
}