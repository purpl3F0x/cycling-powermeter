# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(lsm6ds3tr.c)
zephyr_library_sources(lsm6ds3tr_spi.c)
zephyr_library_sources(lsm6ds3tr_i2c.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3TR_TRIGGER    lsm6ds3tr_trigger.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3TR_SENSORHUB  lsm6ds3tr_shub.c)

zephyr_sources(lsm6ds3tr.c)
zephyr_sources(lsm6ds3tr_spi.c)
zephyr_sources(lsm6ds3tr_i2c.c)

if(DEFINED CONFIG_LSM6DS3TR_TRIGGER)
zephyr_sources(lsm6ds3tr_trigger.c)
endif()

if(DEFINED CONFIG_LSM6DS3TR_SENSORHUB)
zephyr_sources(lsm6ds3tr_shub.c)
endif()


