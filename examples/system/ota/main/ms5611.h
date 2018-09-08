/*
 * ms5611.h
 *
 *  Created on: Aug 8, 2018
 *      Author: Xensr
 */
/**
 * @file ms5611.h
 *
 * ESP-IDF driver for barometic pressure sensor MS5611-01BA03
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MS5611_H__
#define __MS5611_H__

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MS5611_ADDR_CSB_HIGH    0x76
#define MS5611_ADDR_CSB_LOW     0x77

/**
 * Oversampling ratio
 */
typedef enum
{
    MS5611_OSR_256  = 0x00, //!< 256 samples per measurement
    MS5611_OSR_512  = 0x02, //!< 512 samples per measurement
    MS5611_OSR_1024 = 0x04, //!< 1024 samples per measurement
    MS5611_OSR_2048 = 0x06, //!< 2048 samples per measurement
    MS5611_OSR_4096 = 0x08  //!< 4096 samples per measurement
} ms5611_osr_t;

/**
 * Configuration data
 */
typedef struct
{
    uint16_t sens;       //!< C1 Pressure sensitivity                             | SENS_t1
    uint16_t off;        //!< C2 Pressure offset                                  | OFF_t1
    uint16_t tcs;        //!< C3 Temperature coefficient of pressure sensitivity  | TCS
    uint16_t tco;        //!< C4 Temperature coefficient of pressuer offset       | TCO
    uint16_t t_ref;      //!< C5 Reference temperature                            | T_ref
    uint16_t tempsens;   //!< C6 Temperature coefficient of the temperature       | TEMPSENSE
} ms5611_config_data_t;



/**
 * @brief Initialize device descriptior
 * @param dev Device descriptor
 * @param addr I2C address, `MS5611_ADDR_CSB_HIGH` or `MS5611_ADDR_CSB_LOW`
 * @param port I2C port
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */


/**
 * @brief Init MS5611-01BA03
 * Reset device and read calibration data
 * @param dev Device descriptor
 * @param osr Oversampling ratio
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_init(void);

/**
 * @brief Measure pressure and temperature
 * @param dev Device descriptor
 * @param[out] pressure Pressure, Pa
 * @param[out] temperature Temperature, degrees Celsius
 * @return `ESP_OK` on success
 */
esp_err_t ms5611_get_sensor_data(int32_t *pressure, float *temperature);

#ifdef __cplusplus
}
#endif

#endif /* __MS5611_H__ */
