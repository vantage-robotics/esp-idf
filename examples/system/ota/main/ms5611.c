/*
 * ms5611.c
 *
 *  Created on: Aug 8, 2018
 *      Author: Xensr
 */
/**
 * @file ms5611.c
 *
 * ESP-IDF driver for barometic pressure sensor MS5611-01BA03
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016 Bernhard Guillon <Bernhard.Guillon@begu.org>
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "ms5611.h"
#include "Wire.h"
#include <stddef.h>
#include <esp_system.h>
#include <esp_log.h>

#define MS5611_I2C_TIMEOUT 2000
#define OSR MS5611_OSR_1024

#define I2C_ADDRESS 0x76
#define CMD_CONVERT_D1 0x40
#define CMD_CONVERT_D2 0x50
#define CMD_ADC_READ   0x00
#define CMD_RESET      0x1E

#define PROM_ADDR_SENS     0xa2
#define PROM_ADDR_OFF      0xa4
#define PROM_ADDR_TCS      0xa6
#define PROM_ADDR_TCO      0xa8
#define PROM_ADDR_T_REF    0xaa
#define PROM_ADDR_TEMPSENS 0xac

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "MS5611";

ms5611_config_data_t config_data;

// Read a specified number of bytes over I2C at a given subAddress
int16_t _i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	int16_t timeout = MS5611_I2C_TIMEOUT;
	beginTransmission(I2C_ADDRESS);
	iwrite(subAddress);
	endTransmission(true);

	requestFrom(I2C_ADDRESS, count, true);
	while ((available() < count) && timeout--)
		vTaskDelay(1 / portTICK_PERIOD_MS);
	if (timeout)
	{
		for (int i=0; i<count; i++)
		{
			dest[i] = iread();
		}
	}

	return timeout?ESP_OK:ESP_ERR_TIMEOUT;
}

// Write a specified number of bytes over I2C to a given subAddress
uint16_t _i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
{
	beginTransmission(I2C_ADDRESS);
	iwrite(subAddress);
	for (int i=0; i<count; i++)
	{
		iwrite(src[i]);
	}
	endTransmission(true);

	return true;
}

static inline esp_err_t send_command(uint8_t cmd)
{
    //return i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, 1);
	_i2cWriteBytes(cmd, NULL, 0);
	return ESP_OK;

}

static inline uint16_t shuffle(uint16_t val)
{
    return ((val & 0xff00) >> 8) | ((val & 0xff) << 8);
}

static inline esp_err_t read_prom()
{
    uint16_t tmp;

    // FIXME calculate CRC (AN502)

    CHECK(_i2cReadBytes(PROM_ADDR_SENS, &tmp, 2));
    config_data.sens = shuffle(tmp);
    CHECK(_i2cReadBytes(PROM_ADDR_OFF, &tmp, 2));
    config_data.off = shuffle(tmp);
    CHECK(_i2cReadBytes(PROM_ADDR_TCS, &tmp, 2));
    config_data.tcs = shuffle(tmp);
    CHECK(_i2cReadBytes(PROM_ADDR_TCO, &tmp, 2));
    config_data.tco = shuffle(tmp);
    CHECK(_i2cReadBytes(PROM_ADDR_T_REF, &tmp, 2));
    config_data.t_ref = shuffle(tmp);
    CHECK(_i2cReadBytes(PROM_ADDR_TEMPSENS, &tmp, 2));
    config_data.tempsens = shuffle(tmp);

    return ESP_OK;
}

static esp_err_t read_adc(uint32_t *result)
{
    uint8_t tmp[3];

    CHECK(_i2cReadBytes(0, tmp, 3));
    *result = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];

    // If we are to fast the ADC will return 0 instead of the actual result
    return *result == 0 ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

static void wait_conversion()
{
    uint32_t us = 8220;
    switch (OSR)
    {
        case MS5611_OSR_256: us = 500; break;   // 0.5ms
        case MS5611_OSR_512: us = 1100; break;  // 1.1ms
        case MS5611_OSR_1024: us = 2100; break; // 2.1ms
        case MS5611_OSR_2048: us = 4100; break; // 4.1ms
        case MS5611_OSR_4096: us = 8220; break; // 8.22ms
    }
    //ESP_LOGI(TAG,"sleeping for:%d ms",us/1000 + 2);

    vTaskDelay( pdMS_TO_TICKS(10));
    //ets_delay_us(us);
}

static inline esp_err_t get_raw_temperature(uint32_t *result)
{
    CHECK(send_command(CMD_CONVERT_D2 + OSR));
    wait_conversion();
    CHECK(read_adc(result));

    return ESP_OK;
}

static inline esp_err_t get_raw_pressure(uint32_t *result)
{
    CHECK(send_command(CMD_CONVERT_D1 + OSR));
    wait_conversion();
    CHECK(read_adc(result));

    return ESP_OK;
}

static esp_err_t ms5611_reset()
{
    send_command(CMD_RESET);

    return ESP_OK;
}

/////////////////////////Public//////////////////////////////////////


esp_err_t ms5611_init()
{
    // First of all we need to reset the chip
    CHECK(ms5611_reset());
    // Wait a bit for the device to reset
    vTaskDelay( pdMS_TO_TICKS(10));
    // Get the config
    CHECK(read_prom());

    return ESP_OK;
}

esp_err_t ms5611_get_sensor_data(int32_t *pressure, float *temperature)
{
    CHECK_ARG(pressure);
    CHECK_ARG(temperature);

    // Second order temperature compensation see datasheet p8
    uint32_t raw_pressure = 0;
    get_raw_pressure(&raw_pressure);

    uint32_t raw_temperature = 0;
    get_raw_temperature(&raw_temperature);

    // dT = D2 - T_ref = D2 - C5 * 2^8
    int32_t dt = raw_temperature - ((int32_t)config_data.t_ref << 8);
    // Actual temerature (-40...85C with 0.01 resulution)
    // TEMP = 20C +dT * TEMPSENSE =2000 + dT * C6 / 2^23
    int64_t temp = 2000 + (int32_t)(((int64_t)dt * config_data.tempsens) >> 23);
    // Offset at actual temperature
    // OFF=OFF_t1 + TCO * dT = OFF_t1(C2) * 2^16 + (C4*dT)/2^7
    int64_t off = (int64_t)((int64_t)config_data.off << 16)
        + (((int64_t)config_data.tco * dt) >> 7);
    // Senisitivity at actual temperature
    // SENS=SENS_t1 + TCS *dT = SENS_t1(C1) *2^15 + (TCS(C3) *dT)/2^8
    int64_t sens = (int64_t)(((int64_t)config_data.sens) << 15)
        + (((int64_t)config_data.tcs * dt) >> 8);

    // Set defaults for temp >= 2000
    int64_t t_2 = 0;
    int64_t off_2 = 0;
    int64_t sens_2 = 0;
    int64_t help = 0;
    if (temp < 2000)
    {
        // Low temperature
        t_2 = ((dt * dt) >> 31); // T2 = dT^2/2^31
        help = (temp - 2000);
        help = 5 * help * help;
        off_2 = help >> 1;       // OFF_2  = 5 * (TEMP - 2000)^2/2^1
        sens_2 = help >> 2;      // SENS_2 = 5 * (TEMP - 2000)^2/2^2
        if (temp < -1500)
        {
            // Very low temperature
            help = (temp + 1500);
            help = help * help;
            off_2 = off_2 + 7 * help;             // OFF_2  = OFF_2 + 7 * (TEMP + 1500)^2
            sens_2 = sens_2 + ((11 * help) >> 1); // SENS_2 = SENS_2 + 7 * (TEMP + 1500)^2/2^1
        }
    }

    temp = temp - t_2;
    off = off - off_2;
    sens = sens - sens_2;

    // Temperature compensated pressure (10...1200mbar with 0.01mbar resolution
    // P = digital pressure value  * SENS - OFF = (D1 * SENS/2^21 -OFF)/2^15
    *pressure = (((raw_pressure * sens) >> 21) - off) >> 15;
    *temperature = (float)temp / 100.0;

    return ESP_OK;
}



