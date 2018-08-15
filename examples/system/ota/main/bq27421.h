/*
 * bq27421.h
 *
 *  Created on: Aug 9, 2018
 *      Author: Xensr
 */

#ifndef MAIN_BQ27421_H_
#define MAIN_BQ27421_H_

#include "i2cdev.h"

typedef struct {
	i2c_dev_t i2c_dev;
	int charge_design_full;
} bq27421_t;

esp_err_t bq27421_init_desc(bq27421_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
int bq27421_battery_configure_capacity(bq27421_t *dev, uint16_t capacity);


#endif /* MAIN_BQ27421_H_ */
