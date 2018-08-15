/*
 * bq27421.c
 *
 *  Created on: Aug 9, 2018
 *      Author: Xensr
 */

/* Defaults, G1A:
 termination: 3.2 V
 design capacity: 1340 mAh

*/
#include <esp_system.h>
#include <esp_log.h>
#include "bq27421.h"

#define I2C_FREQ_HZ 100000 // 100 kHz

static const char *TAG = "bq27";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/* control register params */
#define BQ27XXX_SEALED			0x20
#define BQ27XXX_SET_CFGUPDATE	0x13
#define BQ27XXX_SOFT_RESET		0x42
#define BQ27XXX_RESET			0x41

#define BQ27XXX_RS			(20) /* Resistor sense mOhm */
#define BQ27XXX_POWER_CONSTANT		(29200) /* 29.2 µV^2 * 1000 */
#define BQ27XXX_CURRENT_CONSTANT	(3570) /* 3.57 µV * 1000 */

#define INVALID_REG_ADDR	0xff

enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,	/* Control */
	BQ27XXX_REG_TEMP,	/* Temperature */
	BQ27XXX_REG_INT_TEMP,	/* Internal Temperature */
	BQ27XXX_REG_VOLT,	/* Voltage */
	BQ27XXX_REG_AI,		/* Average Current */
	BQ27XXX_REG_FLAGS,	/* Flags */
	BQ27XXX_REG_TTE,	/* Time-to-Empty */
	BQ27XXX_REG_TTF,	/* Time-to-Full */
	BQ27XXX_REG_TTES,	/* Time-to-Empty Standby */
	BQ27XXX_REG_TTECP,	/* Time-to-Empty at Constant Power */
	BQ27XXX_REG_NAC,	/* Nominal Available Capacity */
	BQ27XXX_REG_FCC,	/* Full Charge Capacity */
	BQ27XXX_REG_CYCT,	/* Cycle Count */
	BQ27XXX_REG_AE,		/* Available Energy */
	BQ27XXX_REG_SOC,	/* State-of-Charge */
	BQ27XXX_REG_DCAP,	/* Design Capacity */
	BQ27XXX_REG_AP,		/* Average Power */
	BQ27XXX_DM_CTRL,	/* Block Data Control */
	BQ27XXX_DM_CLASS,	/* Data Class */
	BQ27XXX_DM_BLOCK,	/* Data Block */
	BQ27XXX_DM_DATA,	/* Block Data */
	BQ27XXX_DM_CKSUM,	/* Block Data Checksum */
	BQ27XXX_REG_MAX,	/* sentinel */
};

#define BQ27XXX_DM_REG_ROWS \
	[BQ27XXX_DM_CTRL] = 0x61,  \
	[BQ27XXX_DM_CLASS] = 0x3e, \
	[BQ27XXX_DM_BLOCK] = 0x3f, \
	[BQ27XXX_DM_DATA] = 0x40,  \
	[BQ27XXX_DM_CKSUM] = 0x60

uint8_t bq27421_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x02,
		[BQ27XXX_REG_INT_TEMP] = 0x1e,
		[BQ27XXX_REG_VOLT] = 0x04,
		[BQ27XXX_REG_AI] = 0x10,
		[BQ27XXX_REG_FLAGS] = 0x06,
		[BQ27XXX_REG_NAC] = 0x08,
		[BQ27XXX_REG_FCC] = 0x0e,
		[BQ27XXX_REG_SOC] = 0x1c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x18,
		BQ27XXX_DM_REG_ROWS,
	};

esp_err_t bq27421_init_desc(bq27421_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = 0x55;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;

    CHECK(i2c_dev_create_mutex(&dev->i2c_dev));

    return ESP_OK;
}
static int bq27421_battery_seal(bq27421_t *dev)
{
	esp_err_t ret;

	uint8_t cmd[] = {BQ27XXX_REG_CTRL, BQ27XXX_SEALED, 0x00};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, sizeof(cmd));

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "bus error on seal: %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27421_battery_unseal(bq27421_t *dev)
{
	esp_err_t ret;

	uint8_t cmd[] = {BQ27XXX_REG_CTRL, 0x00, 0x80};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, sizeof(cmd));

	if (ret != ESP_OK)
		goto out;

	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, sizeof(cmd));

	if (ret != ESP_OK)
		goto out;

	return 0;

out:
	ESP_LOGE(TAG, "bus error on unseal: %d\n", ret);
	return ret;
}

int bq27421_battery_configure_capacity(bq27421_t *dev, uint16_t capacity)
{
	int ret;
	uint8_t flags = 0;
	uint8_t flags_address = 0x06;
	uint8_t design_cap_msb = 0x4a;
	uint8_t design_cap_lsb = 0x4b;
	uint8_t old_cksum = 0;
	uint8_t new_cksum = 0;

	I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

	ret = bq27421_battery_unseal(dev);
	if (ret != ESP_OK)
		goto out;

	uint8_t config_update[] = {BQ27XXX_REG_CTRL, 0x13, 0x00};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &config_update, sizeof(config_update));
	if (ret != ESP_OK)
		goto out;

	// wait for confirmation that we are in CFGUPDATE mode
	do {
		ret = i2c_dev_read(&dev->i2c_dev, &flags_address, 1, &flags, 1);
		ets_delay_us(100000);
	} while ((flags & 0x10) == 0);

	uint8_t block_data_control[] = {bq27421_regs[BQ27XXX_DM_CTRL], 0x00};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &block_data_control, sizeof(block_data_control));
	if (ret != ESP_OK)
		goto out;

	uint8_t state_subclass[] = {bq27421_regs[BQ27XXX_DM_CLASS], 82};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &state_subclass, sizeof(state_subclass));
	if (ret != ESP_OK)
		goto out;

	// 0x00 for offsets 0 to 31, design capacity is at 10
	uint8_t offset_design_capacity[] = {bq27421_regs[BQ27XXX_DM_BLOCK], 0x00};
	ret = i2c_dev_write(&dev->i2c_dev, NULL, 0, &offset_design_capacity, sizeof(offset_design_capacity));
	if (ret != ESP_OK)
		goto out;

	// read checksum
	ret = i2c_dev_read(&dev->i2c_dev, &bq27421_regs[BQ27XXX_DM_CKSUM], 1, &old_cksum, 1);
	if (ret != ESP_OK)
		goto out;

	ret = i2c_dev_read(&dev->i2c_dev, &design_cap_msb, 1, &design_cap_msb, 1);
	ret = i2c_dev_read(&dev->i2c_dev, &design_cap_lsb, 1, &design_cap_lsb, 1);
	ESP_LOGI(TAG, "checksum:%d,%d",design_cap_msb,design_cap_lsb);

	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
return 0;

out:
	I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return -1;

}
