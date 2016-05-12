/*
 * Driver for DH TMG120 Touch Controller
 *
 * Copyright (C) 2011 Avnet, Inc.
 *
 * based on max11801_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* DH TMG120 touch screen controller is a I2C based multiple
 * touch screen controller, it can support 2 point multiple
 * touch. */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#define REPORT_MODE_NONE		0x0
#define REPORT_MODE_SINGLE		0x1
#define REPORT_MODE_MTTOUCH		0x2

#define MAX_SUPPORT_POINTS		1

#define EVENT_MODE			0
#define EVENT_STATUS			1
#define EVENT_ID_OFFSET			2
#define EVENT_ID_MASK			(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE			(0x1 << 1)
#define EVENT_DOWN_UP			(0X1 << 0)

#define MAX_I2C_CMD_LEN			1
#define MAX_I2C_ENABLE_LEN		2
#define MAX_I2C_DATA_LEN		9
#define MAX_I2C_VER_LEN			2

#define TMG120_COMMAND_REG		0x00
#define TMG120_STATUS_REG		0x01
#define TMG120_FW_VERSION_MSB_REG	0x02
#define TMG120_FW_VERSION_LSB_REG	0x03
#define TMG120_TOUCH_DETECT_REG		0x07
#define TMG120_POS_X1_MSB_REG		0x08
#define TMG120_POS_X1_LSB_REG		0x09
#define TMG120_POS_Y1_MSB_REG		0x0A
#define TMG120_POS_Y1_LSB_REG		0x0B
#define TMG120_POS_X2_MSB_REG		0x0C
#define TMG120_POS_X2_LSB_REG		0x0D
#define TMG120_POS_Y2_MSB_REG		0x0E
#define TMG120_POS_Y2_LSB_REG		0x0F

#define TMG120_X_MAX			800
#define TMG120_X_MIN			0
#define TMG120_Y_MAX			480
#define TMG120_Y_MIN			0

int calibration[7];
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

struct tmg120_pointer {
	bool status;
	u16 x;
	u16 y;
};

struct tmg120_ts {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct tmg120_pointer		events[MAX_SUPPORT_POINTS];
};

static int get_calibrated_x_point(int x_raw, int y_raw)
{
	int x_scaled;
	int x_calibrated;
	int x_scale = calibration[0];
        int y_scale = calibration[1];
        int x_offset = calibration[2];
        int divider = calibration[6];

	if (divider == 0)
	{
		// Calibration coefficients have not yet been calibrated,
		// return the raw value.
		return x_raw;
	}
	else
	{
		// The calibrated x coordinate will be calibrated by using the 
		// following formula:
		// x_calibrated = ((x_raw * x_scale) + (y_raw * y_scale) + x_offset) / divider

		x_scaled = (x_raw * x_scale) + (y_raw * y_scale) + x_offset;
		x_calibrated = x_scaled / divider;
	}
	
	return x_calibrated;
}

static int get_calibrated_y_point(int x_raw, int y_raw)
{
	int y_scaled;
	int y_calibrated;
	int x_scale = calibration[3];
        int y_scale = calibration[4];
        int y_offset = calibration[5];
        int divider = calibration[6];

	if (divider == 0)
	{
		// Calibration coefficients have not yet been calibrated,
		// return the raw value.
		return y_raw;
	}
	else
	{
		// The calibrated y coordinate will be calibrated by using the 
		// following formula:
		//
		// y_calibrated = ((x_raw * x_scale) + (y_raw * y_scale) + y_offset) / divider

		y_scaled = (x_raw * x_scale) + (y_raw * y_scale) + y_offset;
		y_calibrated = y_scaled / divider;
	}
	
	return y_calibrated;
}

static irqreturn_t tmg120_ts_interrupt(int irq, void *dev_id)
{
	struct tmg120_ts *data = dev_id;
	struct input_dev *input_dev = data->input_dev;
	struct i2c_client *client = data->client;
	u8 buf[MAX_I2C_DATA_LEN] = { TMG120_TOUCH_DETECT_REG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	int ret, x1, y1, x1_cal, y1_cal;
	bool down;
	u8 state;

retry:
	ret = i2c_master_send(client, buf, MAX_I2C_CMD_LEN);
	if (ret >= 0)
	{
		ret = i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);
		if (ret == -EAGAIN)
			goto retry;

		if (ret < 0)
			return IRQ_HANDLED;
	}

	/* Determine how many touch points are reported by reading
	 * the first byte of the response.
	 */
	state = buf[0];
	//printk(KERN_INFO "TMG120 reported state %d\n", state);

	/* Determine if a valid number of points are being reported.
	 */
	if (state != REPORT_MODE_NONE
	    && state != REPORT_MODE_SINGLE) 
	{
		/* Invalid points reported. */
		return IRQ_HANDLED;
	}

	/* Convert coordinate byte data to word format. */
	x1 = (buf[1] << 8) | buf[2];
	y1 = (buf[3] << 8) | buf[4];
	
	/* Determine calibrated x and y coordinates. */
	x1_cal = get_calibrated_x_point(x1, y1);
	y1_cal = get_calibrated_y_point(x1, y1);
	
	//printk(KERN_INFO "TMG120 calibration fields: %d, %d, %d, %d, %d, %d, %d\n", calibration[0], calibration[1], calibration[2], calibration[3], calibration[4], calibration[5], calibration[6]);
	//printk(KERN_INFO "TMG120 reported %d raw touch points: %d,%d\n", state, x1, y1);
	//printk(KERN_INFO "TMG120 reporting %d calibrated touch points: %d,%d\n", state, x1_cal, y1_cal);

	/* Determine if a down event is being reported or not. */
	if (state == REPORT_MODE_NONE)
	{
		down = 0;
	}
	else
	{
		down = 1;
	}

	/* Currently, the panel Freescale used on QSB _NOT_
	 * support multi point mode.  
	 */

	/* Process a single point touch event. */
	if (down) 
	{
		input_report_abs(input_dev, ABS_X, x1_cal);
		input_report_abs(input_dev, ABS_Y, y1_cal);
		input_event(data->input_dev, EV_KEY, BTN_TOUCH, 1);
		input_report_abs(input_dev, ABS_PRESSURE, 1);
	}
	else
	{
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_PRESSURE, 0);
	}

	input_sync(input_dev);
	return IRQ_HANDLED;
}

static int tmg120_enable(struct i2c_client *client)
{
	u8 buf[MAX_I2C_ENABLE_LEN] = { TMG120_COMMAND_REG, 0x11 };
	int ret;

	ret = i2c_master_send(client, buf, MAX_I2C_ENABLE_LEN);

	if (ret < 0)
	{
		printk(KERN_INFO " Problem Enabling TMG120 Operation: %d\n", ret);
		return ret;
	}

	return 0;
}

static int tmg120_firmware_version(struct i2c_client *client)
{
	u8 buf[MAX_I2C_VER_LEN] = { TMG120_FW_VERSION_MSB_REG, 0x00 };
	int ret;

	ret = i2c_master_send(client, buf, MAX_I2C_CMD_LEN);
	if (ret >= 0)
	{
		ret = i2c_master_recv(client, buf, MAX_I2C_VER_LEN);
		if (ret >= 0)
		{
			printk(KERN_INFO "Found TMG120 Version: 0x%02X 0x%02X\n", buf[0], buf[1]);
		}
	}

	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int tmg120_status(struct i2c_client *client)
{
	u8 buf[MAX_I2C_CMD_LEN] = { TMG120_STATUS_REG };
	int ret;

	ret = i2c_master_send(client, buf, MAX_I2C_CMD_LEN);
	if (ret >= 0)
	{
		ret = i2c_master_recv(client, buf, MAX_I2C_CMD_LEN);
		if (ret >= 0)
		{
			printk(KERN_INFO "TMG120 Status: 0x%02X\n", buf[0]);
		}
	}

	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int tmg120_ts_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct tmg120_ts *data;
	struct input_dev *input_dev;
	int ret;

	data = kzalloc(sizeof(struct tmg120_ts), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_data;
	}

	data->client = client;
	data->input_dev = input_dev;

	ret = tmg120_firmware_version(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"tmg120_ts: failed to read firmware version\n");
		ret = -EIO;
		goto err_free_dev;
	}

	// Used for debug.
	ret = tmg120_status(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"tmg120_ts: failed to read device status\n");
		ret = -EIO;
		goto err_free_dev;
	}

	// Enable interupt functionality of the TMG120.
	ret = tmg120_enable(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"tmg120_ts: failed to enable touch interrupts\n");
		ret = -EIO;
		goto err_free_dev;
	}

	// Used for debug.
	ret = tmg120_status(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"tmg120_ts: failed to read device status\n");
		ret = -EIO;
		goto err_free_dev;
	}

	/* Assign the input device name as tmg120_ts and the physical media
	 * as I2C.
	 */
	input_dev->name = "tmg120_ts";
	input_dev->phys = "I2C",
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_X, TMG120_X_MIN, TMG120_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, TMG120_Y_MIN, TMG120_Y_MAX, 0, 0);

	input_set_drvdata(input_dev, data);

	/* LZ: use falling edge trigger because low-level
	   sensitive trigger needed 10ms to be processed */
	ret = request_threaded_irq(client->irq, NULL, tmg120_ts_interrupt,
				   /* IRQF_TRIGGER_LOW */ IRQF_TRIGGER_FALLING | IRQF_ONESHOT, 
				   "tmg120_ts", data);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_dev;
	}

	ret = input_register_device(data->input_dev);
	if (ret < 0)
		goto err_free_irq;
	i2c_set_clientdata(client, data);
	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_free_dev:
	input_free_device(input_dev);
err_free_data:
	kfree(data);

	return ret;
}

static int tmg120_ts_remove(struct i2c_client *client)
{
	struct tmg120_ts *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);
	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);
	kfree(data);

	return 0;
}

static const struct i2c_device_id tmg120_ts_id[] = {
	{"tmg120_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tmg120_ts_id);

static struct of_device_id tmg120_ts_dt_ids[] = {
	{ .compatible = "dh,tmg120_ts" },
	{ /* sentinel */ }
};

static struct i2c_driver tmg120_ts_driver = {
	.driver = {
		.name = "tmg120_ts",
		.of_match_table	= of_match_ptr(tmg120_ts_dt_ids),
	},
	.id_table	= tmg120_ts_id,
	.probe		= tmg120_ts_probe,
	.remove		= tmg120_ts_remove,
};

static int __init tmg120_ts_init(void)
{
	printk(KERN_INFO "tmg120_ts_init()\n");

	return i2c_add_driver(&tmg120_ts_driver);
}

static void __exit tmg120_ts_exit(void)
{
	printk(KERN_INFO "tmg120_ts_exit()\n");

	i2c_del_driver(&tmg120_ts_driver);
}

module_init(tmg120_ts_init);
module_exit(tmg120_ts_exit);

MODULE_AUTHOR("Avnet, Inc.");
MODULE_DESCRIPTION("Touchscreen driver for DH Electronic tmg120 touch controller");
MODULE_LICENSE("GPL");
