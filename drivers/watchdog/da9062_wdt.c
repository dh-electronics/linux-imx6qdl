// SPDX-License-Identifier: GPL-2.0+
/*
 * Watchdog device driver for DA9062 and DA9061 PMICs
 * Copyright (C) 2015  Dialog Semiconductor Ltd.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mfd/da9062/registers.h>
#include <linux/mfd/da9062/core.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/of.h>

static const unsigned int wdt_timeout[] = { 0, 2, 4, 8, 16, 32, 65, 131 };
#define DA9062_TWDSCALE_DISABLE		0
#define DA9062_TWDSCALE_MIN		1
#define DA9062_TWDSCALE_MAX		(ARRAY_SIZE(wdt_timeout) - 1)
#define DA9062_WDT_MIN_TIMEOUT		wdt_timeout[DA9062_TWDSCALE_MIN]
#define DA9062_WDT_MAX_TIMEOUT		wdt_timeout[DA9062_TWDSCALE_MAX]
#define DA9062_WDG_DEFAULT_TIMEOUT	wdt_timeout[DA9062_TWDSCALE_MAX-1]
#define DA9062_RESET_PROTECTION_MS	300

struct da9062_watchdog {
	struct da9062 *hw;
	struct watchdog_device wdtdev;
	bool use_sw_pm;
	bool wakeup_from_powerdown;
};

static unsigned timeout = 0;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
				__MODULE_STRING(DA9062_WDG_DEFAULT_TIMEOUT) ")");

static DEFINE_MUTEX(da9062_reset_watchdog_mutex);

static unsigned int da9062_wdt_set_hw_mode(struct da9062_watchdog *wdt)
{
	int ret;
	int val = 0;

	if (wdt->wakeup_from_powerdown)
		val |= DA9062AA_WAKE_UP_MASK;

	ret = regmap_write(wdt->hw->regmap,
			   DA9062AA_CONTROL_F,
			   val);

	return ret;
}

static unsigned int da9062_wdt_get_hw_mode(struct da9062_watchdog *wdt)
{
	int ret;
	int val;

	ret = regmap_read(wdt->hw->regmap, DA9062AA_CONTROL_F, &val);
	if (ret == 0) {
		wdt->wakeup_from_powerdown = false;
		if (val & DA9062AA_WAKE_UP_MASK) {
			wdt->wakeup_from_powerdown = true;
			dev_info(wdt->hw->dev, "HW state: WAKE_UP bit is already set\n");
		}
	}

	return ret;
}

static unsigned int da9062_wdt_read_timeout(struct da9062_watchdog *wdt)
{
	unsigned int val;

	regmap_read(wdt->hw->regmap, DA9062AA_CONTROL_D, &val);

	return wdt_timeout[val & DA9062AA_TWDSCALE_MASK];
}

static unsigned int da9062_wdt_timeout_to_sel(unsigned int secs)
{
	unsigned int i;

	for (i = DA9062_TWDSCALE_MIN; i <= DA9062_TWDSCALE_MAX; i++) {
		if (wdt_timeout[i] >= secs)
			return i;
	}

	return DA9062_TWDSCALE_MAX;
}

/* Must be called under da9062_reset_watchdog_mutex */
static int da9062_reset_watchdog_timer(struct da9062_watchdog *wdt)
{
	int ret;
	int val = DA9062AA_WATCHDOG_MASK;

	if (wdt->wakeup_from_powerdown)
		val |= DA9062AA_WAKE_UP_MASK;

	ret = regmap_write(wdt->hw->regmap,
			   DA9062AA_CONTROL_F,
			   val);

	/*
	 * If this reset function is called too fast
	 * one after the other, the watchdog triggers.
	 * So wait here the reset protection time.
	 */
	mdelay(DA9062_RESET_PROTECTION_MS);

	return ret;
}

static int da9062_wdt_update_timeout_register(struct da9062_watchdog *wdt,
					      unsigned int regval)
{
	struct da9062 *chip = wdt->hw;

	regmap_update_bits(chip->regmap,
				  DA9062AA_CONTROL_D,
				  DA9062AA_TWDSCALE_MASK,
				  DA9062_TWDSCALE_DISABLE);

	usleep_range(150, 300);

	da9062_wdt_set_hw_mode(wdt);

	return regmap_update_bits(chip->regmap,
				  DA9062AA_CONTROL_D,
				  DA9062AA_TWDSCALE_MASK,
				  regval);
}

static int da9062_wdt_start(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	selector = da9062_wdt_timeout_to_sel(wdt->wdtdev.timeout);
	ret = da9062_wdt_update_timeout_register(wdt, selector);
	if (ret)
		dev_err(wdt->hw->dev, "Watchdog failed to start (err = %d)\n",
			ret);
	else
		set_bit(WDOG_HW_RUNNING, &wdd->status);

	return ret;
}

static int da9062_wdt_stop(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	int ret;

	if (test_bit(WDOG_HW_ALWAYS_ENABLED, &wdd->status)) {
		/* Keep watchdog hardware enabled (keep alive by kernel) */
		dev_info(wdt->hw->dev, "Enabling the watchdog (keep alive by kernel)\n");
		ret = da9062_wdt_start(wdd);
	} else {
		ret = regmap_update_bits(wdt->hw->regmap,
					 DA9062AA_CONTROL_D,
					 DA9062AA_TWDSCALE_MASK,
					 DA9062_TWDSCALE_DISABLE);
		if (ret)
			dev_err(wdt->hw->dev, "Watchdog failed to stop (err = %d)\n",
				ret);
	}

	return ret;
}

static int da9062_wdt_ping(struct watchdog_device *wdd)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	int ret;

	/*
	 * Prevent pings from occurring late in system poweroff/reboot sequence
	 * and possibly locking out restart handler from accessing i2c bus.
	 */
	if (system_state > SYSTEM_RUNNING)
		return 0;

	mutex_lock(&da9062_reset_watchdog_mutex);
	ret = da9062_reset_watchdog_timer(wdt);
	mutex_unlock(&da9062_reset_watchdog_mutex);
	if (ret)
		dev_err(wdt->hw->dev, "Failed to ping the watchdog (err = %d)\n",
			ret);

	return ret;
}

static int da9062_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	unsigned int selector;
	int ret;

	selector = da9062_wdt_timeout_to_sel(timeout);
	ret = da9062_wdt_update_timeout_register(wdt, selector);
	if (ret)
		dev_err(wdt->hw->dev, "Failed to set watchdog timeout (err = %d)\n",
			ret);
	else
		wdd->timeout = wdt_timeout[selector];

	return ret;
}

static int da9062_wdt_restart(struct watchdog_device *wdd, unsigned long action,
			      void *data)
{
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);
	struct i2c_client *client = to_i2c_client(wdt->hw->dev);
	int ret;

	/* Don't use regmap because it is not atomic safe */
	ret = i2c_smbus_write_byte_data(client, DA9062AA_CONTROL_F,
					DA9062AA_SHUTDOWN_MASK);
	if (ret < 0)
		dev_alert(wdt->hw->dev, "Failed to shutdown (err = %d)\n",
			  ret);

	/* wait for reset to assert... */
	mdelay(500);

	return ret;
}

static const struct watchdog_info da9062_watchdog_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "DA9062 WDT",
};

static const struct watchdog_ops da9062_watchdog_ops = {
	.owner = THIS_MODULE,
	.start = da9062_wdt_start,
	.stop = da9062_wdt_stop,
	.ping = da9062_wdt_ping,
	.set_timeout = da9062_wdt_set_timeout,
	.restart = da9062_wdt_restart,
};

static const struct of_device_id da9062_compatible_id_table[] = {
	{ .compatible = "dlg,da9062-watchdog", },
	{ },
};

MODULE_DEVICE_TABLE(of, da9062_compatible_id_table);

static int da9062_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	unsigned int reg_timeout;
	struct da9062 *chip;
	struct da9062_watchdog *wdt;

	chip = dev_get_drvdata(dev->parent);
	if (!chip)
		return -EINVAL;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->use_sw_pm = device_property_present(dev, "dlg,use-sw-pm");

	wdt->hw = chip;

	wdt->wdtdev.info = &da9062_watchdog_info;
	wdt->wdtdev.ops = &da9062_watchdog_ops;
	wdt->wdtdev.min_timeout = DA9062_WDT_MIN_TIMEOUT;
	wdt->wdtdev.max_timeout = DA9062_WDT_MAX_TIMEOUT;
	wdt->wdtdev.min_hw_heartbeat_ms = DA9062_RESET_PROTECTION_MS;
	wdt->wdtdev.max_hw_heartbeat_ms = DA9062_WDT_MAX_TIMEOUT * 1000;
	wdt->wdtdev.timeout = DA9062_WDG_DEFAULT_TIMEOUT;
	wdt->wdtdev.status = WATCHDOG_NOWAYOUT_INIT_STATUS;
	wdt->wdtdev.parent = dev;

	da9062_wdt_get_hw_mode(wdt);
	if (of_property_read_bool(pdev->dev.of_node, "wakeup-from-powerdown")) {
		wdt->wakeup_from_powerdown = true;
		dev_info(wdt->hw->dev, "Enable wakeup from powerdown\n");
	}

	if (of_property_read_bool(pdev->dev.of_node, "kernel-monitoring-enabled"))
		set_bit(WDOG_HW_ALWAYS_ENABLED, &wdt->wdtdev.status);

	watchdog_stop_on_reboot(&wdt->wdtdev);

	watchdog_set_restart_priority(&wdt->wdtdev, 128);

	watchdog_set_drvdata(&wdt->wdtdev, wdt);
	dev_set_drvdata(dev, &wdt->wdtdev);

	reg_timeout = da9062_wdt_read_timeout(wdt);
	if (reg_timeout)
		wdt->wdtdev.timeout = reg_timeout;

	/* Set timeout from parameter or DT value if available */
	watchdog_init_timeout(&wdt->wdtdev, timeout, dev);

	if (test_bit(WDOG_HW_ALWAYS_ENABLED, &wdt->wdtdev.status)) {
		da9062_wdt_start(&wdt->wdtdev);
		dev_info(wdt->hw->dev, "Enabling the watchdog (keep alive by kernel)\n");
	}

	ret = devm_watchdog_register_device(dev, &wdt->wdtdev);
	if (ret < 0)
		return ret;

	return da9062_wdt_ping(&wdt->wdtdev);
}

static int __maybe_unused da9062_wdt_suspend(struct device *dev)
{
	struct watchdog_device *wdd = dev_get_drvdata(dev);
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);

	if (!wdt->use_sw_pm)
		return 0;

	if (watchdog_active(wdd))
		return da9062_wdt_stop(wdd);

	return 0;
}

static int __maybe_unused da9062_wdt_resume(struct device *dev)
{
	struct watchdog_device *wdd = dev_get_drvdata(dev);
	struct da9062_watchdog *wdt = watchdog_get_drvdata(wdd);

	if (!wdt->use_sw_pm)
		return 0;

	if (watchdog_active(wdd))
		return da9062_wdt_start(wdd);

	return 0;
}

static SIMPLE_DEV_PM_OPS(da9062_wdt_pm_ops,
			 da9062_wdt_suspend, da9062_wdt_resume);

static struct platform_driver da9062_wdt_driver = {
	.probe = da9062_wdt_probe,
	.driver = {
		.name = "da9062-watchdog",
		.pm = &da9062_wdt_pm_ops,
		.of_match_table = da9062_compatible_id_table,
	},
};
module_platform_driver(da9062_wdt_driver);

MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("WDT device driver for Dialog DA9062 and DA9061");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:da9062-watchdog");
