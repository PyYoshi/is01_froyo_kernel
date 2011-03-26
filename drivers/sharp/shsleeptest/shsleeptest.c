/* drivers/sharp/shsleeptest/shsleeptest.c
 *
 * Copyright (C) 2010 Sharp Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <mach/sharp_smem.h>
#include <sharp/shsleeptest.h>

static int *smem_sleep_test_mode;
static int sleep_test_mode;

static void init_sleep_test_mode(void)
{
	sharp_smem_common_type *sh_smem;

	/* Allocate SMEM */
	sh_smem = sh_smem_get_common_address();
	if (sh_smem != NULL) {
		smem_sleep_test_mode = &sh_smem->sh_sleep_test_mode;
	}
}

int sleep_test_is_enabled(void)
{
	return sleep_test_mode;
}

void notify_sleep_test_mode_to_modem(void)
{
	if (smem_sleep_test_mode != NULL) {
		*smem_sleep_test_mode = sleep_test_mode;
	}
}

static ssize_t show_sleep_test_mode(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	int len;

	len = scnprintf(buf, PAGE_SIZE, "%d\n", sleep_test_mode);

	return len;
}

static ssize_t store_sleep_test_mode(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	long new_mode;

	new_mode = simple_strtol(buf, NULL, 16);
	sleep_test_mode = (new_mode) ? 1 : 0;

	return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_sleep_test_mode, store_sleep_test_mode);

static struct attribute *shsleeptest_attrs[] = {
	&dev_attr_mode.attr,
	NULL,
};

static struct attribute_group shsleeptest_attr_group = {
	.attrs = shsleeptest_attrs,
};

static int shsleeptest_probe(struct platform_device *pdev)
{
	int err;

	init_sleep_test_mode();

	err = sysfs_create_group(&pdev->dev.kobj, &shsleeptest_attr_group);
	if (err) {
		return err;
	}

	return 0;
}

static int shsleeptest_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &shsleeptest_attr_group);

	return 0;
}

static struct platform_device shsleeptest_device = {
	.name = "shsleeptest",
	.id   = -1,
};

static struct platform_driver shsleeptest_driver = {
	.probe	      = shsleeptest_probe,
	.remove	      = shsleeptest_remove,
	.driver	      = {
		.name = "shsleeptest",
	},
};

static int __init shsleeptest_init(void)
{
	platform_device_register(&shsleeptest_device);
	platform_driver_register(&shsleeptest_driver);

	return 0;
}

static void __exit shsleeptest_exit(void)
{
	platform_driver_unregister(&shsleeptest_driver);
	platform_device_unregister(&shsleeptest_device);
}

module_init(shsleeptest_init);
module_exit(shsleeptest_exit);

EXPORT_SYMBOL(sleep_test_is_enabled);
EXPORT_SYMBOL(notify_sleep_test_mode_to_modem);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sharp Corporation");
