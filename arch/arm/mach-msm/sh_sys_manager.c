/* arch/arm/mach-msm/sh_sys_manager.c
 *
 * Copyright (C) 2009 Sharp Corporation
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
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>

#include <mach/sharp_smem.h>

typedef struct {
	unsigned short sh_hw_revision;
	unsigned short sh_model_type;
} sh_sys_manager_information_t;

static unsigned short sh_hw_revision_for_kernel;
static unsigned short sh_model_type_for_kernel;

static ssize_t sh_sys_manager_model_type_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sh_sys_manager_model_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sh_sys_manager_information_t *p_sh_sys_info = NULL;
	ssize_t result;

	p_sh_sys_info = dev->platform_data;
	
	if(p_sh_sys_info != NULL){
		result = scnprintf(buf, PAGE_SIZE, "%04x\n", p_sh_sys_info->sh_model_type);
	} else{
		return 0;
	}

	return result;
}

static struct device_attribute sh_sys_manager_model_type_attr = {
	.attr = {
		.name = "model_type",
		.mode = S_IRUGO | S_IWUSR,
		.owner = THIS_MODULE },
	.show = sh_sys_manager_model_type_show,
	.store = NULL,
};

static ssize_t sh_sys_manager_revision_show(struct device *dev, struct device_attribute *attr, char *buf);
static int sh_sys_manager_probe(struct platform_device *pdev);
static int sh_sys_manager_remove(struct platform_device *pdev);

static ssize_t sh_sys_manager_revision_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sh_sys_manager_information_t *p_sh_sys_info = NULL;
	ssize_t result;

	p_sh_sys_info = dev->platform_data;
	
	if(p_sh_sys_info != NULL){
		result = scnprintf(buf, PAGE_SIZE, "%04x\n", p_sh_sys_info->sh_hw_revision);
	} else{
		return 0;
	}

	return result;
}

static struct device_attribute sh_sys_manager_attr = {
	.attr = {
		.name = "revision",
		.mode = S_IRUGO | S_IWUSR,
		.owner = THIS_MODULE },
	.show = sh_sys_manager_revision_show,
	.store = NULL,
};

unsigned short SH_GetHardwareRevision( void )
{
  return sh_hw_revision_for_kernel;
}
EXPORT_SYMBOL(SH_GetHardwareRevision);

unsigned short SH_GetModelType( void )
{
  return sh_model_type_for_kernel;
}
EXPORT_SYMBOL(SH_GetModelType);

static int sh_sys_manager_probe(struct platform_device *pdev)
{
	int rc;
	sh_sys_manager_information_t *p_sh_sys_info = NULL;
	sharp_smem_common_type *p_sharp_smem_common_type = NULL;

	p_sh_sys_info = kzalloc(sizeof(sh_sys_manager_information_t), GFP_KERNEL);
	if( p_sh_sys_info != NULL){
		pdev->dev.platform_data = p_sh_sys_info;
	} else {
		printk("[ERROR]sh_sys_manager_probe: kzalloc FAILE\n");
	}


	p_sharp_smem_common_type = sh_smem_get_common_address();
	if( p_sharp_smem_common_type != NULL){
		p_sh_sys_info->sh_hw_revision = p_sharp_smem_common_type->sh_hw_revision;
		p_sh_sys_info->sh_model_type = p_sharp_smem_common_type->sh_model_type;
		sh_hw_revision_for_kernel = p_sharp_smem_common_type->sh_hw_revision;
		sh_model_type_for_kernel = p_sharp_smem_common_type->sh_model_type;
		printk("sh_sys_manager_probe: hw_revision %04x\n", p_sh_sys_info->sh_hw_revision);
		printk("sh_sys_manager_probe: model_type %04x\n", p_sh_sys_info->sh_model_type);
	} else {
		printk("[ERROR]sh_sys_manager_probe: smem_alloc FAILE\n");
	}

	/* create sh_sys_manager attributes */
	rc = device_create_file(&pdev->dev, &sh_sys_manager_attr);
	if(rc)
		return rc;

	/* create sh_sys_model_type_manager attributes */
	rc = device_create_file(&pdev->dev, &sh_sys_manager_model_type_attr);
	if(rc)
		return rc;
		
	return 0;
}

static int sh_sys_manager_remove(struct platform_device *pdev)
{
	kfree(pdev->dev.platform_data);

	return 0;
}

static struct platform_driver sh_sys_manager_driver = {
	.probe	= sh_sys_manager_probe,
	.remove = sh_sys_manager_remove,
	.driver	= {
		.name	= "sh_sys_manager",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device sh_sys_manager_device = {
	.name		= "sh_sys_manager",
	.id		= -1,
	.dev		= {
		.platform_data	= NULL,
	},
};

static int __init sh_sys_manager_init(void)
{
	platform_driver_register(&sh_sys_manager_driver);
	platform_device_register(&sh_sys_manager_device);
	return 0;
}

static void __exit sh_sys_manager_exit(void)
{
	platform_driver_unregister(&sh_sys_manager_driver);
}

module_init(sh_sys_manager_init);
module_exit(sh_sys_manager_exit);

MODULE_DESCRIPTION("SHARP SYSTEM MANAGER");
MODULE_LICENSE("GPL");

