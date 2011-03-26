/*
 * Copyright (C) 2009 Sharp.
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <mach/sharp_smem.h>


static int smd_support_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;
}

static ssize_t smd_support_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	unsigned long  smem_comm_data;

	printk("%s\n", __func__);

	p_sh_smem_common_type = sh_smem_get_common_address();
	if( p_sh_smem_common_type != NULL){
		smem_comm_data  = p_sh_smem_common_type->support_FlagData;
		
		if( copy_to_user( buf, (void *)&smem_comm_data, sizeof(smem_comm_data) ) ){
			printk( "copy_to_user failed\n" );
			return -EFAULT;
		}
	}
	return count;
}

static ssize_t smd_support_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	printk("%s\n", __func__);
	return count;
}

static int smd_support_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;
}

static struct file_operations smd_support_fops = {
	.owner		= THIS_MODULE,
	.read		= smd_support_read,
	.write		= smd_support_write,
	.open		= smd_support_open,
	.release	= smd_support_release,
};

static struct miscdevice smd_support_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smd_support",
	.fops = &smd_support_fops,
};

static int __init smd_support_init( void )
{
	int ret;

	ret = misc_register(&smd_support_dev);
	if (ret) {
		return ret;
	}
	return 0;
}

module_init(smd_support_init);

MODULE_DESCRIPTION("smd_support");
MODULE_LICENSE("GPL v2");

