/*
 * Copyright (C) 2010 Sharp.
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


#if 1
struct smem_comm_shdiag {
	unsigned short BootMode;
	unsigned long FlagData;
	unsigned char FirstBoot;
};
#endif

static int smd_shdiag_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;
}


static ssize_t smd_shdiag_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	struct smem_comm_shdiag  smem_comm_data;

	printk("%s\n", __func__);

	p_sh_smem_common_type = sh_smem_get_common_address();
	if( p_sh_smem_common_type != NULL){
		smem_comm_data.BootMode  = p_sh_smem_common_type->shdiag_BootMode;
		smem_comm_data.FlagData  = p_sh_smem_common_type->shdiag_FlagData;
		smem_comm_data.FirstBoot = p_sh_smem_common_type->shdiag_FirstBoot;

		if( copy_to_user( buf, (void *)&smem_comm_data, sizeof(smem_comm_data) ) ){
			printk( "copy_to_user failed\n" );
			return -EFAULT;
		}
	} else {
		printk("[SH_DIAG]smd_shdiag_probe: smem_alloc FAILE\n");
	}
	return count;
}

static ssize_t smd_shdiag_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
	printk("%s\n", __func__);
	return count;
}

static int smd_shdiag_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	return 0;
}

static struct file_operations smd_shdiag_fops = {
	.owner		= THIS_MODULE,
	.read		= smd_shdiag_read,
	.write		= smd_shdiag_write,
	.open		= smd_shdiag_open,
	.release	= smd_shdiag_release,
};

static struct miscdevice smd_shdiag_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smd_shdiag",
	.fops = &smd_shdiag_fops,
};

static int __init smd_shdiag_init( void )
{
	int ret;

	ret = misc_register(&smd_shdiag_dev);
	if (ret) {
		printk("fail to misc_register (smd_shdiag_dev)\n");
		return ret;
	}
	printk("smd_shdiag loaded.\n");
	return 0;
}

module_init(smd_shdiag_init);

MODULE_DESCRIPTION("smd_shdiag");
MODULE_LICENSE("GPL v2");

