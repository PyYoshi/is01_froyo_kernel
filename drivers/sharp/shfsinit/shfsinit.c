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

#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <mach/sharp_smem.h>

#define FLAG_SIZE 4

static int shfsinit_open( struct inode *inode, struct file *filp )
{
    return 0;
}

static ssize_t shfsinit_read( struct file *filp, char __user *buf,
                              size_t count, loff_t *ppos )
{
    sharp_smem_common_type *ptr = NULL;
    unsigned long init_flag;
    char flag[FLAG_SIZE];
    int size;

    printk( "%s\n", __func__ );

    ptr = sh_smem_get_common_address();

    if( ptr == NULL ){
        count = 0;
        return -EFAULT;
    }

    init_flag = ptr->sh_filesystem_init;
    memset( flag, 0x00, sizeof(flag) );
    size = snprintf( flag, FLAG_SIZE, "%ld", init_flag );

    if( copy_to_user( buf, flag, sizeof(flag) ) ){
        return -EFAULT;
    }

    return size;
}

static ssize_t shfsinit_write( struct file *filp, const char __user *buf,
                               size_t count, loff_t *ppos )
{
    return 0;
}

static int shfsinit_release( struct inode *inode, struct file *filp )
{
    return 0;
}

static struct file_operations shfsinit_fops = {
    .owner      = THIS_MODULE,
    .open       = shfsinit_open,
    .release    = shfsinit_release,
    .read       = shfsinit_read,
    .write      = shfsinit_write,
};

static struct miscdevice shfsinit_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shfsinit",
    .fops = &shfsinit_fops,
};

static int __init shfsinit_init( void )
{
    int ret;

    ret = misc_register( &shfsinit_dev );
    if( ret ){
        printk( "failure %s\n", __func__ );
        return ret;
    }
    printk( "success %s.\n", __func__ );
    return 0;
}

module_init(shfsinit_init);

MODULE_DESCRIPTION("shfsinit");
MODULE_LICENSE("GPL");

