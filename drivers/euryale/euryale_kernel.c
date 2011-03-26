/* Copyright (C) 2009 SHARP CORPORATION All rights reserved. This software is licensed under the terms of the GNU General Public License version 2, as published by the Free Software Foundation, and may be copied, distributed, and modified under those terms. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. */
#include <linux/kernel.h>
#include <linux/module.h> 
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
/* -------------------------------------------------------------------- */
#define EURYALE_KER_BASEMINOR		(0)
#define EURYALE_KER_MINORCOUNT		(1)
#define EURYALE_KER_DRVNAME		"euryale"
#define EURYALE_CLASS_NAME		"cls_euryale"
#define EURYALE_SECT_SIZE 		(512)
#define EURYALE_SECT_PER_CLST 		(64*2)
#define euryale_printk			if(0)printk
/* -------------------------------------------------------------------- */
typedef struct{
	int major;
	dev_t dev;
	struct cdev euryale_cdev;
	struct class* euryale_classp;
	int status;
}euryale_info_type;
euryale_info_type euryale_info;
/* -------------------------------------------------------------------- */
typedef struct _euryale_command
{
	int cmd;
	int ret;
	unsigned long sector;
	unsigned long nr;
	char buffer[EURYALE_SECT_SIZE * EURYALE_SECT_PER_CLST];
}euryale_command;
/* -------------------------------------------------------------------- */
static int		euryale_open(struct inode* inode, struct file* filp);
static int		euryale_close(struct inode* inode, struct file* filp);
static ssize_t		euryale_read(struct file* filp, char* buf, size_t count, loff_t* pos);
static ssize_t		euryale_write(struct file* filp, const char* buf, size_t count, loff_t* pos);
/* -------------------------------------------------------------------- */
static struct file_operations euryale_Ops = {
	.owner   = THIS_MODULE,
	.read    = euryale_read,
	.write   = euryale_write,
	.open    = euryale_open,
	.release = euryale_close,
};
/* -------------------------------------------------------------------- */
static struct semaphore 	euryale_sem;
static wait_queue_head_t	euryale_process_q;
static wait_queue_head_t	euryale_read_q;
static wait_queue_head_t	euryale_write_q;
static euryale_command 		euryale_c;
static volatile int		euryale_state = 0;
/* -------------------------------------------------------------------- */
int euryale_write_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer)
{
	int ret;
	unsigned long i;

	euryale_printk("[0] : euryale_write_process start %d , %d\n", (int)sector, (int)current_nr_sectors);

	for(i = 0; i < current_nr_sectors; i += EURYALE_SECT_PER_CLST)
	{
		euryale_printk("[0] : euryale_write_process %d < %d\n", (int)i, (int)current_nr_sectors);

		if(down_interruptible(&euryale_sem))return -1;

		memset(&euryale_c, 0, sizeof(euryale_command));

		euryale_c.cmd = 0;
		euryale_c.sector = sector + i;
		
		if(i + EURYALE_SECT_PER_CLST <= current_nr_sectors)
		{
			euryale_c.nr = EURYALE_SECT_PER_CLST;
		}
		else
		{
			euryale_c.nr = current_nr_sectors - i;
		}
		
		memcpy(euryale_c.buffer, &buffer[i * EURYALE_SECT_SIZE], euryale_c.nr * EURYALE_SECT_SIZE);

		euryale_state = 1;

		up(&euryale_sem);

		euryale_printk("[0] : wake_up_interruptible(&euryale_read_q);\n");

		wake_up_interruptible(&euryale_read_q);

		euryale_printk("[0] : wait_event_interruptible(euryale_process_q, euryale_state == 3);\n");

		wait_event_interruptible(euryale_process_q, euryale_state == 3);

		euryale_printk("[5] : awake\n");

		if(down_interruptible(&euryale_sem))return -1;

		ret = euryale_c.ret;
		euryale_state = 0;

		up(&euryale_sem);

		if(ret != 0)return -1;
	}

	euryale_printk("[6] : euryale_write_process end\n");

	return 0;
}
/* -------------------------------------------------------------------- */
int euryale_read_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer)
{
	int ret;	
	unsigned long i;

	euryale_printk("[1] : euryale_read_process start %d , %d\n", (int)sector, (int)current_nr_sectors);

	for(i = 0; i < current_nr_sectors; i += EURYALE_SECT_PER_CLST)
	{
		euryale_printk("[1] : euryale_read_process %d < %d\n", (int)i, (int)current_nr_sectors);

		if(down_interruptible(&euryale_sem))return -1;

		memset(&euryale_c, 0, sizeof(euryale_command));

		euryale_c.cmd = 1;
		euryale_c.sector = sector + i;
		
		if(i + EURYALE_SECT_PER_CLST <= current_nr_sectors)
		{
			euryale_c.nr = EURYALE_SECT_PER_CLST;
		}
		else
		{
			euryale_c.nr = current_nr_sectors - i;
		}

		euryale_state = 1;

		up(&euryale_sem);

		euryale_printk("[1] : wake_up_interruptible(&euryale_poll_q);\n");

		wake_up_interruptible(&euryale_read_q);

		euryale_printk("[1] : wait_event_interruptible(euryale_process_q, euryale_state == 3);\n");

		wait_event_interruptible(euryale_process_q, euryale_state == 3);

		euryale_printk("[7] : awake\n");

		if(down_interruptible(&euryale_sem))return -1;

		memcpy(&buffer[i * EURYALE_SECT_SIZE], euryale_c.buffer, euryale_c.nr * EURYALE_SECT_SIZE);
		
		ret = euryale_c.ret;
		euryale_state = 0;

		up(&euryale_sem);

		if(ret != 0)return -1;
	}

	euryale_printk("[8] : euryale_read_process end\n");

	return 0;
}
/* -------------------------------------------------------------------- */
static int __init euryale_ker_init(void)
{
	int sdResult;
	struct device* devp;

	euryale_info.major = -1;
	euryale_info.euryale_classp = NULL;
	euryale_info.status = 0;
	
	euryale_info.dev = MKDEV(euryale_info.major, 0);
	
	sdResult = alloc_chrdev_region( &euryale_info.dev, EURYALE_KER_BASEMINOR, EURYALE_KER_MINORCOUNT, EURYALE_KER_DRVNAME );
	if( sdResult < 0 ){
		return -1;
	}
	euryale_info.major = sdResult;
	

	cdev_init( &euryale_info.euryale_cdev, &euryale_Ops );
	euryale_info.euryale_cdev.owner = THIS_MODULE;
	euryale_info.euryale_cdev.ops = &euryale_Ops;

	sdResult = cdev_add(&euryale_info.euryale_cdev, euryale_info.dev, EURYALE_KER_MINORCOUNT);
	if( sdResult < 0 ){
		return -1;
	}

	
	euryale_info.euryale_classp = class_create( THIS_MODULE, EURYALE_CLASS_NAME );
	if (IS_ERR(euryale_info.euryale_classp)){
		return -1;
	}

	devp = device_create( euryale_info.euryale_classp, NULL, euryale_info.dev, NULL, EURYALE_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 ){
		return -1;
	}
	
	sema_init(&euryale_sem, 1);

	init_waitqueue_head(&euryale_process_q);
	init_waitqueue_head(&euryale_read_q);
	init_waitqueue_head(&euryale_write_q);

	return 0;
}
/* -------------------------------------------------------------------- */
static void __exit euryale_ker_term( void )
{
	if( euryale_info.major < 0){
		return;
	}
	
	device_destroy( euryale_info.euryale_classp, euryale_info.dev );
	class_destroy( euryale_info.euryale_classp );
	euryale_info.euryale_classp = NULL;

	cdev_del( &euryale_info.euryale_cdev );
	unregister_chrdev_region( euryale_info.dev, EURYALE_KER_MINORCOUNT );
	euryale_info.major = -1;

	return;
}
/* -------------------------------------------------------------------- */
/*  */
/* -------------------------------------------------------------------- */
static int euryale_open(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static int euryale_close(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t euryale_read(struct file* filp, char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;
	int r;

	euryale_printk("[3] : euryale_read\n");

	r = wait_event_interruptible(euryale_read_q, euryale_state == 1);

	if(r != 0)
	{
		return 0;
	}

	euryale_printk("[3] : awake\n");
	euryale_printk("[3] : %02x%02x%02x%02x%02x%02x%02x%02x\n", euryale_c.buffer[0], euryale_c.buffer[1], euryale_c.buffer[2], euryale_c.buffer[3], euryale_c.buffer[4], euryale_c.buffer[5], euryale_c.buffer[6], euryale_c.buffer[7]);

	do
	{
		if(down_interruptible(&euryale_sem))return 0;

		if(copy_to_user(buf, &euryale_c, sizeof(euryale_command)))break;

		ret = sizeof(euryale_command);
	}
	while(0);

	euryale_state = 2;

	up(&euryale_sem);

	euryale_printk("[3] : wake_up_interruptible(&euryale_write_q);\n");

	wake_up_interruptible(&euryale_write_q);

	return ret;
}
/* -------------------------------------------------------------------- */
static ssize_t euryale_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	ssize_t ret = 0;

	euryale_printk("[4] : euryale_write\n");

	wait_event_interruptible(euryale_write_q, euryale_state == 2);

	euryale_printk("[4] : awake\n");

	do
	{
		if(down_interruptible(&euryale_sem))return 0;

		if(copy_from_user(&euryale_c, buf, sizeof(euryale_command)))break;

		ret = sizeof(euryale_command);
	}
	while(0);

	euryale_state = 3;

	up(&euryale_sem);

	euryale_printk("[4] : wake_up_interruptible(&euryale_rw_q);\n");

	wake_up_interruptible(&euryale_process_q);

	return ret;
}
/* -------------------------------------------------------------------- */
module_init( euryale_ker_init );
module_exit( euryale_ker_term );
/* -------------------------------------------------------------------- */
MODULE_AUTHOR("SHARP");
MODULE_DESCRIPTION("euryale device");
MODULE_LICENSE("GPL");
/* -------------------------------------------------------------------- */

