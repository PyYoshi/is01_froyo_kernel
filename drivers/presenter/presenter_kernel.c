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

/* -------------------------------------------------------------------- */
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
#define PRESENTER_KER_BASEMINOR		(0)
#define PRESENTER_KER_MINORCOUNT	(1)
#define PRESENTER_KER_DRVNAME		"presenter"
#define PRESENTER_CLASS_NAME		"cls_presenter"
/* -------------------------------------------------------------------- */
typedef struct{
	int major;
	dev_t dev;
	struct cdev presenter_cdev;
	struct class* presenter_classp;
	int status;
}presenter_info_type;
presenter_info_type presenter_info;
/* -------------------------------------------------------------------- */
extern pid_t sphinx_get_digest_manager_pid(void);
/* -------------------------------------------------------------------- */
static int		presenter_open(struct inode* inode, struct file* filp);
static int		presenter_close(struct inode* inode, struct file* filp);
static ssize_t		presenter_read(struct file* filp, char* buf, size_t count, loff_t* pos);
static ssize_t		presenter_write(struct file* filp, const char* buf, size_t count, loff_t* pos);
static unsigned int	presenter_poll(struct file* filp, poll_table* wait);
/* -------------------------------------------------------------------- */
static struct file_operations presenter_Ops = {
	.owner   = THIS_MODULE,
	.read    = presenter_read,
	.write   = presenter_write,
	.poll    = presenter_poll,
	.open    = presenter_open,
	.release = presenter_close,
};
/* -------------------------------------------------------------------- */
static wait_queue_head_t	presenter_q;
/* -------------------------------------------------------------------- */
void presenter_set_readable(void)
{
	wake_up_interruptible(&presenter_q);
}
/* -------------------------------------------------------------------- */
static int __init presenter_ker_init(void)
{
	int sdResult;
	struct device* devp;

	presenter_info.major = -1;
	presenter_info.presenter_classp = NULL;
	presenter_info.status = 0;
	
	presenter_info.dev = MKDEV(presenter_info.major, 0);
	
	sdResult = alloc_chrdev_region( &presenter_info.dev, PRESENTER_KER_BASEMINOR, PRESENTER_KER_MINORCOUNT, PRESENTER_KER_DRVNAME );
	if( sdResult < 0 ){
		return -1;
	}
	presenter_info.major = sdResult;
	

	cdev_init( &presenter_info.presenter_cdev, &presenter_Ops );
	presenter_info.presenter_cdev.owner = THIS_MODULE;
	presenter_info.presenter_cdev.ops = &presenter_Ops;

	sdResult = cdev_add(&presenter_info.presenter_cdev, presenter_info.dev, PRESENTER_KER_MINORCOUNT);
	if( sdResult < 0 ){
		return -1;
	}

	
	presenter_info.presenter_classp = class_create( THIS_MODULE, PRESENTER_CLASS_NAME );
	if (IS_ERR(presenter_info.presenter_classp)){
		return -1;
	}

	devp = device_create( presenter_info.presenter_classp, NULL, presenter_info.dev, NULL, PRESENTER_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 ){
		return -1;
	}
	
	init_waitqueue_head(&presenter_q);

	return 0;
}
/* -------------------------------------------------------------------- */
static void __exit presenter_ker_term( void )
{
	if( presenter_info.major < 0){
		return;
	}
	
	device_destroy( presenter_info.presenter_classp, presenter_info.dev );
	class_destroy( presenter_info.presenter_classp );
	presenter_info.presenter_classp = NULL;

	cdev_del( &presenter_info.presenter_cdev );
	unregister_chrdev_region( presenter_info.dev, PRESENTER_KER_MINORCOUNT );
	presenter_info.major = -1;

	return;
}
/* -------------------------------------------------------------------- */
/*  */
/* -------------------------------------------------------------------- */
static int presenter_open(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static int presenter_close(struct inode* inode, struct file* filp)
{
	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t presenter_read(struct file* filp, char* buf, size_t count, loff_t* pos)
{
	if(sphinx_get_digest_manager_pid() != current->pid)return 0;

	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t presenter_write(struct file* filp, const char* buf, size_t count, loff_t* pos)
{
	if(sphinx_get_digest_manager_pid() != current->pid)return 0;

	return 0;
}
/* -------------------------------------------------------------------- */
static unsigned int presenter_poll(struct file* filp, poll_table* wait)
{
	unsigned int retmask = 0;

	if(sphinx_get_digest_manager_pid() != current->pid)return 0;

	interruptible_sleep_on(&presenter_q);
	
	retmask |= (POLLIN | POLLRDNORM);

	return retmask;
}
/* -------------------------------------------------------------------- */
module_init( presenter_ker_init );
module_exit( presenter_ker_term );
/* -------------------------------------------------------------------- */
MODULE_AUTHOR("SHARP");
MODULE_DESCRIPTION("presenter device");
MODULE_LICENSE("GPL");
/* -------------------------------------------------------------------- */

