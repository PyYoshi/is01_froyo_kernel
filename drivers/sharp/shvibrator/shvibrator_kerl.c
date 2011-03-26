/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/pmic.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <sharp/shterm_k.h>

#define PMIC_VIBRATOR_ON_LEV	(3100)  /* 3000->3100 */
#define PMIC_VIBRATOR_OFF_LEV	(   0)


/* VIB Status */
enum{
	SHVIB_OFF = 0,		/* VIb OFF */
	SHVIB_ON		/* VIb ON */
};

/* Return Value */
#define SHVIB_KERNEL_ERROR	(-1)
#define SHVIB_KERNEL_SUCCSESS	( 0)

/* Kernel Define */
#define SHVIB_KER_BASEMINOR	(0)
#define SHVIB_KER_MINORCOUNT	(1)
#define SHVIB_KER_DRVNAME	"shvibrator"

/* Class Define */
#define SHVIB_CLASS_NAME	"cls_shvib"


/* Error Enum */
enum{
	SHVIB_CHRDEVREG_ERR = 0,
	SHVIB_CDEVADD_ERR,
	SHVIB_CLASSCREATE_ERR,
	SHVIB_DEVCREATE_ERR,
};

typedef struct{
	int major;
	dev_t dev;
	struct cdev vib_cdev;
	struct class *vib_classp;
	int status;
}shvib_info_type;
shvib_info_type shvib_info;

/* Local Function */
static void shvibrator_ker_err( int err );
static void shvibrator_ker_infoinit( void );
static void set_shvibrator(int on);
static int shvibrator_start(struct inode *inode, struct file *filp);
static int shvibrator_stop(struct inode *inode, struct file *filp);

static struct file_operations shvibrator_Ops = {
	.owner = THIS_MODULE,
	.open  = shvibrator_start,
	.release = shvibrator_stop,
};

static void set_shvibrator(int on)
{
	if (on != SHVIB_ON) {
		pmic_vib_mot_set_volt(PMIC_VIBRATOR_OFF_LEV);
		pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
		shterm_k_set_info( SHTERM_INFO_VIB, 0 );
	} else {
		pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_HIGH);
		pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__DBUS1);
		pmic_vib_mot_set_volt(PMIC_VIBRATOR_ON_LEV);
		shterm_k_set_info( SHTERM_INFO_VIB, 1 );
	}
}

static int shvibrator_start(struct inode *inode, struct file *filp)
{
	if( shvib_info.status == SHVIB_OFF )
	{
		set_shvibrator(SHVIB_ON);
		shvib_info.status = SHVIB_ON;
	}
	return (int)SHVIB_KERNEL_SUCCSESS;
}

static int shvibrator_stop(struct inode *inode, struct file *filp)
{
	if( shvib_info.status == SHVIB_ON )
	{
		set_shvibrator(SHVIB_OFF);
		shvib_info.status = SHVIB_OFF;
	}
	return (int)SHVIB_KERNEL_SUCCSESS;
}

static int __init shvibrator_ker_init(void)
{
	int sdResult;
	struct device* devp;

	shvibrator_ker_infoinit();
	
	shvib_info.dev = MKDEV(shvib_info.major, 0);
	
	sdResult = alloc_chrdev_region( &shvib_info.dev, SHVIB_KER_BASEMINOR, SHVIB_KER_MINORCOUNT, SHVIB_KER_DRVNAME );
	if( sdResult < 0 ){
		shvibrator_ker_err( SHVIB_CHRDEVREG_ERR );
		return (int)SHVIB_KERNEL_ERROR;
	}
	shvib_info.major = sdResult;
	

	cdev_init( &shvib_info.vib_cdev, &shvibrator_Ops );
	shvib_info.vib_cdev.owner = THIS_MODULE;
	shvib_info.vib_cdev.ops = &shvibrator_Ops;

	sdResult = cdev_add(&shvib_info.vib_cdev, shvib_info.dev, SHVIB_KER_MINORCOUNT);
	if( sdResult < 0 ){
		shvibrator_ker_err( SHVIB_CDEVADD_ERR );
		return (int)SHVIB_KERNEL_ERROR;
	}
	
	shvib_info.vib_classp = class_create( THIS_MODULE, SHVIB_CLASS_NAME );
	if (IS_ERR(shvib_info.vib_classp)) {
		shvibrator_ker_err( SHVIB_CLASSCREATE_ERR );
		return (int)SHVIB_KERNEL_ERROR;
	}

	devp = device_create( shvib_info.vib_classp, NULL, shvib_info.dev, NULL, SHVIB_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 )
	{
		shvibrator_ker_err( SHVIB_DEVCREATE_ERR );
		return (int)SHVIB_KERNEL_ERROR;
	}
	
	return (int)SHVIB_KERNEL_SUCCSESS;
}

static void __exit shvibrator_ker_term( void )
{
	if( shvib_info.major < 0){
		return;
	}
	
	device_destroy( shvib_info.vib_classp, shvib_info.dev );
	class_destroy( shvib_info.vib_classp );
	shvib_info.vib_classp = NULL;

	cdev_del( &shvib_info.vib_cdev );
	unregister_chrdev_region( shvib_info.dev, SHVIB_KER_MINORCOUNT );
	shvib_info.major = -1;

	return;
}

static void shvibrator_ker_infoinit( void )
{
	shvib_info.major = -1;
	shvib_info.vib_classp = NULL;
	shvib_info.status = SHVIB_OFF;
}

static void shvibrator_ker_err( int err )
{
	switch( err )
	{
		case SHVIB_DEVCREATE_ERR:
			class_destroy( shvib_info.vib_classp );
			shvib_info.vib_classp = NULL;
		case SHVIB_CLASSCREATE_ERR:
		case SHVIB_CDEVADD_ERR:
		case SHVIB_CHRDEVREG_ERR:
			unregister_chrdev_region( shvib_info.dev, SHVIB_KER_MINORCOUNT );
			shvib_info.major = -1;
			break;
		default:
			break;
	}
}

module_init( shvibrator_ker_init );
module_exit( shvibrator_ker_term );

MODULE_DESCRIPTION("sh vibrator device");
MODULE_LICENSE("GPL");

