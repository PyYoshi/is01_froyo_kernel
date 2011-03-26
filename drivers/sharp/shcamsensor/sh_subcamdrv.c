/*
 * Copyright (C) 2009-2010 SHARP CORPORATION All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>

#include <mach/gpio.h>
#include <linux/delay.h>
#include <asm/io.h>

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <media/msm_camera.h>
#include <mach/camera.h>

#include "sh_subcamdrv.h"

#include <sharp/shlcdc_kerl.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <asm/pgtable.h>

#define SH_SUBCAM_GPIO_INT				0
#define SH_SUBCAM_GPIO_VSYNC			1
#define SH_SUBCAM_GPIO_AVDD_EN			2
#define SH_SUBCAM_GPIO_MCAM_RST_N		3
#define SH_SUBCAM_GPIO_DATA_0			4
#define SH_SUBCAM_GPIO_DATA_1			5
#define SH_SUBCAM_GPIO_DATA_2			6
#define SH_SUBCAM_GPIO_DATA_3			7
#define SH_SUBCAM_GPIO_DATA_4			8
#define SH_SUBCAM_GPIO_DATA_5			9
#define SH_SUBCAM_GPIO_DATA_6			10
#define SH_SUBCAM_GPIO_DATA_7			11
#define SH_SUBCAM_GPIO_PCLK				12
#define SH_SUBCAM_GPIO_HSYNC			13
#define SH_SUBCAM_GPIO_VSYNC_B			14
#define SH_SUBCAM_GPIO_MCLK				15
#define SH_SUBCAM_GPIO_AFVDD30_EN		16

#define SH_SUBCAM_MINORNUM_BASE			0
#define SH_SUBCAM_DEVICE_COUNT			(1)
#define SH_SUBCAM_DEVICE_NAME			"shsubcam"
#define SH_SUBCAM_NODE_NAME				"shsubcam"
#define SH_SUBCAM_CLASS_NAME			"cls_shsubcam"
#define SH_SUBCAM_IRQ_NAME				"irq_shsubcam"

#define SH_SUBCAM_HCS1_ADDR			0x60000000
#define SH_SUBCAM_HCS1_SIZE			0x1000
#define SH_SUBCAM_HCS2_ADDR			0x94000000
#define SH_SUBCAM_HCS2_SIZE			0x1000

static struct class      *sh_subcam_class;
static dev_t              sh_subcam_devno;
static struct device     *sh_subcam_class_dev;

static int shscamdrv_irq_port = 0;

struct shlcdc_subscribe sh_subcam_gol_csiint_subscribe;

struct sh_subcam_sync_head_t {
	struct file			*file;
	struct cdev			cdev;
	spinlock_t			irq_event_q_lock;
	struct list_head	irq_event_q;
	uint32				irq_event_count;
	wait_queue_head_t	irq_event_wait;
	struct wake_lock	wake_lock;
	struct mutex		mut_lock;
};
static struct sh_subcam_sync_head_t *sh_subcam_sync_head = 0;

struct sh_subcam_irq_msg_t {
	struct list_head	list;
	uint16				irq_number;
};

static int sh_subcamera_node = 0;

static irqreturn_t sh_subcam_drv_gpio_vsync_isr( int irq_num, void *data );
static int sh_subcam_drv_get_irq_num( void __user *argp );
static int sh_subcam_drv_enable_irq( void __user *argp );
static int sh_subcam_drv_disable_irq( void __user *argp );
static int sh_subcam_drv_camif_pad_reg_reset( void __user *argp );
static int sh_subcam_drv_sensor_init( void );
static int sh_subcam_drv_sensor_off( void );
static int sh_subcam_sys_open(struct inode *inode, struct file *filep);
static int sh_subcam_sys_release(struct inode *node, struct file *filep);
static long sh_subcam_sys_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static unsigned int sh_subcam_sys_poll(struct file *filep, struct poll_table_struct *pll_table);
static int sh_subcam_drv_sync_init( struct sh_subcam_sync_head_t *sync );
static int sh_subcam_drv_sync_destroy( struct sh_subcam_sync_head_t *sync );
static void sh_subcam_drv_irq_msg_free( struct sh_subcam_sync_head_t *sync );
static int __init sh_subcam_drv_start( void );
static void __exit sh_subcam_drv_remove( void );

static void sh_subcamdrv_mipi_err_isr( void );
static int sh_subcamdrv_CSI_request_irq( void );
static int sh_subcamdrv_CSI_free_irq( void );

static int sh_subcamdrv_vs_request_irq( void );
static int sh_subcamdrv_vs_free_irq( void );
static int sh_subcamdrv_sys_mmap(struct file *filep, struct vm_area_struct *vma);

static int sh_subcam_drv_timeout_irq_num( void __user *argp );

static irqreturn_t sh_subcam_drv_gpio_vsync_isr( int irq_num, void *data )
{
	irqreturn_t rc = IRQ_HANDLED;
	unsigned long flags = 0;
	struct sh_subcam_irq_msg_t *qmsg = NULL;

	do{
		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags);

		qmsg = kzalloc( sizeof(struct sh_subcam_irq_msg_t ),GFP_KERNEL );
		if ( !qmsg ) {
			spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );
			break;
		}

		qmsg->irq_number = (uint16)CAM_INT_TYPE_VS;

		list_add_tail( &qmsg->list, &sh_subcam_sync_head->irq_event_q );

		sh_subcam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

		wake_up( &sh_subcam_sync_head->irq_event_wait );

	}while(0);

	return rc;
}

static void sh_subcamdrv_mipi_err_isr( void )
{
	unsigned long flags = 0;
	struct sh_subcam_irq_msg_t *qmsg = NULL;

	do{
		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags);

		qmsg = kzalloc( sizeof(struct sh_subcam_irq_msg_t ),GFP_KERNEL );
		if ( !qmsg ) {
			spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );
			break;
		}

		qmsg->irq_number = (uint16)SHSCAM_INT_TYPE_MIPI_ERR;

		list_add_tail( &qmsg->list, &sh_subcam_sync_head->irq_event_q );

		sh_subcam_sync_head->irq_event_count++;

		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

		wake_up( &sh_subcam_sync_head->irq_event_wait );

	}while(0);

}

static int sh_subcam_drv_get_irq_num( void __user *argp )
{
	int ret = 0;
	struct sh_subcam_ctrl_cmd param;
	unsigned long flags = 0;
	struct sh_subcam_irq_msg_t *qmsg = NULL;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;
		param.irq_number  = 0;

		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );

		if ( !list_empty( &sh_subcam_sync_head->irq_event_q ) ){
			qmsg = list_first_entry(&sh_subcam_sync_head->irq_event_q, struct sh_subcam_irq_msg_t, list);
			list_del(&qmsg->list);
			param.irq_number = qmsg->irq_number;
			sh_subcam_sync_head->irq_event_count--;
			kfree(qmsg);
		}
		else{
			param.status = -1;
		}

		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_subcam_drv_timeout_irq_num( void __user *argp )
{
	int ret = 0;
	struct sh_subcam_ctrl_cmd param;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;
		param.irq_number  = (uint16)CAM_INT_TYPE_TIMEOUT;;

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_subcam_drv_enable_irq( void __user *argp )
{
	int ret = 0;
	struct sh_subcam_ctrl_cmd param;
	unsigned long flags = 0;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );

		enable_irq(shscamdrv_irq_port);

		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_subcam_drv_disable_irq( void __user *argp )
{
	int ret = 0;
	struct sh_subcam_ctrl_cmd param;
	unsigned long flags = 0;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );

		disable_irq(shscamdrv_irq_port);

		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_subcam_drv_camif_pad_reg_reset( void __user *argp )
{
	int ret = 0;
	struct sh_subcam_ctrl_cmd param;

	do {
		if (copy_from_user(&param,
							argp,
							sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

		param.status = 0;

		msm_camio_camif_pad_reg_reset();
		udelay(1500);

		if (copy_to_user((void *)argp,
								&param,
								sizeof(struct sh_subcam_ctrl_cmd))) {
			ret = -EFAULT;
			break;
		}

	}while(0);

	return ret;
}

static int sh_subcamdrv_CSI_request_irq( void )
{
	uint16 ret = 0;

	mutex_lock( &sh_subcam_sync_head->mut_lock );
    
	do {

		sh_subcam_gol_csiint_subscribe.event_type = SHLCDC_EVENT_TYPE_CSI;
		sh_subcam_gol_csiint_subscribe.callback   = sh_subcamdrv_mipi_err_isr;
		ret = shlcdc_api_event_subscribe(&sh_subcam_gol_csiint_subscribe);

		if(ret != SHLCDC_RESULT_SUCCESS) {
			break;
    	}

	}while(0);

	mutex_unlock( &sh_subcam_sync_head->mut_lock );

	return ret;
}

static int sh_subcamdrv_CSI_free_irq( void )
{
    int ret = 0;

	mutex_lock( &sh_subcam_sync_head->mut_lock );

	do {
		ret = shlcdc_api_event_unsubscribe(SHLCDC_EVENT_TYPE_CSI);
		if(ret != SHLCDC_RESULT_SUCCESS) {
		}
	}while(0);
	
	mutex_unlock( &sh_subcam_sync_head->mut_lock );

	return ret;

}

static int sh_subcamdrv_vs_request_irq( void )
{
    int ret = 0;
 	unsigned long flags = 0;

	mutex_lock( &sh_subcam_sync_head->mut_lock );

	do {
		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );

		shscamdrv_irq_port = MSM_GPIO_TO_INT( SH_SUBCAM_GPIO_VSYNC );
		ret = request_irq( shscamdrv_irq_port, sh_subcam_drv_gpio_vsync_isr, IRQF_TRIGGER_FALLING, "sh_subcam_drv_vsync", 0 );
		disable_irq(shscamdrv_irq_port);
		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

	}while(0);
	
	mutex_unlock( &sh_subcam_sync_head->mut_lock );
	
	return ret;
}

static int sh_subcamdrv_vs_free_irq( void )
{
    int ret = 0;
	unsigned long flags = 0;

	mutex_lock( &sh_subcam_sync_head->mut_lock );

	do {
		spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );
		free_irq( shscamdrv_irq_port, 0 );
		spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );
	}while(0);
	
	mutex_unlock( &sh_subcam_sync_head->mut_lock );

	return ret;
}
static int sh_subcam_drv_sensor_init( void )
{
	int ret = 0;

	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_INT,		0, GPIO_INPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_VSYNC,		0, GPIO_INPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_AVDD_EN,	0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_MCAM_RST_N, 0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_0,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_1,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_2,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_3,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_4,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_5,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_6,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_7,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_PCLK,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_HSYNC,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_VSYNC_B,	1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_MCLK,		0, GPIO_OUTPUT,	GPIO_NO_PULL,	GPIO_2MA), GPIO_DISABLE );

    return ret;

}

static int sh_subcam_drv_sensor_off( void )
{

	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_INT,		0, GPIO_INPUT,	GPIO_PULL_DOWN,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_VSYNC,		0, GPIO_INPUT,	GPIO_PULL_DOWN,	GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_0,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_1,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_2,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_3,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_4,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_5,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_6,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_DATA_7,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_PCLK,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_16MA),GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_HSYNC,		1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );
	gpio_tlmm_config( GPIO_CFG(SH_SUBCAM_GPIO_VSYNC_B,	1, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE );

	return 0;

}

static int sh_subcam_sys_open(struct inode *inode, struct file *filep)
{
	int ret = 0;

	wake_lock( &sh_subcam_sync_head->wake_lock );
	mutex_lock( &sh_subcam_sync_head->mut_lock );

	do {
		ret = sh_subcam_drv_sensor_init();
		if ( ret < 0 ) {
			break;
		}

	}while(0);

	mutex_unlock( &sh_subcam_sync_head->mut_lock );

	return ret;

}

static int sh_subcam_sys_release(struct inode *node, struct file *filep)
{
	int ret = 0;

	mutex_lock( &sh_subcam_sync_head->mut_lock );

	do {
		ret = sh_subcam_drv_sensor_off();
		if ( ret < 0 ) {
			break;
		}

		sh_subcam_drv_irq_msg_free( sh_subcam_sync_head );
		wake_unlock( &sh_subcam_sync_head->wake_lock );

	}while(0);

	mutex_unlock( &sh_subcam_sync_head->mut_lock );

	return ret;
}

static long sh_subcam_sys_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	do {
		switch (cmd) {
			case SH_SUBCAM_IOCTL_READ_IRQ_KIND:
				ret = wait_event_interruptible_timeout(
							sh_subcam_sync_head->irq_event_wait,
							!list_empty( &sh_subcam_sync_head->irq_event_q ) ,
							msecs_to_jiffies(500));
				if(ret == 0 ){
					sh_subcam_drv_timeout_irq_num( argp );
					return -ETIMEDOUT;
				}
				else{
					ret = sh_subcam_drv_get_irq_num( argp );
				}
				break;
			case SH_SUBCAM_IOCTL_ENABLE_IRQ:
				ret = sh_subcam_drv_enable_irq( argp );
				break;
			case SH_SUBCAM_IOCTL_DISABLE_IRQ:
				ret = sh_subcam_drv_disable_irq( argp );
				break;
			case SH_SUBCAM_IOCTL_CAMIF_PAD_RESET:
				ret = sh_subcam_drv_camif_pad_reg_reset( argp );
				break;
			case SH_SUBCAM_IOCTL_CSI_REQUEST_IRQ:
				ret = sh_subcamdrv_CSI_request_irq();
				break;
			case SH_SUBCAM_IOCTL_CSI_FREE_IRQ:
				ret = sh_subcamdrv_CSI_free_irq();
				break;
			case SH_SUBCAM_IOCTL_VS_REQUEST_IRQ:
				ret =sh_subcamdrv_vs_request_irq();
				break;
				
			case SH_SUBCAM_IOCTL_VS_FREE_IRQ:
				sh_subcamdrv_vs_free_irq();
			default:
				ret = -EFAULT;
				break;
		}
	}while(0);

	return ret;
}

static unsigned int sh_subcam_sys_poll(struct file *filep, struct poll_table_struct *pll_table)
{
	uint32 rc = POLLERR;
	unsigned long flags = 0;

	poll_wait(filep, &sh_subcam_sync_head->irq_event_wait, pll_table);

	spin_lock_irqsave( &sh_subcam_sync_head->irq_event_q_lock, flags );

	if ( !list_empty( &sh_subcam_sync_head->irq_event_q ) ){
		rc = POLLIN | POLLRDNORM;
	}
	else{
	}

	spin_unlock_irqrestore( &sh_subcam_sync_head->irq_event_q_lock, flags );

	return rc;
}

static int sh_subcamdrv_sys_mmap(struct file *filep, struct vm_area_struct *vma)
{
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if(io_remap_pfn_range(vma, vma->vm_start,
						 (unsigned long)SH_SUBCAM_HCS1_ADDR >> PAGE_SHIFT,
						 vma->vm_end - vma->vm_start,
						 vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations sh_subcam_fops = {
	.owner          = THIS_MODULE,
	.open           = sh_subcam_sys_open,
	.unlocked_ioctl = sh_subcam_sys_ioctl,
	.release        = sh_subcam_sys_release,
	.read           = NULL,
	.write          = NULL,
	.poll           = sh_subcam_sys_poll,
	.mmap           = sh_subcamdrv_sys_mmap,
};

static int sh_subcam_drv_sync_init( struct sh_subcam_sync_head_t *sync )
{
	int rc = 0;

	spin_lock_init( &sync->irq_event_q_lock );
	INIT_LIST_HEAD( &sync->irq_event_q );
	init_waitqueue_head( &sync->irq_event_wait );
	wake_lock_init( &sync->wake_lock, WAKE_LOCK_SUSPEND, SH_SUBCAM_DEVICE_NAME );
	sync->irq_event_count = 0;
	mutex_init( &sync->mut_lock );

	return rc;
}

static int sh_subcam_drv_sync_destroy( struct sh_subcam_sync_head_t *sync )
{
	wake_lock_destroy( &sync->wake_lock );

	return 0;
}

static void sh_subcam_drv_irq_msg_free( struct sh_subcam_sync_head_t *sync )
{
	struct sh_subcam_irq_msg_t *qmsg = NULL;

	for (;;){
		if (!list_empty(&sync->irq_event_q)) {
			qmsg = list_first_entry(&sync->irq_event_q, struct sh_subcam_irq_msg_t, list);
			list_del(&qmsg->list);
			sync->irq_event_count--;
			kfree(qmsg);
		}
		else{
			break;
		}
	}

	return;
}

static int __init sh_subcam_drv_start( void )
{
	int rc = -ENODEV;
	int errfunc = 0;

	do {
		if (sh_subcamera_node >= SH_SUBCAM_DEVICE_COUNT) {
			rc = -ENODEV;
			errfunc = 0;
			break;
		}

		if (!sh_subcam_class){
			sh_subcam_class = class_create(THIS_MODULE, SH_SUBCAM_CLASS_NAME);
			if( IS_ERR(sh_subcam_class)) {
				errfunc = 0;
				rc = PTR_ERR(sh_subcam_class);
				break;
			}

			rc = alloc_chrdev_region(&sh_subcam_devno,
									SH_SUBCAM_MINORNUM_BASE,
									SH_SUBCAM_DEVICE_COUNT,
									SH_SUBCAM_DEVICE_NAME);
			if ( rc < 0 ) {
				errfunc = 1;
				break;
			}
		}

		sh_subcam_sync_head = kzalloc( sizeof(struct sh_subcam_sync_head_t ) * SH_SUBCAM_DEVICE_COUNT,
									GFP_KERNEL );
		if ( !sh_subcam_sync_head ) {
			rc = -ENOMEM;
			errfunc = 2;
			break;
		}

		rc = sh_subcam_drv_sync_init( sh_subcam_sync_head );
		if (rc < 0) {
			errfunc = 3;
			break;
		}

		sh_subcam_class_dev = device_create( sh_subcam_class, NULL,
												MKDEV(MAJOR(sh_subcam_devno), SH_SUBCAM_MINORNUM_BASE ),
												NULL, SH_SUBCAM_NODE_NAME );
		if ( IS_ERR( sh_subcam_class_dev ) ){
			rc = PTR_ERR( sh_subcam_class_dev );
			errfunc = 4;
			break;
		}

		cdev_init(&sh_subcam_sync_head->cdev, &sh_subcam_fops);
		sh_subcam_sync_head->cdev.owner = THIS_MODULE;

		rc = cdev_add( &sh_subcam_sync_head->cdev, MKDEV(MAJOR(sh_subcam_devno),
								SH_SUBCAM_MINORNUM_BASE ), SH_SUBCAM_DEVICE_COUNT );
		if ( rc < 0 ) {
			errfunc = 5;
			break;
		}

		sh_subcamera_node++;
		rc = 0;
		errfunc = 0;

	}while(0);

	switch( errfunc ){
		case 6:
			cdev_del(&sh_subcam_sync_head->cdev);
		case 5:
			device_destroy( sh_subcam_class, sh_subcam_devno );
		case 4:
			sh_subcam_drv_sync_destroy( sh_subcam_sync_head );
		case 3:
			kfree( sh_subcam_sync_head );
			sh_subcam_sync_head = NULL;
		case 2:
			unregister_chrdev_region( sh_subcam_devno, SH_SUBCAM_DEVICE_COUNT );
		case 1:
			class_destroy( sh_subcam_class );
			sh_subcam_class = NULL;
		case 0:
		default:
			break;
	}

	return rc;
}

static void __exit sh_subcam_drv_remove( void )
{

	do {
		if ( sh_subcamera_node == 0 ) {
			break;
		}

		cdev_del(&sh_subcam_sync_head->cdev);

		device_destroy( sh_subcam_class, sh_subcam_devno );

		sh_subcam_drv_sync_destroy( sh_subcam_sync_head );

		kfree( sh_subcam_sync_head );
		sh_subcam_sync_head = NULL;

		unregister_chrdev_region( sh_subcam_devno, SH_SUBCAM_DEVICE_COUNT );

		class_destroy( sh_subcam_class );
		sh_subcam_class = NULL;

		sh_subcamera_node = 0;
	}while(0);

	return;
}

module_init(sh_subcam_drv_start);
module_exit(sh_subcam_drv_remove);

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");
