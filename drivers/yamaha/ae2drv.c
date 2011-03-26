/*
 * Copyright (C) 2009 SHARP CORPORATION
 * Copyright (C) 2009 Yamaha CORPORATION
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

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#if 0
#include <asm/arch/mx31_pins.h>
#include <asm/arch/gpio.h>
#else
#include <mach/gpio.h>
#endif

#include "madrv.h"

//#define KDEBUG_FUNC() printk("%s()\n", __FUNCTION__)
#define KDEBUG_FUNC()

//#define YAMAHA_DEBUG_LOG

#define MA_DEVICE_NODE_NAME			"ae2"
#define MA_DEVICE_NAME				"ae2"
#define MA_DEVICE_IRQ_NAME			"irq_ae2"
#define MA_DEVICE_COUNT				(1)
#define MA_CLASS_NAME				"cls_ae2"

extern void gpio_ext_bus_active(void);

struct MaDriverInfo
{
	void				*pMemory;
	unsigned int		dIrq;
	struct cdev			sCdev;
	wait_queue_head_t	sQueue;
	spinlock_t			sLock;
	unsigned int		dIrqCount;
	unsigned int		dMaskIrq;
	unsigned int		dCanceled;
};

static int ma_IoCtl( struct inode *psInode, struct file *psFile, unsigned int dCmd, unsigned long dArg );
static int ma_Open( struct inode *psInode, struct file *psFile );
static int ma_Close( struct inode *psInode, struct file *psFile );

static struct file_operations ma_FileOps =
{
	.owner = THIS_MODULE,
	.ioctl = ma_IoCtl,
	.open = ma_Open,
	.release = ma_Close,
};
static struct MaDriverInfo *gpsDriver = NULL;
static int gsMajor = -1;
static dev_t gsDev;

static struct class *gpsClass = NULL;

/****************************************************************************
 *	NanoWait
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
NanoWait( unsigned int dPeriod )
{
	unsigned int ms = 0, us = 0, ns = 0;
	ms = dPeriod / 1000000;
	us = ( dPeriod - ( ms * 1000000 ) ) / 1000;
	ns = dPeriod - ( ms * 1000000 ) - ( us * 1000 );
	if ( ms > 0 )
	{
		mdelay( ms );
	}
	if ( us > 0 )
	{
		udelay( us );
	}
	if ( ns > 0 )
	{
		ndelay( ns );
	}
	return 0;
}

/****************************************************************************
 *	IoCtl_Wait
 *
 *	Description:
 *			Processing that does weight every Nano second is done. 
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	wait value(ns)
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_Wait( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		Processing that does weight every Nano second is done.
	*/

	(void)psInode;
	(void)psFile;

	return NanoWait( dArg );
}

/****************************************************************************
 *	MilliSleep
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
#if (0)
static inline int
MilliSleep( unsigned int dPeriod )
{
	msleep( dPeriod );
	return 0;
}

#else
static inline int
MilliSleep( unsigned int dPeriod )
{
	struct timeval sStart, sCurrent;
	unsigned long dDiff = 0;
	if ( dPeriod == 0 )
	{
		schedule();
		return 0;
	}
	if ( dPeriod > 4000000 )
	{
		return -EINVAL; /* Too big */
	}
	/* do_gettimeofday might provide more accuracy than jiffies */
	do_gettimeofday( &sStart );
	for ( ;; )
	{
		do_gettimeofday( &sCurrent );
		dDiff = ( sCurrent.tv_sec - sStart.tv_sec ) * 1000000 + ( sCurrent.tv_usec - sStart.tv_usec );
		if ( dDiff > dPeriod * 1000 )
		{
			break;
		}
		schedule();
	}
	return 0;
}
#endif

/****************************************************************************
 *	IoCtl_Sleep
 *
 *	Description:
 *			Processing that does the sleep in each millisecond is done. 
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	sleep value(ms)
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_Sleep( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		Processing that does the sleep in each millisecond is done.
	*/

	(void)psInode;
	(void)psFile;
	(void)dArg;

	return MilliSleep( dArg );
}

/****************************************************************************
 *	WriteRegWait
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
WriteRegWait( unsigned long dAddress, void *pData, unsigned int dSize, unsigned int dWait )
{
	(void)dWait;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
#if 0
	dAddress <<= 1;
#endif
	dAddress += (unsigned long) gpsDriver->pMemory;
#if 1
	NanoWait( dWait );
#endif
	switch ( dSize )
	{
	case sizeof( unsigned char ):
		iowrite8( *(unsigned char *)pData, (void *)dAddress );
		break;
	case sizeof( unsigned short ):
		iowrite16( *(unsigned short *)pData, (void *)dAddress );
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/****************************************************************************
 *	IoCtl_WriteRegWait
 *
 *	Description:
 *			The processing written in the interface register is done. 
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	pointer to ma_IoCtlWriteRegWait structure
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_WriteRegWait( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		Data is copied from the user space, and it writes it in the interface register.
	*/

	struct ma_IoCtlWriteRegWait sParam;
	unsigned char bData = 0;
	unsigned short wData = 0;
#if 1
	unsigned long dFlags = 0;
#else
	unsigned int dFlags = 0;
#endif
	unsigned int dCnt;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	if ( copy_from_user( &sParam, (void*)dArg, sizeof( sParam ) ) )
	{
		return -EFAULT;
	}
	switch ( sParam.dSize )
	{
	case sizeof( unsigned char ):
		for(dCnt=0; dCnt<sParam.dDataLen; ++dCnt) {
			get_user( bData, (((unsigned char*)sParam.pData) + dCnt) );
			spin_lock_irqsave( &gpsDriver->sLock, dFlags);
			WriteRegWait( sParam.dAddress, &bData, sizeof( bData ), sParam.dWait );
			spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
		}
		break;
	case sizeof( unsigned short ):
		for(dCnt=0; dCnt<sParam.dDataLen; ++dCnt) {
			get_user( wData, (((unsigned short*)sParam.pData) + dCnt) );
			spin_lock_irqsave( &gpsDriver->sLock, dFlags);
			WriteRegWait( sParam.dAddress, &wData, sizeof( wData ), sParam.dWait );
			spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/****************************************************************************
 *	ReadRegWait
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
ReadRegWait( unsigned long dAddress, void *pData, unsigned int dSize, unsigned int dWait )
{
	/*
		It reads it from the interface register.
		The read value is copied onto the user space.
	*/

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
#if 0
	dAddress <<= 1;
#endif
	dAddress += (unsigned long) gpsDriver->pMemory;
	NanoWait( dWait );
	switch ( dSize ) {
	case sizeof( unsigned char ):
		*(unsigned char *)pData = ioread8( (void *)dAddress );
		break;
	case sizeof( unsigned short ):
		*(unsigned short *)pData = ioread16( (void *)dAddress );
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/****************************************************************************
 *	IoCtl_ReadRegWait
 *
 *	Description:
 *			The processing read from the interface register is done. 
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	pointer to ma_IoCtlReadRegWait structure
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_ReadRegWait( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		It reads it from the interface register.
		The read value is copied onto the user space.
	*/

	struct ma_IoCtlReadRegWait sParam;
	unsigned char bData = 0;
	unsigned short wData = 0;
#if 1
	unsigned long dFlags;
#else
	unsigned int dFlags;
#endif
	unsigned int dCnt;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	if ( copy_from_user( &sParam, (void*)dArg, sizeof( sParam ) ) )
	{
		return -EFAULT;
	}
	switch ( sParam.dSize ) {
	case sizeof( unsigned char ):
		for(dCnt=0; dCnt<sParam.dDataLen; ++dCnt) {
			spin_lock_irqsave( &gpsDriver->sLock, dFlags );
			ReadRegWait( sParam.dAddress, &bData, sizeof( bData ), sParam.dWait );
			spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
			put_user( bData, (((unsigned char*)sParam.pData) + dCnt) );
		}
		break;
	case sizeof( unsigned short ):
		for(dCnt=0; dCnt<sParam.dDataLen; ++dCnt) {
			spin_lock_irqsave( &gpsDriver->sLock, dFlags);
			ReadRegWait( sParam.dAddress, &wData, sizeof( wData ), sParam.dWait );
			spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
			put_user( wData, (((unsigned short*)sParam.pData) + dCnt) );
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/****************************************************************************
 *	DisableIrq
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
DisableIrq( void )
{
	unsigned char bStatus = 0x00;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	if ( gpsDriver->dMaskIrq == 0 )
	{
		WriteRegWait( 0x00, &bStatus, sizeof( bStatus ), 135 );
	}
	gpsDriver->dMaskIrq++;
	return 0;
}

/****************************************************************************
 *	IoCtl_DisableIrq
 *
 *	Description:
 *			The interruption is prohibited.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	no use
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_DisableIrq( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		The interruption is prohibited.
	*/

	unsigned long dFlags;
	int sdResult = 0;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	spin_lock_irqsave( &gpsDriver->sLock, dFlags );
	sdResult = DisableIrq();
	spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
	return sdResult;
}

/****************************************************************************
 *	EnableIrq
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
EnableIrq( void )
{
	unsigned char bStatus = 0x80;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	if ( gpsDriver->dMaskIrq > 0UL )
	{
		gpsDriver->dMaskIrq--;
	}
	if ( gpsDriver->dMaskIrq == 0UL )
	{
		WriteRegWait( 0x00, &bStatus, sizeof( bStatus ), 135 );
	}
	return 0;
}

/****************************************************************************
 *	IoCtl_EnableIrq
 *
 *	Description:
 *			The interruption is permitted.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	no use
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_EnableIrq( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		The interruption is permitted.
	*/

	unsigned long dFlags;
	int sdResult = 0;
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	spin_lock_irqsave( &gpsDriver->sLock, dFlags );
	sdResult = EnableIrq();
	spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
	return sdResult;
}

/****************************************************************************
 *	ResetIrqMaskCount
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
ResetIrqMaskCount( void )
{
	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}

	gpsDriver->dMaskIrq = 0;
	return 0;
}

/****************************************************************************
 *	IoCtl_ResetIrqMaskCount
 *
 *	Description:
 *			The interruption mask counter is initialized.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	no use
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_ResetIrqMaskCount( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		The interruption mask counter is initialized.
	*/

	unsigned long dFlags = 0;
	int sdResult = 0;

	(void)psInode;
	(void)psFile;
	(void)dArg;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}

	spin_lock_irqsave( &gpsDriver->sLock, dFlags );
	sdResult = ResetIrqMaskCount();
	spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );

	return sdResult;
}

/****************************************************************************
 *	IoCtl_WaitIrq
 *
 *	Description:
 *			The interruption is waited for.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	Pointer to variable that stores interruption frequency
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_WaitIrq( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		Sleep until interruption is notified, or waiting condition is released.
		When getting up, store the counter of interruption reserved to dArg of user space,
		and return it.
		When the interruption has already reserved it, this API call is returned at once. 
	*/

	unsigned long dFlags;
	int sdResult = 0;
	int dIrqCount = 0;

	KDEBUG_FUNC();

	(void)psInode;
	(void)psFile;
	(void)dArg;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	spin_lock_irqsave( &gpsDriver->sLock, dFlags );
	if ( gpsDriver->dIrqCount != 0 )
	{
		dIrqCount = gpsDriver->dIrqCount;
		gpsDriver->dIrqCount = 0;
		spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
	}
	else
	{
		spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
		for ( ;; )
		{
			DEFINE_WAIT( sWait );
			prepare_to_wait( &gpsDriver->sQueue, &sWait, TASK_INTERRUPTIBLE );
			spin_lock_irqsave( &gpsDriver->sLock, dFlags );
			if ( gpsDriver->dCanceled )
			{
				dIrqCount = -1;
				gpsDriver->dCanceled = 0;
				spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
				finish_wait( &gpsDriver->sQueue, &sWait );
				sdResult = -ERESTARTSYS;
				goto out;
			}
			if ( gpsDriver->dIrqCount != 0 )
			{
				dIrqCount = gpsDriver->dIrqCount;
				gpsDriver->dIrqCount = 0;
				spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
				finish_wait( &gpsDriver->sQueue, &sWait );
				sdResult = 0;
				goto out;
			}
			spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );
			if ( signal_pending( current ) )
			{
				dIrqCount = 0;
				finish_wait( &gpsDriver->sQueue, &sWait );
				sdResult = -ERESTARTSYS;
				goto out;
			}
			schedule();
			finish_wait( &gpsDriver->sQueue, &sWait );
		}
	}
out:
	if ( dArg != 0 )
	{
		put_user( dIrqCount, (unsigned int*)dArg );
	}
	return sdResult;
}


/****************************************************************************
 *	IoCtl_CancelWaitIrq
 *
 *	Description:
 *			The interruption waiting state is released.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dArg	no use
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static inline int
IoCtl_CancelWaitIrq( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
	/*
		The interruption waiting state is released.
	*/
	unsigned long dFlags = 0;

	KDEBUG_FUNC();

	(void)psInode;
	(void)psFile;
	(void)dArg;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}

	spin_lock_irqsave( &gpsDriver->sLock, dFlags );
	gpsDriver->dCanceled = 1;
	spin_unlock_irqrestore( &gpsDriver->sLock, dFlags );

	wake_up_interruptible( &gpsDriver->sQueue );
	return 0;
}

/****************************************************************************
 *	IrqIsMine
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
IrqIsMine( void )
{
	unsigned char bStatus = 0;
	ReadRegWait( 0, &bStatus, sizeof( bStatus ), 150 );
	if ( ( bStatus & 0x01 ) == 0 || ( bStatus & 0x80 ) == 0 )
	{
		return 0;
	}
	return 1;
}

#if 1
/****************************************************************************
 *	IoCtl_SetGpio
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static inline int
IoCtl_SetGpio( struct inode *psInode, struct file *psFile, unsigned long dArg )
{
    (void)psInode;
    (void)psFile;

    if( dArg == 0 ){
        /* OFF */
        gpio_direction_output(121, 0);
        gpio_direction_output(27, 0);
#ifdef YAMAHA_DEBUG_LOG
        printk("IoCtl_SetGpio GPIO OFF\n");
#endif
    }else{
        /* ON */
        gpio_direction_output(121, 1);
        gpio_direction_output(27, 1);
#ifdef YAMAHA_DEBUG_LOG
        printk("IoCtl_SetGpio GPIO ON\n");
#endif
    }

	return 0;
}
#endif

/****************************************************************************
 *	ma_IrqHandler
 *
 *	Description:
 *			Interruption handler
 *	Arguments:
 *			sdIrq	interruption number
 *			pDevId	pointer to device ID
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static irqreturn_t
ma_IrqHandler( int sdIrq, void *pDevId )
{
	/*
		It is notified that the interruption entered the interrupt processing thread.
	*/

	KDEBUG_FUNC();
	(void)sdIrq;

	if ( pDevId == NULL || gpsDriver == NULL )
	{
		return IRQ_NONE;
	}
	spin_lock( &gpsDriver->sLock ); /* for SMP */
	if ( !IrqIsMine() )
	{
		spin_unlock( &gpsDriver->sLock );
		return IRQ_NONE;
	}
	DisableIrq();
	gpsDriver->dIrqCount++;
	spin_unlock( &gpsDriver->sLock );
	wake_up_interruptible( &gpsDriver->sQueue );
	return IRQ_HANDLED;
}

/****************************************************************************
 *	ma_IoCtl
 *
 *	Description:
 *			Character type driver IoCtl processing is executed.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *			dCmd	ioctl command number
 *			dArg	ioctl command argument
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static int
ma_IoCtl( struct inode *psInode, struct file *psFile, unsigned int dCmd, unsigned long dArg )
{
	int sdResult = -EFAULT;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}

	switch ( dCmd ) {
	case MA_IOCTL_WAIT:
		sdResult = IoCtl_Wait( psInode, psFile, dArg );
		break;
	case MA_IOCTL_SLEEP:
		sdResult = IoCtl_Sleep( psInode, psFile, dArg );
		break;
	case MA_IOCTL_WRITE_REG_WAIT:
		sdResult = IoCtl_WriteRegWait( psInode, psFile, dArg );
		break;
	case MA_IOCTL_READ_REG_WAIT:
		sdResult = IoCtl_ReadRegWait( psInode, psFile, dArg );
		break;
	case MA_IOCTL_DISABLE_IRQ:
		sdResult = IoCtl_DisableIrq( psInode, psFile, dArg );
		break;
	case MA_IOCTL_ENABLE_IRQ:
		sdResult = IoCtl_EnableIrq( psInode, psFile, dArg );
		break;
	case MA_IOCTL_RESET_IRQ_MASK_COUNT:
		sdResult = IoCtl_ResetIrqMaskCount( psInode, psFile, dArg );
		break;
	case MA_IOCTL_WAIT_IRQ:
		sdResult = IoCtl_WaitIrq( psInode, psFile, dArg );
		break;
	case MA_IOCTL_CANCEL_WAIT_IRQ:
		sdResult = IoCtl_CancelWaitIrq( psInode, psFile, dArg );
		break;
#if 1
	case MA_IOCTL_SET_GPIO:
		sdResult = IoCtl_SetGpio( psInode, psFile, dArg );
		break;
#endif
	default:
		sdResult = -ENOTTY;
		break;
	}

	return sdResult;
}

/****************************************************************************
 *	ma_Open
 *
 *	Description:
 *	Arguments:
 *	Return:
 *
 ****************************************************************************/
static int
ma_Open( struct inode *psInode, struct file *psFile )
{
	/*
		Character type driver Open processes it.
	*/
	int sdResult = 0;

	KDEBUG_FUNC();

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}

#if 1
	gpio_tlmm_config(GPIO_CFG(27,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(28,  0, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(102, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(121, 0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), GPIO_ENABLE);

	gpio_direction_output(121, 1);
	gpio_direction_output(27, 1);
#ifdef YAMAHA_DEBUG_LOG
	printk("ma_Open GPIO ON\n");
#endif
#endif

	/* init */
	init_waitqueue_head( &gpsDriver->sQueue );
	spin_lock_init( &gpsDriver->sLock );
	gpsDriver->dIrqCount = 0;
	gpsDriver->dCanceled = 0;

	/* I/O port setting */
#if 0
	gpsDriver->pMemory = ioremap( ?, 64 );
#else
	gpsDriver->pMemory = ioremap( 0x90000000, 64 );
#endif
	if ( gpsDriver->pMemory == NULL )
	{
		goto err1;
	}

	/* interrrupt setting */
	ResetIrqMaskCount();

#if 0
	gpsDriver->dIrq = ?;
	sdResult = request_irq( gpsDriver->dIrq, ma_IrqHandler, IRQF_SHARED, MA_DEVICE_IRQ_NAME, gpsDriver );
	if ( sdResult < 0 )
	{
		goto err2;
	}
	set_irq_type( gpsDriver->dIrq, IRQT_FALLING );
#else
	gpsDriver->dIrq = MSM_GPIO_TO_INT(28);
	sdResult = request_irq( gpsDriver->dIrq, ma_IrqHandler, IRQF_TRIGGER_FALLING, MA_DEVICE_NAME, gpsDriver );
	if ( sdResult < 0 )
	{
		goto err2;
	}
#endif

	return 0;
err2:
	iounmap( gpsDriver->pMemory );
	gpsDriver->pMemory = NULL;
err1:
#if 1
	gpio_direction_output(121, 0);
	gpio_direction_output(27, 0);
#ifdef YAMAHA_DEBUG_LOG
	printk("ma_Open GPIO Err OFF\n");
#endif
#endif
	return sdResult;
}

/****************************************************************************
 *	ma_Close
 *
 *	Description:
 *			Character type driver Close processing is executed.
 *	Arguments:
 *			psInode	inode info pointer
 *			psFile	file info pointer
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static int
ma_Close( struct inode *psInode, struct file *psFile )
{
	/*
		Character type driver Close processes it.
	*/
	KDEBUG_FUNC();

	(void)psInode;
	(void)psFile;

	if ( gpsDriver == NULL )
	{
		return -ENOTTY;
	}
	if ( gpsDriver->pMemory == NULL )
	{
		return -ENOTTY;
	}

	free_irq( gpsDriver->dIrq, gpsDriver );
	iounmap( gpsDriver->pMemory );
	gpsDriver->pMemory = NULL;

#if 1
	gpio_direction_output(27, 0);
	gpio_direction_output(121, 0);
#endif

	return 0;
}

/****************************************************************************
 *	ma_Init
 *
 *	Description:
 *			The driver is initialized.
 *	Arguments:
 *			none
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static int __init
ma_Init( void )
{
	/*
		Processing that registers the character type driver is done.
	*/

	int sdResult;
	struct device *dev;

	KDEBUG_FUNC();

	sdResult = alloc_chrdev_region( &gsDev, 0, MA_DEVICE_COUNT, MA_DEVICE_NAME );
	if ( sdResult < 0 )
	{
		goto err1;
	}
	gsMajor = sdResult;

	gpsDriver = kzalloc( sizeof( struct MaDriverInfo ), GFP_KERNEL );
	if ( gpsDriver == NULL )
	{
		sdResult = -ENOMEM;
		goto err2;
	}

	cdev_init( &gpsDriver->sCdev, &ma_FileOps );
	gpsDriver->sCdev.owner = THIS_MODULE;
	gpsDriver->sCdev.ops = &ma_FileOps;
	sdResult = cdev_add( &gpsDriver->sCdev, gsDev, MA_DEVICE_COUNT );
	if ( sdResult < 0 )
	{
		goto err3;
	}

	gpsClass = class_create( THIS_MODULE, MA_CLASS_NAME );
	if ( IS_ERR(gpsClass) )
	{
		sdResult = PTR_ERR( gpsClass );
		goto err4;
	}

#if 1
	dev = device_create( gpsClass, NULL, gsDev, NULL, MA_DEVICE_NODE_NAME );
#else
	dev = device_create( gpsClass, NULL, gsDev, MA_DEVICE_NODE_NAME );
#endif
	sdResult = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if ( sdResult < 0 )
	{
		goto err5;
	}

	return 0;

err5:
	class_destroy( gpsClass );
	gpsClass = NULL;
err4:
	cdev_del( &gpsDriver->sCdev );
err3:
	kfree( gpsDriver );
	gpsDriver = NULL;
err2:
	unregister_chrdev_region( gsDev, MA_DEVICE_COUNT );
	gsMajor = -1;
err1:
	return sdResult;
}

/****************************************************************************
 *	ma_Term
 *
 *	Description:
 *			The driver is ended.
 *	Arguments:
 *			none
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static void __exit
ma_Term( void )
{
	/*
		Processing that deletes the character type driver is done.
	*/

	KDEBUG_FUNC();

	if ( gsMajor < 0 )
	{
		return;
	}

	device_destroy( gpsClass, gsDev );

	class_destroy( gpsClass );
	gpsClass = NULL;

	cdev_del( &gpsDriver->sCdev );
	kfree( gpsDriver );
	gpsDriver = NULL;

	unregister_chrdev_region( gsDev, MA_DEVICE_COUNT );
	gsMajor = -1;

	return;
}

module_init( ma_Init );
module_exit( ma_Term );

MODULE_DESCRIPTION("ae2 driver");
MODULE_LICENSE("GPL");
