/* drivers/i2c/busses/i2c-msm.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#define FEATURE_SHLOCAL_SHI2C

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/board.h>
#include <linux/mutex.h>
#include <linux/remote_spinlock.h>
#ifdef FEATURE_SHLOCAL_SHI2C
#include <linux/wakelock.h>
#else  /* FEATURE_SHLOCAL_SHI2C */
#include <linux/pm_qos_params.h>
#endif /* FEATURE_SHLOCAL_SHI2C */
#include <mach/gpio.h>
#ifdef FEATURE_SHLOCAL_SHI2C
#include <mach/msm_rpcrouter.h>
#include <mach/sharp_smem.h>
#endif /* FEATURE_SHLOCAL_SHI2C */

#define DEBUG 0

#ifdef FEATURE_SHLOCAL_SHI2C

#define SH_I2C_WAIT_NEED
#ifdef SH_I2C_WAIT_NEED
#define SH_I2C_WAIT 30
#endif /* SH_I2C_WAIT_NEED */



#define I2C_CLK_STATE_MASK 0xe000
#define I2C_CLK_STATE_BUSIDLE 0x0000



#define XFER_RETRY   2
static int msm_i2c_sub_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num);



#define I2CPROG		0x3000000c
#define I2CVERS		0x00010001

#define ONCRPC_I2C_NULL_PROC 0
#define ONCRPC_I2C_READ_PROC 1
#define ONCRPC_I2C_WRITE_PROC 2
#define ONCRPC_SH_I2C_RECOVERY_PROC 3


#define SH_I2C_CLK_STRETCH_WAIT 20000
static sharp_smem_common_type * sh_smem_common_ptr = NULL;
static struct msm_rpc_endpoint *sh_i2c_endpoint = NULL;
static struct wake_lock sh_i2c_wake_lock;

typedef enum {
  SH_I2C_IDLE_STATE,
  SH_I2C_AARM_RUNNING_STATE,
  SH_I2C_MARM_RUNNING_STATE,
  SH_I2C_RECOVERY_REQ_STATE,
} i2c_smem_state_type;
#endif /* FEATURE_SHLOCAL_SHI2C */

enum {
	I2C_WRITE_DATA          = 0x00,
	I2C_CLK_CTL             = 0x04,
	I2C_STATUS              = 0x08,
	I2C_READ_DATA           = 0x0c,
	I2C_INTERFACE_SELECT    = 0x10,

	I2C_WRITE_DATA_DATA_BYTE            = 0xff,
	I2C_WRITE_DATA_ADDR_BYTE            = 1U << 8,
	I2C_WRITE_DATA_LAST_BYTE            = 1U << 9,

	I2C_CLK_CTL_FS_DIVIDER_VALUE        = 0xff,
	I2C_CLK_CTL_HS_DIVIDER_VALUE        = 7U << 8,

	I2C_STATUS_WR_BUFFER_FULL           = 1U << 0,
	I2C_STATUS_RD_BUFFER_FULL           = 1U << 1,
	I2C_STATUS_BUS_ERROR                = 1U << 2,
	I2C_STATUS_PACKET_NACKED            = 1U << 3,
	I2C_STATUS_ARB_LOST                 = 1U << 4,
	I2C_STATUS_INVALID_WRITE            = 1U << 5,
	I2C_STATUS_FAILED                   = 3U << 6,
	I2C_STATUS_BUS_ACTIVE               = 1U << 8,
	I2C_STATUS_BUS_MASTER               = 1U << 9,
	I2C_STATUS_ERROR_MASK               = 0xfc,

	I2C_INTERFACE_SELECT_INTF_SELECT    = 1U << 0,
	I2C_INTERFACE_SELECT_SCL            = 1U << 8,
	I2C_INTERFACE_SELECT_SDA            = 1U << 9,
	I2C_STATUS_RX_DATA_STATE            = 3U << 11,
	I2C_STATUS_LOW_CLK_STATE            = 3U << 13,
};

struct msm_i2c_dev {
	struct device                *dev;
	void __iomem                 *base;	/* virtual */
	int                          irq;
	struct clk                   *clk;
	struct i2c_adapter           adap_pri;
	struct i2c_adapter           adap_aux;

	spinlock_t                   lock;

	struct i2c_msg               *msg;
	int                          rem;
	int                          pos;
	int                          cnt;
	int                          err;
	int                          flush_cnt;
	int                          rd_acked;
	int                          one_bit_t;
	remote_spinlock_t            rspin_lock;
	int                          suspended;
	struct mutex                 mlock;
	struct msm_i2c_platform_data *pdata;
	void                         *complete;
};

#if DEBUG
static void
dump_status(uint32_t status)
{
	printk("STATUS (0x%.8x): ", status);
	if (status & I2C_STATUS_BUS_MASTER)
		printk("MST ");
	if (status & I2C_STATUS_BUS_ACTIVE)
		printk("ACT ");
	if (status & I2C_STATUS_INVALID_WRITE)
		printk("INV_WR ");
	if (status & I2C_STATUS_ARB_LOST)
		printk("ARB_LST ");
	if (status & I2C_STATUS_PACKET_NACKED)
		printk("NAK ");
	if (status & I2C_STATUS_BUS_ERROR)
		printk("BUS_ERR ");
	if (status & I2C_STATUS_RD_BUFFER_FULL)
		printk("RD_FULL ");
	if (status & I2C_STATUS_WR_BUFFER_FULL)
		printk("WR_FULL ");
	if (status & I2C_STATUS_FAILED)
		printk("FAIL 0x%x", (status & I2C_STATUS_FAILED));
	printk("\n");
}
#endif
#ifdef FEATURE_SHLOCAL_SHI2C

static void sh_i2c_smem_init( void )
{

	sh_smem_common_ptr = sh_smem_get_common_address();

	if( sh_smem_common_ptr == NULL) {
		panic("sh_i2c_smem_init: sh_smem_get_common_address is NULL\n");
	}
}


static int sh_i2c_clk_status_check( struct msm_i2c_dev *dev )
{

	unsigned short loop_count = SH_I2C_CLK_STRETCH_WAIT;


	while ( I2C_CLK_STATE_BUSIDLE != (readl(dev->base + I2C_STATUS) & I2C_CLK_STATE_MASK) )
	{
		if (loop_count == 0)
		{
			printk(KERN_ERR "%s: i2c_not idle %x \n", __func__
													, readl(dev->base + I2C_STATUS) );
			return -EIO;
		}
		udelay(1);
		loop_count--;
    }

	return 0;

}


static void sh_i2c_update_err_info( uint8_t slave_addr )
{

	sh_smem_common_ptr->sh_i2c_last_success_dev = slave_addr;

	sh_smem_common_ptr->sh_i2c_err_counter = 0;
}


static int sh_i2c_rpc_init(void)
{

	sh_i2c_endpoint = msm_rpc_connect_compatible(I2CPROG, I2CVERS, 0);


	if (IS_ERR(sh_i2c_endpoint)) {
		printk(KERN_ERR "%s(): init rpc failed! rc = %lx PROG = %08x vers = %08x\n", 
											__FUNCTION__, 
											PTR_ERR(sh_i2c_endpoint), 
											I2CPROG, 
											I2CVERS );
		sh_i2c_endpoint = NULL;
		return -EIO;
	}
	printk(KERN_INFO "%s(): Success !! I2CPROG = %08x I2CVERS = %08x\n", __func__, I2CPROG, I2CVERS );
	return 0;

}






static int sh_i2c_recoveryreq_a_to_m( void )
{
	int rc = -EIO;

	struct sh_i2c_recover_req {
		struct rpc_request_hdr hdr;
	} req;


	if ( sh_i2c_endpoint == NULL ) { 
		if( sh_i2c_rpc_init() < 0 ){
			return rc;
		}
	}


	rc = msm_rpc_call(sh_i2c_endpoint, ONCRPC_SH_I2C_RECOVERY_PROC, &req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		printk(KERN_ERR "%s(): I2C_Recovery Fail rc = %d\n ", __FUNCTION__, rc );
		return rc;
	}

	return rc;

}
#endif /* FEATURE_SHLOCAL_SHI2C */
static irqreturn_t
msm_i2c_interrupt(int irq, void *devid)
{
	struct msm_i2c_dev *dev = devid;
	uint32_t status = readl(dev->base + I2C_STATUS);
	int err = 0;

#if DEBUG
	dump_status(status);
#endif

	spin_lock(&dev->lock);
	if (!dev->msg) {
		printk(KERN_ERR "%s: IRQ but nothing to do!\n", __func__);
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (status & I2C_STATUS_ERROR_MASK) {
		err = -EIO;
		goto out_err;
	}

	if (dev->msg->flags & I2C_M_RD) {
		if (status & I2C_STATUS_RD_BUFFER_FULL) {

			/*
			 * Theres something in the FIFO.
			 * Are we expecting data or flush crap?
			 */
			if (dev->cnt) { /* DATA */
				uint8_t *data = &dev->msg->buf[dev->pos];

				/* This is in spin-lock. So there will be no
				 * scheduling between reading the second-last
				 * byte and writing LAST_BYTE to the controller.
				 * So extra read-cycle-clock won't be generated
				 * Per I2C MSM HW Specs: Write LAST_BYTE befure
				 * reading 2nd last byte
				 */
				if (dev->cnt == 2)
					writel(I2C_WRITE_DATA_LAST_BYTE,
						dev->base + I2C_WRITE_DATA);
				*data = readl(dev->base + I2C_READ_DATA);
				dev->cnt--;
				dev->pos++;
				if (dev->msg->len == 1)
					dev->rd_acked = 0;
				if (dev->cnt == 0)
					goto out_complete;

			} else {
				/* Now that extra read-cycle-clocks aren't
				 * generated, this becomes error condition
				 */
				dev_err(dev->dev,
					"read did not stop, status - %x\n",
					status);
				err = -EIO;
				goto out_err;
			}
		} else if (dev->msg->len == 1 && dev->rd_acked == 0 &&
				((status & I2C_STATUS_RX_DATA_STATE) ==
				 I2C_STATUS_RX_DATA_STATE))
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
	} else {
		uint16_t data;

		if (status & I2C_STATUS_WR_BUFFER_FULL) {
			dev_err(dev->dev,
				"Write buffer full in ISR on write?\n");
			err = -EIO;
			goto out_err;
		}

		if (dev->cnt) {
			/* Ready to take a byte */
			data = dev->msg->buf[dev->pos];
			if (dev->cnt == 1 && dev->rem == 1)
				data |= I2C_WRITE_DATA_LAST_BYTE;

			status = readl(dev->base + I2C_STATUS);
			/*
			 * Due to a hardware timing issue, data line setup time
			 * may be reduced to less than recommended 250 ns.
			 * This happens when next byte is written in a
			 * particular window of clock line being low and master
			 * not stretching the clock line. Due to setup time
			 * violation, some slaves may miss first-bit of data, or
			 * misinterprete data as start condition.
			 * We introduce delay of just over 1/2 clock cycle to
			 * ensure master stretches the clock line thereby
			 * avoiding setup time violation. Delay is introduced
			 * only if I2C clock FSM is LOW. The delay is not needed
			 * if I2C clock FSM is HIGH or FORCED_LOW.
			 */
			if ((status & I2C_STATUS_LOW_CLK_STATE) ==
					I2C_STATUS_LOW_CLK_STATE)
				udelay((dev->one_bit_t >> 1) + 1);
			writel(data, dev->base + I2C_WRITE_DATA);
			dev->pos++;
			dev->cnt--;
		} else {
			goto out_complete;
		}
	}

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;

 out_err:
	dev->err = err;
 out_complete:
	complete(dev->complete);
	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static int
msm_i2c_poll_writeready(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_WR_BUFFER_FULL))
			return 0;
		if (retries++ > 1000)
			udelay(100);
	}
	return -ETIMEDOUT;
}

static int
msm_i2c_poll_notbusy(struct msm_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + I2C_STATUS);

		if (!(status & I2C_STATUS_BUS_ACTIVE))
			return 0;
		if (retries++ > 1000)
			udelay(100);
	}
	return -ETIMEDOUT;
}

static void
msm_i2c_rmutex_lock(struct msm_i2c_dev *dev)
{
	int gotlock = 0;
	unsigned long flags;
	if (!dev->pdata->rmutex)
		return;
	do {
		remote_spin_lock_irqsave(&dev->rspin_lock, flags);
		#ifdef FEATURE_SHLOCAL_SHI2C
		if (*(dev->pdata->rmutex) == SH_I2C_IDLE_STATE) {
			*(dev->pdata->rmutex) = SH_I2C_AARM_RUNNING_STATE;
			gotlock = 1;
		}
		#else  /* FEATURE_SHLOCAL_SHI2C */
		if (*(dev->pdata->rmutex) == 0) {
			*(dev->pdata->rmutex) = 1;
			gotlock = 1;
		}
		#endif /* FEATURE_SHLOCAL_SHI2C */
		remote_spin_unlock_irqrestore(&dev->rspin_lock, flags);
		/* wait for 1-byte clock interval */
		if (!gotlock)
			udelay(10000000/dev->pdata->clk_freq);
	} while (!gotlock);
}

static void
msm_i2c_rmutex_unlock(struct msm_i2c_dev *dev)
{
	unsigned long flags;
	if (!dev->pdata->rmutex)
		return;
	remote_spin_lock_irqsave(&dev->rspin_lock, flags);
	#ifdef FEATURE_SHLOCAL_SHI2C
	*(dev->pdata->rmutex) = SH_I2C_IDLE_STATE;
	#else  /* FEATURE_SHLOCAL_SHI2C */
	*(dev->pdata->rmutex) = 0;
	#endif /* FEATURE_SHLOCAL_SHI2C */
	remote_spin_unlock_irqrestore(&dev->rspin_lock, flags);
}

static int
msm_i2c_recover_bus_busy(struct msm_i2c_dev *dev, struct i2c_adapter *adap)
{
#ifdef FEATURE_SHLOCAL_SHI2C
	return -EBUSY;
#else  /* FEATURE_SHLOCAL_SHI2C */
	int i;
	int gpio_clk;
	int gpio_dat;
	uint32_t status = readl(dev->base + I2C_STATUS);
	bool gpio_clk_status = false;

	if (!(status & (I2C_STATUS_BUS_ACTIVE | I2C_STATUS_WR_BUFFER_FULL)))
		return 0;

	dev->pdata->msm_i2c_config_gpio(adap->nr, 0);
	/* Even adapter is primary and Odd adapter is AUX */
	if (adap->nr % 2) {
		gpio_clk = dev->pdata->aux_clk;
		gpio_dat = dev->pdata->aux_dat;
	} else {
		gpio_clk = dev->pdata->pri_clk;
		gpio_dat = dev->pdata->pri_dat;
	}

	disable_irq(dev->irq);
	if (status & I2C_STATUS_RD_BUFFER_FULL) {
		dev_warn(dev->dev, "Read buffer full, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE, dev->base + I2C_WRITE_DATA);
		readl(dev->base + I2C_READ_DATA);
	} else if (status & I2C_STATUS_BUS_MASTER) {
		dev_warn(dev->dev, "Still the bus master, status %x, intf %x\n",
			 status, readl(dev->base + I2C_INTERFACE_SELECT));
		writel(I2C_WRITE_DATA_LAST_BYTE | 0xff,
		       dev->base + I2C_WRITE_DATA);
	}

	for (i = 0; i < 9; i++) {
		if (gpio_get_value(gpio_dat) && gpio_clk_status)
			break;
		gpio_direction_output(gpio_clk, 0);
		udelay(5);
		gpio_direction_output(gpio_dat, 0);
		udelay(5);
		gpio_direction_input(gpio_clk);
		udelay(5);
		if (!gpio_get_value(gpio_clk))
			udelay(20);
		if (!gpio_get_value(gpio_clk))
			msleep(10);
		gpio_clk_status = gpio_get_value(gpio_clk);
		gpio_direction_input(gpio_dat);
		udelay(5);
	}
	dev->pdata->msm_i2c_config_gpio(adap->nr, 1);
	udelay(10);

	status = readl(dev->base + I2C_STATUS);
	if (!(status & I2C_STATUS_BUS_ACTIVE)) {
		dev_info(dev->dev, "Bus busy cleared after %d clock cycles, "
			 "status %x, intf %x\n",
			 i, status, readl(dev->base + I2C_INTERFACE_SELECT));
		enable_irq(dev->irq);
		return 0;
	}

	dev_err(dev->dev, "Bus still busy, status %x, intf %x\n",
		 status, readl(dev->base + I2C_INTERFACE_SELECT));
	enable_irq(dev->irq);
	return -EBUSY;
#endif /* FEATURE_SHLOCAL_SHI2C */
}

static int
msm_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
#ifdef FEATURE_SHLOCAL_SHI2C

{
	int	ret;
	int	cnt;

	for (cnt=0; cnt<=XFER_RETRY; cnt++) {
		ret = msm_i2c_sub_xfer(adap, msgs, num);
		if (ret >= 0) {
			return ret;
		}
	}
	return ret;
}
static int
msm_i2c_sub_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)

#endif /* FEATURE_SHLOCAL_SHI2C */
{
	DECLARE_COMPLETION_ONSTACK(complete);
	struct msm_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	int rem = num;
	uint16_t addr;
	long timeout;
	unsigned long flags;
	int check_busy = 1;

	mutex_lock(&dev->mlock);

	if (dev->suspended) {
		mutex_unlock(&dev->mlock);
		return -EIO;
	}

	/* Don't allow power collapse until we release remote spinlock */
#ifdef FEATURE_SHLOCAL_SHI2C
	wake_lock( &sh_i2c_wake_lock );
#else	/* FEATURE_SHLOCAL_SHI2C */
	pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					dev->pdata->pm_lat);
#endif	/* FEATURE_SHLOCAL_SHI2C */
	msm_i2c_rmutex_lock(dev);
	if (adap == &dev->adap_pri)
		writel(0, dev->base + I2C_INTERFACE_SELECT);
	else
		writel(I2C_INTERFACE_SELECT_INTF_SELECT,
				dev->base + I2C_INTERFACE_SELECT);
		/* If other processor did some transactions, we may have
		 * interrupt pending. Clear it
		 */
		get_irq_chip(dev->irq)->ack(dev->irq);

	enable_irq(dev->irq);
#ifdef FEATURE_SHLOCAL_SHI2C

	sh_smem_common_ptr->sh_i2c_access_dev = msgs->addr << 1;
#endif /* FEATURE_SHLOCAL_SHI2C */
	while (rem) {
		addr = msgs->addr << 1;
		if (msgs->flags & I2C_M_RD)
			addr |= 1;

		spin_lock_irqsave(&dev->lock, flags);
		dev->msg = msgs;
		dev->rem = rem;
		dev->pos = 0;
		dev->err = 0;
		dev->flush_cnt = 0;
		dev->cnt = msgs->len;
		dev->complete = &complete;
		spin_unlock_irqrestore(&dev->lock, flags);

		if (check_busy) {
			ret = msm_i2c_poll_notbusy(dev);
			if (ret)
				ret = msm_i2c_recover_bus_busy(dev, adap);
				if (ret) {
					dev_err(dev->dev,
						"Error waiting for notbusy\n");
					goto out_err;
				}
			check_busy = 0;
		}

		if (rem == 1 && msgs->len == 0)
			addr |= I2C_WRITE_DATA_LAST_BYTE;

		/* Wait for WR buffer not full */
		ret = msm_i2c_poll_writeready(dev);
		if (ret) {
			ret = msm_i2c_recover_bus_busy(dev, adap);
			if (ret) {
				dev_err(dev->dev,
				"Error waiting for write ready before addr\n");
				goto out_err;
			}
		}

		/* special case for doing 1 byte read.
		 * There should be no scheduling between I2C controller becoming
		 * ready to read and writing LAST-BYTE to I2C controller
		 * This will avoid potential of I2C controller starting to latch
		 * another extra byte.
		 */
		if ((msgs->len == 1) && (msgs->flags & I2C_M_RD)) {
			uint32_t retries = 0;
			spin_lock_irqsave(&dev->lock, flags);

			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
				dev->base + I2C_WRITE_DATA);

			/* Poll for I2C controller going into RX_DATA mode to
			 * ensure controller goes into receive mode.
			 * Just checking write_buffer_full may not work since
			 * there is delay between the write-buffer becoming
			 * empty and the slave sending ACK to ensure I2C
			 * controller goes in receive mode to receive data.
			 */
			while (retries != 2000) {
				uint32_t status = readl(dev->base + I2C_STATUS);

					if ((status & I2C_STATUS_RX_DATA_STATE)
						== I2C_STATUS_RX_DATA_STATE)
						break;
				retries++;
			}
			if (retries >= 2000) {
				dev->rd_acked = 0;
				spin_unlock_irqrestore(&dev->lock, flags);
				/* 1-byte-reads from slow devices in interrupt
				 * context
				 */
				goto wait_for_int;
			}

			dev->rd_acked = 1;
			writel(I2C_WRITE_DATA_LAST_BYTE,
					dev->base + I2C_WRITE_DATA);
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			writel(I2C_WRITE_DATA_ADDR_BYTE | addr,
					 dev->base + I2C_WRITE_DATA);
		}
		/* Polling and waiting for write_buffer_empty is not necessary.
		 * Even worse, if we do, it can result in invalid status and
		 * error if interrupt(s) occur while polling.
		 */

		/*
		 * Now that we've setup the xfer, the ISR will transfer the data
		 * and wake us up with dev->err set if there was an error
		 */
wait_for_int:

		timeout = wait_for_completion_timeout(&complete, HZ);
		if (!timeout) {
			dev_err(dev->dev, "Transaction timed out\n");
			writel(I2C_WRITE_DATA_LAST_BYTE,
				dev->base + I2C_WRITE_DATA);
			msleep(100);
			/* FLUSH */
			readl(dev->base + I2C_READ_DATA);
			readl(dev->base + I2C_STATUS);
			ret = -ETIMEDOUT;
			goto out_err;
		}
		if (dev->err) {
			dev_err(dev->dev,
				"Error during data xfer (%d)\n",
				dev->err);
			ret = dev->err;
			goto out_err;
		}

		if (msgs->flags & I2C_M_RD)
			check_busy = 1;

		msgs++;
		rem--;
	}

	ret = num;

#ifdef FEATURE_SHLOCAL_SHI2C
	if( sh_i2c_clk_status_check( dev ) < 0 ){
		ret = -EIO;
	}
#endif /* FEATURE_SHLOCAL_SHI2C */
 out_err:
	spin_lock_irqsave(&dev->lock, flags);
	dev->complete = NULL;
	dev->msg = NULL;
	dev->rem = 0;
	dev->pos = 0;
	dev->err = 0;
	dev->flush_cnt = 0;
	dev->cnt = 0;
	spin_unlock_irqrestore(&dev->lock, flags);
	disable_irq(dev->irq);
#ifdef FEATURE_SHLOCAL_SHI2C

	if( ret < 0){
		remote_spin_lock_irqsave(&dev->rspin_lock, flags);
		*(dev->pdata->rmutex) = SH_I2C_RECOVERY_REQ_STATE;
		remote_spin_unlock_irqrestore(&dev->rspin_lock, flags);
		sh_i2c_recoveryreq_a_to_m();
	}else{
		sh_i2c_update_err_info( sh_smem_common_ptr->sh_i2c_access_dev );
		#ifdef SH_I2C_WAIT_NEED
		udelay( SH_I2C_WAIT );
		#endif /* SH_I2C_WAIT_NEED */
		msm_i2c_rmutex_unlock(dev);
	}

#endif /* FEATURE_SHLOCAL_SHI2C */
#ifdef FEATURE_SHLOCAL_SHI2C
	wake_unlock( &sh_i2c_wake_lock );
#else	/* FEATURE_SHLOCAL_SHI2C */
	pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					PM_QOS_DEFAULT_VALUE);
#endif	/* FEATURE_SHLOCAL_SHI2C */
	mutex_unlock(&dev->mlock);
	return ret;
}

static u32
msm_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm msm_i2c_algo = {
	.master_xfer	= msm_i2c_xfer,
	.functionality	= msm_i2c_func,
};

static int
msm_i2c_probe(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev;
	struct resource		*mem, *irq, *ioarea;
	int ret;
	int fs_div;
	int hs_div;
	int i2c_clk;
	int clk_ctl;
	struct clk *clk;
	struct msm_i2c_platform_data *pdata;

	printk(KERN_INFO "msm_i2c_probe\n");

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}
	clk = clk_get(&pdev->dev, "i2c_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Could not get clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get_failed;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	if (!pdata->msm_i2c_config_gpio) {
		dev_err(&pdev->dev, "config_gpio function not initialized\n");
		ret = -ENOSYS;
		goto err_clk_get_failed;
	}
	/* We support frequencies upto FAST Mode(400KHz) */
	if (pdata->clk_freq <= 0 || pdata->clk_freq > 400000) {
		dev_err(&pdev->dev, "clock frequency not supported\n");
		ret = -EIO;
		goto err_clk_get_failed;
	}

	dev = kzalloc(sizeof(struct msm_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->clk = clk;
	dev->pdata = pdata;
	dev->base = ioremap(mem->start, (mem->end - mem->start) + 1);
	if (!dev->base) {
		ret = -ENOMEM;
		goto err_ioremap_failed;
	}

	dev->one_bit_t = USEC_PER_SEC/pdata->clk_freq;
	spin_lock_init(&dev->lock);
	platform_set_drvdata(pdev, dev);

#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
	clk_enable(clk);
#endif /* FEATURE_SHLOCAL_SHI2C */
	if (pdata->rmutex != NULL)
		remote_spin_lock_init(&dev->rspin_lock, pdata->rsl_id);
	/* I2C_HS_CLK = I2C_CLK/(3*(HS_DIVIDER_VALUE+1) */
	/* I2C_FS_CLK = I2C_CLK/(2*(FS_DIVIDER_VALUE+3) */
	/* FS_DIVIDER_VALUE = ((I2C_CLK / I2C_FS_CLK) / 2) - 3 */
	i2c_clk = 19200000; /* input clock */
	fs_div = ((i2c_clk / pdata->clk_freq) / 2) - 3;
	hs_div = 3;
	clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
	writel(clk_ctl, dev->base + I2C_CLK_CTL);
	printk(KERN_INFO "msm_i2c_probe: clk_ctl %x, %d Hz\n",
	       clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3)));

	i2c_set_adapdata(&dev->adap_pri, dev);
	dev->adap_pri.algo = &msm_i2c_algo;
	strlcpy(dev->adap_pri.name,
		"MSM I2C adapter-PRI",
		sizeof(dev->adap_pri.name));

	dev->adap_pri.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adap_pri);
	if (ret) {
		dev_err(&pdev->dev, "Primary i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	i2c_set_adapdata(&dev->adap_aux, dev);
	dev->adap_aux.algo = &msm_i2c_algo;
	strlcpy(dev->adap_aux.name,
		"MSM I2C adapter-AUX",
		sizeof(dev->adap_aux.name));

	dev->adap_aux.nr = pdev->id + 1;
	ret = i2c_add_numbered_adapter(&dev->adap_aux);
	if (ret) {
		dev_err(&pdev->dev, "auxiliary i2c_add_adapter failed\n");
		i2c_del_adapter(&dev->adap_pri);
		goto err_i2c_add_adapter_failed;
	}
	ret = request_irq(dev->irq, msm_i2c_interrupt,
			IRQF_TRIGGER_RISING, pdev->name, dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
#ifdef FEATURE_SHLOCAL_SHI2C
	wake_lock_init(&sh_i2c_wake_lock 
					, WAKE_LOCK_SUSPEND
					, "sh_i2c");
#else  /* FEATURE_SHLOCAL_SHI2C */
	pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c",
					PM_QOS_DEFAULT_VALUE);
#endif /* FEATURE_SHLOCAL_SHI2C */
	disable_irq(dev->irq);
	dev->suspended = 0;
	mutex_init(&dev->mlock);
	/* Config GPIOs for primary and secondary lines */
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
	pdata->msm_i2c_config_gpio(dev->adap_pri.nr, 1);
	pdata->msm_i2c_config_gpio(dev->adap_aux.nr, 1);
#endif /* FEATURE_SHLOCAL_SHI2C */

#ifdef FEATURE_SHLOCAL_SHI2C
	sh_i2c_smem_init();
#endif /* FEATURE_SHLOCAL_SHI2C */

	return 0;

/*	free_irq(dev->irq, dev); */
err_request_irq_failed:
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
err_i2c_add_adapter_failed:
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
	clk_disable(clk);
#endif /* FEATURE_SHLOCAL_SHI2C */
	iounmap(dev->base);
err_ioremap_failed:
	kfree(dev);
err_alloc_dev_failed:
	clk_put(clk);
err_clk_get_failed:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int
msm_i2c_remove(struct platform_device *pdev)
{
	struct msm_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);
#ifdef FEATURE_SHLOCAL_SHI2C
	wake_lock_destroy( &sh_i2c_wake_lock );
#else  /* FEATURE_SHLOCAL_SHI2C */
	pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY, "msm_i2c");
#endif /* FEATURE_SHLOCAL_SHI2C */
	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adap_pri);
	i2c_del_adapter(&dev->adap_aux);
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
	clk_disable(dev->clk);
#endif /* FEATURE_SHLOCAL_SHI2C */

	clk_put(dev->clk);
	iounmap(dev->base);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
#ifdef FEATURE_SHLOCAL_SHI2C

	if( sh_i2c_endpoint != NULL ){
		msm_rpc_close( sh_i2c_endpoint );
	}

#endif /* FEATURE_SHLOCAL_SHI2C */

	return 0;
}

static int msm_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	/* Wait until current transaction finishes
	 * Make sure remote lock is released before we suspend
	 */
	if (dev) {
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
		mutex_lock(&dev->mlock);
		dev->suspended = 1;
		mutex_unlock(&dev->mlock);
#endif /* FEATURE_SHLOCAL_SHI2C */
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
		clk_disable(dev->clk);
#endif /* FEATURE_SHLOCAL_SHI2C */
	}

	return 0;
}

static int msm_i2c_resume(struct platform_device *pdev)
{
	struct msm_i2c_dev *dev = platform_get_drvdata(pdev);
	if (dev) {
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
		clk_enable(dev->clk);
#endif /* FEATURE_SHLOCAL_SHI2C */
#ifdef FEATURE_SHLOCAL_SHI2C

#else  /* FEATURE_SHLOCAL_SHI2C */
		dev->suspended = 0;
#endif /* FEATURE_SHLOCAL_SHI2C */
	}
	return 0;
}

static struct platform_driver msm_i2c_driver = {
	.probe		= msm_i2c_probe,
	.remove		= msm_i2c_remove,
	.suspend	= msm_i2c_suspend,
	.resume		= msm_i2c_resume,
	.driver		= {
		.name	= "msm_i2c",
		.owner	= THIS_MODULE,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
msm_i2c_init_driver(void)
{
	return platform_driver_register(&msm_i2c_driver);
}
subsys_initcall(msm_i2c_init_driver);

static void __exit msm_i2c_exit_driver(void)
{
	platform_driver_unregister(&msm_i2c_driver);
}
module_exit(msm_i2c_exit_driver);

