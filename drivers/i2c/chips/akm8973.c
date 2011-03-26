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
 * This code is based on akm8973.c.
 * The original copyright and notice are described below.
*/


/* drivers/i2c/chips/akm8973.c - akm8973 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/i2c/akm8973.h>
#include <sharp/shterm_k.h>

#define CONFIG_ANDROID_POWER

#ifdef CONFIG_ANDROID_POWER
#include <linux/earlysuspend.h>
#endif

#define DEBUG 0
#define MAX_FAILURE_COUNT 10

#define	DAC_REQUEST		0
#define	DAC_COMPLETE	1

#define	I2C_RETRY		3

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static struct i2c_client *this_client;
struct sh_i2c_compass_platform_data *poSetupData=NULL;


struct akm8973_data {
	struct input_dev *input_dev;
//	struct work_struct work;
#ifdef CONFIG_ANDROID_POWER
	struct early_suspend early_suspend;
#endif
};

/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
//static struct mutex sense_data_mutex;

//static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

//static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;

//static int failure_count = 0;

static short akmd_delay = 0;

#ifdef CONFIG_ANDROID_POWER
static atomic_t suspend_flag = ATOMIC_INIT(0);
#endif

//static struct akm8973_platform_data *pdata;

/* following are the sysfs callback functions */

#define config_ctrl_reg(name,address) \
static ssize_t name##_show(struct device *dev, struct device_attribute *attr, \
			   char *buf) \
{ \
	struct i2c_client *client = to_i2c_client(dev); \
        return sprintf(buf, "%u\n", i2c_smbus_read_byte_data(client,address)); \
} \
static ssize_t name##_store(struct device *dev, struct device_attribute *attr, \
			    const char *buf,size_t count) \
{ \
	struct i2c_client *client = to_i2c_client(dev); \
	unsigned long val = simple_strtoul(buf, NULL, 10); \
	if (val > 0xff) \
		return -EINVAL; \
	i2c_smbus_write_byte_data(client,address, val); \
        return count; \
} \
static DEVICE_ATTR(name, S_IWUSR | S_IRUGO, name##_show, name##_store)

config_ctrl_reg(ms1, AKECS_REG_MS1);

static int AKI2C_RxData(char *rxData, int length)
{
	int nI;
	char rxData_Add[0];
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData_Add,
		},
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		},
	};
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

    /* リトライ対策 */
    rxData_Add[0] = rxData[0];

	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		if (i2c_transfer(this_client->adapter, msgs, 2) >= 0)
		{
			return 0;
		}
		else
		{
			printk(KERN_ERR "AKI2C_RxData: transfer error:%d\n",nI);
		}
	}
	return -EIO;
}

static int AKI2C_TxData(char *txData, int length)
{
	int nI;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		if (i2c_transfer(this_client->adapter, msg, 1) >= 0)
		{
			return 0;
		}
		else
		{
			printk(KERN_ERR "AKI2C_TxData: transfer error:%d\n",nI);
		}
	}
	return -EIO;
}

static int AKECS_Init(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return 0;
}

static void AKECS_Reset(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    gpio_direction_output(poSetupData->gpio_rst, 0);
    udelay(10);
    gpio_direction_output(poSetupData->gpio_rst, 1);
    mdelay(20);    

}

static int AKECS_StartMeasure(void)
{
//	struct akm8973_data *data = i2c_get_clientdata(this_client);
	char buffer[2];
//	int ret;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	/* Set measure mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
//	ret = AKI2C_TxData(buffer, 2);
//	if (ret < 0)
//		return ret;

//	msleep(20);
//	schedule_work(&data->work);
//	return ret;
}

static int AKECS_PowerDown(void)
{
	char buffer[2];
	int ret;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	/* Set powerdown mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_POWERDOWN;
	/* Set data */
	ret = AKI2C_TxData(buffer, 2);
	if (ret < 0)
		return ret;

	/* Dummy read for clearing INT pin */
	buffer[0] = AKECS_REG_TMPS;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int AKECS_StartE2PRead(void)
{
	char buffer[2];
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	/* Set E2P mode */
	buffer[0] = AKECS_REG_MS1;
	buffer[1] = AKECS_MODE_E2P_READ;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

#if 0
static int AKECS_GetData(void)
{
	char buffer[RBUFF_SIZE + 1];
	int ret;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	memset(buffer, 0, RBUFF_SIZE + 1);

	// read C0 - C4
	buffer[0] = AKECS_REG_ST;
	ret = AKI2C_RxData(buffer, RBUFF_SIZE + 1);
	if (ret < 0)
		return ret;

	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, buffer, sizeof(buffer));
	atomic_set(&data_ready, 1);
	wake_up(&data_ready_wq);
	mutex_unlock(&sense_data_mutex);

	return 0;
}
#endif

static int AKECS_SetMode(char mode)
{
	int ret;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	switch (mode) {
	case AKECS_MODE_MEASURE:
		ret = AKECS_StartMeasure();
		break;
	case AKECS_MODE_E2P_READ:
		ret = AKECS_StartE2PRead();
		break;
	case AKECS_MODE_POWERDOWN:
		ret = AKECS_PowerDown();
		break;
	default:
		return -EINVAL;
	}

	/* wait at least 300us after changing mode */
	msleep(1);
	return ret;
}

static int AKECS_TransRBuff(char *rbuf, int size)
{
#if 0
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	wait_event_interruptible_timeout(data_ready_wq,
					 atomic_read(&data_ready), 1000);

	if (!atomic_read(&data_ready)) {
#ifdef CONFIG_ANDROID_POWER
		if (!atomic_read(&suspend_flag)) {
#endif
			printk(KERN_ERR "AKECS_TransRBUFF: Data not ready\n");
			failure_count++;
			if (failure_count >= MAX_FAILURE_COUNT) {
				printk(KERN_ERR
				       "AKECS_TransRBUFF: successive %d failure.\n",
				       failure_count);
				atomic_set(&open_flag, -1);
				wake_up(&open_wq);
				failure_count = 0;
			}
#ifdef CONFIG_ANDROID_POWER
		}
#endif
		return -1;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(&rbuf[1], &sense_data[1], size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);


	failure_count = 0;
	return 0;
#endif

	if(size < RBUFF_SIZE + 1)
	  return -EINVAL;

	// read C0 - C4
	rbuf[0] = AKECS_REG_ST;
	return AKI2C_RxData(rbuf, RBUFF_SIZE + 1);

}

static void AKECS_ReportAccuracy_Value(short *rbuf)
{
	struct akm8973_data *data = i2c_get_clientdata(this_client);
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	/* Report magnetic sensor Accuracy */
	input_report_abs(data->input_dev, ABS_RUDDER, rbuf[0]);

	/* Report acceleration sensor Accuracy */
	input_report_abs(data->input_dev, ABS_WHEEL, rbuf[1]);

	input_sync(data->input_dev);
}

static void AKECS_Report_Value(short *rbuf)
{
	struct akm8973_data *data = i2c_get_clientdata(this_client);
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
#if DEBUG
	printk("AKECS_Report_Value: yaw = %d, pitch = %d, roll = %d\n",
	    rbuf[0], rbuf[1], rbuf[2]);
	printk("                    tmp = %d, m_stat= %d, g_stat=%d\n",
	    rbuf[3], rbuf[4], rbuf[5]);
	printk("      Acceleration:   x = %d LSB, y = %d LSB, z = %d LSB\n",
	    rbuf[6], rbuf[7], rbuf[8]);
	printk("          Magnetic:   x = %d LSB, y = %d LSB, z = %d LSB\n\n",
	    rbuf[9], rbuf[10], rbuf[11]);
#endif
	/* Report magnetic sensor information */
	if (atomic_read(&m_flag)) {
		input_report_abs(data->input_dev, ABS_RX, rbuf[0]);
		input_report_abs(data->input_dev, ABS_RY, rbuf[1]);
		input_report_abs(data->input_dev, ABS_RZ, rbuf[2]);
		input_report_abs(data->input_dev, ABS_RUDDER, rbuf[4]);
	}

	/* Report acceleration sensor information */
	if (atomic_read(&a_flag)) {
		input_report_abs(data->input_dev, ABS_X, rbuf[6]);
		input_report_abs(data->input_dev, ABS_Y, rbuf[7]);
		input_report_abs(data->input_dev, ABS_Z, rbuf[8]);
		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]);
	}

	/* Report temperature information */
	if (atomic_read(&t_flag)) {
		input_report_abs(data->input_dev, ABS_THROTTLE, rbuf[3]);
	}

	if (atomic_read(&mv_flag)) {
		input_report_abs(data->input_dev, ABS_HAT0X, rbuf[9]);
		input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[10]);
		input_report_abs(data->input_dev, ABS_BRAKE, rbuf[11]);
	}

	input_sync(data->input_dev);
}

static int AKECS_GetOpenStatus(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&t_flag, 0);
	atomic_set(&mv_flag, 0);
}

static int AKSUB_DacAdjust(uint8_t *Data,uint8_t *Dac)
{
	uint8_t ret[3]={0};
	uint8_t cnt;
	int data;

	for(cnt=0;cnt<3;cnt++)
	{
		if(((128+16) >= *(Data+cnt)) && ((128-16) <= *(Data+cnt)))
		{
			data = abs(*(Data+cnt) - 128);

			if(data < 12)
			{
				ret[cnt] = DAC_COMPLETE;
				continue;
			}
		}

		if(128 > *(Data+cnt))
		{
			if(*(Dac+cnt) & 0x80)
			{
				if(*(Dac+cnt) == 0xFF)
				{
					ret[cnt] = DAC_COMPLETE;
					continue;
				}
				*(Dac+cnt) = (*(Dac+cnt) + 1) | 0x80;
			}
			else
			{
				if(*(Dac+cnt) == 0x00)
				{
					*(Dac+cnt) = 0x81;
					ret[cnt] = DAC_REQUEST;
					continue;
				}
				*(Dac+cnt) = (*(Dac+cnt) - 1);
			}
		}
		else if(128 < *(Data+cnt))
		{
			if(*(Dac+cnt) & 0x80)
			{
				if(*(Dac+cnt) == 0x80)
				{
					*(Dac+cnt) = 0x01;
					ret[cnt] = DAC_REQUEST;
					continue;
				}
				*(Dac+cnt) = (*(Dac+cnt) - 1) | 0x80;
			}
			else
			{
				if(*(Dac+cnt) == 0x7F)
				{
					ret[cnt] = DAC_COMPLETE;
					continue;
				}
				*(Dac+cnt) = (*(Dac+cnt) + 1);
			}
		}

		ret[cnt] = DAC_REQUEST;
	}
	return (ret[0] & ret[1] & ret[2]);
}


static int AKECS_FactoryShipmentTest(void)
{
	uint8_t GainData[3]={0};
	uint8_t DacData[3]={0};
	uint8_t buffer[10];
	int     loop_cnt=0;
	int ret;

#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	/* AK8973 RESET */
	AKECS_Reset();

	/* Set EEPROM READ mode */
	if(AKECS_StartE2PRead() != 0)
	{
		return -EIO;
	}

	/* Wait over 300us */
    udelay(300);

	/* Read EEPROM  EHXGA, EHYGA, EHZGA  */
	GainData[0] = AKECS_REG_EHXGA;
	/* Read data */
	if(AKI2C_RxData(GainData, 3))
        return -EIO;

	/* Set to PowerDown mode */
	if(AKECS_PowerDown())
	{
	    return -EIO;
	}

	/* Set HXGA, HYGA, HZGA register */
    buffer[0] = AKECS_REG_HXGA;
    buffer[1] = GainData[0];
    buffer[2] = GainData[1];
    buffer[3] = GainData[2];

#if DEBUG
	printk(KERN_INFO "GAIN: 0x%X, 0x%X, 0x%X\n", GainData[0],GainData[1],GainData[2]);
#endif

	if(AKI2C_TxData(buffer,4))
	{
		return -EIO;
	}

	/* Read ST, TMPS, H1X, H1Y, H1Z register */
	buffer[0] = AKECS_REG_ST;
	/* Read data */
	if(AKI2C_RxData(buffer, 5))
	{
		return -EIO;
	}
	
	/* ALL DATA == 0x00 ?  */
	if(0x00 != (buffer[0] | buffer[1] | buffer[2] | buffer[3] | buffer[4]))
	{
		return -EIO;
	}

	/************************************************/
	/*	Step2										*/
	/************************************************/
	/* Set to measurement mode */
	if(AKECS_StartMeasure() != 0)
	{
		return -EIO;
	}

	/* Wait */
	msleep(15);

	/* Read ST register */
	buffer[0] = AKECS_REG_ST;
	/* Read data */
	if(AKI2C_RxData(buffer, 1))
	{
		return -EIO;
	}

	/* ST register INT bit ON?  */
	if(0x01 == (buffer[0] & 0x01))
	{
		/* Read TMPS register */
		buffer[0] = AKECS_REG_TMPS;
		/* Read data */
		if(AKI2C_RxData(buffer, 1))
		{
			return -EIO;
		}
	}
	else
	{
		return -EIO;
	}

	do{
		if(loop_cnt > 256)
		{
			return -EIO;
		}

		/* Set to measurement mode */
		if(AKECS_StartMeasure() != 0)
		{
			return -EIO;
		}

		/* Wait */
		msleep(15);

		/* Read TMPS, H1X, H1Y, H1Z register */
		buffer[0] = AKECS_REG_TMPS;
		/* Read data */
		if(AKI2C_RxData(buffer, 4))
		{
			return -EIO;
		}
#if DEBUG
	printk(KERN_INFO "DATA: 0x%X, 0x%X, 0x%X\n", buffer[1],buffer[2],buffer[3]);
#endif

		ret = AKSUB_DacAdjust(&buffer[1],&DacData[0]);


		/* Set HXDA, HYDA, HZDA register */
	    buffer[0] = AKECS_REG_HXDA;
	    buffer[1] = DacData[0];
	    buffer[2] = DacData[1];
	    buffer[3] = DacData[2];

#if DEBUG
	printk(KERN_INFO "DAC: 0x%X, 0x%X, 0x%X\n", buffer[1],buffer[2],buffer[3]);
#endif

		if(AKI2C_TxData(buffer,4))
		{
			return -EIO;
		}

		loop_cnt++;
	} while(ret != DAC_COMPLETE);



	return 0;
}


static int akm_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
            shterm_k_set_info(SHTERM_INFO_ACCELE, 1);
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
    shterm_k_set_info(SHTERM_INFO_ACCELE, 0);
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static int
akm_aot_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		akmd_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = akmd_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static int akmd_open(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	AKECS_CloseDone();
	return 0;
}

static int
akmd_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
#if DEBUG
	int i;
#endif
	void __user *argp = (void __user *)arg;

	char msg[RBUFF_SIZE + 1], rwbuf[16];//, numfrq[2];
	int ret = -1, status;
	short mode, value[12], delay; /* step_count,*/
//	char *pbuffer = 0;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_MODE:
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_ACCURACY:
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;
		break;

	//case ECS_IOCTL_SET_STEP_CNT:
	//	if (copy_from_user(&step_count, argp, sizeof(step_count)))
	//		return -EFAULT;
	//	break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_INIT:
#if DEBUG
		printk("ECS_IOCTL_INIT %x\n", cmd);
#endif
		ret = AKECS_Init();
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
#if DEBUG
		printk("ECS_IOCTL_RESET %x\n", cmd);
#endif
		AKECS_Reset();
		break;
	case ECS_IOCTL_READ:
#if DEBUG
		printk("ECS_IOCTL_READ %x\n", cmd);
		printk(" len %02x:", rwbuf[0]);
		printk(" addr %02x:", rwbuf[1]);
#endif
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
#if DEBUG
		for(i=0; i<rwbuf[0]; i++){
			printk(" %02x", rwbuf[i+1]);
		}
		printk(" ret = %d\n", ret);
#endif
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
#if DEBUG
		printk("ECS_IOCTL_WRITE %x\n", cmd);
		printk(" len %02x:", rwbuf[0]);
		for(i=0; i<rwbuf[0]; i++){
			printk(" %02x", rwbuf[i+1]);
		}
#endif
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
#if DEBUG
		printk(" ret = %d\n", ret);
#endif
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
#if DEBUG
		printk("ECS_IOCTL_SET_MODE %x mode=%x\n", cmd, mode);
#endif
		ret = AKECS_SetMode((char)mode);
#if DEBUG
		printk(" ret = %d\n", ret);
#endif
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GETDATA:
#if DEBUG
		printk("ECS_IOCTL_GETDATA %x\n", cmd);
#endif
		ret = AKECS_TransRBuff(msg, RBUFF_SIZE+1);
#if DEBUG
		printk(" ret = %d\n", ret);
#endif
		if (ret < 0)
			return ret;
#if DEBUG
		for(i=0; i<ret; i++){
			printk(" %02x", msg[i]);
		}
		printk("\n");
#endif
		break;
	case ECS_IOCTL_SET_YPR:
#if DEBUG
//		printk("ECS_IOCTL_SET_YPR %x ypr=%x\n", cmd, value);
#endif
		AKECS_Report_Value(value);
		break;
	case ECS_IOCTL_SET_ACCURACY:
#if DEBUG
		printk("ECS_IOCTL_SET_ACCURACY\n");
#endif
		AKECS_ReportAccuracy_Value(value);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
#if DEBUG
		printk("ECS_IOCTL_GET_OPEN_STATUS %x start\n", cmd);
#endif
		status = AKECS_GetOpenStatus();
#if DEBUG
		printk("ECS_IOCTL_GET_OPEN_STATUS %x end status=%x\n", cmd, status);
#endif
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
#if DEBUG
		printk("ECS_IOCTL_GET_CLOSE_STATUS %x start\n", cmd);
#endif
		status = AKECS_GetCloseStatus();
#if DEBUG
		printk("ECS_IOCTL_GET_CLOSE_STATUS %x end status=%x\n", cmd, status);
#endif
		break;
//	case ECS_IOCTL_GET_CALI_DATA:
//		pbuffer = get_akm_cal_ram();
//#if DEBUG
//		printk("ECS_IOCTL_GET_CALI_DATA %x pbuffer=%x\n", cmd, pbuffer);
//#endif
//		break;
	case ECS_IOCTL_GET_DELAY:
		delay = akmd_delay;
#if DEBUG
		printk("ECS_IOCTL_GET_DELAY %x delay=%x\n", cmd, delay);
#endif
		break;
	case ECS_IOCTL_FACTORY_TEST:
		ret = AKECS_FactoryShipmentTest();
		if (ret < 0)
			return ret;
		break;
	default:
#if DEBUG
		printk("Unknown cmd %x\n", cmd);
#endif
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GETDATA:
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
		break;
//	case ECS_IOCTL_GET_NUMFRQ:
//		if (copy_to_user(argp, &numfrq, sizeof(numfrq)))
//			return -EFAULT;
//		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;
//	case ECS_IOCTL_GET_CALI_DATA:
//		if (copy_to_user(argp, pbuffer, MAX_CALI_SIZE))
//			return -EFAULT;
//		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

#if 0
static void akm_work_func(struct work_struct *work)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (AKECS_GetData() < 0)
		printk(KERN_ERR "akm_work_func: Get data failed\n");
}
#endif

#ifdef CONFIG_ANDROID_POWER
static void akm8973_early_suspend(struct early_suspend *handler)
{
#if DEBUG
	printk(KERN_ERR "%s\n", __FUNCTION__);
#endif
	atomic_set(&suspend_flag, 1);
	if (atomic_read(&open_flag) == 2)
		AKECS_SetMode(AKECS_MODE_POWERDOWN);

	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
}

static void akm8973_early_resume(struct early_suspend *handler)
{
#if DEBUG
	printk(KERN_ERR "%s\n", __FUNCTION__);
#endif

	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);
}
#endif /* CONFIG_ANDROID_POWER */

static int akm8973_init_client(struct i2c_client *client)
{
//	struct akm8973_data *data;
//	int ret;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

//	data = i2c_get_clientdata(client);
//
//	mutex_init(&sense_data_mutex);
//
//	pdata = client->dev.platform_data;
//	if (pdata == NULL) {
//		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
//		if (pdata == NULL) {
//			ret = -ENOMEM;
//			goto err_alloc_data_failed;
//		} else {
//			pdata->reset = ECS_RST;
//			pdata->clk_on = ECS_CLK_ON;
//			pdata->intr = ECS_INTR;
//		}
//	}
//
//	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&t_flag, 0);
	atomic_set(&mv_flag, 0);

	return 0;

//err_alloc_data_failed:
//	return ret;
}

static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.ioctl = akmd_ioctl,
};

static struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.ioctl = akm_aot_ioctl,
};

static struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8973_aot",
	.fops = &akm_aot_fops,
};

static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8973_dev",
	.fops = &akmd_fops,
};

static int akm8973_ConfigGPIO(struct sh_i2c_compass_platform_data *poSetupData)
{
	if(poSetupData == NULL)
		return -EINVAL;
	return poSetupData->gpio_setup();
}

static int akm8973_ReleaseGPIO(struct sh_i2c_compass_platform_data *poSetupData)
{
	if(poSetupData == NULL)
		return -EINVAL;
	/* GPIOの解放 */
	poSetupData->gpio_shutdown();
	return 0;
}

static int akm8973_Initialize(struct sh_i2c_compass_platform_data *poSetupData)
{
	uint8_t GainData[3];
    uint8_t buffer[4];

#if DEBUG
	printk(KERN_DEBUG "[akm8973]Initialize()\n");
#endif
	/* リセットを行う */
    AKECS_Reset();

    /* EEPROMアクセスモード */
	if(AKECS_StartE2PRead())
	{
#if DEBUG
        printk(KERN_DEBUG "[COMPASS]AKECS_StartE2PRead-->Error\n");
#endif
   	    return -EIO;
    }
 
    udelay(300);

	/* EEPROM GAIN DATA */
	GainData[0] = AKECS_REG_EHXGA;
	/* Read data */
	if(AKI2C_RxData(GainData, 3))
        return -EIO;

#if DEBUG
    printk(KERN_DEBUG "[COMPASS]GainData %Xh,%Xh,%Xh\n",GainData[0],GainData[1],GainData[2]);
#endif

    /* パワーダウンモード */
	if(AKECS_PowerDown())
	{
#if DEBUG
        printk(KERN_DEBUG "[COMPASS]AKECS_PowerDown-->Error\n");
#endif
   	    return -EIO;
    }

    buffer[0] = AKECS_REG_HXGA;
    buffer[1] = GainData[0];

	if(AKI2C_TxData(buffer,2))
	{
#if DEBUG
        printk(KERN_DEBUG "[COMPASS]I2cWriteOne-->Error\n");
#endif
		return -EIO;
	}

    buffer[0] = AKECS_REG_HYGA;
    buffer[1] = GainData[1];
	if(AKI2C_TxData(buffer,2))
	{
#if DEBUG
        printk(KERN_DEBUG "[COMPASS]I2cWriteOne-->Error\n");
#endif
		return -EIO;
	}

    buffer[0] = AKECS_REG_HZGA;
    buffer[1] = GainData[2];
	if(AKI2C_TxData(buffer,2))
	{
#if DEBUG
        printk(KERN_DEBUG "[COMPASS]I2cWriteOne-->Error\n");
#endif
		return -EIO;
	}

	return 0;
}

int akm8973_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct akm8973_data *akm;
	int err = -ENODEV;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	akm = kzalloc(sizeof(struct akm8973_data), GFP_KERNEL);
	if (!akm) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

//	INIT_WORK(&akm->work, akm_work_func);
	i2c_set_clientdata(client, akm);
	akm8973_init_client(client);
	this_client = client;
	poSetupData = client->dev.platform_data;

    if(akm8973_ConfigGPIO(poSetupData))
        goto exit_GPIO_config_failed;

    if(akm8973_Initialize(poSetupData))
        goto exit_GPIO_config_failed;

	akm->input_dev = input_allocate_device();

	if (!akm->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "akm8973_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, akm->input_dev->evbit);

	/* yaw */
	input_set_abs_params(akm->input_dev, ABS_RX, 0, 23040, 0, 0);
	/* pitch */
	input_set_abs_params(akm->input_dev, ABS_RY, -11520, 11520, 0, 0);
	/* roll */
	input_set_abs_params(akm->input_dev, ABS_RZ, -5760, 5760, 0, 0);
	/* x-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_X, -5760, 5760, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_Y, -5760, 5760, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(akm->input_dev, ABS_Z, -5760, 5760, 0, 0);
	/* temparature */
	input_set_abs_params(akm->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(akm->input_dev, ABS_RUDDER, 0, 3, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(akm->input_dev, ABS_WHEEL, 0, 3, 0, 0);
	/* x-axis of raw magnetic vector */
	input_set_abs_params(akm->input_dev, ABS_HAT0X, -2048, 2032, 0, 0);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(akm->input_dev, ABS_HAT0Y, -2048, 2032, 0, 0);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(akm->input_dev, ABS_BRAKE, -2048, 2032, 0, 0);

	akm->input_dev->name = "compass";

	err = input_register_device(akm->input_dev);

	if (err) {
		printk(KERN_ERR
		       "akm8973_probe: Unable to register input device: %s\n",
		       akm->input_dev->name);
		goto exit_input_register_device_failed;
	}

	err = misc_register(&akmd_device);
	if (err) {
		printk(KERN_ERR "akm8973_probe: akmd_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = misc_register(&akm_aot_device);
	if (err) {
		printk(KERN_ERR
		       "akm8973_probe: akm_aot_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = device_create_file(&client->dev, &dev_attr_ms1);

#ifdef CONFIG_ANDROID_POWER
	akm->early_suspend.suspend = akm8973_early_suspend;
	akm->early_suspend.resume = akm8973_early_resume;
	register_early_suspend(&akm->early_suspend);
#endif
	return 0;

exit_misc_device_register_failed:
exit_input_register_device_failed:
	input_free_device(akm->input_dev);
exit_GPIO_config_failed:
    akm8973_ReleaseGPIO(poSetupData);
exit_input_dev_alloc_failed:
	kfree(akm);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

#if 0
static int akm8973_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	strlcpy(info->type, "akm8973", I2C_NAME_SIZE);
	return 0;
}
#endif

static int akm8973_remove(struct i2c_client *client)
{
	struct akm8973_data *akm = i2c_get_clientdata(client);
#if DEBUG
	printk(KERN_INFO "AK8973 compass driver: init\n");
#endif
	input_unregister_device(akm->input_dev);
//	i2c_detach_client(client);
	kfree(akm);
	return 0;
}

static const struct i2c_device_id akm8973_id[] = {
	{ SH_COMPS_I2C_DEVNAME, 0 },
	{ }
};

static struct i2c_driver akm8973_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = akm8973_probe,
	.remove = akm8973_remove,
	.id_table = akm8973_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SH_COMPS_I2C_DEVNAME,
		   },
//	.detect = akm8973_detect,
//	.address_data = &addr_data,
};

static int __init akm8973_init(void)
{
#if DEBUG
	printk(KERN_INFO "AK8973 compass driver: init\n");
#endif
	return i2c_add_driver(&akm8973_driver);
}

static void __exit akm8973_exit(void)
{
#if DEBUG
	printk(KERN_INFO "AK8973 compass driver: exit\n");
#endif
	i2c_del_driver(&akm8973_driver);
}

module_init(akm8973_init);
module_exit(akm8973_exit);

MODULE_AUTHOR("Hou-Kun Chen <hk_chen@htc.com>");
MODULE_DESCRIPTION("AK8973 compass driver");
MODULE_LICENSE("GPL v2");
