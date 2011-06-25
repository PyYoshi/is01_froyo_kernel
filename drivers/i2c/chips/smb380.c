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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/i2c/smb380.h>
#include <linux/hrtimer.h>

#define DEBUG 0

/*+-------------------------------------------------------------------------+*/
/*|	型宣言																	|*/
/*+-------------------------------------------------------------------------+*/
typedef struct i2c_record      I2cAccelRec;
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;
typedef struct work_struct      WorkStruct;
typedef struct input_dev        InputDev;
typedef struct device           Device;

#define	I2C_RETRY			3

struct i2c_record
{
                
    I2cClt  *mpoI2C_Clt;
	int		mnProductInfo;
//	char	mcPhysInfo[QKBD_PHYSLEN];
	int		mnIrqPin;
	int		(*mpfPinSetupFunc)(void);
	void	(*mpfPinShutdownFunc)(void);
	struct delayed_work moCmdQ;
	WorkStruct moIrqWork;
	WorkStruct moTimerWork;
};

typedef struct
{
	uint8_t	mbRegAdr;					/* レジスタアドレス */
	uint8_t mbData;						/* データ */
} I2cWriteData;

#define IRQNO(poKbdRec)  (MSM_GPIO_TO_INT(poKbdRec->mnIrqPin))

#define ACC_DATA_GET(data, acc_data) \
	acc_data = *data >> 6; \
	acc_data |= ((uint16_t)*(data+1) << 2); \
	acc_data <<= 6; \
	acc_data >>= 6;


static I2cClt *this_client;

static int __devinit Smb380_Probe(I2cClt *client, const I2cDevID *poDevId);


static const I2cWriteData goAccInitialize[] =
	{
		{	SMB380_REG_SMB380_CONF2,    0x00	},			/* ALLクリア */
		{	SMB380_REG_SMB380_CONF1,    0x00	},			/* ALLクリア */
		{	SMB380_REG_SMB380_CTRL,     0x01	},			/* スリープ許可 */
		{	0xFF,                       0x00	},			/* 終端 */
	};

static const I2cWriteData goAccEnable[] =
	{
#if 0
		{	0x0A,	0x01	},			/* スリープ許可 */
		{	0x15,	0x40	},			/* ALLクリア */
		{	0x11,	0x40	},			/* 3Shot */
		{	0x10,	0x06	},			/* 復帰割込み閾値設定 */
		{	0x0B,	0x40	},			/* AnyMotionを設定 */
		{	0x0A,	0x00	},			/* スリープ */
		{	0xFF,	0x00	},			/* 終端 */
#else
		{	0x15,	0x20	},			/* NewData IRQ Enable */
		{	0x0A,	0x00	},			/* スリープ解除 */
		{	0xFF,	0x00	},			/* 終端 */
#endif
	};
static const I2cWriteData goAccIntReset[] =
	{
		{	0x15,	0x00	},			/* ALLクリア */
		{	0x0B,	0x00	},			/* ALLクリア */
		{	0x0A,	0x40	},			/* RESET INT */
		{	0x0A,	0x01	},			/* スリープ */
		{	0xFF,	0x00	},			/* 終端 */
	};

#if 1
/*+-------------------------------------------------------------------------+*/
/*|	I2Cリード																|*/
/*+-------------------------------------------------------------------------+*/
static int Smb380_I2cRead(I2cClt *client, uint8_t bRegAdr, uint8_t *pbBuf, uint32_t dwLen)
{
	int nResult;
	int nI;
	unsigned char buf[10];

    buf[0] = bRegAdr;

	for(nI = 0; nI < I2C_RETRY; nI++)
	{
        nResult = i2c_master_send(client, buf, 1);
        if(nResult == 1)
        {
            nResult = i2c_master_recv(client, pbBuf, dwLen);
            if(nResult != dwLen)
            {
                printk(KERN_ERR "Smb380_I2cRead: receive error\n");
                continue;
            }
            return 0;
        }
        else
        {
            printk(KERN_ERR "Smb380_I2cRead: send error\n");
        }
    }

	return -1;
}

/*+-------------------------------------------------------------------------+*/
/*|	I2Cライト																|*/
/*+-------------------------------------------------------------------------+*/
static int Smb380_I2cWriteOne(I2cClt *client, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= client->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(client->adapter, oMsgs, 1);
		if(nResult == 1)
		{
//            printk(KERN_DEBUG "[SMB380]I2cWrite success(%02X,reg:%02X,Data:%02X)=%d\n", client->addr, bRegAdr, bData, nResult);
			return 0;
		}
		printk(KERN_ERR "Smb380_I2cWrite: send error\n");
	}
	return -1;
}

static int Smb380_I2cWriteAny(I2cClt *client, const I2cWriteData *poData)
{
	int nI;
	int nResult;

	/* 設定を行う */
	for(nI = 0; poData[nI].mbRegAdr != 0xFF; nI++)
	{
		nResult = Smb380_I2cWriteOne(client, poData[nI].mbRegAdr, poData[nI].mbData);
		if(nResult < 0)
		{
			return -1;
		}
	}
	return 0;
}
#endif


static int Smb380_open(struct inode *inode, struct file *filp)
{
	uint8_t bRegAdr;
	uint8_t bData;

#if DEBUG
	printk("%s\n", __func__);
#endif

    bRegAdr = SMB380_REG_SMB380_CTRL;
    bData = SMB380_MODE_SLEEP;

    if(Smb380_I2cWriteOne(this_client, bRegAdr, bData) < 0)
        return -EIO;

	return 0;
}


static int Smb380_release(struct inode *inode, struct file *filp)
{
	uint8_t bRegAdr;
	uint8_t bData;

#if DEBUG
printk("%s\n", __func__);
#endif

    bRegAdr = SMB380_REG_SMB380_CTRL;
    bData = SMB380_MODE_SLEEP;
    if(Smb380_I2cWriteOne(this_client, bRegAdr, bData) < 0)
        return -EIO;

	return 0;
}

static int Smb380_ioctl(struct inode *inode, struct file *filp,
			unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
	uint8_t bXYZ[6];
	short nXYZ[3];
	int nI;
	int nErr=0;
    uint8_t mode;
	uint8_t bRegAdr;
	uint8_t bData;

#if DEBUG
    printk("%s\n", __func__);
#endif

	switch (cmd) {
    case SMB380_READ_ACCEL_XYZ:
        if (copy_from_user(&nXYZ, argp, sizeof(nXYZ)))
            return -EFAULT;
		break;
	case SMB380_SET_RANGE:
	case SMB380_SET_MODE:
	case SMB380_SET_BANDWIDTH:
        if (copy_from_user(&mode, argp, sizeof(mode)))
            return -EFAULT;
		break;
    }

    switch (cmd) {
    case SMB380_READ_ACCEL_XYZ:
#if DEBUG
        printk(KERN_DEBUG "[SMB380]IO SMB380_READ_ACCEL_XYZ\n");
#endif
#if 0
        bRegAdr = SMB380_REG_X_AXIS_LSB;
        if(0 <= Smb380_I2cRead(this_client, bRegAdr, bXYZ, 6))
#else

        bRegAdr = SMB380_REG_X_AXIS_LSB;
        for(nI = 0; nI < 6; nI++)
        {
            if(0 > Smb380_I2cRead(this_client, bRegAdr, &bXYZ[nI], 1))
            {
                nErr=-EFAULT;
            }
            bRegAdr++;
        }

        if(nErr == 0)
#endif
        {
            for(nI = 0; nI < 3; nI++)
            {
                ACC_DATA_GET(&bXYZ[nI*2], nXYZ[nI]);
            }
#if DEBUG
            printk(KERN_DEBUG "[SMB380]Read X:%5d Y:%5d Z:%5d\n", nXYZ[0], nXYZ[1], nXYZ[2]);
#endif
            if(copy_to_user(argp,&nXYZ,sizeof(nXYZ))) /* データ返還 */
               return -EFAULT;
        }
        else
        {
#if DEBUG
            printk(KERN_DEBUG "[SMB380]i2c read error\n");
#endif
    		return -EIO;
        }
		break;
    case SMB380_SET_RANGE:
#if DEBUG
        printk(KERN_DEBUG "[SMB380]IO SMB380_SET_RANGE\n");
#endif
	    /* 設定を行う */
	    bRegAdr = SMB380_REG_RANGE_BWIDTH;
        if(Smb380_I2cRead(this_client, bRegAdr, &bData, 1) < 0)
		    return -EIO;

        bData &= 0xe7;
        bData |= (mode << 3);
        if(Smb380_I2cWriteOne(this_client, bRegAdr, bData) < 0)
    		return -EIO;

        /* 加速度安定更新待ち */
        mdelay(2);
		break;
    case SMB380_SET_MODE:
#if DEBUG
        printk(KERN_DEBUG "[SMB380]IO SMB380_SET_MODE\n");
#endif
        bRegAdr = SMB380_REG_SMB380_CTRL;
        bData = (mode & 0x01);
        if(Smb380_I2cWriteOne(this_client, bRegAdr, bData) < 0)
    		return -EIO;
		break;
	case SMB380_SET_BANDWIDTH:
#if DEBUG
        printk(KERN_DEBUG "[SMB380]IO SMB380_SET_BANDWIDTH\n");
#endif
	    /* 設定を行う */
	    bRegAdr = SMB380_REG_RANGE_BWIDTH;
        if(Smb380_I2cRead(this_client, bRegAdr, &bData, 1) < 0)
		    return -EIO;

        bData &= 0xf8;
        bData |= mode;
        if(Smb380_I2cWriteOne(this_client, bRegAdr, bData) < 0)
    		return -EIO;

        /* 加速度安定更新待ち */
        mdelay(2);
		break;
	default:
		break;
	}

	return 0;
}

static struct file_operations Smb380_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= Smb380_open,
	.release	= Smb380_release,
	.ioctl		= Smb380_ioctl,
};

static struct miscdevice Smb380_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smb380_dev",
	.fops = &Smb380_ctl_fops,
};


static int Smb380_ReleaseGPIO(I2cAccelRec *poAccelRec)
{
	if(poAccelRec == NULL)
		return -EINVAL;
	/* GPIOの解放 */
	dev_info(&poAccelRec->mpoI2C_Clt->dev, "releasing gpio pins %d\n", poAccelRec->mnIrqPin);
	poAccelRec->mpfPinShutdownFunc();
	return 0;
}

static int Smb380_ConfigGPIO(I2cAccelRec *poAccelRec)
{
	if(poAccelRec == NULL)
		return -EINVAL;
	return poAccelRec->mpfPinSetupFunc();
}

#if 1
static int Smb380_Initialize(I2cAccelRec *poAccelRec, I2cClt *client)
{
	int nResult = 0;
	uint8_t bData;

#if DEBUG
printk(KERN_DEBUG "[SMB380]Initialize()\n");
#endif
	/* 設定を行う */
	if(0 > Smb380_I2cRead(client, SMB380_REG_RANGE_BWIDTH, &bData, 1))
	{
#if DEBUG
printk(KERN_DEBUG "[SMB380]I2cRead-->Error\n");
#endif
		return -EIO;
	}
	bData &= 0xe0;
	bData |= 0x03;  /* 2G 190Hz */
	nResult = Smb380_I2cWriteOne(client, SMB380_REG_RANGE_BWIDTH, bData);
	if(nResult < 0)
	{
#if DEBUG
printk(KERN_DEBUG "[SMB380]I2cWriteOne-->Error\n");
#endif
		return -EIO;
	}
	nResult = Smb380_I2cWriteAny(client, goAccInitialize);
	if(nResult < 0)
	{
#if DEBUG
printk(KERN_DEBUG "[SMB380]EnableAcc-->Error\n");
#endif
		return -EIO;
	}

	return 0;
}
#endif




static int __devexit Smb380_Remove(I2cClt *client)
{
	I2cAccelRec *poAccelRec = i2c_get_clientdata(client);

#if DEBUG
printk(KERN_DEBUG "[SMB380]Remove()\n");
#endif
	dev_info(&client->dev, "removing driver\n");
	device_init_wakeup(&client->dev, 0);
	Smb380_ReleaseGPIO(poAccelRec);
	kfree(poAccelRec);
	return 0;
}

#ifdef CONFIG_PM
static int Smb380_Suspend(I2cClt *client, pm_message_t oMsg)
{
	I2cAccelRec *poAccelRec = i2c_get_clientdata(client);

#if DEBUG
printk(KERN_DEBUG "[SMB380]Suspend()\n");
#endif
	if(device_may_wakeup(&client->dev))
	{
		enable_irq_wake(IRQNO(poAccelRec));
	}
	return 0;
}

static int Smb380_Resume(I2cClt *client)
{
	I2cAccelRec *poAccelRec = i2c_get_clientdata(client);

#if DEBUG
printk(KERN_DEBUG "[SMB380]Resume()\n");
#endif
	if(device_may_wakeup(&client->dev))
	{
		disable_irq_wake(IRQNO(poAccelRec));
	}
	return 0;
}
#else
#define	Smb380_Suspend		NULL
#define	Smb380_Resume		NULL
#endif

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_ACCEL_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);


/* I2Cドライバ呼び出し用構造体 */
static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_ACCEL_I2C_DEVNAME,
	},
	.probe	  = Smb380_Probe,
	.remove	  = __devexit_p(Smb380_Remove),
	.suspend  = Smb380_Suspend,
	.resume   = Smb380_Resume,
	.id_table = gI2cDevIdTableAcc,
};




static int __devinit Smb380_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	struct sh_i2c_accel_platform_data *poSetupData;
	I2cAccelRec *poAccelRec = NULL;
	int nResult;

#if DEBUG
printk(KERN_DEBUG "[SMB380]Probe()\n");
#endif
	if(!client->dev.platform_data)
	{
		dev_err(&client->dev, "platform device data is required\n");
		return -ENODEV;
	}
	/* Allocating memory */
	poAccelRec = kzalloc(sizeof(I2cAccelRec), GFP_KERNEL);
	if(!poAccelRec)
	{
		return -ENOMEM;
	}

	client->driver = &goI2cAccDriver;
	i2c_set_clientdata(client, poAccelRec);
	poAccelRec->mpoI2C_Clt          = client;
	this_client                     = client;
	poSetupData                     = client->dev.platform_data;

	/* Setup information */
	poAccelRec->mnIrqPin		 = poSetupData->gpio_irq;
	poAccelRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poAccelRec->mpfPinShutdownFunc = poSetupData->gpio_shutdown;
	/* GPIO Seting */
	if(0 == (nResult = Smb380_ConfigGPIO(poAccelRec)))
	{
	    if(Smb380_Initialize(poAccelRec, client))
	        goto exit_misc_device_register_failed;

        nResult = misc_register(&Smb380_device);
        if (nResult)
        {
#if DEBUG
            printk(KERN_DEBUG"Smb380 : misc_register failed\n");
#endif
            goto exit_misc_device_register_failed;
        }

		return 0;
	}

exit_misc_device_register_failed:
	/* GPIOの解放 */
	Smb380_ReleaseGPIO(poAccelRec);
	kfree(poAccelRec);

	return nResult;
}





static int __init Smb380_Init(void)
{
#if DEBUG
printk(KERN_DEBUG "[SMB380]Init()\n");
#endif
	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);
}

static void __exit Smb380_Exit(void)
{
#if DEBUG
printk(KERN_DEBUG "[SMB380]Exit()\n");
#endif
	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}

module_init(Smb380_Init);
module_exit(Smb380_Exit);

MODULE_DESCRIPTION("SMB380 ACCEL sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

