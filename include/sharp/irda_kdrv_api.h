/*
 * irda_kdrv_api.h
 * IrDA SIR/FIR driver module
 *
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

#ifndef _IRDA_KDRV_API_H
#define _IRDA_KDRV_API_H

#include <linux/ioctl.h>

#define IRDA_DEVFILE_NAME	"shirda"
#define IRDA_DEVFILE		"/dev/"IRDA_DEVFILE_NAME

#define IRDA_DRV_IOCTL_MAGIC	'i'

#define IRDA_DRV_IOCTL_SET_QOS		_IOW(IRDA_DRV_IOCTL_MAGIC, 1, \
					     irda_qos_info)
#define IRDA_DRV_IOCTL_GET_QOS		_IOR(IRDA_DRV_IOCTL_MAGIC, 2, \
					     irda_qos_info)
#define IRDA_DRV_IOCTL_IRDACC_INIT 	_IOR(IRDA_DRV_IOCTL_MAGIC, 3, \
					     irda_qos_info)

#define IRDA_DRV_IOCTL_IRDACC_TERM	_IO (IRDA_DRV_IOCTL_MAGIC, 4)

#define IRDA_DRV_IOCTL_CARRIER_SENSE	_IO (IRDA_DRV_IOCTL_MAGIC, 7)

#define IRDA_DRV_IOCTL_CHECK_RX 	_IOR(IRDA_DRV_IOCTL_MAGIC, 8, \
					     int)
#define IRDA_DRV_IOCTL_GPIO_LEDA_ON	_IO (IRDA_DRV_IOCTL_MAGIC, 9)

#define IRDA_DRV_IOCTL_GPIO_LEDA_OFF	_IO (IRDA_DRV_IOCTL_MAGIC, 10)

#define IRDA_DRV_RETUN_TYPE_RECV	(0x10)
#define IRDA_DRV_RETUN_TYPE_SEND	(0x20)
#define IRDA_DRV_RETUN_TYPE_CARRIER	(0x40)
#define IRDA_DRV_RETUN_TYPE_STOP	(0x80)

#define IRDA_DRV_RETUN_RESULT_COMP	(0x00)
#define IRDA_DRV_RETUN_RESULT_ERROR	(0x01)

#define IRDA_DRV_RETUN_RECV_COMP	(IRDA_DRV_RETUN_TYPE_RECV | \
					 IRDA_DRV_RETUN_RESULT_COMP)

#define IRDA_DRV_RETUN_SEND_COMP	(IRDA_DRV_RETUN_TYPE_SEND | \
					 IRDA_DRV_RETUN_RESULT_COMP)

#define IRDA_DRV_RETUN_SEND_ERR		(IRDA_DRV_RETUN_TYPE_SEND | \
					 IRDA_DRV_RETUN_RESULT_ERROR)

#define IRDA_DRV_RETUN_NO_CARRIER	(IRDA_DRV_RETUN_TYPE_CARRIER | \
					 IRDA_DRV_RETUN_RESULT_COMP)

#define IRDA_DRV_RETUN_IRDA_STOP	(IRDA_DRV_RETUN_TYPE_STOP | \
					 IRDA_DRV_RETUN_RESULT_COMP)

#endif
