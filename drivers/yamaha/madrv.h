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

#ifndef MA_DRV_H
#define MA_DRV_H

struct ma_IoCtlWriteRegWait
{
	unsigned long	dAddress;		/* I/F Address */
	const void		*pData;			/* Write Pointer */
	unsigned int	dSize;			/* Write Size(data type size) */
	unsigned int	dDataLen;		/* Data Length */
	unsigned int	dWait;			/* Wait ns */
};

struct ma_IoCtlReadRegWait
{
	unsigned long	dAddress;		/* I/F Address */
	void			*pData;			/* Read Data Store Pointer*/
	unsigned int	dSize;			/* Read Size(data type size) */
	unsigned int	dDataLen;		/* Data Length */
	unsigned int	dWait;			/* Wait ns */
};

#define MA_IOC_MAGIC 					'x'
#define MA_IOCTL_WAIT 					_IOW( MA_IOC_MAGIC, 0, unsigned int )
#define MA_IOCTL_SLEEP 					_IOW( MA_IOC_MAGIC, 1, unsigned int )
#define MA_IOCTL_WRITE_REG_WAIT 		_IOW( MA_IOC_MAGIC, 2, struct ma_IoCtlWriteRegWait )
#define MA_IOCTL_READ_REG_WAIT 			_IOWR( MA_IOC_MAGIC, 3, struct ma_IoCtlReadRegWait )
#define MA_IOCTL_DISABLE_IRQ 			_IO( MA_IOC_MAGIC, 4 )
#define MA_IOCTL_ENABLE_IRQ 			_IO( MA_IOC_MAGIC, 5 )
#define MA_IOCTL_RESET_IRQ_MASK_COUNT	_IO( MA_IOC_MAGIC, 6 )
#define MA_IOCTL_WAIT_IRQ 				_IOR( MA_IOC_MAGIC, 7, int )
#define MA_IOCTL_CANCEL_WAIT_IRQ 		_IO( MA_IOC_MAGIC, 8 )

#if 1
#define MA_IOCTL_SET_GPIO 		        _IOW( MA_IOC_MAGIC, 9, unsigned int )
#endif

#endif
