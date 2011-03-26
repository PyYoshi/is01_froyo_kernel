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

#ifndef __SH_MAINCAM_H
#define __SH_MAINCAM_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "linux/types.h"

#include <mach/board.h>
#include <media/msm_camera.h>

#if 1
typedef unsigned int   uint32;
typedef unsigned short uint16;
typedef unsigned char  uint8;
typedef signed int     int32;
typedef signed short   int16;
typedef signed char    int8;
#endif

#define SH_MAINCAM_IOCTL_MAGIC 'm'

#define SH_MAINCAM_IOCTL_READ_IRQ_KIND		_IOR(SH_MAINCAM_IOCTL_MAGIC, 1, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_ENABLE_IRQ			_IOW(SH_MAINCAM_IOCTL_MAGIC, 2, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_DISABLE_IRQ		_IOW(SH_MAINCAM_IOCTL_MAGIC, 3, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_CAMIF_PAD_RESET	_IOW(SH_MAINCAM_IOCTL_MAGIC, 4, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_REQUEST_IRQ		_IOW(SH_MAINCAM_IOCTL_MAGIC, 5, struct sh_maincam_ctrl_cmd *)

#define SH_MAINCAM_IOCTL_FREE_IRQ			_IOW(SH_MAINCAM_IOCTL_MAGIC, 6, struct sh_maincam_ctrl_cmd *)

#define CAM_INT_TYPE_VS						1
#define CAM_INT_TYPE_LINT3					2
#define CAM_INT_TYPE_TIMEOUT				3

#define D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE	40

struct sh_maincam_ctrl_cmd {
	unsigned char	r_data[D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE];
	unsigned char	w_data[D_SH_MAINCAM_IOCTL_DSP_RW_MAX_SIZE];
	signed int		status;
	signed int		length;
	unsigned short	addr;
	unsigned short	irq_number;
	unsigned char	device_id[5];
	unsigned char	dummy[3];
};

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");

#endif
