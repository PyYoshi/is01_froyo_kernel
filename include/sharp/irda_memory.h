/*
 * irda_memory.h
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

#ifndef _IRDA_MEMORY_H
#define _IRDA_MEMORY_H

#define IRDA_MEM_FRAME_MAX	(8 + 1)
#define IRDA_MEM_1FRAME_SIZE	(2080)

typedef struct {
	unsigned short	rp;
	unsigned short	wp;
} irda_drv_databuff_control_info;

typedef struct {
	irda_send_mode_enum mode;
	unsigned long len;
	unsigned char data[IRDA_MEM_1FRAME_SIZE];
}irda_drv_databuff_info;

typedef struct {
	irda_drv_databuff_control_info control;

	irda_drv_databuff_info databuff[IRDA_MEM_FRAME_MAX];
} irda_drv_data_format_info;

typedef struct {
	irda_drv_data_format_info send;

	irda_drv_data_format_info receive;
} irda_drv_data_info;

#define IRDA_MMAP_LENGTH		(sizeof(irda_drv_data_info))

#endif
