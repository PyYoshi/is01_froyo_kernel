/*
 * irda_common.h
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

#ifndef _IRDA_COMMON_H
#define _IRDA_COMMON_H

#include "ir_types.h"

typedef enum {
	IRDA_SEND_MODE_NORMAL,
	IRDA_SEND_MODE_CONTINUE
} irda_send_mode_enum;

typedef enum {
	IRDA_BAUD_9600 = 1,
	IRDA_BAUD_19200,
	IRDA_BAUD_38400,
	IRDA_BAUD_57600,
	IRDA_BAUD_115200,
	IRDA_BAUD_4000000
} irda_boud_enum;

typedef enum {
	IR_LED_LOW_POWER,
	IR_LED_HIGH_POWER
} ir_led_power_enum;

typedef struct {
	irda_boud_enum baud_rate;
	uint16 connection_address;
	uint16 add_bof;
	uint16 mpi;
	uint16 mtt;
} irda_qos_info;

#define IRDA_KDRV_DEF_CA	(uint16)0xFF
#define IRDA_KDRV_DEF_BOF	(10)
#define IRDA_KDRV_DEF_MPI	(10)
#define IRDA_KDRV_DEF_MTT	(100)

#define IRDA_DRV_SET_ADD_BOF_48	(48)
#define IRDA_DRV_SET_ADD_BOF_32	(32)
#define IRDA_DRV_SET_ADD_BOF_24	(24)
#define IRDA_DRV_SET_ADD_BOF_20	(20)
#define IRDA_DRV_SET_ADD_BOF_16	(16)
#define IRDA_DRV_SET_ADD_BOF_14	(14)
#define IRDA_DRV_SET_ADD_BOF_12	(12)
#define IRDA_DRV_SET_ADD_BOF_10	(10)
#define IRDA_DRV_SET_ADD_BOF_8	(8)
#define IRDA_DRV_SET_ADD_BOF_6	(6)
#define IRDA_DRV_SET_ADD_BOF_5	(5)
#define IRDA_DRV_SET_ADD_BOF_4	(4)
#define IRDA_DRV_SET_ADD_BOF_3	(3)
#define IRDA_DRV_SET_ADD_BOF_2	(2)
#define IRDA_DRV_SET_ADD_BOF_1	(1)
#define IRDA_DRV_SET_ADD_BOF_0	(0)

#if 1
#define IRDA_DRV_SET_MPI_MAX	(1600)
#define IRDA_DRV_SET_MPI_MIN	(10)
#else
#define IRDA_DRV_SET_MPI_MAX	(235)
#define IRDA_DRV_SET_MPI_MIN	(10)
#define IRDA_DRV_SET_MPI_3MS	(300)
#endif

#if 1
#define IRDA_DRV_SET_MTT_MAX	(1600)
#define IRDA_DRV_SET_MTT_MIN	(50)
#define IRDA_DRV_SET_MTT_ZERO	(0)
#else
#define IRDA_DRV_SET_MTT_MAX	(1000)
#define IRDA_DRV_SET_MTT_MIN	(50)
#define IRDA_DRV_SET_MTT_ZERO	(0)
#endif

#endif
