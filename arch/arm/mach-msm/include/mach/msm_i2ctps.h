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

#ifndef _MSM_I2CTPS_H_
#define _MSM_I2CTPS_H_

struct msm_sh_i2ctps_platform_data {
	int  gpio_irq;
	int  gpio_hssp_clk;
	int  gpio_hssp_data;
	int  (*gpio_setup) (void);
	void (*gpio_shutdown)(void);
};
#define	SH_TOUCH_I2C_DEVNAME	"SH_touchpanel"
#define SH_TOUCH_I2C_SLAVE		0x15
#define	SH_TOUCH_IRQ			92
#define	SH_TOUCH_HSSP_CLK		88
#define	SH_TOUCH_HSSP_DATA		89
#define	SH_TOUCH_MAX_X			479
#define	SH_TOUCH_MAX_Y			959
#define	SH_TOUCH_MAX_DISTANCE	1073

#define	FIRMDATA_SIZE			16384

#define TPSIF_DEV_NAME			"tpsif"
#define	TPSIF_DEV_FULLNAME		"/dev/tpsif"

enum
{
	TPSDEV_ENABLE = 0,
	TPSDEV_DISABLE,
	TPSDEV_FW_VERSION,
	TPSDEV_FW_DOWNLOAD,
	TPSDEV_FW_UPDATE,
	TPSDEV_START_TESTMODE,
	TPSDEV_STOP_TESTMODE,
	TPSDEV_GET_SENSOR,
	TPSDEV_SET_FIRMPARAM,
	TPSDEV_CALIBRATION_PARAM,
	TPSDEV_SLEEP_ON,
	TPSDEV_SLEEP_OFF,
	TPSDEV_START_TESTMODE2,
};

typedef struct
{
	uint8_t bVerNo;						/* Version number */
	uint8_t bData[FIRMDATA_SIZE];		/* Firm data */
} TpsFwData;

void msm_i2ctps_flipchange(int nFlipState);
void msm_i2ctps_setsleep(int nIsSleep);
void msm_i2ctps_shutdown(void);

#endif
