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
 * This code is based on msm_i2ckbd.h.
 * The original copyright and notice are described below.
 */

/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MSM_I2CKBD_H_
#define _MSM_I2CKBD_H_

struct msm_i2ckbd_platform_data {
	uint8_t hwrepeat;
	uint8_t scanset1;
	int  gpioreset;
	int  gpioirq;
	int  (*gpio_setup) (void);
	void (*gpio_shutdown)(void);
	void (*hw_reset) (int);
};

#if 1
struct msm_sh_i2ckbd_platform_data {
	int  gpio_kbdirq;
	int  gpio_jogirq[4];
	int  gpio_jogpower;
	int  (*gpio_setup) (void);
	void (*gpio_shutdown)(void);
};

#define SH_KBD_IRQ				42
#define SH_JOG_U_IRQ			38
#define SH_JOG_D_IRQ			39
#define SH_JOG_L_IRQ			41
#define SH_JOG_R_IRQ			40
#define SH_JOG_POWER			48

#define	SH_KBD_I2C_DEVNAME	"SH_qwerty_key"
#define SH_KBD_I2C_SLAVE		0x44

#define KBDIF_DEV_NAME			"kbdif"
#define	KBDIF_DEV_FULLNAME		"/dev/kbdif"

enum
{
	KBDDEV_SLEEP_ON = 0,
	KBDDEV_SLEEP_OFF,
	KBDDEV_ENABLE,
	KBDDEV_DISABLE,
};

void msm_i2ckbd_flipchange(int nFlipState);
void msm_i2ckbd_setsleep(int nIsSleep);

#endif

#endif
