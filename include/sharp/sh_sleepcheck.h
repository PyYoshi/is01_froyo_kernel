/* include/sharp/sh_sleepcheck.h
 *
 * Copyright (C) 2010 Sharp Corporation
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

#ifndef SH_SLEEPCHECK_H
#define SH_SLEEPCHECK_H

/* ==========================================================================================
 * Include files
 * ========================================================================================== */
#include <linux/types.h>

/* ==========================================================================================
 * Definitions
 * ========================================================================================== */
/* Sleep client */
#define SH_SLEEPCHECK_CLIENTS_MAX_NUM 64
/* Sleep client name max string(mARM) */
#define SH_SLEEP_CLIENT_LIST_MAX_STR_MARM 8
/* Sleep client name max string(aARM) */
#define SH_SLEEP_CLIENT_LIST_MAX_STR_AARM 20
/* CLKREGIM clock name max string */
#define SH_SLEEPCHECK_CLKREGIM_LIST_MAX_STR 21

/* ==========================================================================================
 * Structures
 * ========================================================================================== */
typedef struct sh_sleepcheck_clients_t
{
	const char	*name;
	unsigned long block_count;
	unsigned long multi_count;
} sh_sleepcheck_clients;

typedef struct sh_sleepcheck_clients_info_t
{
	sh_sleepcheck_clients	client[SH_SLEEPCHECK_CLIENTS_MAX_NUM];
	unsigned long long curr_not_okts_mask;
	unsigned long block_count;
	unsigned long sleep_count;
	unsigned char num_clients;
} sh_sleepcheck_clients_info;

typedef struct sleepcheck_aarm_time_t
{
	int64_t  t_halt;
	int64_t  t_tcxo_off;
	int64_t  t_pc;
} sleepcheck_aarm_time;

typedef struct sh_sleepcheck_clock_t
{
	const char	*name;
	unsigned char onoffflag;
} sh_sleepcheck_clock;

typedef struct sh_sleepcheck_clock_info_t
{
	sh_sleepcheck_clock	clock[SH_SLEEPCHECK_CLIENTS_MAX_NUM];
	unsigned char num_clocks;
} sh_sleepcheck_clock_info;

/* ==========================================================================================
 * Prototype global functions
 * ========================================================================================== */
void sh_sleepcheck_register( const char *name, int type );
void sh_sleepcheck_unregister( const char *name, int flags );
void sh_sleepcheck_negate_okts( const char *name );
void sh_sleepcheck_assert_okts( const char *name );
void sh_sleepcheck_suspend_count( void );
void sh_sleepcheck_suspend_block_count( void );
void sh_sleepcheck_block_count_client( const char *name, int type );
void sh_sleepcheck_clock_name_init( const char *name, unsigned n );
void sh_sleepcheck_clock_enable( const char *name );
void sh_sleepcheck_clock_disable( const char *name );

#endif /* SH_SLEEPCHECK_H*/
