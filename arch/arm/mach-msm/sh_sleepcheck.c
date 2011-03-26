/* arch/arm/mach-msm/sh_sleepcheck.c
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

/* ==========================================================================================
 * Include files
 * ========================================================================================== */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/pm.h>
#include <mach/msm_rpcrouter.h>
#include "smd_private.h"
#include <sharp/sh_sleepcheck.h>

/* ==========================================================================================
 * Prototype local functions
 * ========================================================================================== */
static int sh_sleepcheck_probe( struct platform_device *pdev );
static int sh_sleepcheck_remove( struct platform_device *pdev );
static ssize_t sh_sleepcheck_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_count_m_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_count_m_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count );
static ssize_t sh_sleepcheck_name_m_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_count_a_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_count_a_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count );
static ssize_t sh_sleepcheck_name_a_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_num_clk_show( struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t sh_sleepcheck_name_clk_show( struct device *dev, struct device_attribute *attr, char *buf );

/* ==========================================================================================
 * Definitions
 * ========================================================================================== */
/* RPC */
#define SHSYS_A2M_PROG									0x300000d0
#define SHSYS_A2M_VERS									0x00010001
#define SHSYS_REMOTE_NULL_PROC							0
#define SHSYS_REMOTE_RPC_GLUE_CODE_INFO_REMOTE_PROC		1
#define SHSYS_API_INITIALIZE_REMOTE_PROC				2
#define SHSYS_SLEEP_INFO_PROC							3
#define SHSYS_SLEEP_COUNTS_GET_PROC						4
#define SHSYS_SLEEP_COUNTS_CLEAR_PROC					5

/* ==========================================================================================
 * module parameter
 * ========================================================================================== */
enum {
	DEBUG_CLOCK = 1U << 0,
};
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* ==========================================================================================
 * Structure definitions
 * ========================================================================================== */
/* Driver definition */
static struct platform_driver sh_sleepcheck_driver = {
	.probe	= sh_sleepcheck_probe,
	.remove = sh_sleepcheck_remove,
	.driver	= {
		.name	= "sh_sleep_chk",
		.owner	= THIS_MODULE,
	},
};

/* Device definition */
static struct platform_device sh_sleepcheck_device = {
	.name	= "sh_sleep_chk",
	.id		= -1,
	.dev	= {
		.platform_data = NULL,
	},
};

/* Device attribute definition */
static DEVICE_ATTR( sh_sleepchk, S_IRUGO | S_IWUGO, sh_sleepcheck_show, NULL );
static DEVICE_ATTR( sh_sleepchk_cnt_m, S_IRUGO | S_IWUGO, sh_sleepcheck_count_m_show, sh_sleepcheck_count_m_store );
static DEVICE_ATTR( sh_sleepchk_name_m, S_IRUGO | S_IWUGO, sh_sleepcheck_name_m_show, NULL );
static DEVICE_ATTR( sh_sleepchk_cnt_a, S_IRUGO | S_IWUGO, sh_sleepcheck_count_a_show, sh_sleepcheck_count_a_store );
static DEVICE_ATTR( sh_sleepchk_name_a, S_IRUGO | S_IWUGO, sh_sleepcheck_name_a_show, NULL );
static DEVICE_ATTR( sh_sleepchk_num_clk, S_IRUGO | S_IWUGO, sh_sleepcheck_num_clk_show, NULL );
static DEVICE_ATTR( sh_sleepchk_name_clk, S_IRUGO | S_IWUGO, sh_sleepcheck_name_clk_show, NULL );

static struct attribute *sh_sleepcheck_attrs[] = {
	&dev_attr_sh_sleepchk.attr,
	&dev_attr_sh_sleepchk_cnt_m.attr,
	&dev_attr_sh_sleepchk_name_m.attr,
	&dev_attr_sh_sleepchk_cnt_a.attr,
	&dev_attr_sh_sleepchk_name_a.attr,
	&dev_attr_sh_sleepchk_num_clk.attr,
	&dev_attr_sh_sleepchk_name_clk.attr,
	NULL,
};

static struct attribute_group sh_sleepcheck_attr_group = {
	.attrs = sh_sleepcheck_attrs,
};

/* RPC request structure definition */
typedef struct sh_sleepcheck_rpc_req_t
{
	struct rpc_request_hdr hdr;
} sh_sleepcheck_rpc_req;

/* RPC response structure definition */
/* for "SHSYS_SLEEP_INFO_PROC" */
typedef struct sh_sleepcheck_rpc_rsp_t
{
	struct rpc_reply_hdr hdr;
	unsigned long timestamp;
	unsigned long t_active;
	unsigned long t_halt;
	unsigned long t_tcxo_off;
	unsigned long sigs;
	unsigned long clients_high;
	unsigned long clients_low;
} sh_sleepcheck_rpc_rsp;

/* for "SHSYS_SLEEP_COUNTS_GET_PROC" */
typedef struct sh_sleepcheck_clt_rpc_rsp_t
{
	struct rpc_reply_hdr hdr;
	unsigned long client_num;
	unsigned long client_info[SH_SLEEPCHECK_CLIENTS_MAX_NUM];
	unsigned long blocked_count;
	unsigned long tasks_count;
} sh_sleepcheck_clt_rpc_rsp;

/* for "SHSYS_SLEEP_COUNTS_CLEAR_PROC" */
typedef struct sh_sleepcheck_clt_clear_rpc_rsp_t
{
	struct rpc_reply_hdr hdr;
	unsigned long clear_ret;
} sh_sleepcheck_clt_clear_rpc_rsp;

/* ==========================================================================================
 * Global variable
 * ========================================================================================== */
static sh_sleepcheck_clients_info sh_sleepcheck_info_aarm;
static sh_sleepcheck_clock_info sh_sleepcheck_info_clk;
static struct msm_rpc_endpoint *sleepcheck_ep;

/* ==========================================================================================
 * Functions
 * ========================================================================================== */
void sh_sleepcheck_register( const char *name, int type )
{
	bool register_flag = false;
	uint8_t index;
	
	if( (type == WAKE_LOCK_SUSPEND) && (SH_SLEEPCHECK_CLIENTS_MAX_NUM > sh_sleepcheck_info_aarm.num_clients) )
	{
		for( index = 0; index < sh_sleepcheck_info_aarm.num_clients; index++ )
		{
			if( name == sh_sleepcheck_info_aarm.client[index].name )
			{
				register_flag = true;
				break;
			}
		}
	
		if( register_flag == true )
		{
			sh_sleepcheck_info_aarm.client[index].multi_count++;
//			pr_info( "%s() : %s multiplex registered. (count=%lu)\n", __func__, name, sh_sleepcheck_info_aarm.client[index].multi_count );
		}
		else
		{
			sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].name = name;
			sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].block_count = 0;
			sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].multi_count = 1;
			sh_sleepcheck_info_aarm.num_clients++;
//			pr_info( "%s() : %s is register. (clients_num=%d)\n", __func__, name, sh_sleepcheck_info_aarm.num_clients );
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_register );

void sh_sleepcheck_unregister( const char *name, int flags )
{
	bool register_flag = false;
	uint8_t index1, index2;
	uint64_t Temp_high;
	uint64_t Temp_low;
	int type;

	type = flags & 0xf/*WAKE_LOCK_TYPE_MASK*/;
	if( type == WAKE_LOCK_SUSPEND )
	{
		for( index1 = 0; index1 < sh_sleepcheck_info_aarm.num_clients; index1++ )
		{
			if( name == sh_sleepcheck_info_aarm.client[index1].name )
			{
				register_flag = true;
				break;
			}
		}
		if( register_flag == false )
		{
//			pr_err( "%s() : %s is not register...\n", __func__, name );
		}
		else
		{
			sh_sleepcheck_info_aarm.client[index1].multi_count--;
			if( sh_sleepcheck_info_aarm.client[index1].multi_count == 0)
			{
				sh_sleepcheck_info_aarm.client[index1].name = NULL;
				sh_sleepcheck_info_aarm.client[index1].block_count = 0;
				for( index2 = index1; index2 < sh_sleepcheck_info_aarm.num_clients - 1; index2++ )
				{
					sh_sleepcheck_info_aarm.client[index2].name = sh_sleepcheck_info_aarm.client[index2 + 1].name;
					sh_sleepcheck_info_aarm.client[index2].block_count = sh_sleepcheck_info_aarm.client[index2 + 1].block_count;
					sh_sleepcheck_info_aarm.client[index2].multi_count = sh_sleepcheck_info_aarm.client[index2 + 1].multi_count;
				}
				if( sh_sleepcheck_info_aarm.num_clients == SH_SLEEPCHECK_CLIENTS_MAX_NUM )
				{
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients - 1].name = NULL;
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients - 1].block_count = 0;
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients - 1].multi_count = 0;
				}
				else
				{
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].name = NULL;
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].block_count = 0;
					sh_sleepcheck_info_aarm.client[sh_sleepcheck_info_aarm.num_clients].multi_count = 0;
				}

				Temp_high = sh_sleepcheck_info_aarm.curr_not_okts_mask;
				Temp_low = sh_sleepcheck_info_aarm.curr_not_okts_mask;
				Temp_high = Temp_high >> 1;
				Temp_high = Temp_high & ( 0xFFFFFFFFFFFFFFFFULL << index1 );
				Temp_low = Temp_low & ~( 0xFFFFFFFFFFFFFFFFULL << index1 );
				sh_sleepcheck_info_aarm.curr_not_okts_mask = Temp_high | Temp_low;

				sh_sleepcheck_info_aarm.num_clients--;

//				pr_info( "%s() : %s is unregister. (clients_num=%d)\n", __func__, name, sh_sleepcheck_info_aarm.num_clients );
			}
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_unregister );

void sh_sleepcheck_negate_okts( const char *name )
{
	uint8_t index;
	uint64_t Temp;

	for( index = 0; index < sh_sleepcheck_info_aarm.num_clients; index++ )
	{
		if( name == sh_sleepcheck_info_aarm.client[index].name )
		{
			Temp = 1 << index;
			sh_sleepcheck_info_aarm.curr_not_okts_mask |= Temp;
//			pr_info( "%s() : %s is negate okts.\n", __func__, name );
			break;
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_negate_okts );

void sh_sleepcheck_assert_okts( const char *name )
{
	uint8_t index;
	uint64_t Temp;

	for( index = 0; index < sh_sleepcheck_info_aarm.num_clients; index++ )
	{
		if( name == sh_sleepcheck_info_aarm.client[index].name )
		{
			Temp = 1 << index;
			sh_sleepcheck_info_aarm.curr_not_okts_mask &= ~Temp;
//			pr_info( "%s() : %s is assert okts.\n", __func__, name );
			break;
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_assert_okts );

void sh_sleepcheck_suspend_count( void )
{
	int i;

	sh_sleepcheck_info_aarm.sleep_count++;

	/* when sleep count overflows, other counts are cleared. */
	if(sh_sleepcheck_info_aarm.sleep_count == 0)
	{
		for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
		{
			sh_sleepcheck_info_aarm.client[i].block_count = 0;
		}
		sh_sleepcheck_info_aarm.block_count = 0;
		sh_sleepcheck_info_aarm.sleep_count = 1;
	}
}
EXPORT_SYMBOL( sh_sleepcheck_suspend_count );

void sh_sleepcheck_suspend_block_count( void )
{
	sh_sleepcheck_info_aarm.block_count++;
}
EXPORT_SYMBOL( sh_sleepcheck_suspend_block_count );

void sh_sleepcheck_block_count_client( const char *name, int type )
{
	uint8_t index;

	if( type == WAKE_LOCK_SUSPEND )
	{
		for( index = 0; index < sh_sleepcheck_info_aarm.num_clients; index++ )
		{
			if( name == sh_sleepcheck_info_aarm.client[index].name )
			{
				sh_sleepcheck_info_aarm.client[index].block_count++;
				break;
			}
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_block_count_client );

void sh_sleepcheck_clock_name_init( const char *name, unsigned n )
{
	if( n < SH_SLEEPCHECK_CLIENTS_MAX_NUM )
	{
		sh_sleepcheck_info_clk.clock[n].name = name;
//		pr_info( "%s() : clock_name[%d] = %s \n", __func__, n, sh_sleepcheck_info_clk.clock[n].name );
	}
}
EXPORT_SYMBOL( sh_sleepcheck_clock_name_init );

void sh_sleepcheck_clock_enable( const char *name )
{
	uint8_t index;

	for( index = 0; index < SH_SLEEPCHECK_CLIENTS_MAX_NUM; index++ )
	{
		if( name == sh_sleepcheck_info_clk.clock[index].name )
		{
			sh_sleepcheck_info_clk.clock[index].onoffflag = 1;
			sh_sleepcheck_info_clk.num_clocks++;
			if (debug_mask & DEBUG_CLOCK)
			{
				pr_info( "%s() : clock_name =%s, onoffflag =%u num_clocks =%u \n", __func__, name, sh_sleepcheck_info_clk.clock[index].onoffflag, sh_sleepcheck_info_clk.num_clocks );
			}
			break;
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_clock_enable );

void sh_sleepcheck_clock_disable( const char *name )
{
	uint8_t index;

	for( index = 0; index < SH_SLEEPCHECK_CLIENTS_MAX_NUM; index++ )
	{
		if( name == sh_sleepcheck_info_clk.clock[index].name )
		{
			sh_sleepcheck_info_clk.clock[index].onoffflag = 0;
			sh_sleepcheck_info_clk.num_clocks--;
			if (debug_mask & DEBUG_CLOCK)
			{
				pr_info( "%s() : clock_name =%s, onoffflag =%u num_clocks =%u \n", __func__, name, sh_sleepcheck_info_clk.clock[index].onoffflag, sh_sleepcheck_info_clk.num_clocks );
			}
			break;
		}
	}
}
EXPORT_SYMBOL( sh_sleepcheck_clock_disable );

/* --------------------------------------------------------------------------
 * probe/remove function
 * -------------------------------------------------------------------------- */
static int sh_sleepcheck_probe( struct platform_device *pdev )
{
	int ret = 0;

	pr_info( "%s() : Called.\n", __func__ );

	/* Create attributes */
	ret = sysfs_create_group( &pdev->dev.kobj, &sh_sleepcheck_attr_group );
	if ( ret )
	{
		pr_err( "%s() : Device create failed.\n", __func__ );
		return ret;
	}

	/* Get RPC endpoint */
	sleepcheck_ep = msm_rpc_connect_compatible( SHSYS_A2M_PROG, SHSYS_A2M_VERS, 0 );
	if( IS_ERR(sleepcheck_ep) )
	{
		pr_err( "%s(): init rpc failed! rc = %ld PROG = %08x vers = %08x\n",
							__func__,
							PTR_ERR(sleepcheck_ep),
							SHSYS_A2M_PROG,
							SHSYS_A2M_VERS );
		ret = PTR_ERR(sleepcheck_ep);
	}

	return ret;
}

static int sh_sleepcheck_remove( struct platform_device *pdev )
{
	sysfs_remove_group( &pdev->dev.kobj, &sh_sleepcheck_attr_group );

	return 0;
}

/* --------------------------------------------------------------------------
 * show/store function
 * -------------------------------------------------------------------------- */
static ssize_t sh_sleepcheck_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	sh_sleepcheck_rpc_rsp sleepcheck_info_marm;
	sh_sleepcheck_rpc_req sleepcheck_rpc_req;
	sleepcheck_aarm_time *sleep_info_aarm = NULL;
	unsigned long active_lock_num;
	unsigned long long not_okts_mask;
	unsigned long long marm_clients;
	struct timespec ts;
	ssize_t len = 0;
	int64_t temp_timestamp_nsec;
	int ret;

	/* --------------------------------------------------------------------- */
	/* Get mARM sleep information by RPC                                     */
	/* --------------------------------------------------------------------- */
	memset( &sleepcheck_info_marm, 0x00, sizeof(sleepcheck_info_marm) );
	ret = msm_rpc_call_reply( sleepcheck_ep, SHSYS_SLEEP_INFO_PROC,
							  &sleepcheck_rpc_req, sizeof(sh_sleepcheck_rpc_req),
							  &sleepcheck_info_marm, sizeof(sh_sleepcheck_rpc_rsp), 5 * HZ);
	if ( ret < 0 )
	{
		pr_err( "%s(): shsys mARM call remote error ! ret = %d\n", __func__, ret );
		return (ssize_t)ret;
	}

	/* --------------------------------------------------------------------- */
	/* Get aARM sleep information                                            */
	/* --------------------------------------------------------------------- */
	sleep_info_aarm = sh_sleepcheck_get_time();

	/* Active time culculate from time stamp of mARM */
	sleepcheck_info_marm.timestamp = be32_to_cpu( sleepcheck_info_marm.timestamp );	/* Get mARM time stamp */
	ts.tv_sec = sleepcheck_info_marm.timestamp >> 15;								/* Convert to second */
	ts.tv_nsec = 0;
	temp_timestamp_nsec = timespec_to_ns( &ts );									/* Convert to nano second */
	temp_timestamp_nsec = temp_timestamp_nsec - ((sleep_info_aarm->t_halt) + (sleep_info_aarm->t_pc));
	ts = ns_to_timespec( temp_timestamp_nsec );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", ts.tv_sec );
	pr_info( "%s() : aarm   act =%08lx sec %08lx nsec\n", __func__, ts.tv_sec, ts.tv_nsec );

	/* Halt time */
	ts = ns_to_timespec( sleep_info_aarm->t_halt );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", ts.tv_sec );
	pr_info( "%s() :        halt=%08lx sec %08lx nsec\n", __func__, ts.tv_sec, ts.tv_nsec );

	/* TCXO down time */
	ts = ns_to_timespec( sleep_info_aarm->t_tcxo_off );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", ts.tv_sec );
	pr_info( "%s() :        tcxo=%08lx sec %08lx nsec\n", __func__, ts.tv_sec, ts.tv_nsec );

	/* PC time */
	ts = ns_to_timespec( sleep_info_aarm->t_pc );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", ts.tv_sec );
	pr_info( "%s() :        pc  =%08lx sec %08lx nsec\n", __func__, ts.tv_sec, ts.tv_nsec );

	/* Active wake lock num */
	active_lock_num = sh_sleepcheck_get_clients();
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", active_lock_num );
	pr_info( "%s() :        active_lock_num=%08lx\n", __func__, active_lock_num );

	/* Not okts task */
	not_okts_mask = sh_sleepcheck_info_aarm.curr_not_okts_mask;
	len += scnprintf( buf + len, PAGE_SIZE - len, "%016llx", not_okts_mask );
	pr_info( "%s() :        not_okts_clients=%016llx\n", __func__, not_okts_mask );

	/* --------------------------------------------------------------------- */
	/* mARM sleep information                                                */
	/* --------------------------------------------------------------------- */
	/* Time stamp */
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_info_marm.timestamp );

	/* Active time */
	sleepcheck_info_marm.t_active = be32_to_cpu( sleepcheck_info_marm.t_active );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_info_marm.t_active );

	/* Halt time */
	sleepcheck_info_marm.t_halt = be32_to_cpu( sleepcheck_info_marm.t_halt );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_info_marm.t_halt );

	/* TCXO down time */
	sleepcheck_info_marm.t_tcxo_off = be32_to_cpu( sleepcheck_info_marm.t_tcxo_off );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_info_marm.t_tcxo_off );

	/* Not okts flag */
	sleepcheck_info_marm.sigs = be32_to_cpu( sleepcheck_info_marm.sigs );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_info_marm.sigs );

	/* Not okts task */
	sleepcheck_info_marm.clients_high = be32_to_cpu( sleepcheck_info_marm.clients_high );
	sleepcheck_info_marm.clients_low  = be32_to_cpu( sleepcheck_info_marm.clients_low );
	marm_clients = ((unsigned long long)sleepcheck_info_marm.clients_high << 32);
	marm_clients += (unsigned long long)sleepcheck_info_marm.clients_low;
	len += scnprintf( buf + len, PAGE_SIZE - len, "%016llx", marm_clients );

	pr_info( "%s() : marm : ts=%08lx, act=%08lx, halt=%08lx, tcxo=%08lx, sigs=%08lx, clnt=%016llx\n",
						__func__,
						sleepcheck_info_marm.timestamp,
						sleepcheck_info_marm.t_active,
						sleepcheck_info_marm.t_halt,
						sleepcheck_info_marm.t_tcxo_off,
						sleepcheck_info_marm.sigs,
						marm_clients );

	return len;
}

static ssize_t sh_sleepcheck_count_m_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	sh_sleepcheck_clt_rpc_rsp sleepcheck_count;
	sh_sleepcheck_rpc_req sleepcheck_rpc_req;
	ssize_t len = 0;
	int ret;
	int i;
	unsigned char c_num;

	/* --------------------------------------------------------------------- */
	/* Get mARM sleep count by RPC                                           */
	/* --------------------------------------------------------------------- */
	memset( &sleepcheck_count, 0x00, sizeof(sh_sleepcheck_clt_rpc_rsp) );
	ret = msm_rpc_call_reply( sleepcheck_ep, SHSYS_SLEEP_COUNTS_GET_PROC,
							  &sleepcheck_rpc_req, sizeof(sh_sleepcheck_rpc_req),
							  &sleepcheck_count, sizeof(sh_sleepcheck_clt_rpc_rsp), 5 * HZ);
	if ( ret < 0 )
	{
		pr_err( "%s(): shsys mARM call remote error ! ret = %d\n", __func__, ret );
		return (ssize_t)ret;
	}

	/* client num */
	sleepcheck_count.client_num = be32_to_cpu( sleepcheck_count.client_num );
	c_num = (unsigned char)sleepcheck_count.client_num;
	len += scnprintf( buf + len, PAGE_SIZE - len, "%02x", c_num );
	pr_info( "%s() : marm   client_num =%02X \n", __func__, c_num );
	/* client blocked count */
	for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
	{
		sleepcheck_count.client_info[i] = be32_to_cpu( sleepcheck_count.client_info[i] );
		len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_count.client_info[i] );
		pr_info( "%s() : marm   client_info[%d] =%08lX \n", __func__, i, sleepcheck_count.client_info[i] );
	}
	/* blocked count */
	sleepcheck_count.blocked_count = be32_to_cpu( sleepcheck_count.blocked_count );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_count.blocked_count );
	pr_info( "%s() : marm   blocked_count =%08lX \n", __func__, sleepcheck_count.blocked_count );
	/* tasks count */
	sleepcheck_count.tasks_count = be32_to_cpu( sleepcheck_count.tasks_count );
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sleepcheck_count.tasks_count );
	pr_info( "%s() : marm   tasks_count =%08lX \n", __func__, sleepcheck_count.tasks_count );

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

static ssize_t sh_sleepcheck_count_m_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	sh_sleepcheck_clt_clear_rpc_rsp sleepcheck_count_clr;
	sh_sleepcheck_rpc_req sleepcheck_rpc_req;
	int ret = 0;

	if( !*buf )
	{
		return (ssize_t)ret;
	}

	/* --------------------------------------------------------------------- */
	/* Clear mARM sleep count by RPC                                         */
	/* --------------------------------------------------------------------- */
	memset( &sleepcheck_count_clr, 0x00, sizeof(sh_sleepcheck_clt_clear_rpc_rsp) );
	ret = msm_rpc_call_reply( sleepcheck_ep, SHSYS_SLEEP_COUNTS_CLEAR_PROC,
							  &sleepcheck_rpc_req, sizeof(sh_sleepcheck_rpc_req),
							  &sleepcheck_count_clr, sizeof(sh_sleepcheck_clt_clear_rpc_rsp), 5 * HZ);
	if ( ret < 0 )
	{
		pr_err( "%s(): shsys mARM call remote error ! ret = %d\n", __func__, ret );
		return (ssize_t)ret;
	}

	/* clear result */
	sleepcheck_count_clr.clear_ret = be32_to_cpu( sleepcheck_count_clr.clear_ret );
	pr_info( "%s() : marm   clear_ret =%lu \n", __func__, sleepcheck_count_clr.clear_ret );

	return (ssize_t)sleepcheck_count_clr.clear_ret;
}

static ssize_t sh_sleepcheck_name_m_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t len = 0;
	int i;
	char *smem_clt_name;
	unsigned smem_size;

	/* --------------------------------------------------------------------- */
	/* Get mARM sleep client name by SMEM                                    */
	/* --------------------------------------------------------------------- */
	smem_clt_name = smem_get_entry( SMEM_SLEEP_STATIC, &smem_size ); 
	if( smem_clt_name != NULL )
	{
		for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
		{
			scnprintf( buf + len, PAGE_SIZE - len, "%s", smem_clt_name + i * (SH_SLEEP_CLIENT_LIST_MAX_STR_MARM + 1) );
			len += SH_SLEEP_CLIENT_LIST_MAX_STR_MARM + 1;
			pr_info( "%s() : marm   client_name[%d] =%s \n", __func__, i + 1, smem_clt_name + i * (SH_SLEEP_CLIENT_LIST_MAX_STR_MARM + 1) );
		}
	}
	else
	{
		pr_info( "%s() : get smem address failed.\n", __func__ );
	}

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

static ssize_t sh_sleepcheck_count_a_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t len = 0;
	int i;

	/* --------------------------------------------------------------------- */
	/* Get aARM sleep count                                                  */
	/* --------------------------------------------------------------------- */
	/* client num */
	len += scnprintf( buf + len, PAGE_SIZE - len, "%02x", sh_sleepcheck_info_aarm.num_clients );
	pr_info( "%s() : aarm   num_clients =%02X \n", __func__, sh_sleepcheck_info_aarm.num_clients );
	/* client blocked count */
	for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
	{
		len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sh_sleepcheck_info_aarm.client[i].block_count );
		pr_info( "%s() : aarm   block_count[%d] =%08lX \n", __func__, i, sh_sleepcheck_info_aarm.client[i].block_count );
	}
	/* blocked count */
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sh_sleepcheck_info_aarm.block_count );
	pr_info( "%s() : aarm   block_count =%08lX \n", __func__, sh_sleepcheck_info_aarm.block_count );
	/* sleep count */
	len += scnprintf( buf + len, PAGE_SIZE - len, "%08lx", sh_sleepcheck_info_aarm.sleep_count );
	pr_info( "%s() : aarm   sleep_count =%08lX \n", __func__, sh_sleepcheck_info_aarm.sleep_count );

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

static ssize_t sh_sleepcheck_count_a_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	int i;

	if( !*buf )
	{
		return 0;
	}

	/* --------------------------------------------------------------------- */
	/* Clear aARM sleep count                                                */
	/* --------------------------------------------------------------------- */
	/* client blocked count */
	for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
	{
		sh_sleepcheck_info_aarm.client[i].block_count = 0;
	}
	/* blocked count */
	sh_sleepcheck_info_aarm.block_count = 0;
	/* sleep count */
	sh_sleepcheck_info_aarm.sleep_count = 0;

	return 1;
}

static ssize_t sh_sleepcheck_name_a_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	char temp_buf[SH_SLEEP_CLIENT_LIST_MAX_STR_AARM + 1] = {0};
	ssize_t len = 0;
	int i;

	/* --------------------------------------------------------------------- */
	/* Get aARM sleep client name                                            */
	/* --------------------------------------------------------------------- */
	for (i = 0; i < sh_sleepcheck_info_aarm.num_clients; i++)
	{
		if(sh_sleepcheck_info_aarm.client[i].name != NULL)
		{
			strncpy( temp_buf, sh_sleepcheck_info_aarm.client[i].name, SH_SLEEP_CLIENT_LIST_MAX_STR_AARM );
			scnprintf( buf + len, PAGE_SIZE - len, "%s", temp_buf );
			pr_info( "%s() : aarm   client_name[%d] =%s \n", __func__, i + 1, temp_buf );
		}
		else
		{
			pr_info( "%s() : aarm   client_name[%d] = NULL!! \n", __func__, i + 1 );
		}
		len += SH_SLEEP_CLIENT_LIST_MAX_STR_AARM + 1;
	}

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

static ssize_t sh_sleepcheck_num_clk_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	ssize_t len = 0;

	/* --------------------------------------------------------------------- */
	/* Get "CLKREGIM" enable clock num                                       */
	/* --------------------------------------------------------------------- */
	/* clock num */
	len += scnprintf( buf + len, PAGE_SIZE - len, "%02x", sh_sleepcheck_info_clk.num_clocks );
	pr_info( "%s() : num_clocks =%u \n", __func__, sh_sleepcheck_info_clk.num_clocks );

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

static ssize_t sh_sleepcheck_name_clk_show( struct device *dev, struct device_attribute *attr, char *buf )
{
	char temp_buf[SH_SLEEPCHECK_CLKREGIM_LIST_MAX_STR + 1] = {0};
	ssize_t len = 0;
	int i;

	/* --------------------------------------------------------------------- */
	/* Get "CLKREGIM" enable clock name                                      */
	/* --------------------------------------------------------------------- */
	for (i = 0; i < SH_SLEEPCHECK_CLIENTS_MAX_NUM; i++)
	{
		if(sh_sleepcheck_info_clk.clock[i].onoffflag == 1)
		{
			if(sh_sleepcheck_info_clk.clock[i].name != NULL)
			{
				strncpy( temp_buf, sh_sleepcheck_info_clk.clock[i].name, SH_SLEEPCHECK_CLKREGIM_LIST_MAX_STR );
				scnprintf( buf + len, PAGE_SIZE - len, "%s", temp_buf );
				pr_info( "%s() : clock_name[%d] =%s \n", __func__, i + 1, temp_buf );
			}
			else
			{
				pr_info( "%s() : clock_name[%d] = NULL!! \n", __func__, i + 1 );
			}
			len += SH_SLEEPCHECK_CLKREGIM_LIST_MAX_STR + 1;
		}
	}

	pr_info( "%s() : read len =%d \n", __func__, len );

	return len;
}

/* --------------------------------------------------------------------------
 * Init/Exit function
 * -------------------------------------------------------------------------- */
static int __init sh_sleepcheck_init( void )
{
	int ret;

	pr_info( "%s() : Called.\n", __func__ );

	/* Platform driver register */
	ret = platform_driver_register( &sh_sleepcheck_driver );
	if( ret )
	{
		pr_err( "%s() : platform driver register failed. (ret=%d)\n", __func__, ret );
		return ret;
	}

	/* Platform device register */
	ret = platform_device_register( &sh_sleepcheck_device );
	if( ret )
	{
		pr_err( "%s() : platform device register failed. (ret=%d)\n", __func__, ret );
		platform_driver_unregister( &sh_sleepcheck_driver );
		return ret;
	}

	return 0;
}

static void __exit sh_sleepcheck_exit( void )
{
	pr_info( "%s() : Called.\n", __func__ );

	platform_driver_unregister( &sh_sleepcheck_driver );
	platform_device_unregister( &sh_sleepcheck_device );
}

module_init(sh_sleepcheck_init);
module_exit(sh_sleepcheck_exit);

MODULE_DESCRIPTION("SH SLEEP CHK");
MODULE_LICENSE("GPL");
