/*
 * Copyright (C) 2009 Sharp.
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


#include <linux/string.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <mach/msm_rpcrouter.h>
#include <asm/uaccess.h>
#include <sharp/shterm_k.h>

/* M2A */
#define SHTERM_PROG  0x300000AA
#define SHTERM_VERS  0x00010001
/* M2A PROC */
#define ONCRPC_SHTERM_SET_INFO_PROC  2
#define ONCRPC_SHTERM_SET_EVENT_PROC 3

/* A2M */
#define SHTERM_A2M_PROG  0x300000AB
#define SHTERM_A2M_VERS  0x00010001
/* A2M PROC */
#define ONCRPC_SHTERM_GET_OFFCHG_LOGSIZE_REMOTE_PROC 3
#define ONCRPC_SHTERM_GET_OFFCHG_DATA_REMOTE_PROC 4
#define ONCRPC_SHTERM_INIT_TIMER_REMOTE_PROC 5
#define ONCRPC_SHTERM_SET_TIMER_REMOTE_PROC 6
#define ONCRPC_SHTERM_REQ_VAL_REMOTE_PROC 7

#define SHTERM_A2M_RPC_CALL_TIMEOUT ( 5 * HZ )

typedef struct _shterm_set_info_args {
    int id;
    int val;
} shterm_set_info_args;

typedef struct _shterm_set_event_args {
    int event_num;
    int one;
    int two;
    int thr;
    int fou;
    int fiv;
    int six;
    int sev;
    int eig;
    int nin;
    int ten;
    int ele;
} shterm_set_event_args;

typedef struct _shterm_set_event_offchg_args {
    int event_num;
    unsigned int time;
    int bat_vol;
    int bat_temp;
    int chg_temp;
    int cam_temp;
    int pmic_temp;
    int pa_temp;
    int avg_cur;
    int avg_vol;
    int chg_vol;
    int chg_cur;
    int latest_cur;
} shterm_set_event_offchg_args;

typedef struct _shterm_api_get_size_efs_rep {
    struct rpc_reply_hdr hdr;
    int ret;
} shterm_api_get_size_efs_rep;

typedef struct _shterm_api_get_offchg_log_rep {
    struct rpc_reply_hdr hdr;
    shterm_set_event_offchg_args data[1];
} shterm_api_get_offchg_log_rep;

typedef struct _shterm_rpc_a2m_req {
    struct rpc_request_hdr hdr;
} shterm_rpc_a2m_req;

typedef struct _shterm_rpc_a2m_rep {
    struct rpc_reply_hdr hdr;
} shterm_rpc_a2m_rep;

typedef struct _shterm_rpc_a2m_cmd_rep {
    struct rpc_reply_hdr hdr;
    int one;
    int two;
    int thr;
    int fou;
    int fiv;
    int six;
    int sev;
    int eig;
    int nin;
    int ten;
    int ele;
} shterm_rpc_a2m_cmd_rep;

typedef struct _shbattlog_data {
    int bat_vol;
    int bat_temp;
    int chg_temp;
    int cam_temp;
    int pmic_temp;
    int pa_temp;
    int avg_cur;
    int avg_vol;
    int chg_vol;
    int chg_cur;
    int latest_cur;
} shbattlog_data;

static shterm_api_get_offchg_log_rep *efs_data;
static int efs_data_size = 0;

static struct msm_rpc_endpoint *endpoint;

static int shterm_rpc_call( struct msm_rpc_server* svr,
                            struct rpc_request_hdr* req,
                            unsigned len )
{
    int ret;
    shterm_set_info_args *i_args;
    shterm_set_event_args *e_args;
    shbattlog_info_t info = {0};

    switch( req->procedure ){
    case ONCRPC_SHTERM_SET_INFO_PROC:
        i_args = (shterm_set_info_args *)(req + 1);
        i_args->id = be32_to_cpu( i_args->id );
        i_args->val = be32_to_cpu( i_args->val );
        ret = shterm_k_set_info( i_args->id, i_args->val );
        break;

    case ONCRPC_SHTERM_SET_EVENT_PROC:
        e_args = (shterm_set_event_args *)(req + 1);
        info.event_num = be32_to_cpu( e_args->event_num );

        switch( info.event_num ){
        case SHBATTLOG_EVENT_HIGH_TEMP:
        case SHBATTLOG_EVENT_FGIC_EX10:
        case SHBATTLOG_EVENT_FGIC_EX20:
        case SHBATTLOG_EVENT_FGIC_EX30:
        case SHBATTLOG_EVENT_FGIC_EX40:
        case SHBATTLOG_EVENT_FGIC_EX50:
        case SHBATTLOG_EVENT_FGIC_EX60:
        case SHBATTLOG_EVENT_FGIC_EX70:
        case SHBATTLOG_EVENT_FGIC_EX80:
        case SHBATTLOG_EVENT_FGIC_EX90:
        case SHBATTLOG_EVENT_FGIC_EX100:
            info.bat_vol = be32_to_cpu( e_args->one );
            info.bat_temp = be32_to_cpu( e_args->two );
            info.chg_temp = be32_to_cpu( e_args->thr );
            info.cam_temp = be32_to_cpu( e_args->fou );
            info.pmic_temp = be32_to_cpu( e_args->fiv );
            info.pa_temp = be32_to_cpu( e_args->six );
            info.avg_cur = be32_to_cpu( e_args->sev );
            info.avg_vol = be32_to_cpu( e_args->eig );
            break;

        case SHBATTLOG_EVENT_BATT_REPORT_NORM:
        case SHBATTLOG_EVENT_BATT_REPORT_CHG:
            info.chg_vol = be32_to_cpu( e_args->one );
            info.chg_cur = be32_to_cpu( e_args->two );
            info.latest_cur = be32_to_cpu( e_args->thr );
            info.bat_vol = be32_to_cpu( e_args->fou );
            info.bat_temp = be32_to_cpu( e_args->fiv );
            info.chg_temp = be32_to_cpu( e_args->six );
            info.cam_temp = be32_to_cpu( e_args->sev );
            info.pmic_temp = be32_to_cpu( e_args->eig );
            info.pa_temp = be32_to_cpu( e_args->nin );
            info.avg_cur = be32_to_cpu( e_args->ten );
            info.avg_vol = be32_to_cpu( e_args->ele );
            break;

        case SHBATTLOG_EVENT_FATAL_BATT:
        case SHBATTLOG_EVENT_INDICATER_0:
        case SHBATTLOG_EVENT_CHG_IDLE_ST:
        case SHBATTLOG_EVENT_CHG_FAST_ST:
        case SHBATTLOG_EVENT_CHG_END:
        case SHBATTLOG_EVENT_BATT_ID_INVALID:
        case SHBATTLOG_EVENT_CHG_COMP:
        case SHBATTLOG_EVENT_CHG_RESTART:
        case SHBATTLOG_EVENT_CHG_ERROR:
        case SHBATTLOG_EVENT_CHG_TRICKLE_ST:
        case SHBATTLOG_EVENT_CHG_MAINT_ST:
        case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
        case SHBATTLOG_EVENT_CHG_HOT_ADD_FAST_ST:
        case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
        case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
        case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
        case SHBATTLOG_EVENT_CHG_ERR_BAT_ID_INVALID_ST:
        case SHBATTLOG_EVENT_FGIC_NOMAL:
        case SHBATTLOG_EVENT_FGIC_INVALID:
            info.bat_vol = be32_to_cpu( e_args->one );
            info.bat_temp = be32_to_cpu( e_args->two );
            info.chg_temp = be32_to_cpu( e_args->thr );
            info.cam_temp = be32_to_cpu( e_args->fou );
            info.pmic_temp = be32_to_cpu( e_args->fiv );
            info.pa_temp = be32_to_cpu( e_args->six );
            break;

        case SHBATTLOG_EVENT_CHG_INSERT_USB:
        case SHBATTLOG_EVENT_CHG_START:
        case SHBATTLOG_EVENT_CHG_REMOVE_USB:
        case SHBATTLOG_EVENT_CHG_INSERT_CHGR:
        case SHBATTLOG_EVENT_CHG_REMOVE_CHGR:
            info.bat_vol = be32_to_cpu( e_args->one );
            info.bat_temp = be32_to_cpu( e_args->two );
            info.chg_temp = be32_to_cpu( e_args->thr );
            info.cam_temp = be32_to_cpu( e_args->fou );
            info.pmic_temp = be32_to_cpu( e_args->fiv );
            info.pa_temp = be32_to_cpu( e_args->six );
            info.chg_vol = be32_to_cpu( e_args->sev );
            info.chg_cur = be32_to_cpu( e_args->eig );
            break;

        case SHBATTLOG_EVENT_OVER_CURRENT1:
        case SHBATTLOG_EVENT_OVER_CURR1_DET:
        case SHBATTLOG_EVENT_OVER_CURR1_RELEASE:
            info.bat_vol = be32_to_cpu( e_args->one );
            info.bat_temp = be32_to_cpu( e_args->two );
            info.chg_temp = be32_to_cpu( e_args->thr );
            info.cam_temp = be32_to_cpu( e_args->fou );
            info.pmic_temp = be32_to_cpu( e_args->fiv );
            info.pa_temp = be32_to_cpu( e_args->six );
            info.latest_cur = be32_to_cpu( e_args->sev );
            break;

        default:
            break;
        }
        ret = shterm_k_set_event( &info );
        //printk( "%s: shterm_k_set_event ret = %d\n", __FUNCTION__, ret );
        break;

    default:
        ret = -1;
        break;
    }

    return 0;
}

static struct msm_rpc_server shterm_rpc_server =
{
    .prog     = SHTERM_PROG,
    .vers     = SHTERM_VERS,
    .rpc_call = shterm_rpc_call,
};

static int shterm_a2m_rpc_get_efs_size_call( void )
{
    int ret = 0;
    int rpc_ret = 0;
    shterm_rpc_a2m_req req;
    shterm_api_get_size_efs_rep rep;

    endpoint = msm_rpc_connect_compatible( SHTERM_A2M_PROG, SHTERM_A2M_VERS, 0 );
    if( IS_ERR(endpoint) ){
        printk( "%s: init rpc failed! ret = %lx\n", __FUNCTION__, PTR_ERR(endpoint) );
        return -1;
    }

    ret = msm_rpc_call_reply( endpoint,
                              ONCRPC_SHTERM_GET_OFFCHG_LOGSIZE_REMOTE_PROC,
                              &req, sizeof(req),
                              &rep, sizeof(rep),
                              SHTERM_A2M_RPC_CALL_TIMEOUT );

    if( ret < 0 ){
        printk( "%s: msm_rpc_call err ret = %d\n", __FUNCTION__, ret );
        return ret;
    }

    rpc_ret = be32_to_cpu( rep.ret );

    return rpc_ret;
}

static int shterm_a2m_rpc_get_efs_data_call( void )
{
    int ret = 0;
    shterm_rpc_a2m_req req;
    int n;

    endpoint = msm_rpc_connect_compatible( SHTERM_A2M_PROG, SHTERM_A2M_VERS, 0 );
    if( IS_ERR(endpoint) ){
        printk( "%s: init rpc failed! ret = %lx\n", __FUNCTION__, PTR_ERR(endpoint) );
        return 0;
    }

    ret = msm_rpc_call_reply( endpoint,
                              ONCRPC_SHTERM_GET_OFFCHG_DATA_REMOTE_PROC,
                              &req, sizeof(req),
                              efs_data, sizeof(shterm_api_get_offchg_log_rep) - sizeof(shterm_set_event_offchg_args) + (sizeof(shterm_set_event_offchg_args) * efs_data_size),
                              SHTERM_A2M_RPC_CALL_TIMEOUT );

    if( ret < 0 ){
        printk( "%s: msm_rpc_call err ret = %d\n", __FUNCTION__, ret );
        return ret;
    }
    for( n = 0; n < efs_data_size; n++ ){
        efs_data->data[n].event_num = be32_to_cpu( efs_data->data[n].event_num );
        efs_data->data[n].time = be32_to_cpu( efs_data->data[n].time );
        efs_data->data[n].bat_vol = be32_to_cpu( efs_data->data[n].bat_vol );
        efs_data->data[n].bat_temp = be32_to_cpu( efs_data->data[n].bat_temp );
        efs_data->data[n].chg_temp = be32_to_cpu( efs_data->data[n].chg_temp );
        efs_data->data[n].cam_temp = be32_to_cpu( efs_data->data[n].cam_temp );
        efs_data->data[n].pmic_temp = be32_to_cpu( efs_data->data[n].pmic_temp );
        efs_data->data[n].pa_temp = be32_to_cpu( efs_data->data[n].pa_temp );
        efs_data->data[n].avg_cur = be32_to_cpu( efs_data->data[n].avg_cur );
        efs_data->data[n].avg_vol = be32_to_cpu( efs_data->data[n].avg_vol );
        efs_data->data[n].chg_vol = be32_to_cpu( efs_data->data[n].chg_vol );
        efs_data->data[n].chg_cur = be32_to_cpu( efs_data->data[n].chg_cur );
        efs_data->data[n].latest_cur = be32_to_cpu( efs_data->data[n].latest_cur );
    }

    return 0;
}

static int shterm_a2m_init_timer_api_call( void )
{
    int ret = 0;
    shterm_rpc_a2m_req req;
    shterm_rpc_a2m_rep rep;

    endpoint = msm_rpc_connect_compatible( SHTERM_A2M_PROG, SHTERM_A2M_VERS, 0 );
    if( IS_ERR(endpoint) ){
        printk( "%s: init rpc failed! ret = %lx\n", __FUNCTION__, PTR_ERR(endpoint) );
        return -1;
    }

    ret = msm_rpc_call_reply( endpoint,
                              ONCRPC_SHTERM_INIT_TIMER_REMOTE_PROC,
                              &req, sizeof(req),
                              &rep, sizeof(rep),
                              SHTERM_A2M_RPC_CALL_TIMEOUT );

    if( ret < 0 ){
        printk( "%s: msm_rpc_call err ret = %d\n", __FUNCTION__, ret );
    }

    return ret;
}

static int shterm_a2m_set_timer_api_call( void )
{
    int ret = 0;
    shterm_rpc_a2m_req req;
    shterm_rpc_a2m_rep rep;

    endpoint = msm_rpc_connect_compatible( SHTERM_A2M_PROG, SHTERM_A2M_VERS, 0 );
    if( IS_ERR(endpoint) ){
        printk( "%s: init rpc failed! ret = %lx\n", __FUNCTION__, PTR_ERR(endpoint) );
        return -1;
    }

    ret = msm_rpc_call_reply( endpoint,
                              ONCRPC_SHTERM_SET_TIMER_REMOTE_PROC,
                              &req, sizeof(req),
                              &rep, sizeof(rep),
                              SHTERM_A2M_RPC_CALL_TIMEOUT );

    if( ret < 0 ){
        printk( "%s: msm_rpc_call err ret = %d\n", __FUNCTION__, ret );
    }

    return ret;
}

static int shterm_a2m_req_val_api_call( shbattlog_data *info )
{
    int ret = 0;
    shterm_rpc_a2m_req req;
    shterm_rpc_a2m_cmd_rep rep;

    endpoint = msm_rpc_connect_compatible( SHTERM_A2M_PROG, SHTERM_A2M_VERS, 0 );
    if( IS_ERR(endpoint) ){
        printk( "%s: init rpc failed! ret = %lx\n", __FUNCTION__, PTR_ERR(endpoint) );
        return -1;
    }

    ret = msm_rpc_call_reply( endpoint,
                              ONCRPC_SHTERM_REQ_VAL_REMOTE_PROC,
                              &req, sizeof(req),
                              &rep, sizeof(rep),
                              SHTERM_A2M_RPC_CALL_TIMEOUT );

    if( ret < 0 ){
        printk( "%s: msm_rpc_call err ret = %d\n", __FUNCTION__, ret );
    }

    info->chg_vol = be32_to_cpu( rep.one );
    info->chg_cur = be32_to_cpu( rep.two );
    info->latest_cur = be32_to_cpu( rep.thr );
    info->bat_vol = be32_to_cpu( rep.fou );
    info->bat_temp = be32_to_cpu( rep.fiv );
    info->chg_temp = be32_to_cpu( rep.six );
    info->cam_temp = be32_to_cpu( rep.sev );
    info->pmic_temp = be32_to_cpu( rep.eig );
    info->pa_temp = be32_to_cpu( rep.nin );
    info->avg_cur = be32_to_cpu( rep.ten );
    info->avg_vol = be32_to_cpu( rep.ele );

    return ret;
}

static int shterm_dev_open( struct inode *inode, struct file *filp )
{
    int size;

    size = shterm_a2m_rpc_get_efs_size_call();
    if( size < 0 ){
        return size;
    }

    /* malloc num byte */
    efs_data = (shterm_api_get_offchg_log_rep *)kzalloc( sizeof(shterm_api_get_offchg_log_rep)
                                                         - sizeof(shterm_set_event_offchg_args)
                                                         + (sizeof(shterm_set_event_offchg_args) * size),
                                                         GFP_KERNEL );
    if( NULL == efs_data ){
        return -1;
    }

    efs_data_size = size;
    return 0;
}

static ssize_t shterm_dev_read( struct file *filp, char __user *buff,
                                size_t len, loff_t *pos )
{
    int ret;

    if( NULL == efs_data ){
        return -1;
    }
    else if( efs_data_size <= 0 ){
        return 0;
    }

    /* RPC call */
    ret = shterm_a2m_rpc_get_efs_data_call();
    if( ret < 0 ){
        return ret;
    }

    /* copy to user area */
    if( copy_to_user(buff, efs_data->data, efs_data_size * sizeof(shterm_set_event_offchg_args)) ){
        return -EFAULT;
    }

    return efs_data_size * sizeof(shterm_set_event_offchg_args);
}

static int shterm_dev_close( struct inode *inode, struct file *filp )
{
    if( filp->private_data ){
        kfree( filp->private_data );
        filp->private_data = NULL;
    }

    kfree( efs_data );
    efs_data_size = 0;

    return 0;
}

static struct file_operations shterm_dev_fops = {
    .owner   = THIS_MODULE,
    .open    = shterm_dev_open,
    .read    = shterm_dev_read,
    .release = shterm_dev_close,
};

static struct miscdevice shterm_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shterm",
    .fops = &shterm_dev_fops,
};

static int shterm_cmd_open( struct inode *inode, struct file *filp )
{
    return 0;
}

static int shterm_cmd_close( struct inode *inode, struct file *filp )
{
    return 0;
}

static ssize_t shterm_cmd_read( struct file *filp, char __user *buff, size_t len, loff_t *pos )
{
    shbattlog_data info = {0};

    shterm_a2m_req_val_api_call( &info );

    /* copy to user area */
    if( copy_to_user(buff, &info, len) ){
        return -EFAULT;
    }

    return len;
}

static ssize_t shterm_cmd_write( struct file *filp, const char __user *buff, size_t len, loff_t *pos )
{
    int proc = 0;
    char char_num[12] = {0};
    int ret = -1;

    if( copy_from_user( char_num, buff, len ) ){
        printk( "copy_from_user %s\n", __FUNCTION__ );
        return -1;
    }

    proc = (int)simple_strtol( char_num, (char **)NULL, 10 );

    if( ONCRPC_SHTERM_INIT_TIMER_REMOTE_PROC == proc ){
        ret = shterm_a2m_init_timer_api_call();
    }
    else if( ONCRPC_SHTERM_SET_TIMER_REMOTE_PROC == proc ){
        ret = shterm_a2m_set_timer_api_call();
    }

    return ret;
}

static struct file_operations shterm_cmd_fops = {
    .owner   = THIS_MODULE,
    .open    = shterm_cmd_open,
    .read    = shterm_cmd_read,
    .write   = shterm_cmd_write,
    .release = shterm_cmd_close,
};

static struct miscdevice shterm_cmd_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shterm_cmd",
    .fops = &shterm_cmd_fops,
};

static int __init shterm_init( void )
{
    int ret;

    ret = msm_rpc_create_server( &shterm_rpc_server );

    ret = misc_register( &shterm_dev );
    if( ret ){
        printk( "misc_register failure %s\n", __FUNCTION__ );
        return ret;
    }

    ret = misc_register( &shterm_cmd_dev );
    if( ret ){
        printk( "misc_register failure %s\n", __FUNCTION__ );
        return ret;
    }

    return ret;
}

module_init(shterm_init);
MODULE_LICENSE("GPL");
