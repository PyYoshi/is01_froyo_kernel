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

#ifndef SHRLOG_TYPE_H /* Double definition is prevented */
#define SHRLOG_TYPE_H

#define OEMLOG_MAGIC_NUM 0x8931baca
#define OEMLOG_FATAL_NUM 0x0930abe4
#define OEMLOG_VERSION 4
#define SH_VER 8
#define TASK_NAME_MAX 11
#define REG_NUMBER 17
#define MAX_STACK 120
#define MAX_STACK_TRACE 5
#define MAX_TCB 100
#define MAX_RUNNING_TASK 10
#define MAX_RESETTYPE 7
#define MAX_RESETHIST 100
#define MAX_CRITSECT 16
#define SMEM_LOG_TABLE_SIZE 120
#define SMEM_LOG_SIZE (20 * 2000)
#define ERR_DATA_SIZE 2144

typedef struct
{
	char						tcb_task_name[TASK_NAME_MAX];
	unsigned long				tcb_add;
	unsigned long				tcb_sp;
	unsigned long				tcb_sigs;
	unsigned long				tcb_wait;
	unsigned long				tcb_pri;
	unsigned long				tcb_trace[MAX_STACK_TRACE];
}tcb_info_struct;

typedef struct
{
	char						task_name[TASK_NAME_MAX];
	unsigned long				reg[REG_NUMBER];
	unsigned long				stk[MAX_STACK];
	unsigned long				cs_num;
	unsigned long				cs_stack[MAX_CRITSECT];
}task_info_struct;

typedef struct
{
	unsigned long				magic_num;
	unsigned long				log_version;
	char						dev_id[SH_VER];
	char						sh_version[SH_VER];
	unsigned long				reset_time;
	unsigned long				curr_mode;
	unsigned long				reset_reason;
	char						err_info[ERR_DATA_SIZE];
} shrlog_system_info;

typedef struct
{
	unsigned long				sh_boot_flag;
	unsigned long				intlock;
	task_info_struct			curr_task;
	task_info_struct			running_info[MAX_RUNNING_TASK];
	unsigned long				running_task_over;
	tcb_info_struct				tcb_info[MAX_TCB];
	unsigned char				detect_info;
	unsigned long				last_timer_item;
	unsigned long				last_timer_tcb;
	unsigned long				last_timer_sig;
	unsigned long				last_timer_func1;
	unsigned long				last_timer_func2;
} shrlog_boot_info;

typedef struct
{
	unsigned char smem_log_table[SMEM_LOG_TABLE_SIZE];
	unsigned char smem_log[SMEM_LOG_SIZE];
	unsigned long smem_log_idx;
} shrlog_smem_info;

typedef struct
{
	char						log_hdr[32];
	shrlog_system_info			system_info;
	shrlog_boot_info			boot_info;
	shrlog_smem_info			smem_info;
	unsigned char				pad[2048];
} shrlog_reset_info;

#endif /* SHRLOG_TYPE_H */
