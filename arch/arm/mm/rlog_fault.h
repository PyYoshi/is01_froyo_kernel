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

#ifndef _RLOG_FAULT_H_
#define _RLOG_FAULT_H_

#include <linux/sched.h>

extern int rlog_app_start( struct task_struct *tsk, unsigned long addr,
                            unsigned int fsr, unsigned int sig, int code,
                            struct pt_regs *regs );

#endif /* _RLOG_FAULT_H_ */
