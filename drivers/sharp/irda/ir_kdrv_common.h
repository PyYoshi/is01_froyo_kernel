/*
 * ir_kdrv_common.h
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

#ifndef _IRDA_KDRV_COMMON_H
#define _IRDA_KDRV_COMMON_H

#undef	SC_IR_KERNEL_DEBUG

#define	SC_IR_KERNEL_DEBUG_E

#define	SC_IR_KERNEL_DEBUG_PRINTK

#define IRLOG_PRINTK_ALERT	KERN_ALERT
#define IRLOG_PRINTK_ERR	KERN_ERR
#define IRLOG_PRINTK_WARNING	KERN_WARNING

#define IRLOG_PRINTK_INFO	KERN_INFO

#define IRLOG_IRCOMMON	"IR_COMMON"
#define IRLOG_IRDRV	"IRDA_DRV"
#define IRLOG_IRDIAG	"IRDA_DIAG"
#define IRLOG_IRREG	"IR_REG"

#define IRLOG_FATAL	"F"
#define IRLOG_ERR	"E"
#define IRLOG_HIGH	"H"
#define IRLOG_MED	"M"
#define IRLOG_LOW	"L"

#define	IRLOG_IN	"[IN]"
#define	IRLOG_OUT	"[OUT]"
#define	IRLOG_INFO	"[INFO]"

#define	IRLOG_MSG_FORM(module, level)	"["module":"level"][%s][%d]"

#define IRLOG_MSG_MED_FORM(module, info)	\
	IRLOG_MSG_FORM(module,IRLOG_MED)info

#ifdef SC_IR_KERNEL_DEBUG_PRINTK

#ifdef SC_IR_KERNEL_DEBUG_E
#define MSG_IRCOM_FATAL(a, b, c, d)		\
		printk(IRLOG_PRINTK_ALERT	\
		       IRLOG_MSG_FORM(IRLOG_IRCOMMON, IRLOG_FATAL) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRCOM_ERR(a, b, c, d)		\
		printk(IRLOG_PRINTK_ERR		\
		       IRLOG_MSG_FORM(IRLOG_IRCOMMON, IRLOG_ERR) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRCOM_HIGH(a, b, c, d)		\
		printk(IRLOG_PRINTK_WARNING	\
		       IRLOG_MSG_FORM(IRLOG_IRCOMMON, IRLOG_HIGH) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRCOM_FATAL(a, b, c, d)
#define MSG_IRCOM_ERR(a, b, c, d)
#define MSG_IRCOM_HIGH(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG
#define MSG_IRCOM_MED_IN(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRCOMMON, IRLOG_IN) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRCOM_MED_OUT(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRCOMMON, IRLOG_OUT) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRCOM_MED_INFO(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRCOMMON, IRLOG_INFO) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRCOM_LOW(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_FORM(IRLOG_IRCOMMON, IRLOG_LOW) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRCOM_MED_IN(a, b, c, d)
#define MSG_IRCOM_MED_OUT(a, b, c, d)
#define MSG_IRCOM_MED_INFO(a, b, c, d)
#define MSG_IRCOM_LOW(a, b, c, d)
#endif

#else
#define MSG_IRCOM_FATAL(a, b, c, d)
#define MSG_IRCOM_ERR(a, b, c, d)
#define MSG_IRCOM_HIGH(a, b, c, d)
#define MSG_IRCOM_MED_IN(a, b, c, d)
#define MSG_IRCOM_MED_OUT(a, b, c, d)
#define MSG_IRCOM_MED_INFO(a, b, c, d)
#define MSG_IRCOM_LOW(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG_PRINTK

#ifdef SC_IR_KERNEL_DEBUG_E
#define MSG_IRDRV_FATAL(a, b, c, d)		\
		printk(IRLOG_PRINTK_ALERT	\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_FATAL) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDRV_ERR(a, b, c, d)		\
		printk(IRLOG_PRINTK_ERR		\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_ERR) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDRV_HIGH(a, b, c, d)		\
		printk(IRLOG_PRINTK_WARNING	\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_HIGH) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRDRV_FATAL(a, b, c, d)
#define MSG_IRDRV_ERR(a, b, c, d)
#define MSG_IRDRV_HIGH(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG
#define MSG_IRDRV_MED_IN(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDRV, IRLOG_IN) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDRV_MED_OUT(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDRV, IRLOG_OUT) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDRV_MED_INFO(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDRV, IRLOG_INFO) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDRV_LOW(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_FORM(IRLOG_IRDRV, IRLOG_LOW) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRDRV_MED_IN(a, b, c, d)
#define MSG_IRDRV_MED_OUT(a, b, c, d)
#define MSG_IRDRV_MED_INFO(a, b, c, d)
#define MSG_IRDRV_LOW(a, b, c, d)
#endif

#else
#define MSG_IRDRV_FATAL(a, b, c, d)
#define MSG_IRDRV_ERR(a, b, c, d)
#define MSG_IRDRV_HIGH(a, b, c, d)
#define MSG_IRDRV_MED_IN(a, b, c, d)
#define MSG_IRDRV_MED_OUT(a, b, c, d)
#define MSG_IRDRV_MED_INFO(a, b, c, d)
#define MSG_IRDRV_LOW(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG_PRINTK

#ifdef SC_IR_KERNEL_DEBUG_E
#define MSG_IRDIAG_FATAL(a, b, c, d)		\
		printk(IRLOG_PRINTK_ALERT	\
		       IRLOG_MSG_FORM(IRLOG_IRDIAG, IRLOG_FATAL) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDIAG_ERR(a, b, c, d)		\
		printk(IRLOG_PRINTK_ERR		\
		       IRLOG_MSG_FORM(IRLOG_IRDIAG, IRLOG_ERR) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDIAG_HIGH(a, b, c, d)		\
		printk(IRLOG_PRINTK_WARNING	\
		       IRLOG_MSG_FORM(IRLOG_IRDIAG, IRLOG_HIGH) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRDIAG_FATAL(a, b, c, d)
#define MSG_IRDIAG_ERR(a, b, c, d)
#define MSG_IRDIAG_HIGH(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG
#define MSG_IRDIAG_MED_IN(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDIAG, IRLOG_IN) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDIAG_MED_OUT(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDIAG, IRLOG_OUT) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDIAG_MED_INFO(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRDIAG, IRLOG_INFO) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRDIAG_LOW(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_FORM(IRLOG_IRDIAG, IRLOG_LOW) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRDIAG_MED_IN(a, b, c, d)
#define MSG_IRDIAG_MED_OUT(a, b, c, d)
#define MSG_IRDIAG_MED_INFO(a, b, c, d)
#define MSG_IRDIAG_LOW(a, b, c, d)
#endif

#else
#define MSG_IRDIAG_FATAL(a, b, c, d)
#define MSG_IRDIAG_ERR(a, b, c, d)
#define MSG_IRDIAG_HIGH(a, b, c, d)
#define MSG_IRDIAG_MED_IN(a, b, c, d)
#define MSG_IRDIAG_MED_OUT(a, b, c, d)
#define MSG_IRDIAG_MED_INFO(a, b, c, d)
#define MSG_IRDIAG_LOW(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG_PRINTK

#ifdef SC_IR_KERNEL_DEBUG_E
#define MSG_IRREG_FATAL(a, b, c, d)		\
		printk(IRLOG_PRINTK_ALERT	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_FATAL) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRREG_ERR(a, b, c, d)		\
		printk(IRLOG_PRINTK_ERR		\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_ERR) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRREG_HIGH(a, b, c, d)		\
		printk(IRLOG_PRINTK_WARNING	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_HIGH) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRREG_FATAL(a, b, c, d)
#define MSG_IRREG_ERR(a, b, c, d)
#define MSG_IRREG_HIGH(a, b, c, d)
#endif

#ifdef SC_IR_KERNEL_DEBUG
#define MSG_IRREG_MED_IN(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRREG, IRLOG_IN) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRREG_MED_OUT(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRREG, IRLOG_OUT) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRREG_MED_INFO(a, b, c, d)	\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_MED_FORM(IRLOG_IRREG, IRLOG_INFO) a, \
		       __func__, __LINE__, b, c, d)
#define MSG_IRREG_LOW(a, b, c, d)		\
		printk(IRLOG_PRINTK_INFO	\
		       IRLOG_MSG_FORM(IRLOG_IRREG, IRLOG_LOW) a, \
		       __func__, __LINE__, b, c, d)
#else
#define MSG_IRREG_MED_IN(a, b, c, d)
#define MSG_IRREG_MED_OUT(a, b, c, d)
#define MSG_IRREG_MED_INFO(a, b, c, d)
#define MSG_IRREG_LOW(a, b, c, d)
#endif

#else
#define MSG_IRREG_FATAL(a, b, c, d)
#define MSG_IRREG_ERR(a, b, c, d)
#define MSG_IRREG_HIGH(a, b, c, d)
#define MSG_IRREG_MED_IN(a, b, c, d)
#define MSG_IRREG_MED_OUT(a, b, c, d)
#define MSG_IRREG_MED_INFO(a, b, c, d)
#define MSG_IRREG_LOW(a, b, c, d)
#endif

#endif
