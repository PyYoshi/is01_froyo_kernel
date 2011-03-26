/*
 * ir_LR388G7.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/pmic.h>
#include <mach/vreg.h>

#include "ir_LR388G7.h"
#include "ir_kdrv_common.h"
#include "sharp/irda_common.h"
#include "sharp/irda_memory.h"

static const uint16	IR_GLT_REG_CLR		= 0x0000;

static const uint16	IR_GLT_IRDADIV		= 0x0100;

static const uint16	IR_GLT_IRDASYS		= 0x0007;
static const uint16	IR_GLT_IRDARST_ENA	= 0x0010;
static const uint16	IR_GLT_IRDARST_DIS	= 0x0000;

static const uint16	IR_GLT_GPIO_OFF		= 0x0000;
static const uint16	IR_GLT_GPIO_ON		= 0x0001;

static const uint16	IR_MTT_MS_MASK		= 0xF0;
static const uint32	IR_MTT_MAX		= 16000;

static const uint16	IR_SD_LOW		= 0x00;
static const uint16	IR_SD_HIGH		= IR_CR15_SD_TERM;

static const int32	IR_FIFO_MAX_SIZE	= 4160;
const int32		IR_FIFO_UNIT_SIZE	= 2;
static const int32	IR_FIFO_UNIT_SIZE_32BIT	= 4;

const int32		IR_LENGTH_ZERO		= 0;

#ifdef IR_FEATURE_WAIT_FOR_SLEEP
static const long	IR_SD_RECOVERY_WAIT_NSEC = (1000 * 1000);
#else
static const uint16	IR_SD_RECOVERY_WAIT	 = 1;
#endif

void __iomem *goliath2_hcs1 = NULL;
void __iomem *goliath2_hcs2 = NULL;

static spinlock_t Irreg_spin_lock;

static TYPE_LED_POWER	g_ir_ledpow = IR_LED_POWER_NON;

static void	ir_reg_set_sd(TYPE_IR_GPIO_VALUE a_val);
static void	ir_reg_gol_irdacc_reset_dis(void);
static void	ir_reg_gol_irdacc_reset_ena(void);
static void	ir_reg_irdacc_ena(void);
static void	ir_reg_irdacc_dis(void);

#define IRDA_ADDR_HCS1			0x60000000
#define IRDA_ADDR_HCS1_SIZE		0x1000
#define IRDA_ADDR_HCS2			0x94000000
#define IRDA_ADDR_HCS2_SIZE		0x800

TYPE_IR_REG_RESULT ir_reg_ioremap_nocache(void)
{
	TYPE_IR_REG_RESULT	w_ret	= IR_REG_RESULT_SUCCESS;

	goliath2_hcs1 = ioremap_nocache(IRDA_ADDR_HCS1, IRDA_ADDR_HCS1_SIZE);
	if (goliath2_hcs1 == NULL) {
		w_ret = IR_REG_RESULT_FAIL;
		MSG_IRREG_FATAL("goliath2_hcs1=0x%x[%d,%d]\n",
						(int)goliath2_hcs1, 0, 0);
	} else {
		goliath2_hcs2 = ioremap_nocache(IRDA_ADDR_HCS2,
							IRDA_ADDR_HCS2_SIZE);
		if (goliath2_hcs2 == NULL) {
			w_ret = IR_REG_RESULT_FAIL;
			MSG_IRREG_FATAL("goliath2_hcs2=0x%x[%d,%d]\n",
						(int)goliath2_hcs2, 0, 0);
		}
	}

	return (w_ret);
}

void ir_reg_iounmap(void)
{
	if (goliath2_hcs1 != NULL) {
		iounmap(goliath2_hcs1);
	}
	if (goliath2_hcs2 != NULL) {
		iounmap(goliath2_hcs2);
	}

	goliath2_hcs1 = NULL;
	goliath2_hcs2 = NULL;

}

TYPE_IR_REG_RESULT ir_reg_irda_hwinit(TYPE_LED_POWER a_val)
{
	TYPE_IR_REG_RESULT	w_ret	= IR_REG_RESULT_SUCCESS;

	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	struct timespec tu;
	#endif

	spin_lock_init(&Irreg_spin_lock);

	ir_reg_gol_irdacc_reset_dis();
	ir_reg_irdacc_ena();

	ir_reg_set_sd(IR_GPIO_LOW);

	ir_reg_led_pwr_sel(a_val);

	if (a_val != IR_LED_POWER_NON) {
		ir_reg_led_leda_on();
	}
	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	tu.tv_sec = 0;
	tu.tv_nsec = IR_SD_RECOVERY_WAIT_NSEC;
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	#else
	mdelay(IR_SD_RECOVERY_WAIT);
	#endif

	g_ir_ledpow = a_val;

	return (w_ret);
}

TYPE_IR_REG_RESULT ir_reg_irda_hwterm(void)
{
	TYPE_IR_REG_RESULT	w_ret	= IR_REG_RESULT_SUCCESS;

	ir_reg_led_leda_off();

	ir_reg_led_pwr_sel(IR_LED_POWER_HIGH);

	ir_reg_irdacc_dis();
	ir_reg_gol_irdacc_reset_ena();

	return (w_ret);
}

void ir_reg_led_leda_on(void)
{
	gpio_tlmm_config(
		GPIO_CFG(IR_QSD_LEDA_EN, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
						GPIO_2MA), GPIO_ENABLE);
	gpio_direction_output(IR_QSD_LEDA_EN, IR_QSD_GPIO_ON);
}

void ir_reg_led_leda_off(void)
{
	gpio_direction_output(IR_QSD_LEDA_EN, IR_QSD_GPIO_OFF);
}

void ir_reg_led_pwr_sel(TYPE_LED_POWER a_val)
{
	if (a_val == IR_LED_POWER_HIGH) {
		IR_WRITE_CR32(IR_CR_CLEAR);
	} else {
		IR_WRITE_CR32(IR_CR32_IRSEL);
	}
}

void ir_reg_led_pwr_sel_set(TYPE_LED_POWER a_val)
{
	g_ir_ledpow = a_val;
	ir_reg_led_pwr_sel(a_val);
}

static void ir_reg_set_sd(TYPE_IR_GPIO_VALUE a_val)
{
	if (a_val == IR_GPIO_LOW) {
		IR_WRITE_CR15(IR_SD_LOW);
	} else {
		IR_WRITE_CR15(IR_SD_HIGH);
	}
}

void ir_reg_irdadrv_sir_set_led(void)
{
	unsigned long spin_lock_flags;

	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	struct timespec tu;
	#endif

	register uint16	w_dmy	= IR_CR15_SD_TERM | IR_CR15_IRTX_AB_OUTPUT;
	register uint16	w_dmy2	= IR_CR15_OPT_IO_A_CNN_ENA;
	spin_lock_irqsave(&Irreg_spin_lock, spin_lock_flags);

	*(volatile uint16*)IR_REG_CR15 = w_dmy;
	*(volatile uint16*)IR_REG_CR14 = w_dmy2;
	*(volatile uint16*)IR_REG_CR15 = w_dmy2;
	w_dmy = IR_READ_SR14;

	spin_unlock_irqrestore(&Irreg_spin_lock, spin_lock_flags);

	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	tu.tv_sec = 0;
	tu.tv_nsec = IR_SD_RECOVERY_WAIT_NSEC;
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	#else
	mdelay(IR_SD_RECOVERY_WAIT);
	#endif
}

void ir_reg_irdadrv_fir_set_led(void)
{
	unsigned long spin_lock_flags;

	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	struct timespec tu;
	#endif

	register uint16	w_dmy;
	register uint16	w_dmy2	= IR_CR15_SD_TERM |
				  IR_CR15_IRTX_AB_OUTPUT |
				  IR_CR15_IRTX_A_OUTDATA;
	register uint16	w_dmy3	= IR_CR15_IRTX_AB_OUTPUT |
				  IR_CR15_IRTX_A_OUTDATA;
	IR_WRITE_CR0(IR_CR0_SYSTEM_RESET | IR_CR0_CAREER_RESET);

	w_dmy = IR_READ_SR14;

	ir_reg_led_pwr_sel(IR_LED_POWER_LOW);

	spin_lock_irqsave(&Irreg_spin_lock, spin_lock_flags);

	*(volatile uint16*)IR_REG_CR15 = w_dmy2;
	*(volatile uint16*)IR_REG_CR14 = w_dmy2;
	*(volatile uint16*)IR_REG_CR15 = w_dmy3;
	w_dmy = IR_READ_SR14;
	IR_WRITE_CR15(IR_CR15_OPT_IO_A_CNN_ENA);

	spin_unlock_irqrestore(&Irreg_spin_lock, spin_lock_flags);

	#ifdef IR_FEATURE_WAIT_FOR_SLEEP
	tu.tv_sec = 0;
	tu.tv_nsec = IR_SD_RECOVERY_WAIT_NSEC;
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	#else
	mdelay(IR_SD_RECOVERY_WAIT);
	#endif

	ir_reg_led_pwr_sel(g_ir_ledpow);
}

static void ir_reg_gol_irdacc_reset_dis(void)
{
	IRM_GLT_WRITE_REG(IR_GLT_REG_IRDASYS, IR_GLT_IRDARST_DIS);
}

static void ir_reg_gol_irdacc_reset_ena(void)
{
	IRM_GLT_WRITE_REG(IR_GLT_REG_IRDASYS, IR_GLT_IRDARST_ENA);
}

static void ir_reg_irdacc_ena(void)
{
	register uint16	w_dmy	= IR_CR15_OPT_IO_A_CNN_ENA;

	IRM_GLT_WRITE_REG(IR_GLT_REG_IRDADIV, IR_GLT_IRDADIV);
	IRM_GLT_WRITE_REG(IR_GLT_REG_IRDASYS, IR_GLT_IRDASYS);

	IR_WRITE_CR0(IR_CR0_SYSTEM_RESET | IR_CR0_CAREER_RESET);

	*(volatile uint16*)IR_REG_CR14 = w_dmy;
	w_dmy = IR_READ_SR14;
}

static void ir_reg_irdacc_dis(void)
{
	IR_WRITE_CR0(IR_CR0_SYSTEM_RESET | IR_CR0_CAREER_RESET);
	IR_WRITE_CR22(IR_CR22_UART_DISENA);
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR27(IR_CR27_FIFO_INIT_MODE);

	IRM_GLT_WRITE_REG(IR_GLT_REG_IRDASYS, IR_GLT_REG_CLR);
}

void ir_reg_irda_int_clear(void)
{
	volatile uint16	w_dmy;

	IR_WRITE_CR3(IR_CR3_TXRX_DISENA);
	IR_WRITE_CR22(IR_CR22_UART_DISENA);

	IR_WRITE_CR20(IR_CR20_RX_CRC_ERR_MSK | IR_CR20_RX_OVERRUN_ERR_MSK |
						IR_CR20_RX_STOP_ERR_MSK |
						IR_CR20_RX_PRTY_ERR_MSK |
						IR_CR20_RX_END_MSK |
						IR_CR20_TX_BUF_SP_ENA_MSK |
						IR_CR20_TX_BUF_SP_MSK |
						IR_CR20_TX_END_MSK);
	IR_WRITE_CR2(IR_CR2_RX_START_MSK | IR_CR2_RX_OVERRUN_ERR_MSK |
						IR_CR2_RX_FLM_ERR_MSK |
						IR_CR2_RX_END_MSK |
						IR_CR2_TIMER_INTRPT_MSK |
						IR_CR2_TX_UNDERRUN_ERR_MSK |
						IR_CR2_TX_END_MSK);

	IR_WRITE_CR0(IR_CR0_TIMER_RESET);
	w_dmy = IR_READ_SR3;
	w_dmy = IR_READ_SR18;

}

void ir_reg_irda_set_baud(TYPE_BAUD a_baud)
{
	const uint16	w_baud_map[6]	= {	IR_CR24_UART_BAUD_9600,
						IR_CR24_UART_BAUD_9600,
						IR_CR24_UART_BAUD_19200,
						IR_CR24_UART_BAUD_38400,
						IR_CR24_UART_BAUD_57600,
						IR_CR24_UART_BAUD_115200 };

	if (a_baud > IR_BAUD_115200) {
		MSG_IRREG_ERR(
			"[ir_reg_irda_set_baud]a_baud = %x[%d,%d]",
			a_baud, 0, 0);
		a_baud = IR_BAUD_9600;
	}

	IR_WRITE_CR24(w_baud_map[a_baud]);
}

void ir_reg_irda_set_mpi(uint32 a_mpi)
{
	uint16	w_mpi;

	if (a_mpi > IR_MTT_MAX) {
		MSG_IRREG_ERR("[ir_reg_irda_set_baud]a_mpi = %d[%d,%d]",
							(int)a_mpi, 0, 0);
		a_mpi = IR_MTT_MAX;
	}

	w_mpi = (uint16)(a_mpi / 10);
	if (a_mpi % 10 != 0) {
		w_mpi++;
	}

	if (w_mpi < IR_MTT_MS_MASK) {
		IR_WRITE_CR30(w_mpi);
	} else {
		w_mpi = (uint16)(a_mpi / 1000);
		if (a_mpi % 1000 != 0) {
			w_mpi++;
		}

		IR_WRITE_CR30(IR_MTT_MS_MASK | (w_mpi - 1));
	}
}

void ir_reg_irda_set_mtt(uint32 a_mtt)
{
	uint16	w_mtt;

	if (a_mtt > IR_MTT_MAX) {
		MSG_IRREG_ERR("[ir_reg_irda_set_mtt]a_mtt = %d[%d,%d]",
							(int)a_mtt, 0, 0);
		a_mtt = IR_MTT_MAX;
	}

	w_mtt = (uint16)(a_mtt / 10);
	if (a_mtt % 10 != 0) {
		w_mtt++;
	}

	if (w_mtt < IR_MTT_MS_MASK) {
		IR_WRITE_CR31(w_mtt);
	} else {
		w_mtt = (uint16)(a_mtt / 1000);
		if (a_mtt % 1000 != 0) {
			w_mtt++;
		}

		IR_WRITE_CR31(IR_MTT_MS_MASK | (w_mtt - 1));
	}
}

void ir_reg_irda_set_bof(uint32 a_bof)
{
	const uint16	w_bof_map[IR_CR28_ADDBOF_MAX + 1] = {
	IR_CR28_ADDBOF_0,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_2, IR_CR28_ADDBOF_3,IR_CR28_ADDBOF_4,
	IR_CR28_ADDBOF_5,IR_CR28_ADDBOF_6, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_8,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_10,IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_12,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_14,IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_16,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_20,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_24,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_32,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1,
	IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_1, IR_CR28_ADDBOF_1,IR_CR28_ADDBOF_48 };

	if (a_bof > IR_CR28_ADDBOF_MAX) {
		MSG_IRREG_ERR(
		"[ir_reg_irda_set_bof]a_bof = %d[%d,%d]", (int)a_bof, 0, 0);
		a_bof = 1;
	}

	IR_WRITE_CR28(w_bof_map[a_bof]);
}

void ir_reg_irda_set_ca(uint8 a_ca)
{
	IR_WRITE_CR8((uint16)a_ca);
}

void ir_reg_irda_set_timer_control(void)
{
	IR_WRITE_CR34(IR_CR34_TAT_MODE | IR_CR34_PIT_MODE);
}

void ir_reg_irda_txrx_dis(void)
{
	ir_reg_irda_fir_txrx_dis();
	ir_reg_irda_sir_txrx_dis();
}

TYPE_IR_REG_RESULT ir_reg_irda_set_txdata(
			uint16* a_dat, int32 a_len, TYPE_IR_SEND_KIND a_kind)
{
	register uint16*	p_dat	= a_dat;
	register int32		w_len	= a_len;
	TYPE_IR_REG_RESULT	w_ret	= IR_REG_RESULT_SUCCESS;

	if ((p_dat == NULL) || (w_len <= IR_LENGTH_ZERO) ||
						(w_len > IR_FIFO_MAX_SIZE)) {
		MSG_IRREG_ERR(
		 "[ir_reg_irda_set_txdata]a_dat = 0x%x, a_len = %d[%d]",
						(int)p_dat, (int)w_len, 0);
		w_ret = IR_REG_RESULT_FAIL;
	} else {
		if (a_kind == IR_SEND_CONTINUE) {
			IR_WRITE_CR6((uint16)w_len);
		} else {
			IR_WRITE_CR4((uint16)w_len);
		}

		while (w_len > IR_LENGTH_ZERO) {
			IR_WRITE_CR6(*p_dat++);
			w_len -= IR_FIFO_UNIT_SIZE;
		}

	}

	return (w_ret);
}

TYPE_IR_REG_RESULT ir_reg_irda_get_rxdata(uint16* a_dat, int32* a_len)
{
	register uint32*	p_dat	= (uint32*)a_dat;
	register int32		i, w_32b, w_16b;
	TYPE_IR_REG_RESULT	w_ret	= IR_REG_RESULT_SUCCESS;

	if (p_dat == NULL || a_len == NULL) {
		if (p_dat == NULL) {
			MSG_IRREG_ERR(
			"[ir_reg_irda_get_rxdata]a_dat = NULL[%d,%d,%d]\n",
								0, 0, 0);
		}
		if (a_len == NULL) {
			MSG_IRREG_ERR(
			"[ir_reg_irda_get_rxdata]a_len = NULL[%d,%d,%d]\n",
								0, 0, 0);
		}
		w_ret = IR_REG_RESULT_FAIL;
	} else {
		*a_len = (int32)IR_READ_SR4;

		if (*a_len <= IRDA_MEM_1FRAME_SIZE) {
			w_32b = *a_len / IR_FIFO_UNIT_SIZE_32BIT;
			w_16b = *a_len % IR_FIFO_UNIT_SIZE_32BIT;
			if (w_16b > IR_FIFO_UNIT_SIZE) {
				w_32b++;
				w_16b = 0;
			}

			for (i = IR_LENGTH_ZERO; i < w_32b; i++) {
				*p_dat++ = IR_READ_SR6_32BIT;
			}
			if (w_16b > IR_LENGTH_ZERO) {
				*p_dat++ = IR_READ_SR6;
			}
		} else {
			MSG_IRREG_ERR(
			"[ir_reg_irda_get_rxdata]size over a_len = %d[%d,%d]\n",
							(int)*a_len, 0, 0);
			w_ret = IR_REG_RESULT_FAIL;
		}
	}

	return (w_ret);
}

void ir_reg_irda_err_reset(void)
{
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR0(IR_CR0_TX_RESET | IR_CR0_RX_RESET | IR_CR0_CAREER_RESET);
}

bool ir_reg_irda_chk_fifo_empty(void)
{
	bool	w_ret	= false;

	if ((IR_READ_SR27 & IR_SR27_FIFO_EMPTY) != IR_SR_CLEAR) {
		w_ret = true;
	}

	return (w_ret);
}

uint16 ir_reg_irda_chk_fifo_free_size(void)
{
	uint16	size = 0;

	size = IR_READ_SR40;

	return (size);
}

uint16 ir_reg_irda_chk_fifo_used_size(void)
{
	uint16	size = 0;

	size = IR_READ_SR42;

	return (size);
}

TYPE_INT_FACTOR ir_reg_irda_sir_get_int_factor(uint16* a_sr18, uint16* a_sr3)
{
	TYPE_INT_FACTOR	w_ret;

	*a_sr18 = IR_READ_SR18;
	*a_sr3 = 0;

	if ((*a_sr18 & IR_SR18_RX_END) != IR_SR_CLEAR) {
		if ((*a_sr18 & IR_SR18_RX_CRC_ERR) != IR_SR_CLEAR) {
			w_ret = IR_INT_ERROR;
		} else {
			w_ret = IR_INT_RX;
		}
	} else if ((*a_sr18 & IR_SR18_TX_END) != IR_SR_CLEAR) {
		w_ret = IR_INT_TX;
	} else {
		*a_sr3 = IR_READ_SR3;
		w_ret = IR_INT_FATAL;
	}

	return (w_ret);
}

void ir_reg_irda_sir_txrx_dis(void)
{
	IR_WRITE_CR22(IR_CR22_UART_DISENA);
}

void ir_reg_irda_sir_set_rx_adr_match(void)
{
	IR_WRITE_CR15(IR_CR15_OPT_IO_A_CNN_ENA);
	IR_WRITE_CR0(IR_CR0_RECEIVE_CNT_SEL);
	IR_WRITE_CR20(IR_CR20_TX_BUF_SP_ENA_MSK |
			IR_CR20_TX_BUF_SP_MSK |
			IR_CR20_TX_END_MSK);
	IR_WRITE_CR10(IR_CR10_SIR_RX_MODE);
	IR_WRITE_CR11(IR_CR11_OPTINPUT_REV);
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR27(IR_CR27_FIFO_RX_MODE);

	IR_WRITE_CR3(IR_CR3_ADR_MCH_ENA);

	IR_WRITE_CR22(IR_CR22_UART_RX_ENA);
}

void ir_reg_irda_sir_set_tx(TYPE_IR_SEND_KIND a_kind)
{
	IR_WRITE_CR15(IR_CR15_OPT_IO_A_CNN_ENA);
	IR_WRITE_CR0(IR_CR0_SEND_CNT_SEL);
	IR_WRITE_CR20(IR_CR20_RX_CRC_ERR_MSK |
			IR_CR20_RX_OVERRUN_ERR_MSK |
			IR_CR20_RX_STOP_ERR_MSK |
			IR_CR20_RX_PRTY_ERR_MSK |
			IR_CR20_RX_END_MSK |
			IR_CR20_TX_BUF_SP_ENA_MSK |
			IR_CR20_TX_BUF_SP_MSK);
	IR_WRITE_CR10(IR_CR10_SIR_TX_MODE);
	if (a_kind == IR_SEND_CONTINUE) {
		IR_WRITE_CR11(IR_CR11_SEND_CONTINUE);
	} else {
		IR_WRITE_CR11(IR_CR_CLEAR);
	}
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR27(IR_CR27_FIFO_TX_MODE);
}

void ir_reg_irda_sir_tx_ena(void)
{
	IR_WRITE_CR22(IR_CR22_UART_TX_ENA);
}

TYPE_INT_FACTOR ir_reg_irda_fir_get_int_factor(uint16* a_sr3, uint16* a_sr0,
						uint16* a_sr1, uint16* a_sr18)
{
	TYPE_INT_FACTOR	w_ret;

	*a_sr3 = IR_READ_SR3;
	*a_sr0 = IR_READ_SR0;
	*a_sr1 = IR_READ_SR1;
	*a_sr18 = 0;

	if ((*a_sr3 & IR_SR3_RX_END) != IR_SR_CLEAR) {
		if (((*a_sr3 & (IR_SR3_RX_OVERRUN_ERR | IR_SR3_RX_FLM_CRC_ERR))
							!= IR_SR_CLEAR) ||
			((*a_sr0 & IR_SR0_RX_FLM_CRC_ERR) != IR_SR_CLEAR) ||
			((*a_sr1 & IR_SR1_STA_FIFO_OVERRUN_ERR) !=
								IR_SR_CLEAR)){
			w_ret = IR_INT_ERROR;
		} else {
			w_ret = IR_INT_RX;
		}
	} else if ((*a_sr3 & IR_SR3_TX_END) != IR_SR_CLEAR) {
		w_ret = IR_INT_TX;
	} else {
		*a_sr18 = IR_READ_SR18;
		w_ret = IR_INT_FATAL;
	}

	return (w_ret);
}

void ir_reg_irda_fir_txrx_dis(void)
{
	IR_WRITE_CR3(IR_CR3_TXRX_DISENA);
}

void ir_reg_irda_fir_set_rx_adr_match(void)
{
	IR_WRITE_CR15(IR_CR15_CONTINU_PKT_RX_ENA |
			IR_CR15_OPT_IO_A_CNN_ENA);
	IR_WRITE_CR0(IR_CR0_RECEIVE_CNT_SEL);
	IR_WRITE_CR1(IR_CR1_DEFSET);
	IR_WRITE_CR2(IR_CR2_RX_START_MSK |
			IR_CR2_TIMER_INTRPT_MSK |
			IR_CR2_TX_UNDERRUN_ERR_MSK |
			IR_CR2_TX_END_MSK);
	IR_WRITE_CR10(IR_CR10_FIR_RX_MODE);
	IR_WRITE_CR11(IR_CR11_OPTINPUT_REV);
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR27(IR_CR27_FIFO_4M_RX_MODE);

	IR_WRITE_CR3(IR_CR3_RX_ENA | IR_CR3_RX_CRC_ENA | IR_CR3_ADR_MCH_ENA);
}

void ir_reg_irda_fir_rx_restart(void)
{
	IR_WRITE_CR0(IR_CR0_INT_RX_END | IR_CR0_INT_ENA);
}

void ir_reg_irda_fir_set_tx(TYPE_IR_SEND_KIND a_kind)
{
	IR_WRITE_CR15(IR_CR15_OPT_IO_A_CNN_ENA);
	IR_WRITE_CR0(IR_CR0_SEND_CNT_SEL);
	IR_WRITE_CR1(IR_CR1_DEFSET);
	IR_WRITE_CR2(IR_CR2_RX_OVERRUN_ERR_MSK |
			IR_CR2_RX_FLM_ERR_MSK |
			IR_CR2_RX_END_MSK |
			IR_CR2_TIMER_INTRPT_MSK |
			IR_CR2_RX_START_MSK);
	IR_WRITE_CR10(IR_CR10_FIR_TX_MODE);
	if (a_kind == IR_SEND_CONTINUE) {
		IR_WRITE_CR11(IR_CR11_SEND_CONTINUE);
		IR_WRITE_CR36(IR_CR36_FIR_INT_MODE);
	} else {
		IR_WRITE_CR11(IR_CR_CLEAR);
		IR_WRITE_CR36(IR_CR_CLEAR);
	}
	IR_WRITE_CR26(IR_CR26_FIFO_RESET);
	IR_WRITE_CR27(IR_CR27_FIFO_4M_TX_MODE);

}

void ir_reg_irda_fir_tx_ena(void)
{
	IR_WRITE_CR3(IR_CR3_TX_ENA | IR_CR3_TX_CRC_ENA);
}

void ir_reg_irda_set_carrier_mode(void)
{
	IR_WRITE_CR10(IR_CR10_SIR_RX_MODE);
	IR_WRITE_CR15(IR_CR15_OPT_IO_A_CNN_ENA);
	IR_WRITE_CR11(IR_CR11_OPTINPUT_REV);
	IR_WRITE_CR0(IR_CR0_TX_RESET | IR_CR0_RX_RESET | IR_CR0_CAREER_RESET);
}

TYPE_IR_REG_RESULT ir_reg_irda_carrier_chk(void)
{
	uint16 a_sr1;
	TYPE_IR_REG_RESULT	w_ret;

	a_sr1 = IR_READ_SR1;

	if ((a_sr1 & IR_SR1_CRRIA_RACH) != IR_SR_CLEAR) {
		IR_WRITE_CR0(IR_CR0_CAREER_RESET);
		w_ret = IR_REG_RESULT_SUCCESS;
	} else {
		w_ret = IR_REG_RESULT_FAIL;
	}

	return (w_ret);
}

void ir_reg_irda_carrier_timer_init(uint16 tm)
{
	uint16 a_sr;

	a_sr = IR_READ_SR11;
	IR_WRITE_CR11(a_sr & ~IR_CR11_TIMER_CYCYE_CHG);
	IR_WRITE_CR12(tm);
	IR_WRITE_CR0(IR_CR0_TIMER_RESET);

	a_sr = IR_READ_SR2;
	IR_WRITE_CR2(a_sr & ~IR_CR2_TIMER_INTRPT_MSK);
}

void ir_reg_irda_carrier_timer_start(void)
{
	uint16 a_sr;

	a_sr = IR_READ_SR11;
	IR_WRITE_CR11(a_sr | IR_CR11_TIMER_ENA);
}

void ir_reg_irda_carrier_timer_term(void)
{
	uint16 a_sr;

	a_sr = IR_READ_SR2;
	IR_WRITE_CR2(a_sr | IR_CR2_TIMER_INTRPT_MSK);
}

void ir_reg_irda_carrier_timer_int_clear(void)
{
	IR_WRITE_CR0(IR_CR0_TIMER_RESET);
}
