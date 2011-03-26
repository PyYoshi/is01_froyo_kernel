/*
 * ir_LR388G7.h
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

#ifndef _IR_REG_H
#define	_IR_REG_H
#include <stdbool.h>
#include <sharp/ir_types.h>

extern void __iomem *goliath2_hcs1;
extern void __iomem *goliath2_hcs2;

#define	IR_GLT_REG_BASE		goliath2_hcs1
#define	IR_GLT_IRDA_BASE	goliath2_hcs2

#define	IR_GLT_REG_SYSCTL	(IR_GLT_REG_BASE + 0x0000)
#define	IR_GLT_REG_INTR		(IR_GLT_REG_BASE + 0x0002)
#define	IR_GLT_REG_INTM		(IR_GLT_REG_BASE + 0x0006)
#define	IR_GLT_REG_IRDADIV	(IR_GLT_REG_BASE + 0x0032)
#define	IR_GLT_REG_IRDASYS	(IR_GLT_REG_BASE + 0x0484)

#define	IR_QSD_LEDA_EN		147
#define	IR_QSD_IRRX_PU		149

#define	IR_QSD_GPIO_ON		(uint16)0x0001
#define	IR_QSD_GPIO_OFF		(uint16)0x0000

#define	IRM_GLT_READ_REG(Reg)	(*((volatile uint16*)(Reg)))
#define	IRM_GLT_WRITE_REG(Reg, Val)				\
{								\
	*((volatile uint16*)(Reg)) = (Val);			\
}
#define	IRM_GLT_MSKBIT_REG(Reg, Val, Msk)			\
{								\
	*((volatile uint16*)(Reg)) =				\
		(*((volatile uint16*)(Reg)) & ~(Msk)) | (Val);	\
}
#define	IRM_GLT_CLRBIT_REG(Reg, Val)				\
{								\
	*((volatile uint16*)(Reg)) &= ~(Val);			\
}
#define	IRM_GLT_SETBIT_REG(Reg, Val)				\
{								\
	*((volatile uint16*)(Reg)) |= (Val);			\
}

#define	IR_REG_CR_OFFSET	0x0500
#define	IR_REG_CR0		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x00)
#define	IR_REG_CR1		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x02)
#define	IR_REG_CR2		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x04)
#define	IR_REG_CR3		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x06)
#define	IR_REG_CR4		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x08)
#define	IR_REG_CR6		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x0c)
#define	IR_REG_CR8		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x10)
#define	IR_REG_CR9		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x12)
#define	IR_REG_CR10		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x14)
#define	IR_REG_CR11		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x16)
#define	IR_REG_CR12		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x18)
#define	IR_REG_CR14		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x1c)
#define	IR_REG_CR15		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x1e)
#define	IR_REG_CR16		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x20)
#define	IR_REG_CR20		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x28)
#define	IR_REG_CR22		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x2c)
#define	IR_REG_CR24		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x30)
#define	IR_REG_CR26		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x34)
#define	IR_REG_CR27		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x36)
#define	IR_REG_CR28		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x38)
#define	IR_REG_CR30		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x3c)
#define	IR_REG_CR31		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x3e)
#define	IR_REG_CR32		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x40)
#define	IR_REG_CR34		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x44)
#define	IR_REG_CR36		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x48)
#define	IR_REG_CR38		(IR_GLT_IRDA_BASE + IR_REG_CR_OFFSET + 0x4c)

#define	IR_REG_SR_OFFSET	0x0580
#define	IR_REG_SR0		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x00)
#define	IR_REG_SR1		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x02)
#define	IR_REG_SR2		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x04)
#define	IR_REG_SR3		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x06)
#define	IR_REG_SR4		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x08)
#define	IR_REG_SR6		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x0c)
#define	IR_REG_SR8		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x10)
#define	IR_REG_SR9		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x12)
#define	IR_REG_SR10		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x14)
#define	IR_REG_SR11		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x16)
#define	IR_REG_SR12		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x18)
#define	IR_REG_SR14		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x1c)
#define	IR_REG_SR15		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x1e)
#define	IR_REG_SR16		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x20)
#define	IR_REG_SR18		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x24)
#define	IR_REG_SR20		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x28)
#define	IR_REG_SR22		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x2c)
#define	IR_REG_SR24		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x30)
#define	IR_REG_SR27		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x36)
#define	IR_REG_SR28		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x38)
#define	IR_REG_SR30		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x3c)
#define	IR_REG_SR31		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x3e)
#define	IR_REG_SR32		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x40)
#define	IR_REG_SR34		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x44)
#define	IR_REG_SR36		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x48)
#define	IR_REG_SR40		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x50)
#define	IR_REG_SR42		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x54)
#define	IR_REG_SR44		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x58)
#define	IR_REG_SR46		(IR_GLT_IRDA_BASE + IR_REG_SR_OFFSET + 0x5c)

#define	IR_READ_SR0		(*(volatile uint16*)IR_REG_SR0)
#define	IR_READ_SR1		(*(volatile uint16*)IR_REG_SR1)
#define	IR_READ_SR2		(*(volatile uint16*)IR_REG_SR2)
#define	IR_READ_SR3		(*(volatile uint16*)IR_REG_SR3)
#define	IR_READ_SR4		(*(volatile uint16*)IR_REG_SR4)
#define	IR_READ_SR6		(*(volatile uint16*)IR_REG_SR6)
#define	IR_READ_SR6_32BIT	(*(volatile uint32*)IR_REG_SR6)
#define	IR_READ_SR8		(*(volatile uint16*)IR_REG_SR8)
#define	IR_READ_SR9		(*(volatile uint16*)IR_REG_SR9)
#define	IR_READ_SR10		(*(volatile uint16*)IR_REG_SR10)
#define	IR_READ_SR11		(*(volatile uint16*)IR_REG_SR11)
#define	IR_READ_SR12		(*(volatile uint16*)IR_REG_SR12)
#define	IR_READ_SR14		(*(volatile uint16*)IR_REG_SR14)
#define	IR_READ_SR15		(*(volatile uint16*)IR_REG_SR15)
#define	IR_READ_SR16		(*(volatile uint16*)IR_REG_SR16)
#define	IR_READ_SR18		(*(volatile uint16*)IR_REG_SR18)
#define	IR_READ_SR20		(*(volatile uint16*)IR_REG_SR20)
#define	IR_READ_SR22		(*(volatile uint16*)IR_REG_SR22)
#define	IR_READ_SR24		(*(volatile uint16*)IR_REG_SR24)
#define	IR_READ_SR27		(*(volatile uint16*)IR_REG_SR27)
#define	IR_READ_SR28		(*(volatile uint16*)IR_REG_SR28)
#define	IR_READ_SR30		(*(volatile uint16*)IR_REG_SR30)
#define	IR_READ_SR31		(*(volatile uint16*)IR_REG_SR31)
#define	IR_READ_SR32		(*(volatile uint16*)IR_REG_SR32)
#define	IR_READ_SR34		(*(volatile uint16*)IR_REG_SR34)
#define	IR_READ_SR36		(*(volatile uint16*)IR_REG_SR36)
#define	IR_READ_SR40		(*(volatile uint16*)IR_REG_SR40)
#define	IR_READ_SR42		(*(volatile uint16*)IR_REG_SR42)
#define	IR_READ_SR44		(*(volatile uint16*)IR_REG_SR44)
#define	IR_READ_SR46		(*(volatile uint16*)IR_REG_SR46)

#define	IR_WRITE_CR0(v)		(*(volatile uint16*)IR_REG_CR0	= (v))
#define	IR_WRITE_CR1(v)		(*(volatile uint16*)IR_REG_CR1	= (v))
#define	IR_WRITE_CR2(v)		(*(volatile uint16*)IR_REG_CR2	= (v))
#define	IR_WRITE_CR3(v)		(*(volatile uint16*)IR_REG_CR3	= (v))
#define	IR_WRITE_CR4(v)		(*(volatile uint16*)IR_REG_CR4	= (v))
#define	IR_WRITE_CR6(v)		(*(volatile uint16*)IR_REG_CR6	= (v))
#define	IR_WRITE_CR8(v)		(*(volatile uint16*)IR_REG_CR8	= (v))
#define	IR_WRITE_CR9(v)		(*(volatile uint16*)IR_REG_CR9	= (v))
#define	IR_WRITE_CR10(v)	(*(volatile uint16*)IR_REG_CR10 = (v))
#define	IR_WRITE_CR11(v)	(*(volatile uint16*)IR_REG_CR11 = (v))
#define	IR_WRITE_CR12(v)	(*(volatile uint16*)IR_REG_CR12 = (v))
#define	IR_WRITE_CR14(v)	(*(volatile uint16*)IR_REG_CR14 = (v))
#define	IR_WRITE_CR15(v)	(*(volatile uint16*)IR_REG_CR15 = (v))
#define	IR_WRITE_CR16(v)	(*(volatile uint16*)IR_REG_CR16 = (v))
#define	IR_WRITE_CR20(v)	(*(volatile uint16*)IR_REG_CR20 = (v))
#define	IR_WRITE_CR22(v)	(*(volatile uint16*)IR_REG_CR22 = (v))
#define	IR_WRITE_CR24(v)	(*(volatile uint16*)IR_REG_CR24 = (v))
#define	IR_WRITE_CR26(v)	(*(volatile uint16*)IR_REG_CR26 = (v))
#define	IR_WRITE_CR27(v)	(*(volatile uint16*)IR_REG_CR27 = (v))
#define	IR_WRITE_CR28(v)	(*(volatile uint16*)IR_REG_CR28 = (v))
#define	IR_WRITE_CR30(v)	(*(volatile uint16*)IR_REG_CR30 = (v))
#define	IR_WRITE_CR31(v)	(*(volatile uint16*)IR_REG_CR31 = (v))
#define	IR_WRITE_CR32(v)	(*(volatile uint16*)IR_REG_CR32 = (v))
#define	IR_WRITE_CR34(v)	(*(volatile uint16*)IR_REG_CR34 = (v))
#define	IR_WRITE_CR36(v)	(*(volatile uint16*)IR_REG_CR36 = (v))
#define	IR_WRITE_CR38(v)	(*(volatile uint16*)IR_REG_CR38 = (v))

#define	IR_CR_CLEAR			(uint16)0x00
#define	IR_CR0_INT_RX_END		(uint16)0x80
#define	IR_CR0_INT_ENA			(uint16)0x40
#define	IR_CR0_TX_RESET			(uint16)0x20
#define	IR_CR0_RX_RESET			(uint16)0x10
#define	IR_CR0_TIMER_RESET		(uint16)0x08
#define	IR_CR0_SYSTEM_RESET		(uint16)0x04
#define	IR_CR0_CAREER_RESET		(uint16)0x02
#define	IR_CR0_SEND_CNT_SEL		(uint16)0x01
#define	IR_CR0_RECEIVE_CNT_SEL		(uint16)0x00
#define	IR_CR1_DEFSET			(uint16)0xAC
#define	IR_CR2_RX_START_MSK		(uint16)0x80
#define	IR_CR2_RX_OVERRUN_ERR_MSK	(uint16)0x40
#define	IR_CR2_RX_FLM_ERR_MSK		(uint16)0x20
#define	IR_CR2_RX_END_MSK		(uint16)0x10
#define	IR_CR2_TIMER_INTRPT_MSK		(uint16)0x08
#define	IR_CR2_TX_UNDERRUN_ERR_MSK	(uint16)0x04
#define	IR_CR2_TX_END_MSK		(uint16)0x01
#define	IR_CR3_TX_ENA			(uint16)0x80
#define	IR_CR3_TX_CRC_ENA		(uint16)0x40
#define	IR_CR3_RX_ENA			(uint16)0x20
#define	IR_CR3_RX_CRC_ENA		(uint16)0x10
#define	IR_CR3_ADR_MCH_ENA		(uint16)0x08
#define	IR_CR3_MLTCST_ENA		(uint16)0x04
#define	IR_CR3_TXRX_DISENA		(uint16)0x00
#define	IR_CR8_QOS_ADR			(uint16)0x00
#define	IR_CR9_ECHO_ENA			(uint16)0x80
#define	IR_CR9_IRDA_SIR_IRRX2		(uint16)0x20
#define	IR_CR9_SHARP_ASK_IRRX2		(uint16)0x10
#define	IR_CR9_TX_SIP			(uint16)0x01
#define	IR_CR10_FIR_TX_MODE		(uint16)0x80
#define	IR_CR10_SIR_TX_MODE		(uint16)0x20
#define	IR_CR10_ASK_TX_MODE		(uint16)0x10
#define	IR_CR10_FIR_RX_MODE		(uint16)0x08
#define	IR_CR10_SIR_RX_MODE		(uint16)0x02
#define	IR_CR10_ASK_RX_MODE		(uint16)0x01
#define	IR_CR11_SEND_CONTINUE		(uint16)0x20
#define	IR_CR11_OPTINPUT_REV		(uint16)0x10
#define	IR_CR11_TIMER_ENA		(uint16)0x08
#define	IR_CR11_TIMER_CYCYE_CHG		(uint16)0x02
#define	IR_CR11_LOOP_BACK_MODE		(uint16)0x01
#define	IR_CR14_DUMMYDATA		(uint16)0x00
#define	IR_CR15_SD_TERM			(uint16)0x80
#define	IR_CR15_IRTX_AB_OUTPUT		(uint16)0x40
#define	IR_CR15_IRTX_A_OUTDATA		(uint16)0x20
#define	IR_CR15_IRTX_B_OUTDATA		(uint16)0x10
#define	IR_CR15_CONTINU_PKT_RX_ENA	(uint16)0x08
#define	IR_CR15_OPT_IO_A_CNN_ENA	(uint16)0x02
#define	IR_CR15_OPT_IO_B_CNN_ENA	(uint16)0x01
#define	IR_CR20_RX_CRC_ERR_MSK		(uint16)0x80
#define	IR_CR20_RX_OVERRUN_ERR_MSK	(uint16)0x40
#define	IR_CR20_RX_STOP_ERR_MSK		(uint16)0x20
#define	IR_CR20_RX_PRTY_ERR_MSK		(uint16)0x10
#define	IR_CR20_RX_END_MSK		(uint16)0x08
#define	IR_CR20_TX_BUF_SP_ENA_MSK	(uint16)0x04
#define	IR_CR20_TX_BUF_SP_MSK		(uint16)0x02
#define	IR_CR20_TX_END_MSK		(uint16)0x01
#define	IR_CR22_UART_PARITY_SET		(uint16)0x18
#define	IR_CR22_UART_STOP_BIT_SET	(uint16)0x04
#define	IR_CR22_UART_TX_ENA		(uint16)0x02
#define	IR_CR22_UART_RX_ENA		(uint16)0x01
#define	IR_CR22_UART_DISENA		(uint16)0x00
#define	IR_CR24_UART_BAUD_9600		(uint16)817

#define	IR_CR24_UART_BAUD_19200		(uint16)401

#define	IR_CR24_UART_BAUD_38400		(uint16)192

#define	IR_CR24_UART_BAUD_57600		(uint16)123

#define	IR_CR24_UART_BAUD_115200	(uint16)53

#define	IR_CR26_FIFO_RESET		(uint16)0x01
#define	IR_CR27_UART_8BIT_DATA		(uint16)0x10
#define	IR_CR27_FIFO_4M_TX_MODE		(uint16)0x08
#define	IR_CR27_FIFO_4M_RX_MODE		(uint16)0x04
#define	IR_CR27_FIFO_TX_MODE		(uint16)0x02
#define	IR_CR27_FIFO_RX_MODE		(uint16)0x01
#define	IR_CR27_FIFO_INIT_MODE		(uint16)0x00
#define	IR_CR28_ADDBOF_NUM1		(uint16)0xE0
#define	IR_CR28_UART_SIR_PULSE_WID	(uint16)0x10
#define	IR_CR28_UART_BOF_EOF_ENA	(uint16)0x08
#define	IR_CR28_UART_SIR_MOD_ENA	(uint16)0x04
#define	IR_CR28_UART_CRC16_ENA		(uint16)0x02
#define	IR_CR28_ADDBOF_NUM2		(uint16)0x01
#define	IR_CR28_DEFAULT		(IR_CR28_UART_BOF_EOF_ENA	|	\
				  IR_CR28_UART_SIR_MOD_ENA	|	\
				  IR_CR28_UART_CRC16_ENA)
#define	IR_CR28_ADDBOF_SHIFT	5
#define	IR_CR28_ADDBOF_MAX	48
#define	IR_CR28_ADDBOF_0	(uint16)(IR_CR28_DEFAULT	| 	\
				(0 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_1	(uint16)(IR_CR28_DEFAULT	|	\
				(1 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_2	(uint16)(IR_CR28_DEFAULT	| 	\
				(2 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_3	(uint16)(IR_CR28_DEFAULT	| 	\
				(3 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_6	(uint16)(IR_CR28_DEFAULT	| 	\
				(4 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_12	(uint16)(IR_CR28_DEFAULT	| 	\
				(5 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_24	(uint16)(IR_CR28_DEFAULT	| 	\
				(6 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_48	(uint16)(IR_CR28_DEFAULT	| 	\
				(7 << IR_CR28_ADDBOF_SHIFT))
#define	IR_CR28_ADDBOF_4	(uint16)(IR_CR28_DEFAULT	| 	\
				(0 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_5	(uint16)(IR_CR28_DEFAULT	|	\
				(1 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_8	(uint16)(IR_CR28_DEFAULT	|	\
				(2 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_10	(uint16)(IR_CR28_DEFAULT	|	\
				(3 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_14	(uint16)(IR_CR28_DEFAULT	|	\
				(4 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_16	(uint16)(IR_CR28_DEFAULT	|	\
				(5 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_20	(uint16)(IR_CR28_DEFAULT	|	\
				(6 << IR_CR28_ADDBOF_SHIFT)	|	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR28_ADDBOF_32	(uint16)(IR_CR28_DEFAULT	|	\
				(7 << IR_CR28_ADDBOF_SHIFT) |	\
				IR_CR28_ADDBOF_NUM2)
#define	IR_CR32_IRSEL			(uint16)0x01
#define	IR_CR34_TAT_START		(uint16)0x40
#define	IR_CR34_TAT_REST		(uint16)0x20
#define	IR_CR34_TAT_MODE		(uint16)0x10
#define	IR_CR34_PIT_START		(uint16)0x04
#define	IR_CR34_PIT_REST		(uint16)0x02
#define	IR_CR34_PIT_MODE		(uint16)0x01
#define	IR_CR36_TX_END_TIMING0		(uint16)0x00
#define	IR_CR36_TX_END_TIMING1		(uint16)0x40
#define	IR_CR36_TX_END_TIMING2		(uint16)0x80
#define	IR_CR36_TX_END_TIMING3		(uint16)0xc0
#define	IR_CR36_RX_START_INT_EN		(uint16)0x20
#define	IR_CR36_UART_INT_MODE		(uint16)0x10
#define	IR_CR36_FIR_INT_MODE		(uint16)0x01
#define	IR_CR38_SIR_INT_END_SEQ		(uint16)0x10

#define	IR_SR_CLEAR			(uint16)0x00
#define	IR_SR0_RX_READ_POS		(uint16)0x80
#define	IR_SR0_RX_OVERRUN_ERR		(uint16)0x40
#define	IR_SR0_RX_FLM_CRC_ERR		(uint16)0x20
#define	IR_SR0_RX_END			(uint16)0x10
#define	IR_SR0_TX_WRITE_POS		(uint16)0x08
#define	IR_SR0_TX_UNDERRUN_ERR		(uint16)0x04
#define	IR_SR0_TX_CRRIA_SENCE		(uint16)0x02
#define	IR_SR0_TX_END			(uint16)0x01
#define	IR_SR1_NEXT_INTRPT_POS		(uint16)0x80
#define	IR_SR1_STA_FIFO_OVERRUN_ERR	(uint16)0x40
#define	IR_SR1_CRRIA_RACH		(uint16)0x02
#define	IR_SR2_RX_START_MSK		(uint16)0x80
#define	IR_SR2_RX_OVERRUN_ERR_MSK	(uint16)0x40
#define	IR_SR2_RX_FLM_ERR_MSK		(uint16)0x20
#define	IR_SR2_RX_END_MSK		(uint16)0x10
#define	IR_SR2_TIMER_INTRPT_MSK		(uint16)0x08
#define	IR_SR2_TX_UNDERRUN_ERR_MSK	(uint16)0x04
#define	IR_SR2_TX_END_MSK		(uint16)0x01
#define	IR_SR3_RX_START			(uint16)0x80
#define	IR_SR3_RX_OVERRUN_ERR		(uint16)0x40
#define	IR_SR3_RX_FLM_CRC_ERR		(uint16)0x20
#define	IR_SR3_RX_END			(uint16)0x10
#define	IR_SR3_TIMER_INTRPT		(uint16)0x08
#define	IR_SR3_TX_UNDERRUN_ERR		(uint16)0x04
#define	IR_SR3_TX_END			(uint16)0x01
#define	IR_SR8_CHIP_VER			(uint16)0x06
#define	IR_SR9_ECHO_ENA			(uint16)0x80
#define	IR_SR9_IRDA_SIR_IRRX2		(uint16)0x20
#define	IR_SR9_SHARP_ASK_IRRX2		(uint16)0x10
#define	IR_SR9_MCLKSEL1			(uint16)0x04
#define	IR_SR9_MCLKSEL0			(uint16)0x02
#define	IR_SR9_TX_SIP			(uint16)0x01
#define	IR_SR10_FIR_TX_MODE		(uint16)0x80
#define	IR_SR10_SIR_TX_MODE		(uint16)0x20
#define	IR_SR10_ASK_TX_MODE		(uint16)0x10
#define	IR_SR10_FIR_RX_MODE		(uint16)0x08
#define	IR_SR10_SIR_RX_MODE		(uint16)0x02
#define	IR_SR10_ASK_RX_MODE		(uint16)0x01
#define	IR_SR11_OPTINPUT_REV		(uint16)0x10
#define	IR_SR11_TIMER_ENA		(uint16)0x08
#define	IR_SR11_TIMER_CYCYE_CHG		(uint16)0x02
#define	IR_SR11_LOOP_BACK_MODE		(uint16)0x01
#define	IR_SR15_SD_TERM			(uint16)0x80
#define	IR_SR15_IRTX_AB_OUTPUT		(uint16)0x40
#define	IR_SR15_IRTX_A_OUTDATA		(uint16)0x20
#define	IR_SR15_IRTX_B_OUTDATA		(uint16)0x10
#define	IR_SR15_CONTINU_PKT_RX_ENA	(uint16)0x08
#define	IR_SR15_OPT_IO_A_CNN_ENA	(uint16)0x02
#define	IR_SR15_OPT_IO_B_CNN_ENA	(uint16)0x01
#define	IR_SR18_RX_CRC_ERR		(uint16)0x80
#define	IR_SR18_RX_OVERRUN_ERR		(uint16)0x40
#define	IR_SR18_RX_STOP_ERR		(uint16)0x20
#define	IR_SR18_RX_PRTY_ERR		(uint16)0x10
#define	IR_SR18_RX_END			(uint16)0x08
#define	IR_SR18_TX_BUF_EMP_REG		(uint16)0x04
#define	IR_SR18_TX_BUF_EMP		(uint16)0x02
#define	IR_SR18_TX_END			(uint16)0x01
#define	IR_SR20_RX_CRC_ERR_MASK		(uint16)0x80
#define	IR_SR20_RX_OVERRUN_ERR_MASK	(uint16)0x40
#define	IR_SR20_RX_STOP_ERR_MASK	(uint16)0x20
#define	IR_SR20_RX_PRTY_ERR_MASK	(uint16)0x10
#define	IR_SR20_RX_END_MASK		(uint16)0x08
#define	IR_SR20_TXBUF_EMP_REG_MASK	(uint16)0x04
#define	IR_SR20_TXBUF_EMP_MASK		(uint16)0x02
#define	IR_SR20_TX_END_MASK		(uint16)0x01
#define	IR_SR22_UART_RX_DATA_NON	(uint16)0x80
#define	IR_SR22_FIFO_FULL		(uint16)0x40
#define	IR_SR22_UART_TX_DATA_NON	(uint16)0x20
#define	IR_SR22_UART_PARITY_SET		(uint16)0x18
#define	IR_SR22_UART_STOP_BIT_SET	(uint16)0x04
#define	IR_SR22_UART_TX_ENA		(uint16)0x02
#define	IR_SR22_UART_RX_ENA		(uint16)0x01
#define	IR_SR22_UART_DISENA		(uint16)0x00
#define	IR_SR27_FIFO_EMPTY		(uint16)0x80
#define	IR_SR27_FIFO_FULL		(uint16)0x40
#define	IR_SR27_UART_8BIT_DATA		(uint16)0x10
#define	IR_SR27_FIFO_4M_TX_MODE		(uint16)0x08
#define	IR_SR27_FIFO_4M_RX_MODE		(uint16)0x04
#define	IR_SR27_FIFO_TX_MODE		(uint16)0x02
#define	IR_SR27_FIFO_RX_MODE		(uint16)0x01
#define	IR_SR27_FIFO_INIT_MODE		(uint16)0x00
#define	IR_SR28_ADDBOF_NUM1		(uint16)0xE0
#define	IR_SR28_UART_SIR_PULSE_WID	(uint16)0x10
#define	IR_SR28_UART_BOF_EOF_ENA	(uint16)0x08
#define	IR_SR28_UART_SIR_MOD_ENA	(uint16)0x04
#define	IR_SR28_UART_CRC16_ENA		(uint16)0x02
#define	IR_SR28_ADDBOF_NUM2		(uint16)0x01
#define	IR_SR32_IRSEL			(uint16)0x01
#define	IR_SR34_TAT_MODE		(uint16)0x10
#define	IR_SR34_PIT_MODE		(uint16)0x01
#define	IR_SR36_TX_END_TIMING		(uint16)0xc0
#define	IR_SR36_RX_START_INT_EN		(uint16)0x20
#define	IR_SR36_UART_INT_MODE		(uint16)0x10
#define	IR_SR36_FIR_INT_MODE		(uint16)0x01

extern const int32	IR_FIFO_UNIT_SIZE;

extern const int32	IR_LENGTH_ZERO;

typedef enum {
	IR_REG_RESULT_SUCCESS = 0,
	IR_REG_RESULT_FAIL
} TYPE_IR_REG_RESULT;

typedef enum {
	IR_GPIO_LOW = 0,
	IR_GPIO_HIGH
} TYPE_IR_GPIO_VALUE;

typedef enum {
	IR_LED_POWER_NON = 0,
	IR_LED_POWER_LOW,
	IR_LED_POWER_HIGH
} TYPE_LED_POWER;

typedef enum {
	IR_BAUD_9600 = 1,
	IR_BAUD_19200,
	IR_BAUD_38400,
	IR_BAUD_57600,
	IR_BAUD_115200,
	IR_BAUD_4000000
} TYPE_BAUD;

typedef enum {
	IR_INT_RX = 0,
	IR_INT_TX,
	IR_INT_ERROR,
	IR_INT_FATAL
} TYPE_INT_FACTOR;

typedef enum {
	IR_SEND_SINGLE = 0,
	IR_SEND_CONTINUE
} TYPE_IR_SEND_KIND;

TYPE_IR_REG_RESULT ir_reg_ioremap_nocache(void);
void ir_reg_iounmap(void);

void ir_reg_led_leda_on(void);
void ir_reg_led_leda_off(void);
void ir_reg_irdadrv_sir_set_led(void);
void ir_reg_irdadrv_fir_set_led(void);

TYPE_IR_REG_RESULT ir_reg_irda_hwinit(TYPE_LED_POWER a_val);
TYPE_IR_REG_RESULT ir_reg_irda_hwterm(void);

void ir_reg_led_pwr_sel(TYPE_LED_POWER a_val);
void ir_reg_led_pwr_sel_set(TYPE_LED_POWER a_val);

void ir_reg_irda_int_clear(void);

void ir_reg_irda_set_baud(TYPE_BAUD a_baud);
void ir_reg_irda_set_mpi(uint32 a_mpi);
void ir_reg_irda_set_mtt(uint32 a_mtt);
void ir_reg_irda_set_bof(uint32 a_bof);
void ir_reg_irda_set_ca(uint8 a_ca);

void ir_reg_irda_set_timer_control(void);

void ir_reg_irda_txrx_dis(void);

TYPE_IR_REG_RESULT ir_reg_irda_set_txdata(
		uint16* a_dat, int32 a_len, TYPE_IR_SEND_KIND a_kind);

TYPE_IR_REG_RESULT ir_reg_irda_get_rxdata(uint16* a_dat, int32* a_len);

void ir_reg_irda_err_reset(void);
bool ir_reg_irda_chk_fifo_empty(void);
uint16 ir_reg_irda_chk_fifo_free_size(void);
uint16 ir_reg_irda_chk_fifo_used_size(void);

TYPE_INT_FACTOR	ir_reg_irda_sir_get_int_factor(uint16* a_sr18, uint16* a_sr3);

void ir_reg_irda_sir_txrx_dis(void);
void ir_reg_irda_sir_set_rx_adr_match(void);

void ir_reg_irda_sir_set_tx(TYPE_IR_SEND_KIND a_kind);

void ir_reg_irda_sir_tx_ena(void);

TYPE_INT_FACTOR ir_reg_irda_fir_get_int_factor(uint16* a_sr3, uint16* a_sr0,
						uint16* a_sr1, uint16* a_sr18);
void ir_reg_irda_fir_txrx_dis(void);
void ir_reg_irda_fir_set_rx_adr_match(void);
void ir_reg_irda_fir_rx_restart(void);

void ir_reg_irda_fir_set_tx(TYPE_IR_SEND_KIND a_kind);
void ir_reg_irda_fir_tx_ena(void);

void ir_reg_irda_set_carrier_mode(void);
TYPE_IR_REG_RESULT ir_reg_irda_carrier_chk(void);
void ir_reg_irda_carrier_timer_init(uint16 tm);
void ir_reg_irda_carrier_timer_start(void);
void ir_reg_irda_carrier_timer_term(void);
void ir_reg_irda_carrier_timer_int_clear(void);

#endif
