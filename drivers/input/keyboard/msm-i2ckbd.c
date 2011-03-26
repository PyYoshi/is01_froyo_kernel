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
 * This code is based on msm-i2ckbd.c.
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
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *
 *  Driver for QWERTY keyboard with I/O communications via
 *  the I2C Interface. The keyboard hardware is a reference design supporting
 *  the standard XT/PS2 scan codes (sets 1&2).
 *
 */

#define	JOG_TIMER_TEST		/* 初回変化を無視する対応 */
#define	ROTATE_90
/* #define	KBD_DEBPRN */

/*+-------------------------------------------------------------------------+*/
/*|	インクルードファイル													|*/
/*+-------------------------------------------------------------------------+*/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <mach/gpio.h>
#include <mach/msm_i2ckbd.h>
#include <linux/syscalls.h>
#ifdef JOG_TIMER_TEST
#include <linux/time.h>
#endif /* JOG_TIMER_TEST */

/*+-------------------------------------------------------------------------+*/
/*|	定数宣言																|*/
/*+-------------------------------------------------------------------------+*/

/* キーマイコンレジスタ番号 */
/*+---------------------------------+*/
/*|	パワーオンリセットレジスタ		|*/
/*+---------------------------------+*/
#define	KPD_RSTINTCLR			0x84
#define	KPD_RSTINTCLR_IRQCLR	0x01
/*+---------------------------------+*/
/*|	クロックモードレジスタ			|*/
/*+---------------------------------+*/
#define	KPD_CLKMODE				0x88
#define	KPD_CLKMODE_MODCTL		0x01
/*+---------------------------------+*/
/*|	クロック設定レジスタ			|*/
/*+---------------------------------+*/
#define	KPD_CLKCFG				0x89
#define	KPD_CLKCFG_CLKSRCSEL	0x40
/*+---------------------------------+*/
/*|	クロック許可レジスタ			|*/
/*+---------------------------------+*/
#define	KPD_CLKEN				0x8A
#define	KPD_CLKEN_TIMOSCEN2		0x20
#define	KPD_CLKEN_TIMOSCEN1		0x10
#define	KPD_CLKEN_TIMOSCEN0		0x08
#define	KPD_CLKEN_TIMEN			0x04
#define	KPD_CLKEN_KBDEN			0x01
/*+---------------------------------+*/
/*|	オートスリープ機能レジスタ		|*/
/*+---------------------------------+*/
#define	KPD_AUTOSLPENA			0x8B
#define	KPD_AUTOSLPENA_ENABLE	0x01
/*+---------------------------------+*/
/*|	オートスリープタイマーレジスタ	|*/
/*+---------------------------------+*/
#define	KPD_AUTOSLPTIMER1		0x8C
#define	KPD_AUTOSLPTIMER2		0x8D
/*+---------------------------------+*/
/*|	I2Cウェイクアップレジスタ		|*/
/*+---------------------------------+*/
#define	KPD_I2CWAKEUPEN			0x8E
#define	KPD_I2CWAKEUPEN_I2CWEN	0x01
/*+---------------------------------+*/
/*|	割込みステータスレジスタ		|*/
/*+---------------------------------+*/
#define	KPD_IRQST				0x91
#define	KPD_IRQST_PORIRQ		0x80
#define	KPD_IRQST_KBDIRQ		0x40
#define	KPD_IRQST_RESERVED		0x20
/* (0x10は空欄) */
#define	KPD_IRQST_TIM2IRQ		0x08
#define	KPD_IRQST_TIM1IRQ		0x04
#define	KPD_IRQST_TIM0IRQ		0x02
#define	KPD_IRQST_GPIIRQ		0x01
/*+---------------------------------+*/
/*|	処理Wait時間の設定レジスタ		|*/
/*+---------------------------------+*/
#define	KPD_KBDSETTLE			0x01
/*+---------------------------------+*/
/*|	デバウンスの設定レジスタ		|*/
/*+---------------------------------+*/
#define	KPD_KBDBOUNCE			0x02
/*+---------------------------------+*/
/*|	マトリックスの設定レジスタ		|*/
/*+---------------------------------+*/
#define	KPD_KBDSIZE				0x03
/*+---------------------------------+*/
/*|	専用キーの設定レジスタ			|*/
/*+---------------------------------+*/
#define	KPD_KBDDEDCFG_H			0x04
#define	KPD_KBDDEDCFG_L			0x05
/*+---------------------------------+*/
/*|	キーコードレジスタ				|*/
/*+---------------------------------+*/
#define	KPD_KEYCODE0			0x0B
#define	KPD_KEYCODE1			0x0C
#define	KPD_KEYCODE2			0x0D
#define	KPD_KEYCODE3			0x0E
/*+---------------------------------+*/
/*|	イベントレジスタ				|*/
/*+---------------------------------+*/
#define	KPD_EVTCODE				0x10
/*+---------------------------------+*/
/*|	KBDロー割込みレジスタ			|*/
/*+---------------------------------+*/
#define	KPD_KBDRIS				0x06
#define	KPD_KBDRIS_RELINT		0x08
#define	KPD_KBDRIS_REVINT		0x04
#define	KPD_KBDRIS_RKLINT		0x02
#define	KPD_KBDRIS_RSINT		0x01
/*+---------------------------------+*/
/*|	KBDマスク割込みレジスタ			|*/
/*+---------------------------------+*/
#define	KPD_KBDMIS				0x07
#define	KPD_KBDMIS_MELINT		0x08
#define	KPD_KBDMIS_MEVINT		0x04
#define	KPD_KBDMIS_MKLINT		0x02
#define	KPD_KBDMIS_MSINT		0x01
/*+---------------------------------+*/
/*|	KBD割込みクリアレジスタ			|*/
/*+---------------------------------+*/
#define	KPD_KBDIC				0x08
#define	KPD_KBDIC_SFOFF			0x80
#define	KPD_KBDIC_EVTIC			0x02
#define	KPD_KBDIC_KBDIC			0x01
/*+---------------------------------+*/
/*|	KBDマスクレジスタ				|*/
/*+---------------------------------+*/
#define	KPD_KBDMSK				0x09
#define	KPD_KBDMSK_MSKELNT		0x08
#define	KPD_KBDMSK_MSKEINT		0x04
#define	KPD_KBDMSK_MSKKLI		0x02
#define	KPD_KBDMSK_MSKSINT		0x01
/*+---------------------------------+*/
/*|	KBD変更特徴設定レジスタ			|*/
/*+---------------------------------+*/
#define	KPD_KBDMFS				0x8F

/*+---------------------------------+*/
/*|	プル設定レジスタ				|*/
/*+---------------------------------+*/
#define	KPD_IOPC0_H				0xAA
#define	KPD_IOPC0_L				0xAB
#define	KPD_IOPC1_H				0xAC
#define	KPD_IOPC1_L				0xAD
#define	KPD_IOPC2_H				0xAE
#define	KPD_IOPC2_L				0xAF
/*+---------------------------------+*/
/*|	GPIOウェイクアップレジスタ		|*/
/*+---------------------------------+*/
#define	KPD_GPIOWAKE0			0xE9
#define	KPD_GPIOWAKE1			0xEA
#define	KPD_GPIOWAKE2			0xEB

#define	KPD_I2C_RETRY			10

#define	KPD_KEYPRESS			1
#define	KPD_KEYRELEASE			0

#define	KPD_KEYBUFF_SIZE		4		/* キーバッファサイズ */
#define	KPD_KEYCODE_MASK		0x7f
#define	KPD_INVALID_KEY			0x7f

#define	KPD_KEY_MAX				128

/* コマンド */
enum
{
	QPHYSLEN			= 128,
	QCVENDOR_ID			= 0x5143,
	QCPRODUCT_ID		= 1,
	QCVERSION_ID		= 1
};

#define	REL_STEP				1		/* 相対移動する距離 */

#define	INITDELAY_TIMER			600
#define	RECOVER_TIMER			1000

#define	KBD_DISABLE_FLIP	0x01
#define	KBD_DISABLE_SLEEP	0x02
#define	KBD_DISABLE_API		0x04

#define	KBD_DISABLE_ON		0xFF
#define	KBD_DISABLE_OFF		0x00

#ifdef JOG_TIMER_TEST
#define	JOG_VOID_TIME			5000	/* 割込み無効時間(ms) */
#endif	/* JOG_TIMER_TEST */

/*+-------------------------------------------------------------------------+*/
/*|	型宣言																	|*/
/*+-------------------------------------------------------------------------+*/
typedef struct i2ckybd_record	I2cKbdRec;
typedef struct i2c_client		I2cClient;
typedef struct i2c_device_id	I2cDevID;
typedef struct work_struct		WorkStruct;
typedef struct input_dev		InputDev;
typedef struct device			Device;

struct i2ckybd_record
{
	I2cClient *mpoClient;
	InputDev *mpoInDev;
	int		mnProductInfo;
	char	mcPhysInfo[QPHYSLEN];
	int		mnIrqPin;
	int		(*mpfPinSetupFunc)(void);
	void	(*mpfPinShutdownFunc)(void);
	uint8_t	mbIsActive;					/* ステータス(1:動作中/0:停止中) */
	struct delayed_work moCmdQ;
	WorkStruct moIrqWork;
	WorkStruct moJogIrqWork;
	int		mnJogIrqPin[4];
	int		mnJogVal[4];
	int		mnJogPowPin;
#ifdef JOG_TIMER_TEST
	unsigned long mdwPrev;
	unsigned long mdwInterval;
#endif /* JOG_TIMER_TEST */
	uint16_t	mwPrev[KPD_KEYBUFF_SIZE];
	uint8_t	mbKbdSleepState;
	uint8_t	mbJogSleepState;
	uint8_t	mbAccessState;				/* アクセス状態 */
};

typedef struct
{
	uint8_t	mbRegAdr;					/* レジスタアドレス */
	uint8_t mbData;						/* データ */
} ShKey_I2cWriteData;

static DEFINE_MUTEX(goKbdAccessMutex);

static I2cKbdRec *gpoKbdRec = NULL;

/*+-------------------------------------------------------------------------+*/
/*|	テーブル																|*/
/*+-------------------------------------------------------------------------+*/

static const uint16_t gwShKeyTable[KPD_KEY_MAX] =
{
					/*      Normal    +Shift     +Alt    +ShiftAlt */
	KEY_1,			/* 00:[   1    ][   !    ][        ][        ] */
	KEY_7,			/* 01:[   7    ][   '    ][ マナー ][        ] */
	KEY_Q,			/* 02:[   Q    ][        ][  TAB   ][        ] */
	KEY_U,			/* 03:[   U    ][        ][        ][        ] */
	KEY_S,			/* 04:[   S    ][        ][        ][        ] */
	KEY_K,			/* 05:[   K    ][        ][        ][        ] */
	KEY_C,			/* 06:[   C    ][        ][        ][        ] */
	KEY_DOT,		/* 07:[   .    ][   >    ][   ]    ][   }    ] */
	0,				/* 08:                                         */
	0,				/* 09:                                         */
	0,				/* 0A:                                         */
	0,				/* 0B:                                         */
	0,				/* 0C:                                         */
	0,				/* 0D:                                         */
	0,				/* 0E:                                         */
	0,				/* 0F:                                         */
	KEY_2,			/* 10:[   2    ][   "    ][        ][        ] */
	KEY_8,			/* 11:[   8    ][   (    ][ Vol↑  ][        ] */
	KEY_W,			/* 12:[   W    ][        ][        ][        ] */
	KEY_I,			/* 13:[   I    ][        ][        ][        ] */
	KEY_D,			/* 14:[   D    ][        ][        ][        ] */
	KEY_L,			/* 15:[   L    ][        ][   ;    ][   +    ] */
	KEY_V,			/* 16:[   V    ][        ][        ][        ] */
	KEY_CHARACTER,	/* 17:[  文字  ][        ][        ][        ] */
	0,				/* 18:                                         */
	0,				/* 19:                                         */
	0,				/* 1A:                                         */
	0,				/* 1B:                                         */
	0,				/* 1C:                                         */
	0,				/* 1D:                                         */
	0,				/* 1E:                                         */
	0,				/* 1F:                                         */
	KEY_3,			/* 20:[   3    ][   #    ][        ][        ] */
	KEY_9,			/* 21:[   9    ][   )    ][ Vol↓  ][        ] */
	KEY_E,			/* 22:[   E    ][        ][        ][        ] */
	KEY_O,			/* 23:[   O    ][        ][        ][        ] */
	KEY_F,			/* 24:[   F    ][        ][        ][        ] */
	KEY_SLASH,		/* 25:[   /    ][   ?    ][   :    ][   *    ] */
	KEY_B,			/* 26:[   B    ][        ][        ][        ] */
	KEY_SPACE,		/* 27:[スペース][        ][        ][        ] */
	KEY_BACK,		/* 28:[  戻る  ][        ][        ][        ] */
	0,				/* 29:                                         */
	0,				/* 2A:                                         */
	0,				/* 2B:                                         */
	0,				/* 2C:                                         */
	0,				/* 2D:                                         */
	0,				/* 2E:                                         */
	0,				/* 2F:                                         */
	KEY_4,			/* 30:[   4    ][   $    ][        ][        ] */
	KEY_0,			/* 31:[   0    ][   ~    ][   ^    ][        ] */
	KEY_R,			/* 32:[   R    ][        ][        ][        ] */
	KEY_P,			/* 33:[   P    ][        ][   @    ][   `    ] */
	KEY_G,			/* 34:[   G    ][        ][        ][        ] */
	KEY_ENTER,		/* 35:[ Enter  ][        ][Private ][        ] */
	KEY_N,			/* 36:[   N    ][        ][        ][        ] */
	KEY_PICTURE,	/* 37:[ 絵・顔 ][        ][        ][        ] */
	KEY_MENU,		/* 38:[  MENU  ][        ][        ][        ] */
	0,				/* 39:                                         */
	0,				/* 3A:                                         */
	0,				/* 3B:                                         */
	0,				/* 3C:                                         */
	0,				/* 3D:                                         */
	0,				/* 3E:                                         */
	0,				/* 3F:                                         */
	KEY_5,			/* 40:[   5    ][   %    ][        ][        ] */
	KEY_MINUS,		/* 41:[   -    ][   =    ][   \    ][   _    ] */
	KEY_T,			/* 42:[   T    ][        ][        ][        ] */
	KEY_SEARCH,		/* 43:[ サーチ ][   `    ][        ][        ] */
	KEY_H,			/* 44:[   H    ][        ][        ][        ] */
	KEY_Z,			/* 45:[   Z    ][        ][        ][        ] */
	KEY_M,			/* 46:[   M    ][        ][        ][        ] */
	KEY_RIGHTSHIFT,	/* 47:[Shift 右][        ][        ][   _    ] */
	BTN_MOUSE,		/* 48:[   OK   ][        ][        ][        ] */
	0,				/* 49:                                         */
	0,				/* 4A:                                         */
	0,				/* 4B:                                         */
	0,				/* 4C:                                         */
	0,				/* 4D:                                         */
	0,				/* 4E:                                         */
	0,				/* 4F:                                         */
	KEY_6,			/* 50:[   6    ][   &    ][        ][        ] */
	KEY_BACKSPACE,	/* 51:[  DEL   ][        ][        ][        ] */
	KEY_Y,			/* 52:[   Y    ][        ][        ][        ] */
	KEY_A,			/* 53:[   A    ][        ][        ][        ] */
	KEY_J,			/* 54:[   J    ][        ][        ][        ] */
	KEY_X,			/* 55:[   X    ][        ][        ][        ] */
	KEY_COMMA,		/* 56:[   ,    ][   <    ][   [    ][    {   ] */
	KEY_HOME,		/* 57:[ ホーム ][        ][        ][        ] */
	0,				/* 58:                                         */
	0,				/* 59:                                         */
	0,				/* 5A:                                         */
	0,				/* 5B:                                         */
	0,				/* 5C:                                         */
	0,				/* 5D:                                         */
	0,				/* 5E:                                         */
	0,				/* 5F:                                         */
#ifdef ROTATE_90
	KEY_RIGHT,		/* 60:[   ↑   ][        ][  Top   ][        ] */
	KEY_LEFT,		/* 61:[   ↓   ][        ][ Bottom ][        ] */
#else
	KEY_UP,			/* 60:[   ↑   ][        ][  Top   ][        ] */
	KEY_DOWN,		/* 61:[   ↓   ][        ][ Bottom ][        ] */
#endif	/* ROTATE_90 */
	KEY_LEFTALT,	/* 62:[  Alt   ][        ][        ][        ] */
	0,				/* 63:                                         */
	0,				/* 64:                                         */
	0,				/* 65:                                         */
	0,				/* 66:                                         */
	0,				/* 67:                                         */
	0,				/* 68:                                         */
	0,				/* 69:                                         */
	0,				/* 6A:                                         */
	0,				/* 6B:                                         */
	0,				/* 6C:                                         */
	0,				/* 6D:                                         */
	0,				/* 6E:                                         */
	0,				/* 6F:                                         */
#ifdef ROTATE_90
	KEY_UP,			/* 70:[   ←   ][        ][  Home  ][        ] */
	KEY_DOWN,		/* 71:[   →   ][        ][  End   ][        ] */
#else
	KEY_LEFT,		/* 70:[   ←   ][        ][  Home  ][        ] */
	KEY_RIGHT,		/* 71:[   →   ][        ][  End   ][        ] */
#endif	/* ROTATE_90 */
	KEY_LEFTSHIFT,	/* 72:[Shift 左][        ][        ][        ] */
	0,				/* 73:                                         */
	0,				/* 74:                                         */
	0,				/* 75:                                         */
	0,				/* 76:                                         */
	0,				/* 77:                                         */
	0,				/* 78:                                         */
	0,				/* 79:                                         */
	0,				/* 7A:                                         */
	0,				/* 7B:                                         */
	0,				/* 7C:                                         */
	0,				/* 7D:                                         */
	0,				/* 7E:                                         */
	0,				/* 7F:                                         */
};

static const ShKey_I2cWriteData goInitData[] =
	{
		{	KPD_CLKEN,			0x00},
#if 1
												/* SYSCLK=16分周(125KHz) */
		{	KPD_CLKCFG,			KPD_CLKCFG_CLKSRCSEL | 0x04},
#else
												/* SYSCLK= 1分周(2MHz) */
		{	KPD_CLKCFG,			KPD_CLKCFG_CLKSRCSEL | 0x00},
#endif
		{	KPD_CLKMODE,		KPD_CLKMODE_MODCTL},
		{	KPD_CLKEN,			KPD_CLKEN_KBDEN},
		{	KPD_KBDSETTLE,		0xA3},			/* 9.68ms */
		{	KPD_KBDBOUNCE,		0xA3},			/* 9.68ms */
		{	KPD_IOPC0_H,		0xAA},			/* KPX:Pull-up */
		{	KPD_IOPC0_L,		0xAA},
		{	KPD_IOPC1_H,		0xAA},			/* KPY:Pull-up */
		{	KPD_IOPC1_L,		0xAA},
		{	KPD_IOPC2_H,		0x5A},
		{	KPD_IOPC2_L,		0x5A},
		{	KPD_KBDSIZE,		0x8A},			/* 8*10 */
		{	KPD_KBDDEDCFG_H,	0xFF},
		{	KPD_KBDDEDCFG_L,	0xFF},
		{	KPD_KBDMSK,			KPD_KBDMSK_MSKELNT | KPD_KBDMSK_MSKEINT | KPD_KBDMSK_MSKKLI},
		{	0xFF,				0x00},			/* 終端 */
	};
/* スリープ突入時の設定 */
static const ShKey_I2cWriteData goSleepData[] =
	{
		{	KPD_AUTOSLPENA,		0x00},			/* オートスリープ解除 */
		{	KPD_CLKMODE,		0x00},			/* クロック停止 */
		{	0xFF,				0x00},			/* 終端 */
	};
/* ウェイクアップ時の設定 */
static const ShKey_I2cWriteData goWakeupData[] =
	{
												/* クロック再開 */
		{	KPD_CLKMODE,		KPD_CLKMODE_MODCTL},
												/* I2C通信でウェイクアップ */
		{	KPD_I2CWAKEUPEN,	KPD_I2CWAKEUPEN_I2CWEN},
		{	KPD_AUTOSLPTIMER1,	0xFF},			/* (下位8Bits) */
		{	KPD_AUTOSLPTIMER2,	0x07},			/* (上位8Bits) */
		{	KPD_GPIOWAKE2,		0x00},			/* IRQ,KPY10～8 */
		{	KPD_GPIOWAKE1,		0x00},			/* KPY7～KPY0 */
		{	KPD_GPIOWAKE0,		0xFF},			/* KPX7～KPX0 */
												/* オートスリープ有効 */
		{	KPD_AUTOSLPENA,		KPD_AUTOSLPENA_ENABLE},
		{	0xFF,				0x00},			/* 終端 */
	};

/*+-------------------------------------------------------------------------+*/
/*|	プロトタイプ宣言														|*/
/*+-------------------------------------------------------------------------+*/
/* I2Cアクセス */
static int ShKey_I2cRead(I2cClient *poClient, uint8_t bRegAdr, uint8_t *pbBuf, uint32_t dwLen);
static int ShKey_I2cWriteOne(I2cClient *poClient, uint8_t bRegAdr, uint8_t bData);
static int ShKey_I2cWriteAny(I2cClient *poClient, const ShKey_I2cWriteData *poData);

/* キードライバ */
static int __init ShKey_Init(void);
static void __exit ShKey_Exit(void);
static int __devinit ShKey_Probe(I2cClient *poClient, const I2cDevID *poDevId);
static int __devexit ShKey_Remove(I2cClient *poClient);
#ifdef CONFIG_PM
static int ShKey_Suspend(I2cClient *poClient, pm_message_t oMsg);
static int ShKey_Resume(I2cClient *poClient);
#else
#define	ShKey_Suspend		NULL
#define	ShKey_Resume		NULL
#endif	/* CONFIG_PM */
static int ShKey_Command(I2cClient *poClient, unsigned int wCmd, void *pArg);
static void ShKey_Connect2InputSys(WorkStruct *poWork);
static InputDev *ShKey_CreateInputDev(I2cKbdRec *poKbdRec);
static int ShKey_ConfigGPIO(I2cKbdRec *poKbdRec);
static int ShKey_ReleaseGPIO(I2cKbdRec *poKbdRec);
static int ShKey_OpenCB(InputDev *poInDev);
static void ShKey_CloseCB(InputDev *pInDev);
static void ShKey_Shutdown(I2cKbdRec *poKbdRec);
static int ShKey_Start(I2cKbdRec *poKbdRec);
static void ShKey_Stop(I2cKbdRec *poKbdRec);
static irqreturn_t ShKey_IrqHandler(int nIrq, void *pvDevId);
static irqreturn_t ShKey_JogHandler(int nIrq, void *pvDevId);
static void ShKey_EnableJogInt(I2cKbdRec *poKbdRec);
static void ShKey_Recover(WorkStruct *poWork);
static void ShKey_NotifyKeyCode(I2cKbdRec *poKbdRec, InputDev *pInDev, uint8_t *pbCode);
static void ShKey_FetchKeys(WorkStruct *poWork);
static int ShKey_FetchKeysMain(I2cKbdRec *poKbdRec);
static void ShKey_FetchJog(WorkStruct *poWork);
static int ShKey_SetState(I2cKbdRec *poKbdRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, int nIsFirst);
static int ShKey_SetKbdSleep(I2cKbdRec *poKbdRec, int nIsSleep, int nIsFirst);
static int ShKey_SetJogSleep(I2cKbdRec *poKbdRec, int nIsSleep, int nIsFirst);
#ifdef JOG_TIMER_TEST
static unsigned long gettimeofday_msec(void);
#endif /* JOG_TIMER_TEST */

/*+-------------------------------------------------------------------------+*/
/*|	I2Cアクセス																|*/
/*+-------------------------------------------------------------------------+*/
/*+-------------------------------------------------------------------------+*/
/*|	I2Cリード																|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_I2cRead(I2cClient *poClient, uint8_t bRegAdr, uint8_t *pbBuf, uint32_t dwLen)
{
	int nResult;
	int nI;
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= poClient->addr,
					.flags	= 0,
					.buf	= (void *)&bRegAdr,
					.len	= 1
				},
			[1] =
				{
					.addr	= poClient->addr,
					.flags	= I2C_M_RD,
					.buf	= (void *)pbBuf,
					.len	= dwLen
				}
		};

	for(nI = 0; nI < KPD_I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(poClient->adapter, oMsgs, 2);
		if(nResult > 0)
		{
			return 0;
		}
		printk(KERN_DEBUG "[ShKey]I2cRead %d(%02X,reg:%02X,Data:%02X,Len:%d)=%d\n", nI, poClient->addr, bRegAdr, pbBuf[0], dwLen, nResult);
	}
	return nResult;
}

/*+-------------------------------------------------------------------------+*/
/*|	I2Cライト																|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_I2cWriteOne(I2cClient *poClient, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= poClient->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < KPD_I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(poClient->adapter, oMsgs, 1);
		if(nResult > 0)
		{
			return 0;
		}
		printk(KERN_DEBUG "[ShKey]I2cWrite %d(%02X,reg:%02X,Data:%02X)=%d\n", nI, poClient->addr, bRegAdr, bData, nResult);
	}
	return nResult;
}

static int ShKey_I2cWriteAny(I2cClient *poClient, const ShKey_I2cWriteData *poData)
{
	int nI;
	int nResult;

	/* 設定を行う */
	for(nI = 0; poData[nI].mbRegAdr != 0xFF; nI++)
	{
		nResult = ShKey_I2cWriteOne(poClient, poData[nI].mbRegAdr, poData[nI].mbData);
		if(nResult < 0)
		{
			return -1;
		}
	}
	return 0;
}

static struct cdev goKbdCDev;
static struct class *gpoKbdClass;
static dev_t gnKbdDev;

static int KbdIf_Open(struct inode *pINode, struct file *poFile)
{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]Open(PID:%ld)\n", sys_getpid());
#endif	/* KBD_DEBPRN */
	return 0;
}

static int KbdIf_Ioctl(struct inode *pINode, struct file *poFile, unsigned int wCmd, unsigned long dwArg)
{
	int nResult = -EINVAL;

#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]Ioctl(PID:%ld,CMD:%d,ARG:%lx)\n", sys_getpid(), wCmd, dwArg);
#endif	/* KBD_DEBPRN */
	mutex_lock(&goKbdAccessMutex);
	if(gpoKbdRec != NULL)
	{
		nResult = ShKey_Command(gpoKbdRec->mpoClient, wCmd, (void *)dwArg);
		switch(nResult)
		{
		case -1:
			nResult = -EIO;
			break;
		case -2:
			nResult = -EINVAL;
			break;
		case -3:
			nResult = -EFAULT;
			break;
		}
	}
	mutex_unlock(&goKbdAccessMutex);
	return nResult;
}

static const struct file_operations goKbdIf_Fops =
{
	.owner	= THIS_MODULE,
	.open	= KbdIf_Open,
	.ioctl	= KbdIf_Ioctl,
};

int __init KbdIf_Setup(void)
{
	dev_t nMajor = 0;
	dev_t nMinor = 0;
	int nResult;

#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]Setup(PID:%ld)\n", sys_getpid());
#endif	/* KBD_DEBPRN */
	nResult = alloc_chrdev_region(&gnKbdDev, 0, 1, KBDIF_DEV_NAME);
	if(!nResult)
	{
		nMajor = MAJOR(gnKbdDev);
		nMinor = MINOR(gnKbdDev);
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]alloc_chrdev_region %d:%d\n", nMajor, nMinor);
#endif	/* KBD_DEBPRN */
	}
	else
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]alloc_chrdev_region error\n");
#endif	/* KBD_DEBPRN */
		return -1;
	}

	cdev_init(&goKbdCDev, &goKbdIf_Fops);

	goKbdCDev.owner = THIS_MODULE;
	goKbdCDev.ops = &goKbdIf_Fops;

	nResult = cdev_add(&goKbdCDev, gnKbdDev, 1);
	if(nResult)
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]cdev_add error\n");
#endif	/* KBD_DEBPRN */
		cdev_del(&goKbdCDev);
		return -1;
	}

	gpoKbdClass = class_create(THIS_MODULE, KBDIF_DEV_NAME);
	if(IS_ERR(gpoKbdClass))
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]class_create error\n");
#endif	/* KBD_DEBPRN */
		cdev_del(&goKbdCDev);
		return -1;
	}
	device_create(gpoKbdClass, NULL, gnKbdDev, &goKbdCDev, KBDIF_DEV_NAME);
	return 0;
}

void __exit KbdIf_Cleanup(void)
{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKbdIF]Cleanup(PID:%ld)\n", sys_getpid());
#endif	/* KBD_DEBPRN */
	device_destroy(gpoKbdClass, gnKbdDev);
	class_destroy(gpoKbdClass);
	cdev_del(&goKbdCDev);
}

module_init(KbdIf_Setup);
module_exit(KbdIf_Cleanup);

/*+-------------------------------------------------------------------------+*/
/*|	キーボードドライバ														|*/
/*+-------------------------------------------------------------------------+*/
module_init(ShKey_Init);
module_exit(ShKey_Exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("I2C QWERTY keyboard driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:SH_qwerty_key");

static const I2cDevID gI2cDevIdTable[] =
{
   { SH_KBD_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTable);

/* I2Cドライバ呼び出し用構造体 */
static struct i2c_driver goI2cKbdDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_KBD_I2C_DEVNAME,
	},
	.probe	  = ShKey_Probe,
	.remove	  = __devexit_p(ShKey_Remove),
	.suspend  = ShKey_Suspend,
	.resume   = ShKey_Resume,
	.id_table = gI2cDevIdTable,
};

static int __init ShKey_Init(void)
{
printk(KERN_DEBUG "[ShKey]Init(PID:%ld)\n", sys_getpid());
	/* I2Cドライバ利用開始 */
	return i2c_add_driver(&goI2cKbdDriver);
}

static void __exit ShKey_Exit(void)
{
printk(KERN_DEBUG "[ShKey]Exit(PID:%ld)\n", sys_getpid());
	/* I2Cドライバ利用終了 */
	i2c_del_driver(&goI2cKbdDriver);
}

/*+-------------------------------------------------------------------------+*/
/*|	キードライバ起動														|*/
/*+-------------------------------------------------------------------------+*/
static int __devinit ShKey_Probe(I2cClient *poClient, const I2cDevID *poDevId)
{
	struct msm_sh_i2ckbd_platform_data *poSetupData;
	I2cKbdRec *poKbdRec = NULL;
	int nResult;

printk(KERN_DEBUG "[ShKey]Probe(PID:%ld)\n", sys_getpid());
	if(!poClient->dev.platform_data)
	{
		dev_err(&poClient->dev, "keyboard platform device data is required\n");
		return -ENODEV;
	}
	/* キードライバ情報用メモリを確保する */
	poKbdRec = kzalloc(sizeof(I2cKbdRec), GFP_KERNEL);
	if(!poKbdRec)
	{
		return -ENOMEM;
	}
	poClient->driver = &goI2cKbdDriver;
	i2c_set_clientdata(poClient, poKbdRec);
	poKbdRec->mpoClient			 = poClient;
	poSetupData					 = poClient->dev.platform_data;
	/* セットアップ情報を得る(board-xxxx.cで定義している) */
	poKbdRec->mnIrqPin			 = poSetupData->gpio_kbdirq;
	poKbdRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poKbdRec->mpfPinShutdownFunc = poSetupData->gpio_shutdown;
	poKbdRec->mnJogIrqPin[0]	 = poSetupData->gpio_jogirq[0];
	poKbdRec->mnJogIrqPin[1]	 = poSetupData->gpio_jogirq[1];
	poKbdRec->mnJogIrqPin[2]	 = poSetupData->gpio_jogirq[2];
	poKbdRec->mnJogIrqPin[3]	 = poSetupData->gpio_jogirq[3];
	poKbdRec->mnJogPowPin		 = poSetupData->gpio_jogpower;
#ifdef JOG_TIMER_TEST
	poKbdRec->mdwPrev			 = gettimeofday_msec();
	poKbdRec->mdwInterval		 = JOG_VOID_TIME;
#endif /* JOG_TIMER_TEST */
	/* グローバル変数に登録しておく */
	gpoKbdRec = poKbdRec;
	poKbdRec->mbAccessState		 = 0;
	poKbdRec->mbKbdSleepState	 = 0;
	poKbdRec->mbJogSleepState	 = 0;
	/* GPIO設定を行う */
	if(0 == (nResult = ShKey_ConfigGPIO(poKbdRec)))
	{
		INIT_WORK(&poKbdRec->moIrqWork, ShKey_FetchKeys);
		INIT_WORK(&poKbdRec->moJogIrqWork, ShKey_FetchJog);
		dev_info(&poClient->dev, "Detected %s, attempting to initialize keyboard\n", SH_KBD_I2C_DEVNAME);
		snprintf(poKbdRec->mcPhysInfo, QPHYSLEN, "%s/%s/event0",
				 poClient->adapter->dev.bus_id, poClient->dev.bus_id);
		INIT_DELAYED_WORK(&poKbdRec->moCmdQ, ShKey_Connect2InputSys);
		schedule_delayed_work(&poKbdRec->moCmdQ, msecs_to_jiffies(INITDELAY_TIMER));
		device_init_wakeup(&poClient->dev, 1);
		return 0;
	}
	/* GPIOの解放 */
	ShKey_ReleaseGPIO(poKbdRec);
	kfree(poKbdRec);
	return nResult;
}

/*+-------------------------------------------------------------------------+*/
/*|	キードライバ切り離し													|*/
/*+-------------------------------------------------------------------------+*/
static int __devexit ShKey_Remove(I2cClient *poClient)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[ShKey]Remove(PID:%ld)\n", sys_getpid());
	dev_info(&poClient->dev, "removing keyboard driver\n");
	device_init_wakeup(&poClient->dev, 0);
	if(poKbdRec->mpoInDev)
	{
		dev_dbg(&poClient->dev, "deregister from input system\n");
		input_unregister_device(poKbdRec->mpoInDev);
		poKbdRec->mpoInDev = NULL;
	}
	gpoKbdRec = NULL;
	ShKey_Shutdown(poKbdRec);
	ShKey_ReleaseGPIO(poKbdRec);
	kfree(poKbdRec);
	return 0;
}

#ifdef CONFIG_PM
/*+-------------------------------------------------------------------------+*/
/*|	キードライバサスペンド													|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_Suspend(I2cClient *poClient, pm_message_t oMsg)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[ShKey]Suspend(PID:%ld)\n", sys_getpid());
	if(device_may_wakeup(&poClient->dev))
	{
		enable_irq_wake(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
	return 0;
}

/*+-------------------------------------------------------------------------+*/
/*|	キードライバレジューム													|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_Resume(I2cClient *poClient)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);

printk(KERN_DEBUG "[ShKey]Resume(PID:%ld)\n", sys_getpid());
	if(device_may_wakeup(&poClient->dev))
	{
		disable_irq_wake(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
	return 0;
}
#endif	/* CONFIG_PM */

static int ShKey_Command(I2cClient *poClient, unsigned int wCmd, void *pArg)
{
	I2cKbdRec *poKbdRec = i2c_get_clientdata(poClient);
	int nResult;

#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]Command(PID:%ld,CMD:%d,ARG:%lx)\n", sys_getpid(), wCmd, (long)pArg);
#endif	/* KBD_DEBPRN */
	/* デバイスがオープンされていないなら */
	if(poKbdRec->mbIsActive == 0)
		return -2;
	switch(wCmd)
	{
	case KBDDEV_SLEEP_ON:
		return ShKey_SetState(poKbdRec, KBD_DISABLE_SLEEP, KBD_DISABLE_ON, 0, 0);
	case KBDDEV_SLEEP_OFF:
		return ShKey_SetState(poKbdRec, KBD_DISABLE_SLEEP, KBD_DISABLE_OFF, 0, 0);
	case KBDDEV_ENABLE:
		return ShKey_SetState(poKbdRec, KBD_DISABLE_API, KBD_DISABLE_OFF, 1, 0);
	case KBDDEV_DISABLE:
		return ShKey_SetState(poKbdRec, KBD_DISABLE_API, KBD_DISABLE_ON, 1, 0);
	default:
		nResult = -2;
		break;
	}
	return nResult;
}

static void ShKey_Connect2InputSys(WorkStruct *poWork)
{
	I2cKbdRec *poKbdRec = container_of(poWork, I2cKbdRec, moCmdQ.work);
	Device *poDev = &poKbdRec->mpoClient->dev;

printk(KERN_DEBUG "[ShKey]Connect2InputSys(PID:%ld)\n", sys_getpid());
	poKbdRec->mpoInDev = ShKey_CreateInputDev(poKbdRec);
	if(poKbdRec->mpoInDev)
	{
		if(input_register_device(poKbdRec->mpoInDev) != 0)
		{
			dev_err(poDev, "Failed to register with input system\n");
			input_free_device(poKbdRec->mpoInDev);
		}
	}
}

static InputDev *ShKey_CreateInputDev(I2cKbdRec *poKbdRec)
{
	Device *poDev = &poKbdRec->mpoClient->dev;
	InputDev *pInDev = input_allocate_device();
	int nI;

printk(KERN_DEBUG "[ShKey]CreateInputDev(PID:%ld)\n", sys_getpid());
	if(pInDev)
	{
		pInDev->name = SH_KBD_I2C_DEVNAME;
		pInDev->phys = poKbdRec->mcPhysInfo;
		pInDev->id.bustype = BUS_I2C;
		pInDev->id.vendor  = QCVENDOR_ID;
		pInDev->id.product = QCPRODUCT_ID;
		pInDev->id.version = QCVERSION_ID;
		pInDev->open = ShKey_OpenCB;
		pInDev->close = ShKey_CloseCB;
		__set_bit(EV_KEY, pInDev->evbit);
		for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
		{
			poKbdRec->mwPrev[nI] = KEY_RESERVED;
		}
		for(nI = 0; nI < KPD_KEY_MAX; nI++)
		{
			if(gwShKeyTable[nI] != 0)
			{
				__set_bit(gwShKeyTable[nI], pInDev->keybit);
			}
		}
		__set_bit(KEY_END, pInDev->keybit);
		__set_bit(KEY_POWER, pInDev->keybit);
		__set_bit(EV_REL, pInDev->evbit);
		__set_bit(REL_X, pInDev->relbit);
		__set_bit(REL_Y, pInDev->relbit);
		input_set_drvdata(pInDev, poKbdRec);
	}
	else
	{
		dev_err(poDev, "Failed to allocate input device for %s\n", SH_KBD_I2C_DEVNAME);
	}
	return pInDev;
}

/*+-------------------------------------------------------------------------+*/
/*|	GPIOの設定																|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_ConfigGPIO(I2cKbdRec *poKbdRec)
{
	if(poKbdRec == NULL)
		return -EINVAL;
	return poKbdRec->mpfPinSetupFunc();
}

/*+-------------------------------------------------------------------------+*/
/*|	GPIOの解放																|*/
/*+-------------------------------------------------------------------------+*/
static int ShKey_ReleaseGPIO(I2cKbdRec *poKbdRec)
{
	if(poKbdRec == NULL)
		return -EINVAL;
	/* GPIOの解放 */
	dev_info(&poKbdRec->mpoClient->dev, "releasing keyboard gpio pins %d\n", poKbdRec->mnIrqPin);
	poKbdRec->mpfPinShutdownFunc();
	return 0;
}

static int ShKey_OpenCB(InputDev *poInDev)
{
	I2cKbdRec *poKbdRec = input_get_drvdata(poInDev);

printk(KERN_DEBUG "[ShKey]OpenCB(PID:%ld)\n", sys_getpid());
	dev_dbg(&poKbdRec->mpoClient->dev, "ENTRY: input_dev open callback\n");
	return ShKey_Start(poKbdRec);
}

static void ShKey_CloseCB(InputDev *pInDev)
{
	I2cKbdRec *poKbdRec = input_get_drvdata(pInDev);
	Device *poDev = &poKbdRec->mpoClient->dev;

printk(KERN_DEBUG "[ShKey]CloseCB(PID:%ld)\n", sys_getpid());
	dev_dbg(poDev, "ENTRY: close callback\n");
	ShKey_Shutdown(poKbdRec);
}

static void ShKey_Shutdown(I2cKbdRec *poKbdRec)
{
printk(KERN_DEBUG "[ShKey]Shutdown(PID:%ld)\n", sys_getpid());
	/* 動作中なら */
	if(poKbdRec->mbIsActive)
	{
		ShKey_Stop(poKbdRec);
		/* ワークメモリ解放 */
		flush_work(&poKbdRec->moIrqWork);
		flush_work(&poKbdRec->moJogIrqWork);
	}
}

static int ShKey_Start(I2cKbdRec *poKbdRec)
{
	I2cClient *poClient = poKbdRec->mpoClient;
	int nResult = 0;
	int nI;
	int nTrigger;

printk(KERN_DEBUG "[ShKey]Start(PID:%ld)\n", sys_getpid());
	mutex_lock(&goKbdAccessMutex);
	/* 停止中にする */
	poKbdRec->mbIsActive = 0;
	nResult = ShKey_I2cWriteAny(poClient, goInitData);
	if(nResult < 0)
	{
printk(KERN_DEBUG "[ShKey]Start-->Error\n");
		mutex_unlock(&goKbdAccessMutex);
		return -EIO;
	}
	/* 割込みハンドラの登録 */
	nResult = request_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin), &ShKey_IrqHandler,
						 IRQF_TRIGGER_LOW | IRQF_DISABLED,
					     SH_KBD_I2C_DEVNAME, poKbdRec);
	if(nResult < 0)
	{
		printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_KBD_I2C_DEVNAME, nResult);
		mutex_unlock(&goKbdAccessMutex);
		return -EIO;
	}
	for(nI = 0; nI < 4; nI++)
	{
		/* 現在値を保存 */
		poKbdRec->mnJogVal[nI] = gpio_get_value(poKbdRec->mnJogIrqPin[nI]);
		nTrigger = (poKbdRec->mnJogVal[nI] == 0 ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW) | IRQF_DISABLED;
		/* 割込み設定 */
		nResult = request_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]),
						 &ShKey_JogHandler, nTrigger, SH_KBD_I2C_DEVNAME, poKbdRec);
		if(nResult < 0)
		{
			printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_KBD_I2C_DEVNAME, nResult);
			ShKey_Stop(poKbdRec);
			mutex_unlock(&goKbdAccessMutex);
			return -EIO;
		}
	}
	/* 動作中にする */
	poKbdRec->mbIsActive = 1;
	ShKey_SetState(poKbdRec, KBD_DISABLE_API, KBD_DISABLE_OFF, 0, 1);
	mutex_unlock(&goKbdAccessMutex);
	return 0;
}

static void ShKey_Stop(I2cKbdRec *poKbdRec)
{
printk(KERN_DEBUG "[ShKey]Stop(PID:%ld)\n", sys_getpid());
	mutex_lock(&goKbdAccessMutex);
	/* 停止中にする */
	poKbdRec->mbIsActive = 0;
	/* 割込み登録解除 */
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin), poKbdRec);
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[0]), poKbdRec);
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[1]), poKbdRec);
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[2]), poKbdRec);
	free_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[3]), poKbdRec);
	mutex_unlock(&goKbdAccessMutex);
}

/*+-------------------------------------------------------------------------+*/
/*|	IRQハンドラ																|*/
/*+-------------------------------------------------------------------------+*/
static irqreturn_t ShKey_IrqHandler(int nIrq, void *pvDevId)
{
	I2cKbdRec *poKbdRec = pvDevId;

//printk(KERN_DEBUG "[ShKey]IrqHandler(PID:%ld,IRQ:%02X)\n", sys_getpid(), nIrq);
	disable_irq_nosync(nIrq);
	schedule_work(&poKbdRec->moIrqWork);
	return IRQ_HANDLED;
}

static irqreturn_t ShKey_JogHandler(int nIrq, void *pvDevId)
{
	I2cKbdRec *poKbdRec = pvDevId;
	int nI;

//printk(KERN_DEBUG "[ShKey]JogHandler(PID:%ld,IRQ:%02X)\n", sys_getpid(), nIrq);
	for(nI = 0; nI < 4; nI++)
	{
		/* ４方向とも割込み禁止 */
		disable_irq_nosync(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]));
	}
	/* 割込みハンドラ実行 */
	schedule_work(&poKbdRec->moJogIrqWork);
	return IRQ_HANDLED;
}
static void ShKey_EnableJogInt(I2cKbdRec *poKbdRec)
{
	int nTrigger;
	int nI;

	for(nI = 0; nI < 4; nI++)
	{
		/* 現在値を保存 */
		poKbdRec->mnJogVal[nI] = gpio_get_value(poKbdRec->mnJogIrqPin[nI]);
		nTrigger = (poKbdRec->mnJogVal[nI] == 0 ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW) | IRQF_DISABLED;
		/* 割込みトリガの設定変更 */
		set_irq_type(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]), nTrigger);
		/* 割込み許可 */
		enable_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]));
	}
}

static void ShKey_Recover(WorkStruct *poWork)
{
	int nResult;
	I2cKbdRec *poKbdRec;

printk(KERN_DEBUG "[ShKey]Recover(PID:%ld)\n", sys_getpid());
	poKbdRec = container_of(poWork, I2cKbdRec, moCmdQ.work);

	dev_info(&poKbdRec->mpoClient->dev, "keyboard recovery requested\n");

	nResult = ShKey_Start(poKbdRec);
	if(nResult != 0)
	{
		dev_err(&poKbdRec->mpoClient->dev, "recovery failed with (nResult=%d)\n", nResult);
	}
}

static void ShKey_NotifyKeyCode(I2cKbdRec *poKbdRec, InputDev *pInDev, uint8_t *pbCode)
{
	long lCode;
	int nI;
	int nJ;
	int nCnt;
	int wCode[KPD_KEYBUFF_SIZE];

	/* 停止中なら何もしない */
	if(!poKbdRec->mbIsActive)
	{
		return;
	}
	nCnt = 0;
	/* キーバッファ分ループ */
	for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
	{
		/* キーが押されているなら */
		if(pbCode[nI] != KPD_INVALID_KEY)
		{
			wCode[nI] = gwShKeyTable[pbCode[nI] & 0x7f];
			/* 前回から押したままかをチェック */
			for(nJ = 0; nJ < KPD_KEYBUFF_SIZE; nJ++)
			{
				if(poKbdRec->mwPrev[nJ] == wCode[nI])
				{
					break;
				}
			}
			/* 新しく押下した */
			if(nJ == KPD_KEYBUFF_SIZE)
			{
				lCode = (long)wCode[nI];
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]Key Press  <%3ld>\n", lCode);
#endif	/* KBD_DEBPRN */
				input_report_key(pInDev, lCode, KPD_KEYPRESS);
				nCnt++;
			}
			else
			{
				/* 押し続けているキーはバッファから消去 */
				poKbdRec->mwPrev[nJ] = KEY_RESERVED;
			}
		}
		else
		{
			wCode[nI] = KEY_RESERVED;
		}
	}
	/* キーバッファ分ループ */
	for(nJ = 0; nJ < KPD_KEYBUFF_SIZE; nJ++)
	{
		/* 前回通知時にキー押しで今回キー押し判定ではない場合は離したと判断する */
		if(poKbdRec->mwPrev[nJ] != KEY_RESERVED)
		{
			lCode = (long)poKbdRec->mwPrev[nJ];
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]Key Release<%3ld>\n", lCode);
#endif	/* KBD_DEBPRN */
			input_report_key(pInDev, lCode, KPD_KEYRELEASE);
			nCnt++;
		}
	}
	if(nCnt != 0)
	{
		input_sync(pInDev);
	}
	/* キーバッファ分ループ */
	for(nI = 0; nI < KPD_KEYBUFF_SIZE; nI++)
	{
		/* 今回押下キーを記録 */
		poKbdRec->mwPrev[nI] = wCode[nI];
	}
}

static void ShKey_FetchKeys(WorkStruct *poWork)
{
	I2cKbdRec *poKbdRec = container_of(poWork, I2cKbdRec, moIrqWork);

//printk(KERN_DEBUG "[ShKey]FetchKeys(PID:%ld)\n", sys_getpid());
	mutex_lock(&goKbdAccessMutex);
	if(ShKey_FetchKeysMain(poKbdRec) != 0)
	{
		dev_err(&poKbdRec->mpoClient->dev, "Failed read from keyboard \n");
		ShKey_Stop(poKbdRec);
		INIT_DELAYED_WORK(&poKbdRec->moCmdQ, ShKey_Recover);
		schedule_delayed_work(&poKbdRec->moCmdQ, msecs_to_jiffies(RECOVER_TIMER));
	}
	else
	{
		/* 次の割り込み許可 */
		enable_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
	}
	mutex_unlock(&goKbdAccessMutex);
}

static int ShKey_FetchKeysMain(I2cKbdRec *poKbdRec)
{
	InputDev *pInDev = poKbdRec->mpoInDev;
	I2cClient *poClient = poKbdRec->mpoClient;
	uint8_t bIRQST;
	uint8_t bKBDMIS;
	uint8_t bKey[KPD_KEYBUFF_SIZE];
	uint8_t bEvt = 0x00;

	/* ステータスレジスタの読み込み */
	if(0 != ShKey_I2cRead(poClient, KPD_IRQST, &bIRQST, 1))
	{
printk(KERN_DEBUG "[ShKey]i2c read error\n");
		return -1;
	}
	if(bIRQST & KPD_IRQST_PORIRQ)
	{
//printk(KERN_DEBUG "[ShKey]IRQST %02X\n", bIRQST);
		/* パワーオンリセットを実行 */
		ShKey_I2cWriteOne(poClient, KPD_RSTINTCLR, KPD_RSTINTCLR_IRQCLR);
printk(KERN_DEBUG "[ShKey]KPD_RSTINTCLR(%02X)<<%02X\n", KPD_RSTINTCLR, KPD_RSTINTCLR_IRQCLR);
	}
	else if(bIRQST & KPD_IRQST_KBDIRQ)
	{
		/* クロック停止中なら以下のレジスタにアクセスできないのでここで止める */
		if(poKbdRec->mbKbdSleepState != 0)
		{
			return 0;
		}
		/* ステータスレジスタを読み込む */
		if(0 != ShKey_I2cRead(poClient, KPD_KBDMIS, &bKBDMIS, 1))
		{
printk(KERN_DEBUG "[ShKey]i2c read error\n");
			return -1;
		}
		if(bKBDMIS & KPD_KBDMIS_MELINT)
		{
printk(KERN_DEBUG "[ShKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
			ShKey_I2cWriteOne(poClient, KPD_KBDIC, KPD_KBDIC_KBDIC);
		}
		if(bKBDMIS & KPD_KBDMIS_MEVINT)
		{
			if(0 != ShKey_I2cRead(poClient, KPD_EVTCODE, &bEvt, 1))
			{
printk(KERN_DEBUG "[ShKey]i2c read error\n");
				return -1;
			}
printk(KERN_DEBUG "[ShKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
		}
		if(bKBDMIS & KPD_KBDMIS_MKLINT)
		{
printk(KERN_DEBUG "[ShKey]IRQST %02X KBDMIS %02X\n", bIRQST, bKBDMIS);
			ShKey_I2cWriteOne(poClient, KPD_KBDIC, KPD_KBDIC_EVTIC);
		}
		if(bKBDMIS & KPD_KBDMIS_MSINT)
		{
			/* キーコードレジスタを読み込む */
			if(0 != ShKey_I2cRead(poClient, KPD_KEYCODE0, bKey, KPD_KEYBUFF_SIZE))
			{
printk(KERN_DEBUG "[ShKey]i2c read error\n");
				return -1;
			}
			/* 通知する */
			ShKey_NotifyKeyCode(poKbdRec, pInDev, bKey);
		}
	}
	else
	{
printk(KERN_DEBUG "[ShKey]Other IRQ(%02X)\n", bIRQST);
		return -1;
	}
	return 0;
}

static void ShKey_FetchJog(WorkStruct *poWork)
{
#ifdef ROTATE_90
								/*		上			下			左			右	*/
	static const int nMoveX[4] = {	 REL_STEP,	-REL_STEP,			0,	 		0};
	static const int nMoveY[4] = {			0,	 		0,	-REL_STEP,	 REL_STEP};
#else
								/*		上			下			左			右	*/
	static const int nMoveX[4] = {			0,			0,	-REL_STEP,	 REL_STEP};
	static const int nMoveY[4] = {	-REL_STEP,	 REL_STEP,			0,			0};
#endif	/* ROTATE_90 */
	I2cKbdRec *poKbdRec = container_of(poWork, I2cKbdRec, moJogIrqWork);
	InputDev *pInDev = poKbdRec->mpoInDev;
	int nI;
	int nX;
	int nY;
	int nVal;
#ifdef JOG_TIMER_TEST
	unsigned long dwNow;
#endif /* JOG_TIMER_TEST */

	mutex_lock(&goKbdAccessMutex);
#ifdef JOG_TIMER_TEST
	dwNow = gettimeofday_msec();
#endif /* JOG_TIMER_TEST */
	nX = nY = 0;
	for(nI = 0; nI < 4; nI++)
	{
		/* 現在値を読み込む */
		nVal = gpio_get_value(poKbdRec->mnJogIrqPin[nI]);
		/* 前回値と異なるなら */
		if(nVal != poKbdRec->mnJogVal[nI])
		{
			nX += nMoveX[nI];
			nY += nMoveY[nI];
		}
	}
#ifdef JOG_TIMER_TEST
	/* 前回イベントからの時刻が一定以上なら無視する */
	if((dwNow - poKbdRec->mdwPrev) > poKbdRec->mdwInterval)
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]Event lost(%lu>%lu)\n", (dwNow - poKbdRec->mdwPrev), poKbdRec->mdwInterval);
#endif	/* KBD_DEBPRN */
	}
	else
	{
#endif /* JOG_TIMER_TEST */
	if(nX != 0)
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]REL_X<%d>\n", nX);
#endif	/* KBD_DEBPRN */
		input_report_rel(pInDev, REL_X, nX);
	}
	if(nY != 0)
	{
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]REL_Y<%d>\n", nY);
#endif	/* KBD_DEBPRN */
		input_report_rel(pInDev, REL_Y, nY);
	}
	if(nX != 0 || nY != 0)
	{
		input_sync(pInDev);
	}
#ifdef JOG_TIMER_TEST
	}
	poKbdRec->mdwPrev = dwNow;
#endif /* JOG_TIMER_TEST */
	/* 次の割り込み許可 */
	ShKey_EnableJogInt(poKbdRec);
	mutex_unlock(&goKbdAccessMutex);
}

static int ShKey_SetState(I2cKbdRec *poKbdRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, int nIsFirst)
{
	uint8_t bNew;
	int nResult = 0;

	bValue &= bMask;
	bNew = (poKbdRec->mbAccessState & ~bMask) | bValue;
#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]SetState[%02X]->[%02X]\n", poKbdRec->mbAccessState, bNew);
#endif	/* KBD_DEBPRN */
	/* ２重チェックありなら */
	if(bCheck)
	{
		if((poKbdRec->mbAccessState & bMask) == bValue)
		{
printk(KERN_DEBUG "[ShKey]StateCheck NG\n");
			return -2;
		}
	}
	/* 状態を記録する */
	poKbdRec->mbAccessState = bNew;
	if(poKbdRec->mbIsActive)
	{
		/* Enable+フリップ開きなら */
		if((poKbdRec->mbAccessState & (KBD_DISABLE_API | KBD_DISABLE_FLIP)) == 0)
		{
			nResult = ShKey_SetKbdSleep(poKbdRec, 0, nIsFirst);
		}
		else
		{
			nResult = ShKey_SetKbdSleep(poKbdRec, 1, nIsFirst);
		}
		if(nResult == 0)
		{
			/* Enable+Sleep非指定なら */
			if((poKbdRec->mbAccessState & (KBD_DISABLE_API | KBD_DISABLE_SLEEP)) == 0)
			{
				nResult = ShKey_SetJogSleep(poKbdRec, 0, nIsFirst);
			}
			else
			{
				nResult = ShKey_SetJogSleep(poKbdRec, 1, nIsFirst);
			}
		}
	}
	return nResult;
}


static int ShKey_SetKbdSleep(I2cKbdRec *poKbdRec, int nIsSleep, int nIsFirst)
{
	I2cClient *poClient = poKbdRec->mpoClient;
	int nResult = 0;

#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]SetKbdSleep<%d>\n", nIsSleep);
#endif	/* KBD_DEBPRN */
	/* 初回設定なら */
	if(nIsFirst)
	{
		/* スリープなら */
		if(nIsSleep == 1)
		{
			/* 割り込み禁止 */
			disable_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
			/* スリープ設定 */
			nResult = ShKey_I2cWriteAny(poClient, goSleepData);
		}
		/* 起動なら(割込み設定は不要) */
		else
		{
			/* 起動設定 */
			nResult = ShKey_I2cWriteAny(poClient, goWakeupData);
		}
	}
	else
	{
		/* 現在の状態と異なるなら */
		if(poKbdRec->mbKbdSleepState != (uint8_t)nIsSleep)
		{
			/* スリープなら */
			if(nIsSleep == 1)
			{
				/* 割り込み禁止 */
				disable_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
				/* スリープ設定 */
				nResult = ShKey_I2cWriteAny(poClient, goSleepData);
			}
			/* 起動なら */
			else
			{
				/* 起動設定 */
				nResult = ShKey_I2cWriteAny(poClient, goWakeupData);
				/* 割り込み解除 */
				enable_irq(MSM_GPIO_TO_INT(poKbdRec->mnIrqPin));
			}
		}
	}
	poKbdRec->mbKbdSleepState = (uint8_t)nIsSleep;
	return nResult;
}

static int ShKey_SetJogSleep(I2cKbdRec *poKbdRec, int nIsSleep, int nIsFirst)
{
	int nResult = 0;
	int nI;

#ifdef KBD_DEBPRN
printk(KERN_DEBUG "[ShKey]SetJogSleep<%d>\n", nIsSleep);
#endif	/* KBD_DEBPRN */
	/* 初回設定なら */
	if(nIsFirst)
	{
		/* スリープなら */
		if(nIsSleep == 1)
		{
			for(nI = 0; nI < 4; nI++)
			{
				/* ４方向とも割込み禁止 */
				disable_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]));
				gpio_tlmm_config(GPIO_CFG(poKbdRec->mnJogIrqPin[nI], 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
			}
			/* ホールIC電源オフ */
			gpio_direction_output(poKbdRec->mnJogPowPin, 1);
		}
		/* 起動なら(割込み設定は不要) */
		else
		{
			/* ホールIC電源オン */
			gpio_direction_output(poKbdRec->mnJogPowPin, 0);
			for(nI = 0; nI < 4; nI++)
			{
				gpio_tlmm_config(GPIO_CFG(poKbdRec->mnJogIrqPin[nI], 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
			}
		}
	}
	else
	{
		/* 現在の状態と異なるなら */
		if(poKbdRec->mbJogSleepState != (uint8_t)nIsSleep)
		{
			/* スリープなら */
			if(nIsSleep == 1)
			{
				for(nI = 0; nI < 4; nI++)
				{
					/* ４方向とも割込み禁止 */
					disable_irq(MSM_GPIO_TO_INT(poKbdRec->mnJogIrqPin[nI]));
					gpio_tlmm_config(GPIO_CFG(poKbdRec->mnJogIrqPin[nI], 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
				}
				/* ホールIC電源オフ */
				gpio_direction_output(poKbdRec->mnJogPowPin, 1);
			}
			/* 起動なら */
			else
			{
				/* ホールIC電源オン */
				gpio_direction_output(poKbdRec->mnJogPowPin, 0);
				for(nI = 0; nI < 4; nI++)
				{
					gpio_tlmm_config(GPIO_CFG(poKbdRec->mnJogIrqPin[nI], 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
				}
				/* 割り込み解除 */
				ShKey_EnableJogInt(poKbdRec);
			}
		}
	}
	poKbdRec->mbJogSleepState = (uint8_t)nIsSleep;
	return nResult;
}

/*+-------------------------------------------------------------------------+*/
/*|	外部公開I/F																|*/
/*+-------------------------------------------------------------------------+*/
void msm_i2ckbd_flipchange(int nFlipState)
{
	mutex_lock(&goKbdAccessMutex);
	if(gpoKbdRec != NULL)
	{
		/* フリップオープンなら */
		if(nFlipState == 0x00)
			ShKey_SetState(gpoKbdRec, KBD_DISABLE_FLIP, KBD_DISABLE_OFF, 0, 0);
		else
			ShKey_SetState(gpoKbdRec, KBD_DISABLE_FLIP, KBD_DISABLE_ON, 0, 0);
	}
	mutex_unlock(&goKbdAccessMutex);
}

void msm_i2ckbd_setsleep(int nIsSleep)
{
	mutex_lock(&goKbdAccessMutex);
	if(gpoKbdRec != NULL)
	{
		/* スリープ解除なら */
		if(nIsSleep == 0x00)
			ShKey_SetState(gpoKbdRec, KBD_DISABLE_SLEEP, KBD_DISABLE_OFF, 0, 0);
		else
			ShKey_SetState(gpoKbdRec, KBD_DISABLE_SLEEP, KBD_DISABLE_ON, 0, 0);
	}
	mutex_unlock(&goKbdAccessMutex);
}

#ifdef JOG_TIMER_TEST
static unsigned long gettimeofday_msec(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}
#endif /* JOG_TIMER_TEST */
