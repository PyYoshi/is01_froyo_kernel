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
 */

/*
 * SHARP DISPLAY DRIVER FOR KERNEL
 */

#ifndef SHLCDC_KERN_H
#define SHLCDC_KERN_H

/*
 * TYPES
 */

enum {
    SHLCDC_RESULT_SUCCESS,
    SHLCDC_RESULT_FAILURE,
    SHLCDC_RESULT_FAILURE_I2C_TMO,
    NUM_SHLCDC_RESULT
};

enum {
    SHLCDC_EVENT_TYPE_IRRC,
    SHLCDC_EVENT_TYPE_SDHC,
    SHLCDC_EVENT_TYPE_GPIO,
    SHLCDC_EVENT_TYPE_PVSYNC,
    SHLCDC_EVENT_TYPE_I2C,
    SHLCDC_EVENT_TYPE_CAMVIEW,
    SHLCDC_EVENT_TYPE_SDDET_H,
    SHLCDC_EVENT_TYPE_SDDET_L,
    SHLCDC_EVENT_TYPE_CSTM,
    SHLCDC_EVENT_TYPE_CSI,
    NUM_SHLCDC_EVENT_TYPE
};

enum {
    SHLCDC_DEV_TYPE_HANDSET,
    SHLCDC_DEV_TYPE_DISP,
    SHLCDC_DEV_TYPE_MDDI,
    SHLCDC_DEV_TYPE_LED,
    SHLCDC_DEV_TYPE_CAM,
    SHLCDC_DEV_TYPE_SUBCAM,
    SHLCDC_DEV_TYPE_SD,
    SHLCDC_DEV_TYPE_IR,
    SHLCDC_DEV_TYPE_TP,
    SHLCDC_DEV_TYPE_DIAG,
    NUM_SHLCDC_DEV_TYPE
};

enum {
    SHLCDC_DEV_PWR_OFF,
    SHLCDC_DEV_PWR_ON,
    NUM_SHLCDC_DEV_PWR
};

enum {
    SHLCDC_IR_IRSD_MODE_LO,
    SHLCDC_IR_IRSD_MODE_HI,
    SHLCDC_IR_IRSD_MODE_IRDACC_CTL,
    NUM_SHLCDC_IR_IRSD_MODE
};

enum {
    SHLCDC_IR_IRSEL_MODE_LO,
    SHLCDC_IR_IRSEL_MODE_HI,
    SHLCDC_IR_IRSEL_MODE_IRDACC_CTL,
    NUM_SHLCDC_IR_IRSEL_MODE
};

enum {
    SHLCDC_TP_PSOC_STBY_LO,
    SHLCDC_TP_PSOC_STBY_HI,
    NUM_SHLCDC_TP_PSOC_STBY
};

enum {
    SHLCDC_TP_PSOC_RESET_LO,
    SHLCDC_TP_PSOC_RESET_HI,
    NUM_SHLCDC_TP_PSOC_RESET
};

struct shlcdc_subscribe {
    int event_type;
    void (*callback)(void);
};

int shlcdc_api_set_power_mode(int dev_type, int dev_pwr_req);
int shlcdc_api_event_subscribe(struct shlcdc_subscribe *subscribe);
int shlcdc_api_event_unsubscribe(int event_type);
int shlcdc_api_ir_set_irsd_mode(int irsd_mode);
int shlcdc_api_ir_set_irsel_mode(int irsel_mode);
int shlcdc_api_tp_set_psoc_stby_mode(int stby_mode);
int shlcdc_api_tp_set_psoc_reset_mode(int reset_mode);

#endif /* SHLCDC_KERN_H */
