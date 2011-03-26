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

#ifndef SHDISP_KERN_H
#define SHDISP_KERN_H

/*
 * TYPES
 */

enum {
    SHDISP_RESULT_SUCCESS,
    SHDISP_RESULT_FAILURE,
    SHDISP_RESULT_FAILURE_I2C_TMO,
    SHDISP_RESULT_FAILURE_VEILVIEW_UNINIT,
    SHDISP_RESULT_FAILURE_VEILVIEW_REQ_DUP,
    SHDISP_RESULT_FAILURE_VEILVIEW_REQ_INVALID,
    NUM_SHDISP_RESULT
};

enum {
    SHDISP_MAIN_BKL_MODE_OFF,
    SHDISP_MAIN_BKL_MODE_FIX,
    SHDISP_MAIN_BKL_MODE_AUTO,
    SHDISP_MAIN_BKL_MODE_DTV_OFF,
    SHDISP_MAIN_BKL_MODE_DTV_FIX,
    SHDISP_MAIN_BKL_MODE_DTV_AUTO,
    NUM_SHDISP_MAIN_BKL_MODE
};

enum {
    SHDISP_MAIN_BKL_PARAM_0,
    SHDISP_MAIN_BKL_PARAM_1,
    SHDISP_MAIN_BKL_PARAM_2,
    SHDISP_MAIN_BKL_PARAM_3,
    SHDISP_MAIN_BKL_PARAM_4,
    SHDISP_MAIN_BKL_PARAM_5,
    SHDISP_MAIN_BKL_PARAM_6,
    NUM_SHDISP_MAIN_BKL_PARAM
};

struct shdisp_tri_led {
    unsigned long red;
    unsigned long green;
    unsigned long blue;
    int ext_mode;
};

struct shdisp_tri_led_ex {
    unsigned long red1;
    unsigned long green1;
    unsigned long blue1;
    int led_mode1;
    int period1;
    int ontime1;
    unsigned long red2;
    unsigned long green2;
    unsigned long blue2;
    int led_mode2;
    int period2;
    int ontime2;
    int ext_mode;
};

struct shdisp_main_bkl_ctl {
    int mode;
    int param;
};

struct shdisp_gam_adj_point {
    unsigned char gam_lvl;
    unsigned short gam_val;
};

struct shdisp_gam_data {
    unsigned short status;
    struct shdisp_gam_adj_point adj[3][8];
    unsigned char def_table[3][2][256];
};

struct shdisp_boot_context {
    int driver_is_initialized;
    unsigned short hw_revision;
    int handset_color;
    int upper_unit_is_connected;
    int panel_ver;
    int lcdc_ver;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    struct shdisp_gam_data gam_data;
    unsigned char alpha;
    int vdfreq;
    int tpll;
    int boot_mode;
};

int shdisp_api_main_disp_on(void);
int shdisp_api_main_disp_off(void);
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl);
int shdisp_api_main_bkl_off(void);
int shdisp_api_shutdown(void);

#endif /* SHDISP_KERN_H */
