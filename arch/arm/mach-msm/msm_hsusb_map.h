/* linux/arch/arm/mach-msm/msm_hsusb_map.h
 *
 * Copyright (c) 2010 Sharp Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_MSM_HSUSB_MAP_H
#define __ARCH_ARM_MACH_MSM_MSM_HSUSB_MAP_H


#include <mach/msm_hsusb_desc.h>


static struct usb_function_map usb_functions_map[] = {
        {"obex", 0},
        {"mdlm", 1},
        {"modem", 2},
        {"mass_storage", 3},
        {"mtp", 4},
        {"adb", 5},
        {"diag", 6},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
        {
                .product_id         = USB_PID_MODE1,
                .functions          = 0x67, /* 1100111 */
        },

        {
                .product_id         = USB_PID_MODE2,
                .functions          = 0x70, /* 1110000 */
        },

        {
                .product_id         = USB_PID_MODE3,
                .functions          = 0x68, /* 1101000 */
        },

        {
                .product_id         = USB_PID_MODE4,
                .functions          = 0x61, /* 1100001 */
        },
};

#endif
