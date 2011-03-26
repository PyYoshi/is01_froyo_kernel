/* arch/arm/mach-msm/sh_smem.c
 *
 * Copyright (C) 2009 Sharp Corporation
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
#include <linux/kernel.h>

#include "smd_private.h"
#include <mach/sharp_smem.h>

static sharp_smem_common_type *p_sharp_smem_common_type = NULL;

/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void )
{
    if( p_sharp_smem_common_type == NULL){
        p_sharp_smem_common_type = smem_alloc( SMEM_ID_VENDOR0, sizeof(sharp_smem_common_type));
    }
    
    return p_sharp_smem_common_type;
}
EXPORT_SYMBOL(sh_smem_get_common_address);
