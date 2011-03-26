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
 * Definitions for smb380 chip.
 */
#ifndef SMB380_H
#define SMB380_H

#include <linux/ioctl.h>


struct sh_i2c_accel_platform_data {
    int     gpio_irq;
    int     (*gpio_setup) (void);
    void    (*gpio_shutdown)(void);
};
#define	SH_ACCEL_I2C_DEVNAME    "SH_SMB380"
#define SH_ACCEL_I2C_SLAVE      0x38
#define	SH_ACCEL_IRQ            37


/* SMB380 register address */
#define SMB380_REG_CHIP_ID              0x00
#define SMB380_REG_VERSION              0x01
#define SMB380_REG_X_AXIS_LSB           0x02
#define SMB380_REG_X_AXIS_MSB           0x03
#define SMB380_REG_Y_AXIS_LSB           0x04
#define SMB380_REG_Y_AXIS_MSB           0x05
#define SMB380_REG_Z_AXIS_LSB           0x06
#define SMB380_REG_Z_AXIS_MSB           0x07
#define SMB380_REG_TEMP_RD              0x08
#define SMB380_REG_SMB380_STATUS        0x09
#define SMB380_REG_SMB380_CTRL          0x0a
#define SMB380_REG_SMB380_CONF1         0x0b
#define SMB380_REG_LG_THRESHOLD         0x0c
#define SMB380_REG_LG_DURATION          0x0d
#define SMB380_REG_HG_THRESHOLD         0x0e
#define SMB380_REG_HG_DURATION          0x0f
#define SMB380_REG_MOTION_THRS          0x10
#define SMB380_REG_HYSTERESIS           0x11
#define SMB380_REG_CUSTOMER1            0x12
#define SMB380_REG_CUSTOMER2            0x13
#define SMB380_REG_RANGE_BWIDTH         0x14
#define SMB380_REG_SMB380_CONF2         0x15



/* SMB380 ioctl */
#define SMB380_IOC_MAGIC                0xA2
#define SMB380_READ_ACCEL_XYZ   _IOR(SMB380_IOC_MAGIC,0x10, short[3])
#define SMB380_SET_RANGE        _IOW(SMB380_IOC_MAGIC,0x11, unsigned char)
#define SMB380_SET_MODE         _IOW(SMB380_IOC_MAGIC,0x12, unsigned char)
#define SMB380_SET_BANDWIDTH    _IOW(SMB380_IOC_MAGIC,0x13, unsigned char)



#define SMB380_RANGE_2G         0 /* sets range to 2G mode */
#define SMB380_RANGE_4G         1 /* sets range to 4G mode */
#define SMB380_RANGE_8G         2 /* sets range to 8G mode */
#define SMB380_MODE_NORMAL      0 /* sets mode to NORMAL mode */
#define SMB380_MODE_SLEEP       1 /* sets mode to SLEEP mode */
#define SMB380_BW_25HZ          0 /* sets bandwidth to 25HZ */
#define SMB380_BW_50HZ          1 /* sets bandwidth to 50HZ */
#define SMB380_BW_100HZ         2 /* sets bandwidth to 100HZ */
#define SMB380_BW_190HZ         3 /* sets bandwidth to 190HZ */
#define SMB380_BW_375HZ         4 /* sets bandwidth to 375HZ */
#define SMB380_BW_750HZ         5 /* sets bandwidth to 750HZ */
#define SMB380_BW_1500HZ        6 /* sets bandwidth to 1500HZ */


#endif /* SMB380_H */

