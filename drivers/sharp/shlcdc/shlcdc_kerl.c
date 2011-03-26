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
 * SHARP LCD CONTROLLER DRIVER FOR KERNEL
 */

/*
 * INCLUDE FILES
 */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shlcdc_kerl.h>
#include <mach/sharp_smem.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <asm/io.h>
#include <asm/pgtable.h>

/*
 * MACROS
 */

//#define KERN_DBG_ENABLE_A
//#define KERN_DBG_ENABLE_B
//#define KERN_DBG_ENABLE_C

#define SHLCDC_FILE "shlcdc_kerl.c"

#ifdef KERN_DBG_ENABLE_A
#define KERN_DBG_A(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_B
#define KERN_DBG_B(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_C
#define KERN_DBG_C(fmt, args...)     \
        printk(KERN_INFO "[%s][%s] " fmt, SHLCDC_FILE, __func__, ## args)
#else /* !KERN_DBG_ENABLE_C */
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_C */
#else /* !KERN_DBG_ENABLE_B */
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_B */
#else /* !KERN_DBG_ENABLE_A */
#define KERN_DBG_A(fmt, args...)
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_A */

#define SHLCDC_IOC_MAGIC 'l'
#define SHLCDC_IOCTL_SET_VDLINK_MUTEX _IOW(SHLCDC_IOC_MAGIC, 0, int)

/*
 * TYPES
 */

struct shlcdc_event {
    int cmd;
    int event_type;
};

/*
 * VARIABLES
 */

static dev_t shlcdc_dev;
static dev_t shlcdc_major = 0;
static dev_t shlcdc_minor = 0;
static struct cdev shlcdc_cdev;
static struct class *shlcdc_class;
static void *shlcdc_base_addr = NULL;
static wait_queue_head_t shlcdc_wq_for_read;
static atomic_t shlcdc_atomic_for_read;
static signed long shlcdc_event_counter_for_read;
static int shlcdc_driver_is_initialized = 0;
static struct shlcdc_event shlcdc_event_info;
static struct shdisp_boot_context shdisp_boot_ctx;

DECLARE_MUTEX(shdisp_vdlink_mutex);

/*
 * PROTOTYPES
 */

void shlcdc_wake_up(int cmd, int event_type);
static int shlcdc_open(struct inode *inode, struct file *filp);
static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos);
static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos);
static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma);
static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg);
static int shlcdc_release(struct inode *inode, struct file *filp);

static struct file_operations shlcdc_fops = {
    .owner   = THIS_MODULE,
    .open    = shlcdc_open,
    .write   = shlcdc_write,
    .read    = shlcdc_read,
    .mmap    = shlcdc_mmap,
    .ioctl   = shlcdc_ioctl,
    .release = shlcdc_release,
};

/*
 * FUNCTIONS
 */

/*
 * shlcdc_wake_up
 */

void shlcdc_wake_up(int cmd, int event_type)
{
    shlcdc_event_info.cmd = cmd;
    shlcdc_event_info.event_type = event_type;

    atomic_inc(&shlcdc_atomic_for_read);
    wake_up_interruptible(&shlcdc_wq_for_read);

    return;
}

/*
 * shlcdc_open
 */

static int shlcdc_open(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized == 0) {
        shlcdc_event_counter_for_read = atomic_read(&shlcdc_atomic_for_read);
    }

    shlcdc_driver_is_initialized++;

    return 0;
}

/*
 * shlcdc_write
 */

static int shlcdc_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    return 0;
}

/*
 * shlcdc_read
 */

static int shlcdc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    DECLARE_WAITQUEUE(wait, current);
    int ret = 0;
    int event_count;

    add_wait_queue(&shlcdc_wq_for_read, &wait);

    do {
        set_current_state(TASK_INTERRUPTIBLE);

        event_count = atomic_read(&shlcdc_atomic_for_read);

        if (event_count != shlcdc_event_counter_for_read) {
            if (copy_to_user(buf, &shlcdc_event_info, count) != 0) {
                ret = -EFAULT;
            } else {
                shlcdc_event_counter_for_read = event_count;
                ret = count;
            }
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }

        if (signal_pending(current)) {
            ret = -ERESTARTSYS;
            break;
        }

        schedule();
    } while (1);

    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&shlcdc_wq_for_read, &wait);

    return ret;
}

/*
 * shlcdc_mmap
 */

static int shlcdc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    ret = io_remap_pfn_range(vma, vma->vm_start,
                       (unsigned long)shlcdc_base_addr >> PAGE_SHIFT,
                       vma->vm_end - vma->vm_start, vma->vm_page_prot);

    return 0;
}

/*
 * shlcdc_ioctl
 */

static int shlcdc_ioctl(struct inode *inode, struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
    int ret, sw;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
    case SHLCDC_IOCTL_SET_VDLINK_MUTEX:
        ret = copy_from_user(&sw, argp, sizeof(int));
        if (ret != 0) {
            return ret;
        }
        if (sw == 0) {
            up(&shdisp_vdlink_mutex);
        } else {
            down(&shdisp_vdlink_mutex);
        }
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

/*
 * shlcdc_release
 */

static int shlcdc_release(struct inode *inode, struct file *filp)
{
    if (shlcdc_driver_is_initialized > 0) {
        shlcdc_driver_is_initialized--;

        if (shlcdc_driver_is_initialized == 0) {
            atomic_set(&shlcdc_atomic_for_read, 0);
        }
    }

    return 0;
}

/*
 * shlcdc_init
 */

static int __init shlcdc_init(void)
{
    int ret;
    sharp_smem_common_type *sh_smem_common;

    sh_smem_common = sh_smem_get_common_address();

    memcpy(&shdisp_boot_ctx,
           &(sh_smem_common->shdisp_boot_ctx),
           sizeof(struct shdisp_boot_context));

    if ((shdisp_boot_ctx.hw_revision & 0x000F) == 0x0000) {
        shlcdc_base_addr = (unsigned char*)0x94000000;
    } else {
        shlcdc_base_addr = (unsigned char*)0x60000000;
    }

    ret = alloc_chrdev_region(&shlcdc_dev, 0, 1, "shlcdc");

    if (!ret) {
        shlcdc_major = MAJOR(shlcdc_dev);
        shlcdc_minor = MINOR(shlcdc_dev);
    } else {
        goto shlcdc_err_1;
    }

    cdev_init(&shlcdc_cdev, &shlcdc_fops);

    shlcdc_cdev.owner = THIS_MODULE;
    shlcdc_cdev.ops = &shlcdc_fops;

    ret = cdev_add(&shlcdc_cdev, shlcdc_dev, 1);

    if (ret) {
        goto shlcdc_err_2;
    }

    shlcdc_class = class_create(THIS_MODULE, "shlcdc");

    if (IS_ERR(shlcdc_class)) {
        goto shlcdc_err_2;
    }

    device_create(shlcdc_class, NULL,
                  shlcdc_dev, &shlcdc_cdev, "shlcdc");

    init_waitqueue_head(&shlcdc_wq_for_read);

    atomic_set(&shlcdc_atomic_for_read, 0);;

    return 0;

shlcdc_err_2:
    cdev_del(&shlcdc_cdev);

shlcdc_err_1:
    return -1;
}
module_init(shlcdc_init);

/*
 * shlcdc_exit
 */

static void shlcdc_exit(void)
{
    device_destroy(shlcdc_class, shlcdc_dev);
    class_destroy(shlcdc_class);
    cdev_del(&shlcdc_cdev);

    return;
}
module_exit(shlcdc_exit);

MODULE_DESCRIPTION("SHARP LCD CONTROLLER DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
