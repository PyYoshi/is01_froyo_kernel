/* ******************************************************************************** */
/*																					*/
/*	TunerDev.c	Tuner Control device													*/
/*																					*/
/*																					*/
/*																					*/
/* ******************************************************************************** */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/smp_lock.h>
#include <mach/vreg.h>

#include "gpio_def.h"

extern int gpio_direction_output(unsigned gpio, int value);
extern int gpio_direction_input(unsigned gpio);
extern int gpio_get_value(unsigned gpio);
extern void gpio_set_value(unsigned gpio, int value);

static int gpio_init(void);
static int gpio_get(unsigned int no, int *val);
static int gpio_set(unsigned int no, int val);
static int tuner_vreg_enable(void);
static int tuner_vreg_disable(void);

static stGPIO_DEF use_gpiono[] = {
	
	{GPIO_PWRDWN_PORTNO   , DirctionOut, 1, 0},
	{GPIO_GTDION_PORTNO   , DirctionOut, 0, 0},
	{GPIO_LDO_PORTNO      , DirctionOut, 0, 0}
};



static int tuner_open(struct inode *inode, struct file *file)
{
	int ret;

	ret  = gpio_init();
	if (ret == 1) {
		
		printk("%s:%d !!!tuner_open gpio_init() error \n", __FILE__, __LINE__);
		return (-1);
	}
	return (0);
}


static int tuner_release(struct inode *inode, struct file *file)
{
	return (0);
}

static long tuner_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	ioctl_cmd 		  *io_cmd = (ioctl_cmd *)arg;
	
	switch ( cmd ) {
	case IOC_GPIO_VAL_SET:
		ret = gpio_set(io_cmd->no, io_cmd->val);
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->no, io_cmd->val, ret);
			return -EINVAL;
		}
		break;
	case IOC_GPIO_VAL_GET:
		ret = gpio_get(io_cmd->no, &(io_cmd->val));
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl get error[cmd %x val %d][ret:%d]\n", __FILE__, __LINE__, io_cmd->no, io_cmd->val, ret);
			return -EINVAL;
		}
		break;
	case IOC_VREG_ENABLE:
		ret = tuner_vreg_enable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]¥n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	case IOC_VREG_DISABLE:
		ret = tuner_vreg_disable();
		if (ret != 0) {
			printk("%s:%d  !!!tuner_ioctl set error[ret:%d]¥n", __FILE__, __LINE__, ret);
			return -EINVAL;
		}
		break;
	default:
		/* NOTE:  returning a fault code here could cause trouble
		 * in buggy userspace code.  Some old kernel bugs returned
		 * zero in this case, and userspace code might accidentally
		 * have depended on that bug.
		 */
		return -ENOTTY;
	}
	return 0;
}

static struct file_operations tuner_fops = {
	.owner				= THIS_MODULE,
	.unlocked_ioctl		= tuner_ioctl,
	.open				= tuner_open,
	.release			= tuner_release,
};

static struct miscdevice tuner_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tunctrl",
	.fops = &tuner_fops,
};


static int __init tuner_init(void)
{
	int ret;

	ret = misc_register(&tuner_dev);
	if (ret) {
		printk("%s.%s.%d !!! fail to misc_register (MISC_DYNAMIC_MINOR)\n", __FILE__, __func__, __LINE__);
		return ret;
	}
	
	return 0;
}

static void __exit tuner_cleanup(void)
{
	misc_deregister(&tuner_dev);
}



/************************************************************************/
static int gpio_init(void)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	int errcnt = 0;
	stGPIO_DEF *p = &use_gpiono[0];
	int i;
	
	for (i=0; i<loop; i++, p++) {
		if (p->direction == DirctionIn) {
			
			if (gpio_direction_input(p->no) < 0) {
				
				errcnt ++;
				printk( "%s:%d gpio_direction_input error NO.%d \n", __FILE__,__LINE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
		if (p->direction == DirctionOut) {
			
			if (gpio_direction_output(p->no, p->out_val) < 0) {
				
				errcnt ++;
				printk("%s: gpio_direction_output error NO.%d \n", __FILE__, p->no);
				continue;
			}
			p->init_done = 1;
			continue;
		}
	}

	if (errcnt != 0) {
		printk("%s: gpio_init error count %d\n", __FILE__, errcnt);
		return 1;
	}
	return 0;
}

static int gpio_set(unsigned int no, int value)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = use_gpiono;
	int flag = 0;
	int i;
	
	
	for(i=0; i<loop; i++, p++){
		if (p->no == no){
			flag = 1;
			break;
		}
	}
	if (flag == 0) {
		
		printk("%s: !!! gpio_set() error No.%d value %d \n", __FILE__, no, value);
		return EINVAL;
	}
	gpio_set_value(no, value);
	return 0;
}

static int gpio_get(unsigned int no, int *val)
{
	int loop = sizeof(use_gpiono)/sizeof(stGPIO_DEF);
	stGPIO_DEF *p = &use_gpiono[0];
	int flag = 0;
	int i;
	
	*val = 0;

	
	for(i=0; i<loop; i++, p++){
		if (p->no == no){
			flag = 1;
			break;
		}
	}
	if (flag == 0) {
		
		printk("%s: !!! gpio_get() No.%d error \n", __FILE__, no);
		return EINVAL;
	}
	*val = gpio_get_value(no);

	return 0;
}

static int tuner_vreg_enable(void)
{
	struct vreg *vreg_gp3;
	int rc;

	vreg_gp3 = vreg_get(NULL, "gp3");
	if (IS_ERR(vreg_gp3)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)¥n",
			__func__, "gp3", PTR_ERR(vreg_gp3));
		return -1;
	}

	rc = vreg_set_level(vreg_gp3, 2900);
	if (rc) {
		printk(KERN_ERR "%s: vreg gp3 set level failed (%d)¥n",
			__func__, rc);
		return -2;
	}

	rc = vreg_enable(vreg_gp3);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)¥n",
			__func__, rc);
		return -3;
	}

	return 0;
}

static int tuner_vreg_disable(void)
{
	struct vreg *vreg_gp3;
	int rc;

	vreg_gp3 = vreg_get(NULL, "gp3");
	if (IS_ERR(vreg_gp3)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)¥n",
			__func__, "gp3", PTR_ERR(vreg_gp3));
		return -1;
	}

	rc = vreg_set_level(vreg_gp3, 0);
	if (rc) {
		printk(KERN_ERR "%s: vreg gp3 set level failed (%d)¥n",
			__func__, rc);
		return -2;
	}

	rc = vreg_disable(vreg_gp3);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)¥n",
			__func__, rc);
		return -3;
	}

	return 0;
}

MODULE_LICENSE("GPL");
module_init(tuner_init);
module_exit(tuner_cleanup);
