/* 
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

#include <asm/mach/time.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/list.h>

typedef enum
{
	SHTIMER_CMD_INVALID,
	SHTIMER_CMD_SET,
	SHTIMER_CMD_CANCEL,
	SHTIMER_CMD_WAIT,
	SHTIMER_CMD_WAKELOCK,
	SHTIMER_CMD_WAKEUNLOCK,
} shtimer_cmd;

#define SHTIMER_NONE_SIG   0x00000000
#define SHTIMER_CANCEL_SIG 0x00000001
#define SHTIMER_EXPIRE_SIG 0x00000002

#define SLEEP_NEGATE_TIME 50000000

static LIST_HEAD(shtimer_head);

typedef struct
{
	struct list_head link;
	struct hrtimer timer;
	struct wake_lock wake_lock;
	char wake_name[16];
	unsigned long discriptor;
	unsigned long pending;
	unsigned long waiting;
	wait_queue_head_t wait_queue;
	ktime_t tv;
	ktime_t base_tv;
} shtimer_list;

static unsigned long shtimer_discriptor;
static DEFINE_SPINLOCK(shtimer_slock);
static ktime_t wakeup_time;
static int64_t sleep_ns = 0;

extern void msm_pm_set_shtimer_sleep_time(int64_t max_sleep_time_ns);

void shtimer_set_sleep_time(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shtimer_slock, flags);
	msm_pm_set_shtimer_sleep_time(ktime_to_ns(wakeup_time));
	spin_unlock_irqrestore(&shtimer_slock, flags);
}

void shtimer_get_sleep_time(int64_t ns)
{
	unsigned long flags;

	spin_lock_irqsave(&shtimer_slock, flags);
	sleep_ns = ns;
	spin_unlock_irqrestore(&shtimer_slock, flags);
}

int shtimer_resume(struct platform_device *pdev)
{
	shtimer_list *list;
	struct list_head *item;
	unsigned long flags;

	if(sleep_ns == 0) return 0;

	spin_lock_irqsave(&shtimer_slock, flags);
	list_for_each(item, &shtimer_head){
		list = list_entry(item, shtimer_list, link);

		if(list->waiting){
			list->tv = ktime_sub(list->tv, ns_to_ktime(sleep_ns));
			if(list->tv.tv64 < (s64)0){
				list->tv.tv64 = 0;
			}
			list->base_tv = ktime_get();
			spin_unlock_irqrestore(&shtimer_slock, flags);
			hrtimer_start(&list->timer, list->tv, HRTIMER_MODE_REL);
			spin_lock_irqsave(&shtimer_slock, flags);
		}
	}
	sleep_ns = 0;
	spin_unlock_irqrestore(&shtimer_slock, flags);

	return 0;
}

int shtimer_suspend(struct platform_device *pdev, pm_message_t state)
{
	shtimer_list *list, *wakelist = NULL;
	struct list_head *item;
	unsigned long flags;
	ktime_t elapse_time;

	wakeup_time.tv.sec = 0x1000;
	wakeup_time.tv.nsec = 0;

	spin_lock_irqsave(&shtimer_slock, flags);
	list_for_each(item, &shtimer_head){
		list = list_entry(item, shtimer_list, link);

		if(list->waiting){
			spin_unlock_irqrestore(&shtimer_slock, flags);
			hrtimer_try_to_cancel(&list->timer);
			spin_lock_irqsave(&shtimer_slock, flags);

			elapse_time = ktime_sub(ktime_get(), list->base_tv);
			if((list->tv.tv64 < elapse_time.tv64) ||
			   (list->pending & SHTIMER_EXPIRE_SIG) != 0){
				/* already expired */
				list->tv.tv64 = 0;
			}
			else{
				list->tv = ktime_sub(list->tv, elapse_time);
			}

			if(list->tv.tv64 < wakeup_time.tv64){
				wakeup_time = list->tv;
				wakelist = list;
			}
		}
	}
	spin_unlock_irqrestore(&shtimer_slock, flags);

	if(wakeup_time.tv.sec == 0 && wakeup_time.tv.nsec <= SLEEP_NEGATE_TIME){
		sleep_ns = 1;
		shtimer_resume(pdev);
		if(wakelist) wake_lock_timeout(&wakelist->wake_lock, HZ / 20);
		return -EBUSY;
	}

	return 0;
}

static long shtimer_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned long flags, discriptor;
	shtimer_list *list = NULL;
	struct list_head *item;
	struct timespec tv = {0, 0};

	if(file->private_data == NULL) return 0;
	discriptor = *(unsigned long*)file->private_data;

	spin_lock_irqsave(&shtimer_slock, flags);
	list_for_each(item, &shtimer_head){
		list = list_entry(item, shtimer_list, link);

		if(list->discriptor == discriptor) break;
	}
	spin_unlock_irqrestore(&shtimer_slock, flags);

	if(item == &shtimer_head) return 0;

	switch(cmd){
	case SHTIMER_CMD_CANCEL:
		hrtimer_try_to_cancel(&list->timer);

		spin_lock_irqsave(&shtimer_slock, flags);
		wake_lock(&list->wake_lock);
		list->pending |= SHTIMER_CANCEL_SIG;
		list->waiting = 0;
		wake_up(&list->wait_queue);
		spin_unlock_irqrestore(&shtimer_slock, flags);
		break;

	case SHTIMER_CMD_SET:
		if (copy_from_user(&tv, (void __user *)arg, sizeof(struct timespec))) {
			ret = 0;
			break;
		}

		hrtimer_try_to_cancel(&list->timer);

		spin_lock_irqsave(&shtimer_slock, flags);
		wake_up(&list->wait_queue);
		list->pending = SHTIMER_NONE_SIG;
		list->waiting = 1;
		list->tv = timespec_to_ktime(tv);
		list->base_tv = ktime_get();
		spin_unlock_irqrestore(&shtimer_slock, flags);

		hrtimer_start(&list->timer, list->tv, HRTIMER_MODE_REL);

	case SHTIMER_CMD_WAIT:
		spin_lock_irqsave(&shtimer_slock, flags);
		if(cmd == SHTIMER_CMD_SET && SLEEP_NEGATE_TIME < list->tv.tv64){
			/* aARM wake up cost is over 50ms */
			wake_unlock(&list->wake_lock);
		}
		spin_unlock_irqrestore(&shtimer_slock, flags);
		ret = wait_event_interruptible(list->wait_queue, list->pending);
		if(ret){
			ret = EINTR;
			goto freeze_request;
		}

		spin_lock_irqsave(&shtimer_slock, flags);
		wake_lock(&list->wake_lock);
		if((list->pending & SHTIMER_EXPIRE_SIG) == 0 ||
		   (list->pending & SHTIMER_CANCEL_SIG) != 0 ){
			ret = -1;
		}
		spin_unlock_irqrestore(&shtimer_slock, flags);
		break;

	case SHTIMER_CMD_WAKELOCK:
		spin_lock_irqsave(&shtimer_slock, flags);
		wake_lock(&list->wake_lock);
		spin_unlock_irqrestore(&shtimer_slock, flags);
		break;

	case SHTIMER_CMD_WAKEUNLOCK:
		spin_lock_irqsave(&shtimer_slock, flags);
		wake_unlock(&list->wake_lock);
		spin_unlock_irqrestore(&shtimer_slock, flags);
		break;

	default:
		ret = 0;
		break;
	}

freeze_request:

	return ret;
}

static enum hrtimer_restart shtimer_timer_expired(struct hrtimer *timer)
{
	unsigned long flags;
	shtimer_list *list;
	struct list_head *item;

	spin_lock_irqsave(&shtimer_slock, flags);
	list_for_each(item, &shtimer_head){
		list = list_entry(item, shtimer_list, link);

		if(&list->timer == timer) break;
	}

	if(item == &shtimer_head){
		spin_unlock_irqrestore(&shtimer_slock, flags);
		return HRTIMER_NORESTART;
	}

	wake_lock(&list->wake_lock);
	list->pending |= SHTIMER_EXPIRE_SIG;
	list->waiting = 0;
	wake_up(&list->wait_queue);
	spin_unlock_irqrestore(&shtimer_slock, flags);

	return HRTIMER_NORESTART;
}

static int shtimer_open(struct inode *inode, struct file *file)
{
	unsigned long *discriptor;
	shtimer_list *list;

	discriptor = (unsigned long*)kmalloc(sizeof(void), GFP_KERNEL);
	if(discriptor == NULL) return 0;

	list = (shtimer_list*)kmalloc(sizeof(shtimer_list), GFP_KERNEL);
	if(list == NULL){
		kfree(discriptor);
		return 0;
	}

	memset(list, 0x00, sizeof(shtimer_list));

	hrtimer_init(&list->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	list->timer.function = shtimer_timer_expired;
	list->discriptor = shtimer_discriptor;
	init_waitqueue_head(&list->wait_queue);

	sprintf(list->wake_name, "shtimer%02d", (int)shtimer_discriptor);
	wake_lock_init(&list->wake_lock, WAKE_LOCK_SUSPEND, list->wake_name);

	*discriptor = shtimer_discriptor;
	shtimer_discriptor++;
	file->private_data = (void*)discriptor;

	list_add(&list->link, &shtimer_head);

	return 0;
}

static int shtimer_release(struct inode *inode, struct file *file)
{
	unsigned long discriptor;
	shtimer_list *list;
	struct list_head *item;

	if(file->private_data == NULL) return 0;
	discriptor = *(unsigned long*)file->private_data;

	list_for_each(item, &shtimer_head){
		list = list_entry(item, shtimer_list, link);

		if(list->discriptor == discriptor) break;
	}

	if(item == &shtimer_head) return 0;

	hrtimer_try_to_cancel(&list->timer);
	wake_lock_destroy(&list->wake_lock);
	list_del(&list->link);
	if(list->discriptor) kfree(file->private_data);
	if(list) kfree(list);
	file->private_data = NULL;

	return 0;
}

static struct file_operations shtimer_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = shtimer_ioctl,
	.open = shtimer_open,
	.release = shtimer_release,
};

static struct miscdevice shtimer_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "shtimer",
	.fops = &shtimer_fops,
};

static int __devinit shtimer_probe(struct platform_device *pdev)
{
	shtimer_misc.parent = &pdev->dev;

	return misc_register(&shtimer_misc);
}

static int shtimer_remove(struct platform_device *pdev)
{
	misc_deregister(&shtimer_misc);

	return 0;
}

static struct platform_device shtimer_device = {
	.name = "shtimer",
	.id   = -1,
};

static struct platform_driver shtimer_driver = {
	.probe = shtimer_probe,
	.suspend = shtimer_suspend,
	.resume = shtimer_resume,
	.remove = shtimer_remove,
	.driver	      = {
		.name = "shtimer",
	},
};

static int __init shtimer_init(void)
{
	int ret;

	ret = platform_device_register(&shtimer_device);
	if(ret){
		printk("device register failure %s\n", __FUNCTION__);
		return ret;
	}

	ret = platform_driver_register(&shtimer_driver);
	if(ret){
		platform_driver_unregister(&shtimer_driver);
		printk("driver register failure %s\n", __FUNCTION__);
		return ret;
	}

	return 0;
}

static void  __exit shtimer_exit(void)
{
	platform_driver_unregister(&shtimer_driver);
	platform_device_unregister(&shtimer_device);
}

module_init(shtimer_init);
module_exit(shtimer_exit);
MODULE_LICENSE("GPL");
