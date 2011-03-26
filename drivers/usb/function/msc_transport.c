/* drivers/usb/function/msc_transport.c
 *
 * Function Device for the Android MSC Protocol
 *
 * Copyright (c) 2010 Sharp Corporation.
 *
 * This code borrows from adb.c, which is
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/miscdevice.h>
#include <linux/fs.h>


#include <linux/kref.h>
#include <linux/platform_device.h>
#include <linux/usb/mass_storage_function.h>

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/switch.h>

#include "usb_function.h"
#include <mach/msm_hsusb_desc.h>

/*
 * Refer to Documentation/ioctl-number.txt and Documentation/ioctl/
 * to choose magic-numbers
 */
#define USB_MSC_IOC_MAGIC 0xFF

#define USB_MSC_FUNC_IOC_SET_STALL _IOW(USB_MSC_IOC_MAGIC, 0x20, int)
#define USB_MSC_FUNC_IOC_GET_MOUNT_STS _IOW(USB_MSC_IOC_MAGIC, 0x21, int)


#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(x)
#endif

/* Defines */
#define DRIVER_NAME			"usb_mass_storage"
/* s D_SHUSB_MSC_LUN_IF */
#define MAX_LUNS			1
/* e D_SHUSB_MSC_LUN_IF */

#define RXN_MAX 512
#define TXN_MAX 4096


/* MSC setup class requests */
#define USB_MSC_GET_MAX_LUN_REQUEST   0xFE
#define USB_MSC_RESET_REQUEST         0xFF

#define	USB_MSC_NTY_ONLINE_STATE				(1)
#define	USB_MSC_NTY_OFFLINE_STATE				(2)
#define USB_MSC_NTY_MOUNT_ENABLE				(3)
#define USB_MSC_NTY_MOUNT_DISABLE				(4)
#define	USB_MSC_NTY_RESET_STATE					(5)

#define	USB_CTRL_NTY_MAX_STR_LEN	(48)
#define	USB_ONLINE_STATE_CHANGE_STR		"online"
#define	USB_OFFLINE_STATE_CHANGE_STR	"offline"
#define	USB_MOUNT_ENABLE_STR			"mount"
#define	USB_MOUNT_DISABLE_STR			"umount"
#define	USB_RESET_STATE_STR				"reset"

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 128 /* 16 -> 128 */
#define TX_REQ_MAX 16 /* 4 -> 16 */

#define MSC_FUNCTION_NAME "mass_storage"

#define USB_DIR_MASK	USB_DIR_IN

struct msc_context
{
	int online;
	int on_mode;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
#if 0
	/* we don't use enable_device for msc. */
	atomic_t enable_excl;
#endif
	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	volatile unsigned long	sleep_flag;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;
        /* reference counting: wait until all LUNs are released */
        struct kref  ref;
	unsigned int nluns;
	struct lun   *luns;
	struct platform_device *pdev;
	unsigned int mount_enb;
	unsigned bound;
	struct switch_dev sdev;
	struct work_struct sw_work;

	/* for control */
	atomic_t open_ctrl_excl;
	atomic_t ctrl_read_excl;
	wait_queue_head_t ctrl_read_wq;
	unsigned char ctrl_set_size;
	unsigned char *ctrl_read_buf;
};

enum shusb_state{
  SHUSB_STATE_WAIT_CBW,
  SHUSB_STATE_DATA_IN,
  SHUSB_STATE_DATA_OUT,
  SHUSB_STATE_WAIT_CSW,
};
static struct msc_context _context;

#define USB_SC_SCSI     0x06            /* Transparent SCSI */
#define USB_PR_BULK     0x50            /* Bulk-only */
static struct usb_interface_descriptor intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	2,
	.bInterfaceClass 	= USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass 	= USB_SC_SCSI,
	.bInterfaceProtocol 	= USB_PR_BULK,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval =		0,
};
static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval =		0,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		0,
};

struct lun {
        struct file     *filp;
        loff_t          file_length;
        loff_t          num_sectors;

        unsigned int    ro : 1;
        unsigned int    prevent_medium_removal : 1;
        unsigned int    registered : 1;
        unsigned int    info_valid : 1;

        u32             sense_data;
        u32             sense_data_info;
        u32             unit_attention_data;

        struct device   dev;
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sharp Corporation");

static struct usb_function usb_func_msc;

static void msc_control_notify( unsigned char type,void* opt );

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}


/* add a request to the tail of a list */
static void req_put(struct msc_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct msc_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void msc_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct msc_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	ctxt->sleep_flag = 1;

	wake_up(&ctxt->write_wq);
}

static void msc_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct msc_context *ctxt = req->context;

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
		DBG("complete_out err(%d)\n", req->actual);
	} else {
		req_put(ctxt, &ctxt->rx_done, req);
		DBG("complete_out(%d)\n", req->actual);
	}

	wake_up(&ctxt->read_wq);
}

#define D_MSC_TRANSPORT_READ_REQ_ON_TIME

static ssize_t msc_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct msc_context *ctxt = &_context;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	int	timeout_onflg = 0;	/* timeout-flg */
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	int req_q_cnt =0;
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

	DBG("msc_read(%d)\n", count);

	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("msc_read: waiting for online state\n");
		ret = wait_event_interruptible_timeout(ctxt->read_wq, 
								(ctxt->online || ctxt->error), HZ / 10 );
		DBG("wait req 1\n");
		if (ret < 0) {
			_unlock(&ctxt->read_excl);
			return ret;
		}
		else if (!ctxt->online) {
			_unlock(&ctxt->read_excl);
			return -EIO;
		}
	}
	
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	req_q_cnt = count % usb_ept_get_max_packet(ctxt->out);
	if(req_q_cnt == 0)
		req_q_cnt = count / usb_ept_get_max_packet(ctxt->out);
	else
		req_q_cnt = count / usb_ept_get_max_packet(ctxt->out) + 1;
	DBG("msc_read: count = %d , req_q_cnt = %d \n",count , req_q_cnt);
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* if we have idle read requests, get them queued */
		while ((req = req_get(ctxt, &ctxt->rx_idle))) {
requeue_req:
#if 1
			req->length = usb_ept_get_max_packet(ctxt->out);
			if(req->length > RXN_MAX)
				req->length = RXN_MAX;
#else
			req->length = TXN_MAX;
#endif

#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
			if(req_q_cnt == 0) {
				DBG("msc_read: req_q_cnt == 0 ,so break \n");
				req_put(ctxt, &ctxt->rx_idle, req);
				break;
			}
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("msc_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				req_put(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
				req_q_cnt--;
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */
				DBG("rx %p queue\n", req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;

		DBG("xfer =%d\n", xfer);
			if (copy_to_user(buf, ctxt->read_buf, xfer)) {
				r = -EFAULT;
				break;
			}
			ctxt->read_buf += xfer;
			ctxt->read_count -= xfer;
			buf += xfer;
			count -= xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_count == 0) {
				req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
				ctxt->read_req = 0;
				timeout_onflg = 1;
		
				/* for short packet  */
				if( (r - count) % usb_ept_get_max_packet(ctxt->out) != 0) {
					r = r - count;
					break;
				}
			}
			DBG("read exit(%d)\n", xfer);
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		DBG("wait req 2\n");
		if( timeout_onflg == 0 )
			ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error));
		else
			ret = wait_event_interruptible_timeout(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error), HZ / 10 );


		DBG("wait exit\n");
		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0)
				goto requeue_req;

			ctxt->read_req = req;
			ctxt->read_count = req->actual;
			ctxt->read_buf = req->buf;
			DBG("rx %p %d\n", req, req->actual);
		}
		else if((ret == 0) && (!ctxt->error)){ /* for timeout */
			r = r - count;
			break;
		}

		if (ret < 0) {
			r = ret;
			break;
		}
	}

fail:
#ifdef D_MSC_TRANSPORT_READ_REQ_ON_TIME
	if(r < 0) {
		DBG("msc_read: case r< 0 \n");
		/* refresh out-direction buffer */
		usb_ept_fifo_flush(ctxt->out);

		/* if we have a stale request being read, recycle it */
		ctxt->read_buf = 0;
		ctxt->read_count = 0;
		if (ctxt->read_req) {
			req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
			ctxt->read_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = req_get(ctxt, &ctxt->rx_done)))
			req_put(ctxt, &ctxt->rx_idle, req);
	}
#endif /* D_MSC_TRANSPORT_READ_REQ_ON_TIME */
	_unlock(&ctxt->read_excl);
	DBG("msc_read ret= %d \n", r);
	return r;
}

#define D_MSC_TRANSPORT_WRITE_WAIT_COMP

static ssize_t msc_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct msc_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	int req_q_cnt;
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */

	DBG("msc_write(%d)\n", count);

	if (_lock(&ctxt->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (ctxt->error) {
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(ctxt->write_wq,
					       ((req = req_get(ctxt, &ctxt->tx_idle)) || ctxt->error));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			xfer = count > TXN_MAX ? TXN_MAX : count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}

			req->length = xfer;
			ret = usb_ept_queue_xfer(ctxt->in, req);
			if (ret < 0) {
				DBG("msc_write: xfer error %d\n", ret);
				ctxt->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}


	if (req)
		req_put(ctxt, &ctxt->tx_idle, req);

#ifdef D_MSC_TRANSPORT_WRITE_WAIT_COMP
	if( r > 0 ) {
		do {
			unsigned long flags;

			req_q_cnt = 0;

			ctxt->sleep_flag = 0;
			spin_lock_irqsave(&ctxt->lock, flags);
			list_for_each_entry(req,&ctxt->tx_idle,list)
				req_q_cnt++;
			spin_unlock_irqrestore(&ctxt->lock, flags);

			DBG("msc_write: wait complete q_cnt=%d \n", req_q_cnt);

			if(req_q_cnt != TX_REQ_MAX) {
#if 0
				ret = interruptible_sleep_on_timeout( &ctxt->write_wq, HZ / 10 );
#else
				ret = wait_event_interruptible_timeout( ctxt->write_wq, 
											(ctxt->sleep_flag == 1), HZ / 10 );
#endif
				if (ret < 0) {
					r = ret;
					break;
				}
			}
			else {
				break;
			}

		} while (!ctxt->error);
	}
#endif /* D_MSC_TRANSPORT_WRITE_WAIT_COMP */
	_unlock(&ctxt->write_excl);
	return r;
}

static int msc_open(struct inode *ip, struct file *fp)
{
	struct msc_context *ctxt = &_context;

	DBG("msc_open\n");

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int msc_release(struct inode *ip, struct file *fp)
{
	struct msc_context *ctxt = &_context;

	DBG("msc_release\n");

	_unlock(&ctxt->open_excl);
	return 0;
}

static int msc_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	struct msc_context *ctxt = &_context;
	int ret = 0;
	unsigned long flags;
	struct usb_request *req_out;

	DBG("msc_ioctl(cmd=%d)\n", cmd);

	switch (cmd) {
	case USB_MSC_FUNC_IOC_SET_STALL: {
			int direction = arg; /* in:1 */
			DBG("msc_ioctl(USB_MSC_FUNC_IOC_SET_STALL)(dir=%d)\n", direction);
			spin_lock_irqsave(&ctxt->lock, flags);
			if( direction == 1 ) {
				usb_ept_set_halt(ctxt->in);
			}
			else {
				usb_ept_set_halt(ctxt->out);

				/* refresh out-direction buffer */
				usb_ept_fifo_flush(ctxt->out);

				/* if we have a stale request being read, recycle it */
				ctxt->read_buf = 0;
				ctxt->read_count = 0;
				if (ctxt->read_req) {
					req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
					ctxt->read_req = 0;
				}

				/* retire any completed rx requests from previous session */
				while ((req_out = req_get(ctxt, &ctxt->rx_done)))
					req_put(ctxt, &ctxt->rx_idle, req_out);
			}
			spin_unlock_irqrestore(&ctxt->lock, flags);
		}
		break;
	case USB_MSC_FUNC_IOC_GET_MOUNT_STS:
		ret = ctxt->mount_enb;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

/* s D_SHUSB_MSC_LUN_IF */
static ssize_t show_file(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t		rc;
	struct msc_context *ctxt = &_context;

	DBG("show_file\n");
	/* store mount enable or disable */
	if( ctxt->mount_enb == 1 ) {
		*buf = '1';
		rc = 1;
	}
	else {
		*buf = 0x00;
		rc = 0;
	}

	return rc;
}

static ssize_t store_file(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct msc_context *ctxt = &_context;

	DBG("store_file: \"%s\"\n", buf);
	
	if( count == 1 && *buf == 0x00 ){
		ctxt->mount_enb = 0;
		msc_control_notify(USB_MSC_NTY_MOUNT_DISABLE , NULL);
	}
	else {
		ctxt->mount_enb = 1;
		msc_control_notify(USB_MSC_NTY_MOUNT_ENABLE , NULL);
	}

	return  count;
}


static DEVICE_ATTR(file, 0444, show_file, store_file);
/* e D_SHUSB_MSC_LUN_IF */

static struct file_operations msc_fops = {
	.owner =   THIS_MODULE,
	.read =    msc_read,
	.write =   msc_write,
	.open =    msc_open,
	.release = msc_release,
	.ioctl =   msc_ioctl,
};

static struct miscdevice msc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msc_transport",
	.fops = &msc_fops,
};



static void msc_control_notify( unsigned char type,void* opt )
{

	struct msc_context *ctxt = &_context;

	if(!ctxt->ctrl_read_buf) {
		return;
	}

	DBG("msc_control_notify: \"%d\"\n", type);

	switch(type) {
	case USB_MSC_NTY_ONLINE_STATE:
		ctxt->ctrl_set_size = strlen(USB_ONLINE_STATE_CHANGE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_ONLINE_STATE_CHANGE_STR , ctxt->ctrl_set_size );
		break;
	case USB_MSC_NTY_OFFLINE_STATE:
		ctxt->ctrl_set_size = strlen(USB_OFFLINE_STATE_CHANGE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_OFFLINE_STATE_CHANGE_STR ,  ctxt->ctrl_set_size );
		break;
	case USB_MSC_NTY_MOUNT_ENABLE:
		ctxt->ctrl_set_size = strlen(USB_MOUNT_ENABLE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_MOUNT_ENABLE_STR ,  ctxt->ctrl_set_size );
		break;
	case USB_MSC_NTY_MOUNT_DISABLE:
		ctxt->ctrl_set_size = strlen(USB_MOUNT_DISABLE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_MOUNT_DISABLE_STR ,  ctxt->ctrl_set_size );
		break;
	case USB_MSC_NTY_RESET_STATE:
		ctxt->ctrl_set_size = strlen(USB_RESET_STATE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_RESET_STATE_STR ,  ctxt->ctrl_set_size );
		break;
		
	default:
		return;
	}

	wake_up(&ctxt->ctrl_read_wq);

	return;
}


static ssize_t msc_control_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	/* status notify */
	struct msc_context *ctxt = &_context;
	int size;
	int ret = 0;

	if (_lock(&ctxt->ctrl_read_excl))
		return -EBUSY;

	ret = wait_event_interruptible(ctxt->ctrl_read_wq, ctxt->ctrl_set_size );
	if (ret < 0) {
		_unlock(&ctxt->ctrl_read_excl);
		return ret;
	}

	if( count < ctxt->ctrl_set_size ) {
		_unlock(&ctxt->ctrl_read_excl);
		return -EFAULT;
	}

	DBG("msc_control_read: \"%s\"\n", ctxt->ctrl_read_buf);

	if (copy_to_user(buf, ctxt->ctrl_read_buf, ctxt->ctrl_set_size)) {
		size =  -EFAULT;
	}
	else {
		size = ctxt->ctrl_set_size;
		ctxt->ctrl_set_size = 0;
	}

	_unlock(&ctxt->ctrl_read_excl);

	return size;
}

static int msc_control_open(struct inode *ip, struct file *fp)
{
	struct msc_context *ctxt = &_context;

	if (_lock(&ctxt->open_ctrl_excl))
		return -EBUSY;

	ctxt->ctrl_set_size = 0;
	if( ctxt->ctrl_read_buf )
		kfree(ctxt->ctrl_read_buf);
	ctxt->ctrl_read_buf = kzalloc( USB_CTRL_NTY_MAX_STR_LEN, GFP_KERNEL);

	if( ctxt->on_mode )
		msc_control_notify(USB_MSC_NTY_ONLINE_STATE , NULL);
	else
		msc_control_notify(USB_MSC_NTY_OFFLINE_STATE , NULL);

	return 0;
}

static int msc_control_release(struct inode *ip, struct file *fp)
{
	struct msc_context *ctxt = &_context;

	_unlock(&ctxt->open_ctrl_excl);

	ctxt->ctrl_set_size = 0;
	if( ctxt->ctrl_read_buf ) {
		kfree(ctxt->ctrl_read_buf);
		ctxt->ctrl_read_buf = 0;
	}
	return 0;
}


static struct file_operations msc_control_fops = {
	.owner =   THIS_MODULE,
	.read =    msc_control_read,
	.open =    msc_control_open,
	.release = msc_control_release,
};

static struct miscdevice msc_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msc_control",
	.fops = &msc_control_fops,
};

static void msc_lun_release(struct kref *ref)
{
        struct msc_context  *ctxt = container_of(ref, struct msc_context, ref);

        kfree(ctxt->luns);
}

static void lun_release(struct device *dev)
{
        struct msc_context *ctxt = dev_get_drvdata(dev);

        kref_put(&ctxt->ref, msc_lun_release);
}

static void msc_unbind(void *_ctxt)
{
	struct msc_context *ctxt = _ctxt;
	struct usb_request *req;

	if (!ctxt->bound)
		return;


	while ((req = req_get(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = req_get(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}

	if (ctxt->in) {
		usb_ept_fifo_flush(ctxt->in);
		usb_ept_enable(ctxt->in,  0);
		usb_free_endpoint(ctxt->in);
	}
	if (ctxt->out) {
		usb_ept_fifo_flush(ctxt->out);
		usb_ept_enable(ctxt->out,  0);
		usb_free_endpoint(ctxt->out);
	}

	ctxt->online = 0;
	ctxt->error = 1;

	ctxt->on_mode = 0;
	schedule_work(&ctxt->sw_work);

	ctxt->mount_enb = 0;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	ctxt->bound = 0;
}

static void msc_bind(void *_ctxt)
{
	struct msc_context *ctxt = _ctxt;
	struct usb_request *req;
	int n;

	intf_desc.bInterfaceNumber =
		usb_msm_get_next_ifc_number(&usb_func_msc);
	intf_desc.iInterface =
		usb_msm_get_next_strdesc_id(USB_MSC_STRING_DESC_WORD);


	ctxt->in = usb_alloc_endpoint(USB_DIR_IN);
	if (ctxt->in) {
		hs_bulk_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->in->num;
		fs_bulk_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->in->num;
	}

	ctxt->out = usb_alloc_endpoint(USB_DIR_OUT);
	if (ctxt->out) {
		hs_bulk_out_desc.bEndpointAddress = USB_DIR_OUT|ctxt->out->num;
		fs_bulk_out_desc.bEndpointAddress = USB_DIR_OUT|ctxt->out->num;
	}

#if 1
	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, RXN_MAX);
#else
	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 4096);
#endif
		if (req == 0) {
			printk(KERN_ERR "msc_bind() :err usb_ept_alloc_req for out \n");
			ctxt->bound = 1;
			goto fail;
		}
		req->context = ctxt;
		req->complete = msc_complete_out;
		req_put(ctxt, &ctxt->rx_idle, req);
	}

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 4096);
		if (req == 0) {
			printk(KERN_ERR "msc_bind() :err usb_ept_alloc_req for in \n");
			ctxt->bound = 1;
			goto fail;
		}
		req->context = ctxt;
		req->complete = msc_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

#if 0
	dev_attr_file.attr.mode = 0644;

	rc = device_create_file(&curlun->dev, &dev_attr_file);
	if (rc != 0) {
		ERROR(fsg, "device_create_file failed: %d\n", rc);
		device_unregister(&curlun->dev);
		goto fail;
	}
#endif


	ctxt->bound = 1;
	return;

fail:
	printk(KERN_ERR "msc_bind() could not allocate requests\n");
	msc_unbind(ctxt);
}

static void msc_configure_switch(struct work_struct *w)
{
	struct msc_context *ctxt = container_of(w,
					    struct msc_context,
					    sw_work);

	if( ctxt->on_mode )
		msc_control_notify(USB_MSC_NTY_ONLINE_STATE , NULL);
	else
		msc_control_notify(USB_MSC_NTY_OFFLINE_STATE ,NULL);

	switch_set_state(&ctxt->sdev, ctxt->on_mode);
}

static void msc_configure(int configured, void *_ctxt)
{
	struct msc_context *ctxt = _ctxt;
	struct usb_request *req;

	if (configured) {
		ctxt->online = 1;

		ctxt->on_mode = 1;
		schedule_work(&ctxt->sw_work);

		if (usb_msm_get_speed() == USB_SPEED_HIGH) {
			usb_configure_endpoint(ctxt->in, &hs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &hs_bulk_out_desc);
		} else {
			usb_configure_endpoint(ctxt->in, &fs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &fs_bulk_out_desc);
		}
		usb_ept_enable(ctxt->in,  1);
		usb_ept_enable(ctxt->out, 1);

		/* if we have a stale request being read, recycle it */
		ctxt->read_buf = 0;
		ctxt->read_count = 0;
		if (ctxt->read_req) {
			req_put(ctxt, &ctxt->rx_idle, ctxt->read_req);
			ctxt->read_req = 0;
		}

		/* retire any completed rx requests from previous session */
		while ((req = req_get(ctxt, &ctxt->rx_done)))
			req_put(ctxt, &ctxt->rx_idle, req);

	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

#if 0
	schedule_work(&ctxt->sw_work);
#endif

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static int msc_setup(struct usb_ctrlrequest* req, void* buf, int len, void *_ctxt)
{
	int ret = -EOPNOTSUPP;

	if ((req->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		switch( req->bRequest ){
		case USB_MSC_GET_MAX_LUN_REQUEST:	/*  Request */
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_IN)
					|| req->wValue != 0 || req->wIndex != 0)
				break;
			((u8*)buf)[0] = 0;

			ret = 1;
			break;
		case USB_MSC_RESET_REQUEST:		/* reset request */
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_OUT))
				break;
			msc_control_notify(USB_MSC_NTY_RESET_STATE , NULL);
			ret = 0;
			break;
		default:
			break;
		}
	}
	else if ((req->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		/* Vendor Request */
	}

	return ret;
}

void msc_disconnect(void *_ctxt)
{
	struct msc_context *ctxt = _ctxt;

	ctxt->on_mode = 0;
	schedule_work(&ctxt->sw_work);
	return;
}


static struct usb_function usb_func_msc = {
	.bind = msc_bind,
	.unbind = msc_unbind,
	.configure = msc_configure,
	.setup = msc_setup,
	.disconnect = msc_disconnect,

	.name = MSC_FUNCTION_NAME,
	.context = &_context,

};


static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	struct msc_context  *ctxt = container_of(sdev, struct msc_context, sdev);
	return sprintf(buf, "%s\n", (ctxt->on_mode ? "online" : "offline"));
}

#if 0
static int msc_set_interface(int ifc_num, int alt_set, void *_ctxt)
{
	struct msc_context *ctxt = _ctxt;

	if (ctxt == NULL) {
		return 1;
	}

	if ((ifc_num == intf_desc.bInterfaceNumber)
			&& (alt_set == 0)) {
		return 0;
	}

	return 1;
}
#endif

static int msc_get_interface(int ifc_num, void *_ctxt)
{
	return 0;
}

struct usb_descriptor_header *msc_hs_descriptors[5];
struct usb_descriptor_header *msc_fs_descriptors[5];

static int __init msc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct msc_context *ctxt = &_context;

        int                     rc = 0;
        int                     i;
        struct lun              *curlun;

	DBG("msc_init()\n");

	ctxt->pdev = pdev;
	ctxt->nluns = 1;

	ctxt->read_count = 0;

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
	init_waitqueue_head(&ctxt->write_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
#if 0
	/* we don't use enable_device for msc. */
	atomic_set(&ctxt->enable_excl, 0);
#endif

	/* for control */
	atomic_set(&ctxt->open_ctrl_excl, 0);
	atomic_set(&ctxt->ctrl_read_excl, 0);
	init_waitqueue_head(&ctxt->ctrl_read_wq);
	

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	INIT_WORK(&ctxt->sw_work, msc_configure_switch);

	msc_hs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
	msc_hs_descriptors[1] =
		(struct usb_descriptor_header *)&hs_bulk_in_desc;
	msc_hs_descriptors[2] =
		(struct usb_descriptor_header *)&hs_bulk_out_desc;
	msc_hs_descriptors[3] = NULL;

	msc_fs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
	msc_fs_descriptors[1] =
		(struct usb_descriptor_header *)&fs_bulk_in_desc;
	msc_fs_descriptors[2] =
		(struct usb_descriptor_header *)&fs_bulk_out_desc;
	msc_fs_descriptors[3] = NULL;

	usb_func_msc.hs_descriptors = msc_hs_descriptors;
	usb_func_msc.fs_descriptors = msc_fs_descriptors;
#if 0
	usb_func_msc.set_interface = msc_set_interface;
#endif
	usb_func_msc.get_interface = msc_get_interface;

	ret = misc_register(&msc_device);
	if (ret) {
		printk(KERN_ERR "msc Can't register misc device  %d \n",
						MISC_DYNAMIC_MINOR);
		return ret;
	}

	ret = misc_register(&msc_control_device);
	if (ret) {
		printk(KERN_ERR "msc Can't register misc device  %d \n",
						MISC_DYNAMIC_MINOR);
		misc_deregister(&msc_device);
		return ret;
	}

	ret = usb_function_register(&usb_func_msc);
	if (ret) {
		misc_deregister(&msc_device);
		misc_deregister(&msc_control_device);
		return ret;
	}

	ctxt->sdev.name = DRIVER_NAME;
	ctxt->sdev.print_name = print_switch_name;
	ctxt->sdev.print_state = print_switch_state;
	ret = switch_dev_register(&ctxt->sdev);
	if (ret < 0){
//		return retval;
		goto fail_probe2;
	}


        dev_attr_file.attr.mode = 0644;

        /* Find out how many LUNs there should be */
        i = ctxt->nluns;
        if (i == 0)
                i = 1;
        if (i > MAX_LUNS) {
#if 0
                ERROR(ctxt->nluns, "invalid number of LUNs: %d\n", i);
#endif
                rc = -EINVAL;
                goto fail_probe;
        }

        /* initialize ref before Create the LUNs */
        kref_init(&ctxt->ref);

        /* Create the LUNs, open their backing files, and register the
         * LUN devices in sysfs. */
        ctxt->luns = kzalloc(i * sizeof(struct lun), GFP_KERNEL);
        if (!ctxt->luns) {
                rc = -ENOMEM;
                goto fail_probe;
        }
        ctxt->nluns = i;

        for (i = 0; i < ctxt->nluns; ++i) {
                curlun = &ctxt->luns[i];
                curlun->ro = 0;
                curlun->dev.release = lun_release;
                curlun->dev.parent = &ctxt->pdev->dev;
                dev_set_drvdata(&curlun->dev, ctxt);
                snprintf(curlun->dev.bus_id, BUS_ID_SIZE,
                                "lun%d", i);

                rc = device_register(&curlun->dev);
                if (rc != 0) {
#if 0
                        INFO(ctxt->nluns, "failed to register LUN%d: %d\n", i, rc);
#endif
                        goto fail_probe;
                }
                rc = device_create_file(&curlun->dev, &dev_attr_file);
                if (rc != 0) {
#if 0
                        ERROR(ctxt->nluns, "device_create_file failed: %d\n", rc);
#endif
                        device_unregister(&curlun->dev);
                        goto fail_probe;
                }
                curlun->registered = 1;
                kref_get(&ctxt->ref);
        }
	return ret;
fail_probe:
	switch_dev_unregister(&ctxt->sdev);
fail_probe2:
	misc_deregister(&msc_device);
	misc_deregister(&msc_control_device);
	return rc;

}

static int __exit msc_remove(struct platform_device *pdev)
{
	struct msc_context *ctxt = &_context;
	int i;
	struct lun *curlun;

	misc_deregister(&msc_device);
	misc_deregister(&msc_control_device);
	switch_dev_unregister(&ctxt->sdev);
	usb_function_unregister(&usb_func_msc);

        /* Unregister the sysfs attribute files and the LUNs */
        for (i = 0; i < ctxt->nluns; ++i) {
                curlun = &ctxt->luns[i];
                if (curlun->registered) {
                        device_remove_file(&curlun->dev, &dev_attr_file);
                        device_unregister(&curlun->dev);
                        curlun->registered = 0;
                }
        }

	kref_put(&ctxt->ref, msc_lun_release);

	return 0;
}



static struct platform_driver msc_pf_driver = {
	.probe = msc_probe,
	.remove = __exit_p(msc_remove),
	.driver = { .name = DRIVER_NAME, },
};

static int __init msc_transport_init(void)
{
	return platform_driver_register(&msc_pf_driver);
}
module_init(msc_transport_init);


static void __exit msc_transport_exit(void)
{
	platform_driver_unregister(&msc_pf_driver);
}
module_exit(msc_transport_exit);

