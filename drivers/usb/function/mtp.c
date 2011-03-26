/* drivers/usb/function/mtp.c
 *
 * Function Driver for USB Media Transfer Protocol
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
#define USB_MTP_IOC_MAGIC 0xFF

#define USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET _IOW(USB_MTP_IOC_MAGIC, 0x20, int)
#define USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET _IOW(USB_MTP_IOC_MAGIC, 0x21, int)
#define USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET _IOW(USB_MTP_IOC_MAGIC, 0x22, int)
#define USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET _IOW(USB_MTP_IOC_MAGIC, 0x23, int)
#define USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET _IOW(USB_MTP_IOC_MAGIC, 0x24, int)
#define USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET _IOW(USB_MTP_IOC_MAGIC, 0x25, int)
#define USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET _IOW(USB_MTP_IOC_MAGIC, 0x26, int)
#define USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET _IOW(USB_MTP_IOC_MAGIC, 0x27, int)

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(x)
#endif

/* Defines */
#define DRIVER_NAME			"switch_mtp"

#define TXN_MAX 4096


/* MTP setup class requests */
#define	USB_MTP_CANCEL_REQUEST				(0x64)
#define	USB_MTP_GET_EXTENDED_DATA			(0x65)
#define	USB_MTP_DEVICE_RESET_REQUEST		(0x66)
#define	USB_MTP_GET_DEVICE_STATUS			(0x67)

#define	USB_MTP_NTY_CANCEL_REQUEST				(1)
#define	USB_MTP_NTY_GET_EXTENDED_DATA			(2)
#define	USB_MTP_NTY_DEVICE_RESET_REQUEST		(3)
#define	USB_MTP_NTY_GET_DEVICE_STATUS			(4)
#define	USB_MTP_NTY_ONLINE_STATE				(5)
#define	USB_MTP_NTY_OFFLINE_STATE				(6)

#define	USB_CTRL_NTY_MAX_STR_LEN	(48)
#define	USB_CANCEL_REQUEST_STR		"Cancel_Request"
#define	USB_GET_DEVICE_STATUS_STR	"Get_Device_Status"
#define	USB_DEVICE_RESET_STR		"Device_Reset"

#define	USB_ONLINE_STATE_CHANGE_STR		"online"
#define	USB_OFFLINE_STATE_CHANGE_STR	"offline"

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4
#define NOTIFY_REQ_MAX 1

#define MTP_FUNCTION_NAME "mtp"

#define USB_DIR_MASK	USB_DIR_IN

struct mtp_context
{
	int online;
	int on_mode;
	int error;
	int cancel_request;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
#if 0
	/* we don't use enable_device for mtp. */
	atomic_t enable_excl;
#endif
	spinlock_t lock;

	struct usb_endpoint *out;
	struct usb_endpoint *in;
	struct usb_endpoint *notify;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;
	struct list_head notify_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;
	unsigned bound;
	struct work_struct switch_notify;
	struct switch_dev sdev;

	/* for_control */
	atomic_t ctrl_read_excl;
	atomic_t open_ctrl_excl;
	wait_queue_head_t ctrl_read_wq;
	unsigned char ctrl_set_size;
	unsigned char *ctrl_read_buf;

};

static struct mtp_context _context;

static struct usb_interface_descriptor intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_STILL_IMAGE,
	.bInterfaceSubClass =	0x01,
	.bInterfaceProtocol =	0x01,
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

static struct usb_endpoint_descriptor hs_notify_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		16,
};
static struct usb_endpoint_descriptor fs_notify_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval =		32,
};

static struct usb_function usb_func_mtp;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sharp Corporation");

static void mtp_control_notify( unsigned char type,void* opt );

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


static void mtp_notify(struct work_struct *w)
{
	struct mtp_context *ctxt = container_of(w, struct mtp_context, switch_notify);

	if( ctxt->on_mode )
		mtp_control_notify(USB_MTP_NTY_ONLINE_STATE , NULL);
	else
		mtp_control_notify(USB_MTP_NTY_OFFLINE_STATE ,NULL);


	switch_set_state(&ctxt->sdev, ctxt->on_mode);
}

/* add a request to the tail of a list */
static void req_put(struct mtp_context *ctxt, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct mtp_context *ctxt, struct list_head *head)
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

static void mtp_complete_in(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->tx_idle, req);

	wake_up(&ctxt->write_wq);
}

static void mtp_complete_out(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_context *ctxt = req->context;

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
	} else {
		ctxt->cancel_request = 0;
		req_put(ctxt, &ctxt->rx_done, req);
	}

	wake_up(&ctxt->read_wq);
}

static void mtp_complete_notify(struct usb_endpoint *ept, struct usb_request *req)
{
	struct mtp_context *ctxt = req->context;

	if (req->status != 0)
		ctxt->error = 1;

	req_put(ctxt, &ctxt->notify_idle, req);
/*
	wake_up(&ctxt->write_wq);
*/
}

static ssize_t mtp_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_context *ctxt = &_context;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	int	timeout_onflg = 0;	/* timeout-flg */


	DBG("mtp_read(%d)\n", count);

	if (_lock(&ctxt->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(ctxt->online || ctxt->error)) {
		DBG("mtp_read: waiting for online state\n");
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0) {
			_unlock(&ctxt->read_excl);
			return ret;
		}
	}

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
			if(req->length > TXN_MAX)
				req->length = TXN_MAX;
#else
			req->length = TXN_MAX;
#endif
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("mtp_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				req_put(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("rx %p queue\n", req);
			}
		}

		/* if we have data pending, give it to userspace */
		if (ctxt->read_count > 0) {
			xfer = (ctxt->read_count < count) ? ctxt->read_count : count;

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
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		if( timeout_onflg == 0 )
			ret = wait_event_interruptible(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error));
		else
			ret = wait_event_interruptible_timeout(ctxt->read_wq,
					       ((req = req_get(ctxt, &ctxt->rx_done)) || ctxt->error), HZ / 10 );

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
	_unlock(&ctxt->read_excl);
	return r;
}

static ssize_t mtp_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct mtp_context *ctxt = &_context;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG("mtp_write(%d)\n", count);

	if (_lock(&ctxt->write_excl))
		return -EBUSY;

	while (count > 0) {
		if (ctxt->cancel_request) {
			r = -EAGAIN;
			break;
		}
		if (ctxt->error) {
			r = -EIO;
			break;
		}


		/* get an idle tx request to use */
		req = 0;
#if 1
		ret = wait_event_interruptible(ctxt->write_wq,
							((req = req_get(ctxt, &ctxt->tx_idle)) 
							|| ctxt->error || ctxt->cancel_request));
#else
		ret = wait_event_interruptible(ctxt->write_wq,
					       ((req = req_get(ctxt, &ctxt->tx_idle)) || ctxt->error));
#endif

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
				DBG("mtp_write: xfer error %d\n", ret);
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

	_unlock(&ctxt->write_excl);

	DBG("mtp_write return=%d\n", r);

	return r;
}

static int mtp_open(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int mtp_release(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;

	_unlock(&ctxt->open_excl);
	return 0;
}

static struct file_operations mtp_fops = {
	.owner =   THIS_MODULE,
	.read =    mtp_read,
	.write =   mtp_write,
	.open =    mtp_open,
	.release = mtp_release,
};

static struct miscdevice mtp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp",
	.fops = &mtp_fops,
};

#if 0
/* we don't use enable_device for mtp. */
static int mtp_enable_open(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;

	if (_lock(&ctxt->enable_excl))
		return -EBUSY;

	printk(KERN_INFO "enabling mtp function\n");
	usb_function_enable(MTP_FUNCTION_NAME, 1);
	/* clear the error latch */
	ctxt->error = 0;

	return 0;
}

static int mtp_enable_release(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;

	printk(KERN_INFO "disabling mtp function\n");
	usb_function_enable(MTP_FUNCTION_NAME, 0);
	_unlock(&ctxt->enable_excl);
	return 0;
}
#endif

#if 0
/* we don't use enable_device for mtp. */
static struct file_operations mtp_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    mtp_enable_open,
	.release = mtp_enable_release,
};

static struct miscdevice mtp_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_enable",
	.fops = &mtp_enable_fops,
};
#endif


static void mtp_control_notify( unsigned char type,void* opt )
{

	struct mtp_context *ctxt = &_context;

	if(!ctxt->ctrl_read_buf) {
		return;
	}

	switch(type) {
	case USB_MTP_NTY_CANCEL_REQUEST:
		ctxt->ctrl_set_size = strlen(USB_CANCEL_REQUEST_STR);
		memcpy(ctxt->ctrl_read_buf, USB_CANCEL_REQUEST_STR , ctxt->ctrl_set_size );
		break;
	case USB_MTP_NTY_GET_DEVICE_STATUS:
		ctxt->ctrl_set_size = strlen(USB_GET_DEVICE_STATUS_STR);
		memcpy(ctxt->ctrl_read_buf, USB_GET_DEVICE_STATUS_STR , ctxt->ctrl_set_size );
		break;
	case USB_MTP_NTY_DEVICE_RESET_REQUEST:
		ctxt->ctrl_set_size = strlen(USB_DEVICE_RESET_STR);
		memcpy(ctxt->ctrl_read_buf, USB_DEVICE_RESET_STR , ctxt->ctrl_set_size );
		break;
	case USB_MTP_NTY_ONLINE_STATE:
		ctxt->ctrl_set_size = strlen(USB_ONLINE_STATE_CHANGE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_ONLINE_STATE_CHANGE_STR , ctxt->ctrl_set_size );
		break;
	case USB_MTP_NTY_OFFLINE_STATE:
		ctxt->ctrl_set_size = strlen(USB_OFFLINE_STATE_CHANGE_STR);
		memcpy(ctxt->ctrl_read_buf, USB_OFFLINE_STATE_CHANGE_STR ,  ctxt->ctrl_set_size );
		break;
	default:
		return;
	}

	wake_up(&ctxt->ctrl_read_wq);

	return;
}

static ssize_t mtp_control_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct mtp_context *ctxt = &_context;
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
		return -1;
	}

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

static int mtp_control_open(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;


	if (_lock(&ctxt->open_ctrl_excl))
		return -EBUSY;

	ctxt->ctrl_set_size = 0;
	if( ctxt->ctrl_read_buf )
		kfree(ctxt->ctrl_read_buf);
	ctxt->ctrl_read_buf = kzalloc( USB_CTRL_NTY_MAX_STR_LEN, GFP_KERNEL);

	if( ctxt->on_mode )
		mtp_control_notify(USB_MTP_NTY_ONLINE_STATE , NULL);
	else
		mtp_control_notify(USB_MTP_NTY_OFFLINE_STATE , NULL);

	return 0;
}

static int mtp_control_release(struct inode *ip, struct file *fp)
{
	struct mtp_context *ctxt = &_context;

	_unlock(&ctxt->open_ctrl_excl);

	ctxt->ctrl_set_size = 0;
	if( ctxt->ctrl_read_buf ) {
		kfree(ctxt->ctrl_read_buf);
		ctxt->ctrl_read_buf = 0;
	}
	return 0;
}

static int mtp_control_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct mtp_context *ctxt = &_context;
	
	unsigned short sts[2];

	switch (cmd) {
	case USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_CANCEL_REQUEST_SET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_CANCEL_REQUEST_GET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_SET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_GET_EXTENDED_EVENT_DATA_GET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_SET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_DEVICE_RESET_REQUEST_GET\n");
		ret = -EOPNOTSUPP;
		break;

	case USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_SET\n");

		sts[0] = 0x04;					/* length */
		sts[1] = (unsigned short)arg;	/* code */
		/*	any parameter x is not send (pending)	*/

		DBG("mtp_control_ioctl %d \n",sts[1]);

		send_setup0_in_response(sts, sts[0]);

		if ( sts[1] == 0x2001 ) {
			DBG("mtp_control_ioctl:reset cancel_request \n");
			if (atomic_read(&ctxt->read_excl)) {
				int ret_xfer;
				struct usb_request *req = 0;

				req = req_get(ctxt, &ctxt->rx_idle);
				if (req) {
					req->length = usb_ept_get_max_packet(ctxt->out);
					if(req->length > TXN_MAX)
						req->length = TXN_MAX;
					ret_xfer = usb_ept_queue_xfer(ctxt->out, req);
					if (ret_xfer < 0) {
						ctxt->error = 1;
						req_put(ctxt, &ctxt->rx_idle, req);
					}
				}
			}
		}
		break;

	case USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET:
		DBG("mtp_control_ioctl USB_MTP_FUNC_IOC_GET_DEVICE_STATUS_GET\n");
		ret = -EOPNOTSUPP;
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}

static struct file_operations mtp_control_fops = {
	.owner =   THIS_MODULE,
	.read =    mtp_control_read,
	.open =    mtp_control_open,
	.release = mtp_control_release,
	.ioctl =   mtp_control_ioctl,
};

static struct miscdevice mtp_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_control",
	.fops = &mtp_control_fops,
};

static void mtp_unbind(void *_ctxt)
{
	struct mtp_context *ctxt = _ctxt;
	struct usb_request *req;

	if (!ctxt->bound)
		return;

	while ((req = req_get(ctxt, &ctxt->rx_idle))) {
		usb_ept_free_req(ctxt->out, req);
	}
	while ((req = req_get(ctxt, &ctxt->tx_idle))) {
		usb_ept_free_req(ctxt->in, req);
	}
	while ((req = req_get(ctxt, &ctxt->notify_idle))) {
		usb_ept_free_req(ctxt->notify, req);
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
	if (ctxt->notify) {
		usb_ept_fifo_flush(ctxt->notify);
		usb_ept_enable(ctxt->notify,  0);
		usb_free_endpoint(ctxt->notify);
	}
	ctxt->online = 0;
	ctxt->error = 1;

	ctxt->on_mode = 0;
	schedule_work(&ctxt->switch_notify);

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
	ctxt->bound = 0;
}

static void mtp_bind(void *_ctxt)
{
	struct mtp_context *ctxt = _ctxt;
	struct usb_request *req;
	int n;

	intf_desc.bInterfaceNumber =
		usb_msm_get_next_ifc_number(&usb_func_mtp);

	intf_desc.iInterface =
		usb_msm_get_next_strdesc_id(USB_MTP_STRING_DESC_WORD);

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

	ctxt->notify = usb_alloc_endpoint(USB_DIR_IN);
	if (ctxt->notify) {
		hs_notify_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->notify->num;
		fs_notify_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->notify->num;
	}


	for (n = 0; n < RX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 4096);
		if (req == 0) {
			printk(KERN_ERR "mtp_bind() :err usb_ept_alloc_req for out \n");
			ctxt->bound = 1;
			goto fail;
		}
		req->context = ctxt;
		req->complete = mtp_complete_out;
		req_put(ctxt, &ctxt->rx_idle, req);
	}

	for (n = 0; n < TX_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 4096);
		if (req == 0) {
			printk(KERN_ERR "mtp_bind() :err usb_ept_alloc_req for in \n");
			ctxt->bound = 1;
			goto fail;
		}
		req->context = ctxt;
		req->complete = mtp_complete_in;
		req_put(ctxt, &ctxt->tx_idle, req);
	}

	for (n = 0; n < NOTIFY_REQ_MAX; n++) {
		req = usb_ept_alloc_req(ctxt->notify, 4096);
		if (req == 0) {
			printk(KERN_ERR "mtp_bind() :err usb_ept_alloc_req for notify \n");
			ctxt->bound = 1;
			goto fail;
		}
		req->context = ctxt;
		req->complete = mtp_complete_notify;
		req_put(ctxt, &ctxt->notify_idle, req);
	}


	ctxt->bound = 1;
	return;

fail:
	printk(KERN_ERR "mtp_bind() could not allocate requests\n");
	mtp_unbind(ctxt);
}

static void mtp_configure(int configured, void *_ctxt)
{
	struct mtp_context *ctxt = _ctxt;
	struct usb_request *req;

	if (configured) {
		ctxt->online = 1;
		ctxt->cancel_request = 0;

		ctxt->error = 0;	/* reset error flag */

		ctxt->on_mode = 1;
		schedule_work(&ctxt->switch_notify);

		if (usb_msm_get_speed() == USB_SPEED_HIGH) {
			usb_configure_endpoint(ctxt->in, &hs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &hs_bulk_out_desc);
			usb_configure_endpoint(ctxt->notify, &hs_notify_in_desc);
		} else {
			usb_configure_endpoint(ctxt->in, &fs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &fs_bulk_out_desc);
			usb_configure_endpoint(ctxt->notify, &fs_notify_in_desc);
		}
		usb_ept_enable(ctxt->in,  1);
		usb_ept_enable(ctxt->out, 1);
		usb_ept_enable(ctxt->notify, 1);

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
#if 0 /* deconfigured is not err. */
		ctxt->error = 1;
#endif
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_wq);
}

static int mtp_setup(struct usb_ctrlrequest* req, void* buf, int len, void *_ctxt)
{
	int ret = -EOPNOTSUPP;
	struct mtp_context *ctxt = _ctxt;
	struct usb_request *req_out;

	if ((req->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		switch( req->bRequest ){
		case USB_MTP_CANCEL_REQUEST:		/* Cancel Request */
		DBG("mtp_setup USB_MTP_CANCEL_REQUEST\n");
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_OUT))
				break;
			mtp_control_notify(USB_MTP_NTY_CANCEL_REQUEST,NULL);


			/* flush out-transaction */
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

			ctxt->cancel_request = 1;
			usb_ept_fifo_flush(ctxt->in);
			wake_up(&ctxt->write_wq);

			ctxt->error = 0; 

			ret = 6;
			break;
		case USB_MTP_GET_EXTENDED_DATA:		/* Get Extended Data */
		DBG("mtp_setup USB_MTP_GET_EXTENDED_DATA\n");
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_IN))
				break;
			break;
		case USB_MTP_DEVICE_RESET_REQUEST:	/* Device Reset Request */
		DBG("mtp_setup USB_MTP_DEVICE_RESET_REQUEST\n");
		#if 0
			/* because windows send in-direction,skip checking direction  */
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_OUT))
				break;
		#endif
			mtp_control_notify(USB_MTP_NTY_DEVICE_RESET_REQUEST,NULL);
			ret = 0;
			break;
		case USB_MTP_GET_DEVICE_STATUS:		/* Get Device Status */
			DBG("mtp_setup USB_MTP_GET_DEVICE_STATUS\n");
			if ((req->bRequestType & (USB_DIR_MASK)) != (USB_DIR_IN))
				break;
			DBG("mtp_setup USB_MTP_GET_DEVICE_STATUS OK\n");
			mtp_control_notify(USB_MTP_NTY_GET_DEVICE_STATUS,NULL);
#if 1
			ret = USB_SETUP0_WAIT_SENDING_DATA_RES;
#endif
			break;
		default:
		DBG("mtp_setup other\n");
			break;
		}
	}
	else if ((req->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		/* Vendor Request */
	}

	return ret;
}

void mtp_disconnect(void *_ctxt)
{
	struct mtp_context *ctxt = _ctxt;

	ctxt->on_mode = 0;
	schedule_work(&ctxt->switch_notify);
	return;
}


static struct usb_function usb_func_mtp = {
	.bind = mtp_bind,
	.unbind = mtp_unbind,
	.configure = mtp_configure,
	.setup = mtp_setup,
	.disconnect = mtp_disconnect,

	.name = MTP_FUNCTION_NAME,
	.context = &_context,

};


#if 1
static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	struct mtp_context  *ctxt = container_of(sdev, struct mtp_context, sdev);
	return sprintf(buf, "%s\n", (ctxt->on_mode ? "online" : "offline"));
}
#endif

static int mtp_get_interface(int ifc_num, void *_ctxt)
{
        return 0;
}

struct usb_descriptor_header *mtp_hs_descriptors[5];
struct usb_descriptor_header *mtp_fs_descriptors[5];
static int __init mtp_init(void)
{
	int ret = 0;
	struct mtp_context *ctxt = &_context;
	DBG("mtp_init()\n");

	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
	init_waitqueue_head(&ctxt->ctrl_read_wq); /* for control */

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
#if 0
	/* we don't use enable_device for mtp. */
	atomic_set(&ctxt->enable_excl, 0);
#endif
	atomic_set(&ctxt->ctrl_read_excl, 0); /* for control */

	spin_lock_init(&ctxt->lock);

	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	INIT_LIST_HEAD(&ctxt->notify_idle);

	INIT_WORK(&ctxt->switch_notify, mtp_notify);

	mtp_hs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
	mtp_hs_descriptors[1] =
		(struct usb_descriptor_header *)&hs_bulk_in_desc;
	mtp_hs_descriptors[2] =
		(struct usb_descriptor_header *)&hs_bulk_out_desc;
	mtp_hs_descriptors[3] =
		(struct usb_descriptor_header *)&hs_notify_in_desc;
	mtp_hs_descriptors[4] = NULL;

	mtp_fs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
	mtp_fs_descriptors[1] =
		(struct usb_descriptor_header *)&fs_bulk_in_desc;
	mtp_fs_descriptors[2] =
		(struct usb_descriptor_header *)&fs_bulk_out_desc;
	mtp_fs_descriptors[3] =
		(struct usb_descriptor_header *)&fs_notify_in_desc;
	mtp_fs_descriptors[4] = NULL;

	usb_func_mtp.hs_descriptors = mtp_hs_descriptors;
	usb_func_mtp.fs_descriptors = mtp_fs_descriptors;
	usb_func_mtp.get_interface = mtp_get_interface;

	ret = misc_register(&mtp_device);
	if (ret) {
		printk(KERN_ERR "mtp Can't register misc device  %d \n",
						MISC_DYNAMIC_MINOR);
		return ret;
	}

	ret = misc_register(&mtp_control_device);
	if (ret) {
		printk(KERN_ERR "mtp Can't register misc device  %d \n",
						MISC_DYNAMIC_MINOR);
		misc_deregister(&mtp_device);
		return ret;
	}

#if 0
	/* we don't use enable_device for mtp. */
	ret = misc_register(&mtp_enable_device);
	if (ret) {
		printk(KERN_ERR "mtp Can't register misc enable device  %d \n",
						MISC_DYNAMIC_MINOR);
		misc_deregister(&mtp_device);
		misc_deregister(&mtp_control_device);
		return ret;
	}
#endif

	ret = usb_function_register(&usb_func_mtp);
	if (ret) {
		misc_deregister(&mtp_device);
		misc_deregister(&mtp_control_device);
#if 0
		/* we don't use enable_device for mtp. */
		misc_deregister(&mtp_enable_device);
#endif
	}

#if 1
	ctxt->sdev.name = DRIVER_NAME;
	ctxt->sdev.print_name = print_switch_name;
	ctxt->sdev.print_state = print_switch_state;
	ret = switch_dev_register(&ctxt->sdev);
	if (ret < 0){
//		return retval;
	}
#endif


	return ret;
}

module_init(mtp_init);

static void __exit mtp_exit(void)
{
	misc_deregister(&mtp_device);
	misc_deregister(&mtp_control_device);
#if 0
	/* we don't use enable_device for mtp. */
	misc_deregister(&mtp_enable_device);
#endif
	usb_function_unregister(&usb_func_mtp);
}
module_exit(mtp_exit);
