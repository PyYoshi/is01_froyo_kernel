/* drivers/usb/function/obex.c
 *
 * Function Driver for Object Exchange Protocol
 *
 * Copyright (c) 2010 Sharp Corporation.
 *
 * This code also borrows from serial.c, which is
 * Copyright 2003 (C) Al Borchers (alborchers@steinerpoint.com)
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2009 SHARP Inc. All rights reserved.
 *
 * This code is based in part on the Gadget Zero driver, which
 * is Copyright (C) 2003 by David Brownell, all rights reserved.
 *
 * This code also borrows from usbserial.c, which is
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * Copyright (C) 2000 Al Borchers (alborchers@steinerpoint.com)
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/wait.h>
#include <linux/serial.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/switch.h>

#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/unaligned.h>

#include <linux/usb/cdc.h>
#include "usb_function.h"
#include <mach/msm_hsusb_desc.h>

#include <linux/workqueue.h>
/* Defines */
#define SWITCH_NAME		"switch_obex"
#define DRIVER_NAME		"obex"

#define SWITCH_NAME_ALT	"obex_open_sts"

#define OBEX_VERSION_STR		"v0.0"

#define OBEX_LONG_NAME			"obex Function"
#define OBEX_SHORT_NAME			"obex"

static int obex_instances = 1;
#define MAX_INSTANCES 1

#define OBEX_MAJOR			0
#define OBEX_MINOR_START		0

#define OBEX_NUM_PORTS			1

#define OBEX_NO_CONFIG_ID		0
#define OBEX_ACM_CONFIG_ID		2

#define OBEX_MAX_DESC_LEN		256

/* defines for maintaining serial states */
#define	MSR_CTS		(1 << 4)
#define	MSR_DSR		(1 << 5)
#define	MSR_RI		(1 << 6)
#define	MSR_CD		(1 << 7)
#define	MCR_DTR		(1 << 0)
#define	MCR_RTS		(1 << 1)
#define	MCR_LOOP	(1 << 4)

/* USB CDC control line state defines */
#define USB_CDC_SET_CONTROL_LINE_STATE_DTR 0x1
#define USB_CDC_SET_CONTROL_LINE_STATE_RTS 0x2

#define OBEX_DEFAULT_READ_Q_SIZE	16
#define OBEX_DEFAULT_WRITE_Q_SIZE	16
#define OBEX_DEFAULT_INT_REQ		1

#define OBEX_DEFAULT_WRITE_BUF_SIZE	8192
#define OBEX_TMP_BUF_SIZE		8192

#define OBEX_CLOSE_TIMEOUT		15

#define OBEX_DEFAULT_USE_ACM		0

#define OBEX_DEFAULT_DTE_RATE		9600
#define OBEX_DEFAULT_DATA_BITS		8
#define OBEX_DEFAULT_PARITY		USB_CDC_NO_PARITY
#define OBEX_DEFAULT_CHAR_FORMAT	USB_CDC_1_STOP_BITS

/* #define OBEX_DEBUG */

/* debug settings */
#ifdef OBEX_DEBUG
static int debug = 1;

#define obex_debug(format, arg...) \
	do { if (debug) printk(KERN_DEBUG format, ## arg); } while (0)
#define obex_debug_level(level, format, arg...) \
	do { if (debug >= level) printk(KERN_DEBUG format, ## arg); } while (0)

#else

#define obex_debug(format, arg...) \
	do { } while (0)
#define obex_debug_level(level, format, arg...) \
	do { } while (0)

#endif /* OBEX_DEBUG */

#define OBEX_LOG2_NOTIFY_INTERVAL	5	/* 1 << 5 == 32 msec */
#define OBEX_NOTIFY_MAXPACKET		8
#define OBEX_CONFIGURED			1
#define OBEX_UNCONFIGURED		0

/* Structures */

struct obex_dev;

/* circular buffer */
struct obex_buf {
	unsigned int buf_size;
	char *buf_buf;
	char *buf_get;
	char *buf_put;
};

/* list of requests */
struct obex_req_entry {
	struct list_head re_entry;
	struct usb_request *re_req;
};

/* the port structure holds info for each port, one for each minor number */
struct obex_port {
	struct obex_dev *port_dev;	/* pointer to device struct */
	struct tty_struct *port_tty;/* pointer to tty struct */
	spinlock_t port_lock;
	int port_num;
	int port_open_count;
	int port_in_use;			/* open/close in progress */
	wait_queue_head_t port_write_wait;	/* waiting to write */
	struct obex_buf *port_write_buf;
	struct usb_cdc_line_coding port_line_coding;
	struct list_head	read_pool;
	struct list_head	read_queue;
	struct list_head	write_pool;
	unsigned            n_read;
	unsigned int msr;
	unsigned int prev_msr;
	unsigned int mcr;
	wait_queue_head_t msr_change_wait;
	struct work_struct push_work;
};

/*-------------------------------------------------------------*/
/*Allocate DMA buffer in non interrupt context(obex_bind)*/

struct obex_reqbuf {
	void *buf;
};

/*-------------------------------------------------------------*/

/* the device structure holds info for the USB device */
struct obex_dev {
	/* lock for set/reset config */
	spinlock_t dev_lock;
	/* configuration number */
	int dev_config;
	/* address of in endpoint */
	struct usb_endpoint *dev_in_ep;
	/* address of out endpoint */
	struct usb_endpoint *dev_out_ep;
	/* list of write requests */
	struct list_head dev_req_list;
	/* round robin port scheduled */
	int dev_sched_port;
	struct obex_port *dev_port[OBEX_NUM_PORTS];	/* the ports */
	u16 interface_num;

	/*interface, endpoint descriptors*/
	struct usb_interface_descriptor obex_ifc_desc;
	struct usb_cdc_header_desc obex_cdc_header_desc;
	struct usb_cdc_obex_desc obex_cdc_obex_desc;
	struct usb_cdc_union_desc obex_cdc_union_ifc_desc;
	struct usb_interface_descriptor obex_data_zero_ifc_desc;
	struct usb_interface_descriptor obex_data_one_ifc_desc;
	struct usb_endpoint_descriptor obex_hs_bulkin_desc, obex_fs_bulkin_desc;
	struct usb_endpoint_descriptor obex_hs_bulkout_desc, obex_fs_bulkout_desc;
	struct usb_descriptor_header **obex_fullspeed_header;
	struct usb_descriptor_header **obex_highspeed_header;

	struct usb_function *func;
	int configured;
	int bound;
	int alt_set;
	struct switch_dev sdev;
	struct work_struct sw_work;

	struct switch_dev sw_alt_dev;
	struct work_struct sw_alt_work;

	/* notify for connection */
	int on_mode;
};

/* Functions */

/* module */
static int __init obex_module_init(void);
static void __exit obex_module_exit(void);

/* tty driver */
static int obex_open(struct tty_struct *tty, struct file *file);
static void obex_close(struct tty_struct *tty, struct file *file);
static int obex_write(struct tty_struct *tty,
		    const unsigned char *buf, int count);
static int obex_put_char(struct tty_struct *tty, unsigned char ch);
static void obex_flush_chars(struct tty_struct *tty);
static int obex_write_room(struct tty_struct *tty);
static int obex_chars_in_buffer(struct tty_struct *tty);
static void obex_throttle(struct tty_struct *tty);
static void obex_unthrottle(struct tty_struct *tty);
static int obex_break(struct tty_struct *tty, int break_state);
static int obex_ioctl(struct tty_struct *tty, struct file *file,
		    unsigned int cmd, unsigned long arg);
static void obex_set_termios(struct tty_struct *tty, struct ktermios *old);
static unsigned obex_start_rx(struct obex_dev *dev);

static int obex_send(struct obex_dev *dev);
static int obex_send_packet(struct obex_dev *dev, char *packet, unsigned int size);
static void obex_read_complete(struct usb_endpoint *ep, struct usb_request *req);
static void obex_write_complete(struct usb_endpoint *ep, struct usb_request *req);
static void obex_write_complete_sendzero
			       (struct usb_endpoint *ep, struct usb_request *req);
static int obex_tiocmget(struct tty_struct *tty, struct file *file);
static int obex_tiocmset(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear);

/* Function driver */
static void obex_bind(void *);
static void obex_unbind(void *);
static int obex_setup(struct usb_ctrlrequest *req,
		void *buf, int len, void *_ctxt);

static void obex_configure(int config, void *_ctxt);
static void obex_disconnect(void *_ctxt);
static void obex_reset_config(struct obex_dev *dev);

static struct usb_request *obex_alloc_req(struct usb_endpoint *ep,
					unsigned int len);
static void obex_free_req(struct usb_endpoint *ep, struct usb_request *req);

static int obex_alloc_ports(struct obex_dev *dev, gfp_t kmalloc_flags);
static void obex_free_ports(struct obex_dev *dev);

static int obex_set_interface(int ifc_num, int alt_set, void *_ctxt);
static int obex_get_interface(int ifc_num, void *_ctxt);
static void obex_configure_switch(struct work_struct *w);
static void obex_setinterface_switch(struct work_struct *w);

/* circular buffer */
static struct obex_buf *obex_buf_alloc(unsigned int size, gfp_t kmalloc_flags);
static void obex_buf_free(struct obex_buf *gb);
static void obex_buf_clear(struct obex_buf *gb);
static unsigned int obex_buf_data_avail(struct obex_buf *gb);
static unsigned int obex_buf_space_avail(struct obex_buf *gb);
static unsigned int obex_buf_put(struct obex_buf *gb, const char *buf,
			       unsigned int count);
static unsigned int obex_buf_get(struct obex_buf *gb, char *buf,
			       unsigned int count);
/* creates, returns async_icount struct with current port msr,mcr values */
static struct async_icount obex_current_icount(struct obex_port *port);

/* Globals */
static struct obex_dev **obex_devices;

static struct semaphore obex_open_close_sem[OBEX_NUM_PORTS];

static unsigned int obex_read_q_size = OBEX_DEFAULT_READ_Q_SIZE;
static unsigned int obex_write_q_size = OBEX_DEFAULT_WRITE_Q_SIZE;

static unsigned int obex_write_buf_size = OBEX_DEFAULT_WRITE_BUF_SIZE;

static struct workqueue_struct *obex_tty_wq;


/* tty driver struct */
static const struct tty_operations obex_tty_ops = {
	.open = obex_open,
	.close = obex_close,
	.write = obex_write,
	.put_char = obex_put_char,
	.flush_chars = obex_flush_chars,
	.write_room = obex_write_room,
	.ioctl = obex_ioctl,
	.set_termios = obex_set_termios,
	.throttle = obex_throttle,
	.unthrottle = obex_unthrottle,
	.break_ctl = obex_break,
	.chars_in_buffer = obex_chars_in_buffer,
	.tiocmget = obex_tiocmget,
	.tiocmset = obex_tiocmset,
};
static struct tty_driver *obex_tty_driver;

/* Function  driver struct */
static struct usb_function usb_function_obex[MAX_INSTANCES];

/* Module */
MODULE_DESCRIPTION(OBEX_LONG_NAME);
MODULE_AUTHOR("Sharp Corporation");
MODULE_LICENSE("GPL");

module_init(obex_module_init);
module_exit(obex_module_exit);

/******************************************************************************/

/*
 * CDC-OBEX Class specific Descriptors
 */

/* Communications Class OBEX Model Interface Descriptor */
static void obex_init_comm_class_ifc_desc(struct usb_interface_descriptor *ifc_desc)
{
	ifc_desc->bLength = sizeof(struct usb_interface_descriptor);
	ifc_desc->bDescriptorType = USB_DT_INTERFACE; /* 0x04 Interface */
	ifc_desc->bAlternateSetting = 0;
	ifc_desc->bNumEndpoints = 0;
	ifc_desc->bInterfaceClass = USB_CLASS_COMM; 
					/* 0x02 [Class] Communications and CDC Control */
	ifc_desc->bInterfaceSubClass = USB_CDC_SUBCLASS_OBEX;
					/* 0x0B OBEX Model (Communications Class SubClass) */
	ifc_desc->bInterfaceProtocol = 0;
}

/* Header */    /* USB_CDC_HEADER_TYPE */
static void obex_init_obex_cdc_header_ifc_desc(struct usb_cdc_header_desc *header_desc)
{
	header_desc->bLength = sizeof(struct usb_cdc_header_desc);
	header_desc->bDescriptorType = USB_DT_CS_INTERFACE;
					/* 0x24 (USB_TYPE_CLASS | USB_DT_INTERFACE) */
	header_desc->bDescriptorSubType = USB_CDC_HEADER_TYPE;
					/* 0x00 USB_CDC_HEADER_TYPE */
	header_desc->bcdCDC = __constant_cpu_to_le16(0x0110);
}

/*  OBEX */     /* USB_CDC_OBEX_TYPE */
static void obex_init_obex_cdc_obex_ifc_desc(struct usb_cdc_obex_desc *obex_desc)
{
	obex_desc->bLength = sizeof(struct usb_cdc_obex_desc);
	obex_desc->bDescriptorType = USB_DT_CS_INTERFACE;
					/* 0x24 (USB_TYPE_CLASS | USB_DT_INTERFACE) */
	obex_desc->bDescriptorSubType = USB_CDC_OBEX_TYPE;
					/* 0x15 USB_CDC_OBEX_TYPE */
	obex_desc->bcdVersion = __constant_cpu_to_le16(0x0100);
}

/* Union */	/* USB_CDC_UNION_TYPE */
static void obex_init_obex_cdc_union_ifc_desc(struct usb_cdc_union_desc *union_desc)
{
	union_desc->bLength = sizeof(struct usb_cdc_union_desc);
	union_desc->bDescriptorType = USB_DT_CS_INTERFACE;
					/* 0x24 (USB_TYPE_CLASS | USB_DT_INTERFACE) */
	union_desc->bDescriptorSubType = USB_CDC_UNION_TYPE;
					/* 0x06 USB_CDC_UNION_TYPE */
	union_desc->bMasterInterface0 = 0;
	union_desc->bSlaveInterface0 = 1;
}

/* Data Class Interface Descriptor for OBEX facilities, Setting 0 */
static void obex_init_data_class_setting0_ifc_desc(struct usb_interface_descriptor *ifc_desc)
{
	ifc_desc->bLength = sizeof(struct usb_interface_descriptor);
	ifc_desc->bDescriptorType = USB_DT_INTERFACE;   /* 0x04 Interface */
	ifc_desc->bAlternateSetting = 0;
	ifc_desc->bNumEndpoints = 0;
	ifc_desc->bInterfaceClass = USB_CLASS_CDC_DATA; /* 0x0A [Class] CDC-Data */
	ifc_desc->bInterfaceSubClass = 0;
	ifc_desc->bInterfaceProtocol = 0;
}

/* Data Class Interface Descriptor for OBEX facilities, non-zero Setting */
static void obex_init_data_class_setting1_ifc_desc(struct usb_interface_descriptor *ifc_desc)
{
	ifc_desc->bLength = sizeof(struct usb_interface_descriptor);
	ifc_desc->bDescriptorType = USB_DT_INTERFACE;   /* 0x04 Interface */
	ifc_desc->bAlternateSetting = 1;
	ifc_desc->bNumEndpoints = 2;
	ifc_desc->bInterfaceClass = USB_CLASS_CDC_DATA; /* 0x0A [Class] CDC-Data */
	ifc_desc->bInterfaceSubClass = 0;
	ifc_desc->bInterfaceProtocol = 0;
}

#define HIGHSPEED	1
#define	FULLSPEED	2

#define BULK	1
#define INTERRUPT	2
static void obex_init_ep_desc(struct usb_endpoint_descriptor *ep_desc,
				unsigned type, unsigned speed)
{
	ep_desc->bLength =		USB_DT_ENDPOINT_SIZE;
	ep_desc->bDescriptorType =	USB_DT_ENDPOINT;

	if (type == BULK) {
		ep_desc->bmAttributes = USB_ENDPOINT_XFER_BULK;
		if (speed == HIGHSPEED)
			ep_desc->wMaxPacketSize = 512;
		else
			ep_desc->wMaxPacketSize = 64;
	} else {

		ep_desc->bmAttributes = USB_ENDPOINT_XFER_INT;
		ep_desc->wMaxPacketSize = 64;
		ep_desc->bInterval = 4;
	}
}

static void obex_init_header_desc(struct obex_dev *dev)
{
	dev->obex_highspeed_header[0] =
		(struct usb_descriptor_header *)&dev->obex_ifc_desc;
	dev->obex_highspeed_header[1] =
		(struct usb_descriptor_header *)&dev->obex_cdc_header_desc;
	dev->obex_highspeed_header[2] =
		(struct usb_descriptor_header *)&dev->obex_cdc_obex_desc;
	dev->obex_highspeed_header[3] =
		(struct usb_descriptor_header *)&dev->obex_cdc_union_ifc_desc;
	dev->obex_highspeed_header[4] =
		(struct usb_descriptor_header *)&dev->obex_data_zero_ifc_desc;
	dev->obex_highspeed_header[5] =
		(struct usb_descriptor_header *)&dev->obex_data_one_ifc_desc;
	dev->obex_highspeed_header[6] =
		(struct usb_descriptor_header *)&dev->obex_hs_bulkin_desc;
	dev->obex_highspeed_header[7] =
		(struct usb_descriptor_header *)&dev->obex_hs_bulkout_desc;
	dev->obex_highspeed_header[8] = NULL;

	dev->obex_fullspeed_header[0] =
		(struct usb_descriptor_header *)&dev->obex_ifc_desc;
	dev->obex_fullspeed_header[1] =
		(struct usb_descriptor_header *)&dev->obex_cdc_header_desc;
	dev->obex_fullspeed_header[2] =
		(struct usb_descriptor_header *)&dev->obex_cdc_obex_desc;
	dev->obex_fullspeed_header[3] =
		(struct usb_descriptor_header *)&dev->obex_cdc_union_ifc_desc;
	dev->obex_fullspeed_header[4] =
		(struct usb_descriptor_header *)&dev->obex_data_zero_ifc_desc;
	dev->obex_fullspeed_header[5] =
		(struct usb_descriptor_header *)&dev->obex_data_one_ifc_desc;
	dev->obex_fullspeed_header[6] =
		(struct usb_descriptor_header *)&dev->obex_fs_bulkin_desc;
	dev->obex_fullspeed_header[7] =
		(struct usb_descriptor_header *)&dev->obex_fs_bulkout_desc;
	dev->obex_fullspeed_header[8] = NULL;
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	struct obex_dev  *dev = container_of(sdev, struct obex_dev, sdev);
	return sprintf(buf, "%s\n", (dev->on_mode ? "online" : "offline"));
}

static ssize_t print_switch_name_alt_sts(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME_ALT);
}

static ssize_t print_switch_state_alt_sts(struct switch_dev *sdev, char *buf)
{
	struct obex_dev  *dev = container_of(sdev, struct obex_dev, sw_alt_dev);
	return sprintf(buf, "%s\n", (dev->alt_set ? "open" : "close"));
}

/*****************************************************************************/
/*
 *  obex_module_init
 *
 *  Register as a USB gadget driver and a tty driver.
 */

char *obex_a[] = {"obex"};

static int __init obex_module_init(void)
{
	int i, retval;
	struct usb_function *func;

	if (obex_instances > MAX_INSTANCES || obex_instances == 0) {
		printk(KERN_ERR "Incorrect obex_instances entered \n");
		return -ENODEV;
	}

	obex_tty_wq = create_singlethread_workqueue("obex_tty");
	if (obex_tty_wq == 0)
		return -ENOMEM;
	obex_tty_driver = alloc_tty_driver(OBEX_NUM_PORTS);
	if (!obex_tty_driver) {
		destroy_workqueue(obex_tty_wq);
		return -ENOMEM;
	}
	obex_tty_driver->owner = THIS_MODULE;
	obex_tty_driver->driver_name = OBEX_SHORT_NAME;
	obex_tty_driver->name = "ttyOBEX";
	obex_tty_driver->major = OBEX_MAJOR;
	obex_tty_driver->minor_start = OBEX_MINOR_START;
	obex_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	obex_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	obex_tty_driver->flags =  TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
				| TTY_DRIVER_RESET_TERMIOS;
	obex_tty_driver->init_termios = tty_std_termios;
	obex_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL
	    | CLOCAL;
	obex_tty_driver->init_termios.c_iflag = 0;
	obex_tty_driver->init_termios.c_oflag = 0;
	obex_tty_driver->init_termios.c_lflag = 0;

	tty_set_operations(obex_tty_driver, &obex_tty_ops);

	for (i = 0; i < OBEX_NUM_PORTS; i++)
		sema_init(&obex_open_close_sem[i], 1);

	retval = tty_register_driver(obex_tty_driver);
	if (retval) {
		/*usb_function_unregister(&usb_func_serial); */
		put_tty_driver(obex_tty_driver);
		printk(KERN_ERR
		       "obex_module_init: cannot register tty driver,ret = %d\n",
		       retval);
		return retval;
	}
	for (i = 0; i < MAX_INSTANCES; i++){
		tty_register_device(obex_tty_driver, i, NULL);
	}

	obex_devices = kzalloc(sizeof(struct obex_dev *) * obex_instances,
				GFP_KERNEL);
	if (!obex_devices){
		return -ENOMEM;
	}

	for (i = 0; i < obex_instances; i++) {
		func = &usb_function_obex[i];

		obex_devices[i] = kzalloc(sizeof(struct obex_dev), GFP_KERNEL);
		if (!obex_devices[i])
			return -ENOMEM;
		spin_lock_init(&obex_devices[i]->dev_lock);
		INIT_LIST_HEAD(&obex_devices[i]->dev_req_list);
		INIT_WORK(&obex_devices[i]->sw_work, obex_configure_switch);
		INIT_WORK(&obex_devices[i]->sw_alt_work, obex_setinterface_switch);
		obex_devices[i]->func = func;
		/*3 + 3 - Interfaces, 2 Endpoints-> Total 8 + 1 for NULL*/
		obex_devices[i]->obex_fullspeed_header =
		kmalloc(sizeof(struct usb_descriptor_header *) * 9, GFP_KERNEL);
		obex_devices[i]->obex_highspeed_header =
		kmalloc(sizeof(struct usb_descriptor_header *) * 9, GFP_KERNEL);

		obex_init_comm_class_ifc_desc(&obex_devices[i]->obex_ifc_desc);
		obex_init_obex_cdc_header_ifc_desc(&obex_devices[i]->obex_cdc_header_desc);
		obex_init_obex_cdc_obex_ifc_desc(&obex_devices[i]->obex_cdc_obex_desc);
		obex_init_obex_cdc_union_ifc_desc(&obex_devices[i]->obex_cdc_union_ifc_desc);
		obex_init_data_class_setting0_ifc_desc(&obex_devices[i]->obex_data_zero_ifc_desc);
		obex_init_data_class_setting1_ifc_desc(&obex_devices[i]->obex_data_one_ifc_desc);

		obex_init_ep_desc(&obex_devices[i]->obex_hs_bulkin_desc, BULK,
				HIGHSPEED);
		obex_init_ep_desc(&obex_devices[i]->obex_hs_bulkout_desc, BULK,
				HIGHSPEED);

		obex_init_ep_desc(&obex_devices[i]->obex_fs_bulkin_desc, BULK,
				FULLSPEED);
		obex_init_ep_desc(&obex_devices[i]->obex_fs_bulkout_desc, BULK,
				FULLSPEED);

		obex_init_header_desc(obex_devices[i]);

		/*Initializing Directions*/
		obex_devices[i]->obex_hs_bulkin_desc.bEndpointAddress = USB_DIR_IN;
		obex_devices[i]->obex_hs_bulkout_desc.bEndpointAddress =
								USB_DIR_OUT;
		obex_devices[i]->obex_fs_bulkin_desc.bEndpointAddress = USB_DIR_IN;
		obex_devices[i]->obex_fs_bulkout_desc.bEndpointAddress =
								USB_DIR_OUT;
		obex_devices[i]->sdev.name = SWITCH_NAME;
		obex_devices[i]->sdev.print_name = print_switch_name;
		obex_devices[i]->sdev.print_state = print_switch_state;
		retval = switch_dev_register(&obex_devices[i]->sdev);

		if (retval < 0){
			return retval;
		}

		obex_devices[i]->sw_alt_dev.name = SWITCH_NAME_ALT;
		obex_devices[i]->sw_alt_dev.print_name = print_switch_name_alt_sts;
		obex_devices[i]->sw_alt_dev.print_state = print_switch_state_alt_sts;
		retval = switch_dev_register(&obex_devices[i]->sw_alt_dev);

		if (retval < 0){
			return retval;
		}

		func->bind = obex_bind;
		func->unbind = obex_unbind;
		func->configure = obex_configure;
		func->disconnect = obex_disconnect;
		func->setup = obex_setup;
		func->name = obex_a[i];
		func->context = obex_devices[i];
		func->fs_descriptors = obex_devices[i]->obex_fullspeed_header;
		func->hs_descriptors = obex_devices[i]->obex_highspeed_header;
		func->set_interface = obex_set_interface;
		func->get_interface = obex_get_interface;

		retval = usb_function_register(func);
		if (retval) {
			printk(KERN_ERR
	      "obex_module_init: cannot register Function driver, ret = %d\n",
			       retval);
			return retval;
		}
	}

	return 0;
}

static int obex_set_interface(int ifc_num, int alt_set, void *_ctxt)
{
	struct obex_dev *dev = _ctxt;

	if (dev == NULL) {
		return 1;
	}

	if ((ifc_num == dev->obex_data_one_ifc_desc.bInterfaceNumber)
			&& (alt_set == 1)) {
		dev->alt_set = 1;
		usb_ept_enable(dev->dev_in_ep, 1);
		usb_ept_enable(dev->dev_out_ep, 1);
	} else {
		dev->alt_set = 0;
		usb_ept_enable(dev->dev_in_ep, 0);
		usb_ept_enable(dev->dev_out_ep, 0);
	}

	schedule_work(&dev->sw_alt_work);

	return 0;
}

static int obex_get_interface(int ifc_num, void *_ctxt)
{
	struct obex_dev *dev = _ctxt;
	return dev->alt_set;
}

/*
* obex_module_exit
*
* Unregister as a tty driver and a USB gadget driver.
*/
static void __exit obex_module_exit(void)
{
	int i;
	for (i = 0; i < obex_instances; i++)
		usb_function_unregister(&usb_function_obex[i]);

	for (i = 0; i < obex_instances; ++i) {
		kfree(obex_devices[i]->obex_fullspeed_header);
		kfree(obex_devices[i]->obex_highspeed_header);
		kfree(obex_devices[i]);
	}
	for (i = 0; i < MAX_INSTANCES; i++)
		tty_unregister_device(obex_tty_driver, i);
	tty_unregister_driver(obex_tty_driver);
	put_tty_driver(obex_tty_driver);
	printk(KERN_INFO "obex_module_exit: %s %s unloaded\n", OBEX_LONG_NAME,
	       OBEX_VERSION_STR);
}

/* TTY Driver */
/*
 * obex_open
 */
static int obex_open(struct tty_struct *tty, struct file *file)
{
	int port_num;
	unsigned long flags;
	struct obex_port *port;
	struct obex_dev *dev;
	struct obex_buf *buf;
	struct semaphore *sem;
	int ret;

	port_num = tty->index;

	obex_debug("obex_open: (%d,%p,%p)\n", port_num, tty, file);

	if (port_num < 0 || port_num >= OBEX_NUM_PORTS) {
		printk(KERN_ERR "obex_open: (%d,%p,%p) invalid port number\n",
		       port_num, tty, file);
		return -ENODEV;
	}

	dev = obex_devices[tty->index];

	if (dev == NULL) {
		printk(KERN_ERR "obex_open: (%d,%p,%p) NULL device pointer\n",
		       port_num, tty, file);
		return -ENODEV;
	}

	sem = &obex_open_close_sem[port_num];
	if (down_interruptible(sem)) {
		printk(KERN_ERR
	       "obex_open: (%d,%p,%p) interrupted waiting for semaphore\n",
		       port_num, tty, file);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&dev->dev_lock, flags);
	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR "obex_open: (%d,%p,%p) NULL port pointer\n",
		       port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}

	spin_unlock_irqrestore(&dev->dev_lock, flags);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "obex_open: (%d,%p,%p) port disconnected (1)\n",
		       port_num, tty, file);
		ret = -EIO;
		goto exit_unlock_port;
	}

	if (port->port_open_count > 0) {
		++port->port_open_count;
		obex_debug("obex_open: (%d,%p,%p) already open\n",
			 port_num, tty, file);
#if 0
		ret = 0;
#else
		ret = -EBUSY;
#endif
		goto exit_unlock_port;
	}

	tty->driver_data = NULL;

	/* mark port as in use, we can drop port lock and sleep if necessary */
	port->port_in_use = 1;

	/* allocate write buffer on first open */
	if (port->port_write_buf == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		buf = obex_buf_alloc(obex_write_buf_size, GFP_KERNEL);
		spin_lock_irqsave(&port->port_lock, flags);

		/* might have been disconnected while asleep, check */
		if (port->port_dev == NULL) {
			printk(KERN_ERR
			       "obex_open: (%d,%p,%p) port disconnected (2)\n",
			       port_num, tty, file);
			port->port_in_use = 0;
			ret = -EIO;
			goto exit_unlock_port;
		}

		port->port_write_buf = buf;
		if (port->port_write_buf == NULL) {
			printk(KERN_ERR
	       "obex_open: (%d,%p,%p) cannot allocate port write buffer\n",
			       port_num, tty, file);
			port->port_in_use = 0;
			ret = -ENOMEM;
			goto exit_unlock_port;
		}

	}

	/* wait for carrier detect (not implemented) */

	/* might have been disconnected while asleep, check */
	if (port->port_dev == NULL) {
		printk(KERN_ERR "obex_open: (%d,%p,%p) port disconnected (3)\n",
		       port_num, tty, file);
		port->port_in_use = 0;
		ret = -EIO;
		goto exit_unlock_port;
	}

	tty->driver_data = port;
	port->port_tty = tty;
	port->port_tty->low_latency = 1;
	port->port_open_count = 1;
	port->port_in_use = 0;

	obex_debug("obex_open: (%d,%p,%p) completed\n", port_num, tty, file);
	port->msr |= MSR_CTS;
	/* Queue RX requests */
	port->n_read = 0;
	obex_start_rx(dev);
	wake_up_interruptible(&port->msr_change_wait);
	ret = 0;

exit_unlock_port:
	spin_unlock_irqrestore(&port->port_lock, flags);
	up(sem);
	return ret;

exit_unlock_dev:
	spin_unlock_irqrestore(&dev->dev_lock, flags);
	up(sem);
	return ret;

}

/*
 * obex_close
 */

#define OBEX_WRITE_FINISHED_EVENT_SAFELY(p)			\
({								\
	int cond;						\
								\
	spin_lock_irq(&(p)->port_lock);				\
	cond = !(p)->port_dev || !obex_buf_data_avail((p)->port_write_buf); \
	spin_unlock_irq(&(p)->port_lock);			\
	cond;							\
})

static void obex_close(struct tty_struct *tty, struct file *file)
{
	struct obex_port *port = tty->driver_data;
	struct semaphore *sem;

	if (port == NULL) {
		printk(KERN_ERR "obex_close: NULL port pointer\n");
		return;
	}

	obex_debug("obex_close: (%d,%p,%p)\n", port->port_num, tty, file);

	sem = &obex_open_close_sem[port->port_num];
	down(sem);

	spin_lock_irq(&port->port_lock);

	port->msr &= ~MSR_CTS;
	wake_up_interruptible(&port->msr_change_wait);

	if (port->port_open_count == 0) {
		printk(KERN_ERR
		       "obex_close: (%d,%p,%p) port is already closed\n",
		       port->port_num, tty, file);
		goto exit;
	}

	if (port->port_open_count > 1) {
		--port->port_open_count;
		goto exit;
	}

	/* free disconnected port on final close */
	if (port->port_dev == NULL)
		goto exit;


	/* mark port as closed but in use, we can drop port lock */
	/* and sleep if necessary */
	port->port_in_use = 1;
	port->port_open_count = 0;

	/* wait for write buffer to drain, or */
	/* at most OBEX_CLOSE_TIMEOUT seconds */
	if (obex_buf_data_avail(port->port_write_buf) > 0) {
		if(port->port_dev->configured == OBEX_UNCONFIGURED){
			obex_buf_clear(port->port_write_buf);
		}
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->port_write_wait,
						 OBEX_WRITE_FINISHED_EVENT_SAFELY
						 (port), OBEX_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
	}

	/* free disconnected port on final close */
	/* (might have happened during the above sleep) */
	if (port->port_dev == NULL)
		goto exit;


	obex_buf_clear(port->port_write_buf);

	/* Flush bulk-out pipe */
	usb_ept_fifo_flush(port->port_dev->dev_out_ep);
	tty->driver_data = NULL;
	port->port_tty = NULL;
	port->port_in_use = 0;

	obex_debug("obex_close: (%d,%p,%p) completed\n", port->port_num, tty, file);

exit:
	spin_unlock_irq(&port->port_lock);
	up(sem);
	if (port->port_dev == NULL){
		kfree(port);
	}
}

/*
 * obex_write
 */
static int obex_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	unsigned long flags;
	struct obex_port *port = tty->driver_data;
	int ret;

	if (port == NULL) {
		printk(KERN_ERR "obex_write: NULL port pointer\n");
		return -EIO;
	}

	obex_debug("obex_write: (%d,%p) writing %d bytes\n", port->port_num, tty,
		 count);

	if (count == 0)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "obex_write: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		ret = -EIO;
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "obex_write: (%d,%p) port is closed\n",
		       port->port_num, tty);
		ret = -EBADF;
		goto exit;
	}

	count = obex_buf_put(port->port_write_buf, buf, count);

	if (port->port_dev->dev_config)
		obex_send(obex_devices[tty->index]);
	spin_unlock_irqrestore(&port->port_lock, flags);

	obex_debug("obex_write: (%d,%p) wrote %d bytes\n", port->port_num, tty,
		 count);

	return count;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return ret;
}

/*
 * obex_put_char
 */
static int obex_put_char(struct tty_struct *tty, unsigned char ch)
{
	unsigned long flags;
	int ret = 0;
	struct obex_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "obex_put_char: NULL port pointer\n");
		goto out;
	}

	obex_debug("obex_put_char: (%d,%p) char=0x%x, called from %p, %p, %p\n",
		 port->port_num, tty, ch, __builtin_return_address(0),
		 __builtin_return_address(1), __builtin_return_address(2));

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "obex_put_char: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		goto exit_unlock;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "obex_put_char: (%d,%p) port is closed\n",
		       port->port_num, tty);
		goto exit_unlock;
	}

	ret = obex_buf_put(port->port_write_buf, &ch, 1);

exit_unlock:
	spin_unlock_irqrestore(&port->port_lock, flags);
out:
	return ret;
}

/*
 * obex_flush_chars
 */
static void obex_flush_chars(struct tty_struct *tty)
{
	unsigned long flags;
	struct obex_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "obex_flush_chars: NULL port pointer\n");
		return;
	}

	obex_debug("obex_flush_chars: (%d,%p)\n", port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR
		       "obex_flush_chars: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "obex_flush_chars: (%d,%p) port is closed\n",
		       port->port_num, tty);
		goto exit;
	}

	if (port->port_dev->dev_config)
		obex_send(obex_devices[tty->index]);
	spin_unlock_irqrestore(&port->port_lock, flags);


	return;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * obex_write_room
 */
static int obex_write_room(struct tty_struct *tty)
{

	int room = 0;
	unsigned long flags;
	struct obex_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	    && port->port_write_buf != NULL)
		room = obex_buf_space_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	obex_debug("obex_write_room: (%d,%p) room=%d\n", port->port_num, tty, room);

	return room;
}

/*
 * obex_chars_in_buffer
 */
static int obex_chars_in_buffer(struct tty_struct *tty)
{
	int chars = 0;
	unsigned long flags;
	struct obex_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	    && port->port_write_buf != NULL)
		chars = obex_buf_data_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	obex_debug("obex_chars_in_buffer: (%d,%p) chars=%d\n",
		 port->port_num, tty, chars);

	return chars;
}

/*
 * obex_throttle
 */
static void obex_throttle(struct tty_struct *tty)
{
}

/*
 * obex_unthrottle
 */
static void obex_unthrottle(struct tty_struct *tty)
{
	struct obex_port		*port = tty->driver_data;
	unsigned long		flags;

	spin_lock_irqsave(&port->port_lock, flags);
	queue_work(obex_tty_wq, &port->push_work);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * obex_break
 */
static int obex_break(struct tty_struct *tty, int break_state)
{
	return 0;
}

/*
 * obex_ioctl
 */
static int obex_ioctl(struct tty_struct *tty, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	struct obex_port *port = tty->driver_data;
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount cnow;
	struct async_icount cprev;

	if (port == NULL) {
		printk(KERN_ERR "obex_ioctl: NULL port pointer\n");
		return -EIO;
	}

	obex_debug("obex_ioctl: (%d,%p,%p) cmd=0x%4.4x, arg=%lu\n",
		 port->port_num, tty, file, cmd, arg);

	/* handle ioctls */
	switch (cmd) {
	case TIOCMIWAIT:
		cprev = obex_current_icount(port);
		while (1) {
			add_wait_queue(&port->msr_change_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&port->msr_change_wait, &wait);
			if (signal_pending(current))
				return -ERESTARTSYS;
			cnow = obex_current_icount(port);
			if (cnow.rng == cprev.rng &&
				cnow.dsr == cprev.dsr &&
				cnow.dcd == cprev.dcd &&
				cnow.cts == cprev.cts)
				return -EIO;
			if (((arg & TIOCM_RI) &&
					(cnow.rng != cprev.rng)) ||
				((arg & TIOCM_DSR) &&
					(cnow.dsr != cprev.dsr)) ||
				((arg & TIOCM_CD)  &&
					(cnow.dcd != cprev.dcd)) ||
				((arg & TIOCM_CTS) &&
					(cnow.cts != cprev.cts)))
				return 0;
			cprev = cnow;
		}
		break;
	}

	/* could not handle ioctl */
	return -ENOIOCTLCMD;
}

/*
 * obex_set_termios
 */
static void obex_set_termios(struct tty_struct *tty, struct ktermios *old)
{
}

/*
* obex_send
*
* This function finds available write requests, calls
* obex_send_packet to fill these packets with data, and
* continues until either there are no more write requests
* available or no more data to send.  This function is
* run whenever data arrives or write requests are available.
*/
static int obex_send(struct obex_dev *dev)
{
	struct obex_port *port = dev->dev_port[0];
	struct list_head *pool = &port->write_pool;
	int status = 0;
	bool do_tty_wake = false;
	struct usb_endpoint *ep = dev->dev_in_ep;
	unsigned int MaxPacketSize;
	unsigned int RemainLen;

	while (!list_empty(pool)) {
		struct usb_request *req;
		int len;
		req = list_entry(pool->next, struct usb_request, list);
		len = obex_send_packet(dev, req->buf, usb_ept_get_max_packet(ep));
		if (len == 0) {
			wake_up_interruptible(&port->port_write_wait);
			break;
		}
		do_tty_wake = true;

		req->length = len;
		list_del(&req->list);

		MaxPacketSize = usb_ept_get_max_packet(ep);
		RemainLen = obex_buf_data_avail(dev->dev_port[0]->port_write_buf);
		if( ( req->length == MaxPacketSize ) &&
		    ( RemainLen == 0 ) ) {
			req->complete = obex_write_complete_sendzero;
		}
		else {
			req->complete = obex_write_complete;
		}

		/* Drop lock while we call out of driver; completions
		 * could be issued while we do so.  Disconnection may
		 * happen too; maybe immediately before we queue this!
		 * NOTE that we may keep sending data for a while after
		 * the TTY closed (dev->ioport->port_tty is NULL).
		 */
		spin_unlock(&port->port_lock);
		status = usb_ept_queue_xfer(ep, req);
		spin_lock(&port->port_lock);

		if (status) {
			printk(KERN_ERR "%s: %s err %d\n",
					__func__, "queue", status);
			list_add(&req->list, pool);
			break;
		}

	}

	if (do_tty_wake && port->port_tty){
		tty_wakeup(port->port_tty);
	}
	return status;

}

/*
 * obex_send_packet
 *
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.  If there is any error a negative
 * error number is returned.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int obex_send_packet(struct obex_dev *dev, char *packet, unsigned int size)
{
	unsigned int len;
	struct obex_port *port;

	if (dev == NULL) {
		printk(KERN_ERR "obex_recv_packet:NULL device pointer\n");
		return -EIO;
	}

	/* TEMPORARY -- only port 0 is supported right now */
	port = dev->dev_port[0];
	if (port == NULL) {
		printk(KERN_ERR
		       "obex_send_packet: port=%d, NULL port pointer\n", 0);
		return -EIO;
	}


	len = obex_buf_data_avail(port->port_write_buf);
	if (len < size)
		size = len;
	if (size != 0)
		size = obex_buf_get(port->port_write_buf, packet, size);

	if (port->port_tty){
		tty_wakeup(port->port_tty);
	}

	return size;
}

static void obex_rx_push(struct work_struct *work)
{
	struct obex_port *port = container_of(work,
					struct obex_port,
					push_work);
	struct tty_struct *tty;
	struct list_head *queue = &port->read_queue;
	bool do_push = false;
	struct obex_dev *dev = port->port_dev;

	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	tty = port->port_tty;
	while (!list_empty(queue)) {
		struct usb_request	*req;

		req = list_first_entry(queue, struct usb_request, list);

		/* discard data if tty was closed */
		if (!tty)
			goto recycle;

		if (req->actual) {
			char		*packet = req->buf;
			unsigned	size = req->actual;
			unsigned	n;
			int		count;
			/* we may have pushed part of this packet already... */
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}
			/*printk(KERN_INFO "tty_push:%d\n",size);*/
			count = tty_insert_flip_string(tty, packet, size);
			if (count == 0)
				printk(KERN_INFO "%s: tty buffer is full: throttle\n",
							__func__);
			if (count)
				do_push = true;
			if (count != size) {
				/* stop pushing; TTY layer can't handle more */
				port->n_read += count;
				break;
			}
			port->n_read = 0;
		}
recycle:
		list_move(&req->list, &port->read_pool);
	}
	if (tty && do_push) {
		spin_unlock_irq(&port->port_lock);
		tty_flip_buffer_push(tty);
		wake_up_interruptible(&tty->read_wait);
		spin_lock_irq(&port->port_lock);
		/* tty may have been closed */
		tty = port->port_tty;
	}
	if (!list_empty(queue) && tty) {
		if (!test_bit(TTY_THROTTLED, &tty->flags)) {
			if (do_push){
				queue_work(obex_tty_wq, &port->push_work);
			}
		}
	}
	obex_start_rx(dev);
	spin_unlock_irq(&port->port_lock);
}

static void obex_configure_switch(struct work_struct *w)
{
	struct obex_dev *dev = container_of(w,
					    struct obex_dev,
					    sw_work);

	switch_set_state(&dev->sdev, dev->on_mode);
}

static void obex_setinterface_switch(struct work_struct *w)
{
	struct obex_dev *dev = container_of(w,
					    struct obex_dev,
					    sw_alt_work);

	switch_set_state(&dev->sw_alt_dev, dev->alt_set);
}

/*
* obex_read_complete
*/
static void obex_read_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	/* used global variable */
	struct obex_dev *dev = (struct obex_dev *)req->device;
	struct obex_port *port;
	struct tty_struct *tty;

	if (dev == NULL) {
		printk(KERN_ERR "obex_read_complete: NULL device pointer\n");
		return;
	}

	port = dev->dev_port[0];
	tty = port->port_tty;
	switch (req->status) {
	case 0:
		spin_lock(&port->port_lock);
		list_add_tail(&req->list, &port->read_queue);
		if (!test_bit(TTY_THROTTLED, &tty->flags))
			queue_work(obex_tty_wq, &port->push_work);
		spin_unlock(&port->port_lock);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		obex_debug("obex_read_complete: shutdown\n");
		obex_free_req(ep, req);
		break;

	case -ENODEV:
		list_add_tail(&req->list, &port->read_pool);
		/* Implemented handling in future if needed */
		break;
	default:
		list_add_tail(&req->list, &port->read_pool);
		printk(KERN_ERR
		"obex_read_complete: unexpected status error, status=%d\n",
			req->status);
		/* goto requeue; */
		break;
	}
}

/*
* obex_write_complete
*/
static void obex_write_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	struct obex_dev *dev = (struct obex_dev *)req->device;
	struct obex_port	*port = dev->dev_port[0];

	if (dev == NULL) {
		printk(KERN_ERR "obex_write_complete: NULL device pointer\n");
		return;
	}
	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);

	switch (req->status) {
	default:
		/* presumably a transient fault */
		printk(KERN_ERR "%s: unexpected status %d\n",
				__func__, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */

		if (dev->dev_config)
			obex_send(dev);

		break;

	case -ESHUTDOWN:
		/* disconnect */
		printk(KERN_DEBUG "%s: shutdown\n", __func__);
		break;
	}
	spin_unlock(&port->port_lock);
}


/*
* obex_write_complete_sendzero
*/
static void obex_write_complete_sendzero(struct usb_endpoint *ep, struct usb_request *req)
{
	struct obex_dev *dev = (struct obex_dev *)req->device;
	struct obex_port	*port = dev->dev_port[0];
/*
	struct usb_endpoint *ep = dev->dev_in_ep; */
	unsigned int RemainLen;
	if (dev == NULL) {
		printk(KERN_ERR "obex_write_complete_sendzero: NULL device pointer\n");
		return;
	}
	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);

	switch (req->status) {
	default:
		/* presumably a transient fault */
		printk(KERN_ERR "%s: unexpected status %d\n",
				__func__, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */
		/* send zero length packet */

		RemainLen = obex_buf_data_avail(port->port_write_buf);

		if( RemainLen == 0 ) {
			if(!list_empty(&port->write_pool)) {
				req = list_entry(port->write_pool.next,
					       struct usb_request, list);
				list_del(&req->list);
				req->length = 0;
				req->complete = obex_write_complete;
				usb_ept_queue_xfer(ep, req);
			}
		}
		else if (dev->dev_config) {
			obex_send(dev);
		}

		break;

	case -ESHUTDOWN:
		/* disconnect */
		printk(KERN_DEBUG "%s: shutdown\n", __func__);
		break;
	}
	spin_unlock(&port->port_lock);
}

/* Function Driver */
/*
 * obex_bind
 *
 * Called on module load.  Allocates and initializes the device
 * structure and a control request.
 */
static void obex_bind(void *_ctxt)
{
	struct usb_endpoint *ep;
	struct obex_dev *dev = _ctxt;
	struct usb_function *func = dev->func;
	int ret;
	int intf;

	if (func == NULL) {
		pr_err("%s: NULL function pointer\n", __func__);
		return;
	}

	ret = obex_alloc_ports(dev, GFP_KERNEL);
	if (ret != 0) {
		pr_err("%s: cannot allocate ports\n", __func__);
		obex_unbind(_ctxt);
		return;
	}

	ret = usb_msm_get_next_ifc_number(func);
	dev->obex_ifc_desc.bInterfaceNumber = ret;

	intf = usb_msm_get_next_strdesc_id(USB_OBEX_STRING_DESC_WORD);
	dev->obex_ifc_desc.iInterface = intf;

	dev->obex_cdc_union_ifc_desc.bMasterInterface0 = ret;
	dev->obex_cdc_union_ifc_desc.bSlaveInterface0 = ret+1;

	ret = usb_msm_get_next_ifc_number(func);
	dev->obex_data_zero_ifc_desc.bInterfaceNumber = ret;
	dev->obex_data_zero_ifc_desc.iInterface = intf;

	dev->obex_data_one_ifc_desc.bInterfaceNumber = ret;

	dev->obex_data_one_ifc_desc.iInterface = intf;

	/*Configuring IN Endpoint*/
	ep = dev->dev_in_ep = usb_alloc_endpoint(USB_DIR_IN);
	if (!ep) {
		pr_err("%s: in endpoint allocation failed\n", __func__);
		return;
	}
	dev->obex_hs_bulkin_desc.bEndpointAddress = USB_DIR_IN | ep->num;
	dev->obex_fs_bulkin_desc.bEndpointAddress = USB_DIR_IN | ep->num;
	pr_debug("%s: bulk_in_endpoint Number = %d\n",
						__func__, ep->num);

	/*Configuring OUT endpoint*/
	ep = dev->dev_out_ep = usb_alloc_endpoint(USB_DIR_OUT);
	if (!ep) {
		pr_err("out endpoint allocation failed\n");
		return;
	}
	dev->obex_hs_bulkout_desc.bEndpointAddress = USB_DIR_OUT | ep->num;
	dev->obex_fs_bulkout_desc.bEndpointAddress = USB_DIR_OUT | ep->num;
	pr_debug("%s: bulk_out_endpoint Number = %d\n",
						__func__, ep->num);

	dev->bound = 1;
	return;
}
/*
 * obex_unbind
 *
 * Called on module unload.  Frees the control request and device
 * structure.
 */
static void /* __init_or_exit */ obex_unbind(void *_ctxt)
{
	struct obex_dev *dev = _ctxt;

	if (!dev) {
		pr_err("%s: error: null device\n", __func__);
		return;
	}
	if (!dev->bound)
		return;

	if (dev->dev_in_ep) {
		usb_ept_fifo_flush(dev->dev_in_ep);
		if(dev->alt_set == 0){
			usb_ept_enable(dev->dev_in_ep,  0);
		}
		usb_free_endpoint(dev->dev_in_ep);
	}
	if (dev->dev_out_ep) {
		usb_ept_fifo_flush(dev->dev_out_ep);
		if(dev->alt_set == 0){
			usb_ept_enable(dev->dev_out_ep,  0);
		}
		usb_free_endpoint(dev->dev_out_ep);
	}

	obex_free_ports(dev);
	dev->on_mode = 0;
	schedule_work(&dev->sw_work);

	dev->alt_set = 0;
	schedule_work(&dev->sw_alt_work);

	dev->bound = 0;
	pr_debug("%s: %s %s\n", __func__, OBEX_LONG_NAME, OBEX_VERSION_STR);
}
static int obex_setup(struct usb_ctrlrequest *ctrl,
		void *buf, int len, void *_ctxt)
{
	int ret = -EOPNOTSUPP;
	struct obex_dev *dev = _ctxt;
	struct obex_port *port;/* ACM only has one port */
#if 0 /* not supported */
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
#endif

	if (dev == NULL) {
		printk(KERN_ERR"obex_setup:device pointer NULL\n");
		return 0;
	}
	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR"obex_setup: port pointer is NULL\n");
		return 0;
	}
	switch (ctrl->bRequest) {
#if 0 /* not supported */
	case USB_CDC_REQ_SET_LINE_CODING:
		ret = min(wLength,
		(u16)sizeof(struct usb_cdc_line_coding));
		if (port) {
			spin_lock(&port->port_lock);
			memcpy(&port->port_line_coding, buf , ret);
			spin_unlock(&port->port_lock);
		}
	break;
#endif

#if 0 /* not supported */
	case USB_CDC_REQ_GET_LINE_CODING:
		port = dev->dev_port[0];/* ACM only has one port */
		ret = min(wLength, (u16) sizeof(struct usb_cdc_line_coding));
		if (port) {
			spin_lock(&port->port_lock);
			memcpy(buf, &port->port_line_coding, ret);
			spin_unlock(&port->port_lock);
		}
		break;
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		port = dev->dev_port[0];/* ACM only has one port */
		if (wValue & USB_CDC_SET_CONTROL_LINE_STATE_DTR) {
			port->mcr |= MCR_DTR;
			port->msr |= MSR_DSR;
			wake_up_interruptible(&port->msr_change_wait);
		} else	{
			port->mcr &= ~MCR_DTR;
			port->msr &= ~MSR_DSR;
			wake_up_interruptible(&port->msr_change_wait);
		}
		if (wValue & USB_CDC_SET_CONTROL_LINE_STATE_RTS)
			port->mcr |= MCR_RTS;
		else
			port->mcr &= ~MCR_RTS;

		if (port->prev_msr != port->msr) {
			dev->interface_num = wIndex;
			port->prev_msr = port->msr;
		}

		ret = 0;
		break;
#endif
	default:
		break;
	}

	return ret;
}
static void obex_disconnect(void *_ctxt)
{
	struct obex_dev *dev = _ctxt;
	struct obex_port *port = dev->dev_port[0];
	unsigned long flags;

	/* tell the TTY glue not to do I/O here any more */
	spin_lock_irqsave(&port->port_lock, flags);
	dev->dev_config = 0;
	if (port->port_open_count > 0 || port->port_in_use) {
		wake_up_interruptible(&port->port_write_wait);
		if (port->port_tty) {
			wake_up_interruptible(&port->port_tty->read_wait);
			wake_up_interruptible(&port->port_tty->write_wait);
			tty_hangup(port->port_tty);
		}
	}
	port->mcr &= ~(MCR_DTR | MCR_RTS);
	port->msr &= ~(MSR_DSR | MSR_CD | MSR_RI);
	spin_unlock_irqrestore(&port->port_lock, flags);
	dev->on_mode = 0;
	schedule_work(&dev->sw_work);

	dev->alt_set = 0;
	usb_ept_enable(dev->dev_in_ep, 0);
	usb_ept_enable(dev->dev_out_ep, 0);
	schedule_work(&dev->sw_alt_work);

}
/*
 * obex_configure
 *
 * Configures the device by enabling device specific
 * optimizations, setting up the endpoints, allocating
 * read and write requests and queuing read requests.
 *
 * The device lock must be held when calling this function.
 */
static void obex_configure(int config, void *_ctxt)
{
	int i, ret = 0;
	unsigned MaxPacketSize;
	struct obex_dev *dev = _ctxt;
	struct usb_endpoint *ep;
	struct usb_request *req;
	struct obex_port *port;
	struct list_head *rhead;
	struct list_head *whead;
	unsigned started = 0;

	if (dev == NULL) {
		printk(KERN_ERR "obex_configure: NULL device pointer\n");
		return;
	}
	if (!dev->bound)
		return;

	port = dev->dev_port[0];
	rhead = &port->read_pool;
	whead = &port->write_pool;
	if (port == NULL) {
		printk(KERN_ERR "obex_configure:port is NULL\n");
		return;
	}


	if (!config) {
		obex_debug("obex_configure: Deconfigure\n");
		dev->configured = OBEX_UNCONFIGURED;
		obex_reset_config(dev);
		return;
	}
	dev->dev_config = config;

	port->msr |= MSR_DSR;
	wake_up_interruptible(&port->msr_change_wait);

	if (dev->dev_in_ep == NULL || dev->dev_out_ep == NULL) {
		printk(KERN_ERR "obex_configure : cannot find endpoints\n");
		ret = -ENODEV;
		goto reset_config;
	}

	if (usb_msm_get_speed() == USB_SPEED_HIGH) {
		usb_configure_endpoint(dev->dev_in_ep, &dev->obex_hs_bulkin_desc);
		usb_configure_endpoint(dev->dev_out_ep,
					&dev->obex_hs_bulkout_desc);
	} else {
		usb_configure_endpoint(dev->dev_in_ep, &dev->obex_fs_bulkin_desc);
		usb_configure_endpoint(dev->dev_out_ep,
					&dev->obex_fs_bulkout_desc);
	}
	if(dev->alt_set == 1){
		usb_ept_enable(dev->dev_in_ep, 1);
		usb_ept_enable(dev->dev_out_ep, 1);
	}

	obex_debug("obex_configure: endpoint sizes and buffers\n");
	/* allocate and queue read requests */
	ep = dev->dev_out_ep;
	MaxPacketSize = usb_ept_get_max_packet(ep);
	for (i = 0; i < obex_read_q_size; i++) {
		req = obex_alloc_req(ep, MaxPacketSize);
		if (req) {
			req->device = (void *)dev;
			req->length = MaxPacketSize;
			req->complete = obex_read_complete;
			list_add_tail(&req->list, rhead);
			obex_debug("obex_configure: queuing read request(%d)\n", i);
		} else {
			printk(KERN_ERR
			"obex_configure: cannot allocate read request(%d)\n", i);
			goto reset_config;
		}
	}

	/* allocate write requests, and put on free list */
	ep = dev->dev_in_ep;
	MaxPacketSize = usb_ept_get_max_packet(ep);
	for (i = 0; i < obex_write_q_size; i++) {
		req = obex_alloc_req(ep, MaxPacketSize);
		if (req) {
			req->device = (void *)dev;
			req->length = MaxPacketSize;
			req->complete = obex_write_complete;
			list_add_tail(&req->list, whead);
		} else {
			printk(KERN_ERR
			"obex_configure: cannot allocate write request(%d)\n", i);
			goto reset_config;
		}
	}

	if (port->port_open_count) {
		unsigned long flags;
		spin_lock_irqsave(&port->port_lock, flags);
		started = obex_start_rx(dev);
		obex_send(dev);
		spin_unlock_irqrestore(&port->port_lock, flags);
		if (started)
			tty_wakeup(port->port_tty);
	}

	dev->configured = OBEX_CONFIGURED;
#if 0
	switch_set_state(&dev->sdev, dev->dev_config);
#endif
	dev->on_mode = 1;
	schedule_work(&dev->sw_work);
	return;

reset_config:
	printk(KERN_ERR "obex_configure(end): error, calling obex_reset_config\n");
	obex_reset_config(dev);
	return;
}

static unsigned obex_start_rx(struct obex_dev *dev)
{
	struct obex_port *port = dev->dev_port[0];
	struct list_head *pool = &port->read_pool;
	unsigned ret = 0;
	struct usb_endpoint *ep = dev->dev_out_ep;
	unsigned started = 0;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		struct tty_struct	*tty;
		tty = port->port_tty;
		if (!tty) {
			printk(KERN_ERR "%s: tty is null\n", __func__);
			break;
		}

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		spin_unlock(&port->port_lock);
		ret = usb_ept_queue_xfer(ep, req);
		spin_lock(&port->port_lock);
		if (ret) {
			list_add(&req->list, pool);
			break;
		}
		started++;

	}
	return started;
}
/*
 * obex_reset_config
 *
 * Mark the device as not configured, disable all endpoints,
 * which forces completion of pending I/O and frees queued
 * requests, and free the remaining write requests on the
 * free list.
 *
 * The device lock must be held when calling this function.
 */
static void obex_reset_config(struct obex_dev *dev)
{
	struct obex_port *port;
	struct usb_request *req;
	unsigned long flags;

	if (dev == NULL) {
		printk(KERN_ERR "obex_reset_config: NULL device pointer\n");
		return;
	}

	port = dev->dev_port[0];



	if (dev->dev_out_ep)
		usb_free_endpoint_all_req(dev->dev_out_ep);
	if (dev->dev_in_ep)
		usb_free_endpoint_all_req(dev->dev_in_ep);

	spin_lock_irqsave(&port->port_lock, flags);
	dev->dev_config = OBEX_NO_CONFIG_ID;

#if 0
	switch_set_state(&dev->sdev, dev->dev_config);
	schedule_work(&dev->sw_work);
#endif

	/* free write requests on the free list */
	while (!list_empty(&port->write_pool)) {
		req = list_entry(port->write_pool.next,
				       struct usb_request, list);
		list_del(&req->list);
		obex_free_req(dev->dev_in_ep, req);
	}

	/* free read requests from read pool */
	while (!list_empty(&port->read_pool)) {
		req = list_entry(port->read_pool.next,
				       struct usb_request, list);
		list_del(&req->list);
		obex_free_req(dev->dev_out_ep, req);
	}

	/* free read requests from read queue */
	while (!list_empty(&port->read_queue)) {
		req = list_entry(port->read_queue.next,
				       struct usb_request, list);
		list_del(&req->list);
		obex_free_req(dev->dev_out_ep, req);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * obex_alloc_req
 *
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */
static struct usb_request *obex_alloc_req(struct usb_endpoint *ep,
					unsigned int len)
{
	struct usb_request *req;
	if (ep == NULL)
		return NULL;
	req = usb_ept_alloc_req(ep, len);
	return req;
}

/*
 * obex_free_req
 *
 * Free a usb_request and its buffer.
 */
static void obex_free_req(struct usb_endpoint *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL){
		usb_ept_free_req(ep, req);
	}
}

/*
 * obex_alloc_ports
 *
 * Allocate all ports and set the obex_dev struct to point to them.
 * Return 0 if successful, or a negative error number.
 *
 * The device lock is normally held when calling this function.
 */
static int obex_alloc_ports(struct obex_dev *dev, gfp_t kmalloc_flags)
{
	int i;
	struct obex_port *port;

	if (dev == NULL)
		return -EIO;

	for (i = 0; i < OBEX_NUM_PORTS; i++) {
		port = kzalloc(sizeof(struct obex_port), kmalloc_flags);
		if (port == NULL)
			return -ENOMEM;

		INIT_WORK(&port->push_work, obex_rx_push);
		INIT_LIST_HEAD(&port->read_pool);
		INIT_LIST_HEAD(&port->read_queue);
		INIT_LIST_HEAD(&port->write_pool);
		port->msr = 0;
		port->prev_msr = 0;
		port->mcr = 0;
		init_waitqueue_head(&port->msr_change_wait);
		port->port_dev = dev;
		port->port_num = i;
		port->port_line_coding.dwDTERate =
		    cpu_to_le32(OBEX_DEFAULT_DTE_RATE);
		port->port_line_coding.bCharFormat = OBEX_DEFAULT_CHAR_FORMAT;
		port->port_line_coding.bParityType = OBEX_DEFAULT_PARITY;
		port->port_line_coding.bDataBits = OBEX_DEFAULT_DATA_BITS;
		spin_lock_init(&port->port_lock);
		init_waitqueue_head(&port->port_write_wait);

		dev->dev_port[i] = port;
	}

	return 0;
}

/*
 * obex_free_ports
 *
 * Free all closed ports.  Open ports are disconnected by
 * freeing their write buffers, setting their device pointers
 * and the pointers to them in the device to NULL.  These
 * ports will be freed when closed.
 *
 * The device lock is normally held when calling this function.
 */
static void obex_free_ports(struct obex_dev *dev)
{
	int i;
	unsigned long flags;
	struct obex_port *port;

	if (dev == NULL)
		return;

	for (i = 0; i < OBEX_NUM_PORTS; i++) {
		port = dev->dev_port[i];
		if (port != NULL) {
			dev->dev_port[i] = NULL;

			spin_lock_irqsave(&port->port_lock, flags);

			if (port->port_write_buf != NULL) {
				obex_buf_free(port->port_write_buf);
				port->port_write_buf = NULL;
			}

			if (port->port_open_count > 0 || port->port_in_use) {
				port->port_dev = NULL;
				wake_up_interruptible(&port->port_write_wait);
				if (port->port_tty) {
					wake_up_interruptible
					    (&port->port_tty->read_wait);
					wake_up_interruptible
					    (&port->port_tty->write_wait);
				}
				spin_unlock_irqrestore(&port->port_lock, flags);
			} else {
				spin_unlock_irqrestore(&port->port_lock, flags);
				kfree(port);
			}

		}
	}
}

/* Circular Buffer */

/*
 * obex_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */
static struct obex_buf *obex_buf_alloc(unsigned int size, gfp_t kmalloc_flags)
{
	struct obex_buf *gb;

	if (size == 0)
		return NULL;

	gb = kmalloc(sizeof(struct obex_buf), kmalloc_flags);
	if (gb == NULL)
		return NULL;

	gb->buf_buf = kmalloc(size, kmalloc_flags);
	if (gb->buf_buf == NULL) {
		kfree(gb);
		return NULL;
	}

	gb->buf_size = size;
	gb->buf_get = gb->buf_put = gb->buf_buf;

	return gb;
}

/*
 * obex_buf_free
 *
 * Free the buffer and all associated memory.
 */
void obex_buf_free(struct obex_buf *gb)
{
	if (gb) {
		kfree(gb->buf_buf);
		kfree(gb);
	}
}

/*
 * obex_buf_clear
 *
 * Clear out all data in the circular buffer.
 */
void obex_buf_clear(struct obex_buf *gb)
{
	if (gb != NULL)
		gb->buf_get = gb->buf_put;
	/* equivalent to a get of all data available */
}

/*
 * obex_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */
unsigned int obex_buf_data_avail(struct obex_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_put - gb->buf_get)
		    % gb->buf_size;
	else
		return 0;
}

/*
 * obex_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
unsigned int obex_buf_space_avail(struct obex_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_get - gb->buf_put - 1)
		    % gb->buf_size;
	else
		return 0;
}

/*
 * obex_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Return the number of bytes copied.
 */
unsigned int obex_buf_put(struct obex_buf *gb, const char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = obex_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf + len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else		/* count == len */
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

/*
 * obex_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
unsigned int obex_buf_get(struct obex_buf *gb, char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = obex_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf + len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else		/* count == len */
			gb->buf_get = gb->buf_buf;
	}

	return count;
}

/*
* obex_tiocmget
*/
static int obex_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct obex_port *port;
	unsigned int mcr, msr;
	unsigned int result = 0;
	struct obex_dev *dev = obex_devices[tty->index];

	if (dev == NULL)
		return -EIO;

	port = dev->dev_port[0];
	if (port == NULL)
		return -EIO;

	mcr = port->mcr;
	msr = port->msr;

	result = ((mcr & MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & MSR_CD) ? TIOCM_CD : 0)
		| ((msr & MSR_RI) ? TIOCM_RI : 0)
		| ((msr & MSR_DSR) ? TIOCM_DSR : 0)
		| ((msr & MSR_CTS) ? TIOCM_CTS : 0);
	return result;
}

/*
* obex_tiocmset
*/
static int obex_tiocmset(struct tty_struct *tty, struct file *file,
	unsigned int set, unsigned int clear)
{
	struct obex_port *port;
	unsigned int mcr;
	unsigned int msr;
	struct obex_dev *dev = obex_devices[tty->index];

	if (dev == NULL)
		return -EIO;
	port = dev->dev_port[0];

	if (port == NULL)
		return -EIO;

	mcr = port->mcr;
	msr = port->msr;
	if (dev->configured != OBEX_CONFIGURED)
		return -EIO;

	if (set & TIOCM_DTR)
		mcr |= MCR_DTR;
	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_LOOP)
		mcr |= MCR_LOOP;
	if (set & TIOCM_RI)
		msr |= MSR_RI;
	if (set & TIOCM_CD)
		msr |= MSR_CD;

	if (clear & TIOCM_DTR)
		mcr &= ~MCR_DTR;
	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_LOOP)
		mcr &= ~MCR_LOOP;
	if (clear & TIOCM_RI)
		msr &= ~MSR_RI;
	if (clear & TIOCM_CD)
		msr &= ~MSR_CD;

	port->mcr = mcr;
	port->msr = msr;

	if (port->prev_msr != port->msr) {
		port->prev_msr = port->msr;
	}

	return 0;
}

static struct async_icount obex_current_icount(struct obex_port *port)
{
	struct async_icount icount;

	if (port->msr & MSR_RI)
		icount.rng = TIOCM_RI;
	else
		icount.rng = 0;

	if (port->msr & MSR_DSR)
		icount.dsr = TIOCM_DSR;
	else
		icount.dsr = 0;

	if (port->msr & MSR_CD)
		icount.dcd = TIOCM_CD;
	else
		icount.dcd = 0;

	if (port->msr & MSR_CTS){
		icount.cts = TIOCM_CTS;
	}
	else{
		icount.cts = 0;
	}
	return icount;
}
