#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/vmalloc.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>

#define MODNAME "stheno"

#define MINOR_COUNT 1 //

//
static int stheno_major = 0;

//
static struct semaphore stheno_sem;
static struct request_queue *stheno_queue;
static spinlock_t stheno_lock;
static struct task_struct* stheno_thread;
static wait_queue_head_t stheno_process_q;
static int stheno_processing = 0;

//
struct gendisk *stheno_gd;
#define SECT_SIZE	(512)   // sector size(byte)
#define MAX_SECTORS (64*2)
#define SECT_NUM	(0x4000000) // amount of sector
#define stheno_printk			if(0)printk


typedef struct _stheno_segment_info_ {
    unsigned int   cmd;                             /* READ/WRITE */
    sector_t       start_sector;
    unsigned long  current_pos;
    unsigned long  nr_sectors;
    unsigned int   dirty;
    unsigned char  buffer[SECT_SIZE * MAX_SECTORS];
} stheno_segment_info;

static stheno_segment_info seg_info;

//
//
//
//
//
extern int euryale_write_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer);
extern int euryale_read_process(unsigned long sector, unsigned long current_nr_sectors, char* buffer);

static int stheno_do_request(void* arg)
{
    while(1)
    {
        wait_event_interruptible(stheno_process_q, stheno_processing == 1);

        stheno_processing = 0;

        do
        {
            struct request *req;
            int ret;

            spin_lock_irq( stheno_queue->queue_lock );
            req = elv_next_request( stheno_queue );
            spin_unlock_irq( stheno_queue->queue_lock );

            if(req == NULL)break;

            // ignore not fs cmd
            if( ! blk_fs_request( req ) )
            {
                printk( KERN_ERR "skip no fs request\n" );
                end_request( req, 0 );
                continue;
            }

            // ignore nr_sectors == 0 request
            if( req->nr_sectors == 0 || req->current_nr_sectors == 0)
            {
                printk( KERN_ERR "skip nr_sectors == (%d) current_nr_sectors == (%d) request\n", (int)req->nr_sectors, (int)req->current_nr_sectors );
                end_request( req, 0 );
                continue;
            }

            stheno_printk( KERN_NOTICE "stheno_request: REQUEST START! %s sect(%d) curr_count(%d) count(%d)\n", (rq_data_dir( req ) == WRITE) ? "write" : "read",  (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);

            if( seg_info.dirty )
            {
                stheno_printk( KERN_NOTICE "stheno_request:                %s dirty(%d) seg_start_sect(%d) seg_pos(%d) seg_nr_count(%d)\n", (rq_data_dir( req ) == WRITE) ? "write" : "read",  (int)seg_info.dirty,  (int)seg_info.start_sector, (int)seg_info.current_pos, (int)seg_info.nr_sectors );
            }

            if( seg_info.dirty && rq_data_dir( req ) != seg_info.cmd )
            {
                if( seg_info.cmd == WRITE )
                {
                    stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) start\n", (int)seg_info.start_sector, (int)seg_info.current_pos);
                    ret = euryale_write_process(seg_info.start_sector, seg_info.current_pos, seg_info.buffer);
                    stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) ret(%d) end\n", (int)seg_info.start_sector, (int)seg_info.current_pos, ret);
                    if( ret < 0 )
                    {
                        printk( KERN_ERR "stheno_request: segment write sect(%d) count(%d) ERROR\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                    }
                }
                seg_info.dirty = 0;
            }
    
            if( rq_data_dir( req ) == WRITE )
            {
                if( !seg_info.dirty
                    && req->current_nr_sectors < req->nr_sectors )
                {
                    stheno_printk( KERN_NOTICE "stheno_request: new_segment(%d) cur_count(%d) count(%d)\n", (int)seg_info.dirty, (int)req->current_nr_sectors, (int)req->nr_sectors);

                    seg_info.cmd = rq_data_dir( req ); 
                    seg_info.start_sector = req->sector;
                    seg_info.nr_sectors = req->nr_sectors;
                    memcpy( seg_info.buffer, req->buffer, req->current_nr_sectors * SECT_SIZE);
                    seg_info.current_pos = req->current_nr_sectors;
                    seg_info.dirty = 1;

                    end_request( req, 1 );
                    continue;
                }
                else if( seg_info.dirty
                    && seg_info.start_sector + seg_info.current_pos == req->sector
                    && seg_info.nr_sectors - seg_info.current_pos == req->nr_sectors
                    && seg_info.nr_sectors - seg_info.current_pos >= req->current_nr_sectors )
                {
                    stheno_printk( KERN_NOTICE "stheno_request: add_segment(%d) cur_count(%d) count(%d)\n", (int)seg_info.dirty, (int)req->current_nr_sectors, (int)req->nr_sectors);

                    memcpy( seg_info.buffer + (seg_info.current_pos * SECT_SIZE), req->buffer, req->current_nr_sectors * SECT_SIZE);
                    seg_info.current_pos += req->current_nr_sectors;
                    if( seg_info.current_pos == seg_info.nr_sectors )
                    {
                        stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) start\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                        ret = euryale_write_process(seg_info.start_sector, seg_info.current_pos, seg_info.buffer);
                        stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) end\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                        seg_info.dirty = 0;
                        if( ret < 0 )
                        {
                            printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) ERROR\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                        }
                    }

                    end_request( req, 1 );
                    continue;
                }
                else if( seg_info.dirty )
                {
                    stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) start\n", (int)seg_info.start_sector, (int)seg_info.current_pos);
                    ret = euryale_write_process(seg_info.start_sector, seg_info.current_pos, seg_info.buffer);
                    stheno_printk( KERN_NOTICE "stheno_request: segment write sect(%d) count(%d) end\n", (int)seg_info.start_sector, (int)seg_info.current_pos);
                    seg_info.dirty = 0;
                    if( ret < 0 )
                    {
                        printk( KERN_ERR "stheno_request: segment write sect(%d) count(%d) ERROR\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                    }
                }
            }
            else /* READ */
            {
                 if( !seg_info.dirty 
                    && req->current_nr_sectors < req->nr_sectors )
                {
                    seg_info.cmd = rq_data_dir( req ); 
                    seg_info.start_sector = req->sector;
                    seg_info.nr_sectors = req->nr_sectors;

                    stheno_printk( KERN_NOTICE "stheno_request: segment read sect(%d) count(%d) start\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                    ret = euryale_read_process(seg_info.start_sector, seg_info.nr_sectors, seg_info.buffer);
                    stheno_printk( KERN_NOTICE "stheno_request: segment read sect(%d) count(%d) end\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                    if( ret < 0 )
                    {
                        printk( KERN_ERR "stheno_request: segment read sect(%d) count(%d) ERROR\n", (int)seg_info.start_sector, (int)seg_info.nr_sectors);
                        end_request( req, 0 );
                        continue;
                    }

                    memcpy(req->buffer, seg_info.buffer, req->current_nr_sectors * SECT_SIZE);
                    seg_info.current_pos = req->current_nr_sectors;
                    seg_info.dirty = 1;

                    end_request( req, 1 );
                    continue;
                }
                else if( seg_info.dirty
                    && seg_info.start_sector + seg_info.current_pos == req->sector
                    && seg_info.nr_sectors - seg_info.current_pos == req->nr_sectors
                    && seg_info.nr_sectors - seg_info.current_pos >= req->current_nr_sectors )
                {
                    memcpy(req->buffer, seg_info.buffer + (seg_info.current_pos * SECT_SIZE), req->current_nr_sectors * SECT_SIZE);
                    seg_info.current_pos += req->current_nr_sectors;
                    if( seg_info.current_pos == seg_info.nr_sectors )
                    {
                        seg_info.dirty = 0;
                    }

                    end_request( req, 1 );
                    continue;
                }
                else
                {
                    seg_info.dirty = 0;
                }
            }

            if( rq_data_dir( req ) == WRITE )
            {
                stheno_printk( KERN_NOTICE "stheno_request: write sect(%d) cur_count(%d) count(%d) start\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                ret = euryale_write_process(req->sector, req->current_nr_sectors, req->buffer);
                stheno_printk( KERN_NOTICE "stheno_request: write sect(%d) cur_count(%d) count(%d) end\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                if( ret < 0 )
                {
                    printk( KERN_ERR "stheno_request: write sect(%d) cur_count(%d) count(%d) ERROR\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                }
            }
            else
            {
                stheno_printk( KERN_NOTICE "stheno_request: read sect(%d) cur_count(%d) count(%d) start\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                ret = euryale_read_process(req->sector, req->current_nr_sectors, req->buffer);
                stheno_printk( KERN_NOTICE "stheno_request: read sect(%d) cur_count(%d) count(%d) end\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                if( ret < 0 )
                {
                    printk( KERN_ERR "stheno_request: read sect(%d) cur_count(%d) count(%d) ERROR\n", (int)req->sector, (int)req->current_nr_sectors, (int)req->nr_sectors);
                    end_request( req, 0 );
                    continue;
                }
            }

            //
            end_request( req, 1 );
        }
        while(1);

    }
    
    return 0;
}

static void stheno_request( struct request_queue *queue )
{
    stheno_processing = 1;

    wake_up_interruptible( &stheno_process_q );
}

static int stheno_getgeo(struct block_device* bdev, struct hd_geometry* geo)
{
    memset(geo, 0, sizeof(struct hd_geometry));

    geo->heads = 128;
    geo->sectors = 63;
    geo->cylinders = SECT_NUM / geo->heads / geo->sectors;
    geo->start = 0;

    return 0;
}

//
//
static struct block_device_operations stheno_fops = {
    .owner   = THIS_MODULE,
    .open    = NULL,
    .release = NULL,
    .ioctl   = NULL,
    .getgeo  = stheno_getgeo
};


//
static int __init stheno_module_init( void )
{
    //
    stheno_major = register_blkdev( 0, MODNAME );
    if( stheno_major <= 0 ){
	printk( KERN_WARNING "register_blkdev failed\n" );
        return stheno_major;
    }

    //
    spin_lock_init( &stheno_lock );
    stheno_queue = blk_init_queue(
        stheno_request, //
        &stheno_lock );
    if( ! stheno_queue ){
	printk( KERN_WARNING "blk_init_queue failed\n" );        
        unregister_blkdev( stheno_major, MODNAME );
        return -ENOMEM;
    }
    blk_queue_hardsect_size( stheno_queue, SECT_SIZE );
    blk_queue_max_sectors( stheno_queue, MAX_SECTORS );
	blk_queue_bounce_limit(stheno_queue, BLK_BOUNCE_ANY);

    //
    stheno_gd = alloc_disk( MINOR_COUNT );
    if( ! stheno_gd ){
	printk( KERN_WARNING "alloc_disk failed\n" );        
        blk_cleanup_queue( stheno_queue );
        unregister_blkdev( stheno_major, MODNAME );
        return -ENOMEM;
    }
    sprintf( stheno_gd->disk_name, "%s", MODNAME ); //
    stheno_gd->queue = stheno_queue; //
    stheno_gd->major = stheno_major;
    stheno_gd->first_minor = 0;
    stheno_gd->fops = &stheno_fops;  //
    set_capacity( stheno_gd, SECT_NUM );

    sema_init(&stheno_sem, 1);
    init_waitqueue_head(&stheno_process_q);

    stheno_thread = kthread_create(stheno_do_request, 0, "sthenod");
    
    wake_up_process(stheno_thread);

    //
    //
    add_disk( stheno_gd );

    printk( KERN_INFO "stheno is loaded\n" );
    printk( KERN_INFO "major = %d\n", stheno_major );

    return 0;
}


//
static void __exit stheno_module_exit( void )
{
    //
    //
    del_gendisk( stheno_gd );
    put_disk( stheno_gd );

    //
    blk_cleanup_queue( stheno_queue );

    //
    unregister_blkdev( stheno_major, MODNAME );

    printk( KERN_INFO "stheno is removed\n" );
}
	

module_init( stheno_module_init );
module_exit( stheno_module_exit );
/* Copyright (C) 2009 SHARP CORPORATION All rights reserved. This software is licensed under the terms of the GNU General Public License version 2, as published by the Free Software Foundation, and may be copied, distributed, and modified under those terms. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. */
MODULE_DESCRIPTION( MODNAME );
MODULE_LICENSE( "GPL2" );

