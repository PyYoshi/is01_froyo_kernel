/*
 * ***************************************************************************
 *
 *  FILE:     send.c
 * 
 *  PURPOSE:
 *      Code for adding a signal request to the from-host queue.
 *      When the driver bottom-half is run, it will take requests from the
 *      queue and pass them to the UniFi.
 *      
 * Copyright (C) 2005-2009 by Cambridge Silicon Radio Ltd.
 *
 * Refer to LICENSE.txt included with this source code for details on
 * the license terms.
 *
 * ***************************************************************************
 */
#include "driver/unifi.h"
#include "driver/conversions.h"
#include "driver/sigs.h"
#include "card.h"

static CsrUint32
unifi_frame_priority_to_queue(CsrUint32 priority)
{
    switch (priority) {
        case CSR_QOS_UP0:
        case CSR_QOS_UP3:
            return 1;
        case CSR_QOS_UP1:
        case CSR_QOS_UP2:
            return 0;
        case CSR_QOS_UP4:
        case CSR_QOS_UP5:
            return 2;
        case CSR_QOS_UP6:
        case CSR_QOS_UP7:
            return 3;
        default:
            return 1;
    }
}

/*
 * ---------------------------------------------------------------------------
 *  send_signal
 *
 *      This function queues a signal for sending to UniFi.  It first checks
 *      that there is space on the fh_signal_queue for another entry, then
 *      claims any bulk data slots required and copies data into them. Then
 *      increments the fh_signal_queue write count.
 *      
 *      The fh_signal_queue is later processed by the driver bottom half
 *      (in unifi_bh()).
 *
 *      This function call unifi_pause_xmit() to pause the flow of data plane
 *      packets when:
 *        - the fh_signal_queue ring buffer is full
 *        - there are less than UNIFI_MAX_DATA_REFERENCES (2) bulk data
 *          slots available.
 *
 *  Arguments:
 *      card            Pointer to card context structure
 *      sigptr          Pointer to the signal to write to UniFi.
 *      siglen          Number of bytes pointer to by sigptr.
 *      bulkdata        Array of pointers to an associated bulk data.
 *      sigq            To which from-host queue to add the signal.
 *
 *  Returns:
 *      0 on success
 *      -CSR_EIO if there were insufficient data slots or no free signal queue entry
 *
 * Notes:
 *      Calls unifi_pause_xmit() when the last slots are used.
 * ---------------------------------------------------------------------------
 */
static CsrInt32
send_signal(card_t *card, const CsrUint8 *sigptr, CsrUint32 siglen,
            const bulk_data_param_t *bulkdata,
            q_t *sigq, CsrUint32 priority_q, CsrUint32 run_bh)
{
    CsrUint16 i, data_slot_size;
    card_signal_t *csptr;
    CsrInt16 qe;
    CsrInt32 r;
    CsrInt16 debug_print = 0;

    data_slot_size = CardGetDataSlotSize(card);

    /* Check that the fh_data_queue has a free slot */
    if (!q_slots_free(sigq)) {
        unifi_error(card->ospriv, "send_signal: %s full\n", sigq->name);

        return -CSR_ENOSPC;
    }

    /*
     * Now add the signal to the From Host signal queue
     */
    /* Get next slot on queue */
    qe = q_next_w_slot(sigq);
    csptr = q_slot_data(sigq, qe);

    /* Make up the card_signal struct */
    csptr->signal_length = (CsrUint16)siglen;
    CsrMemCpy((void*)csptr->sigbuf, (void*)sigptr, siglen);

    for (i = 0; i < UNIFI_MAX_DATA_REFERENCES; ++i) {
        if ((bulkdata != NULL) && (bulkdata->d[i].data_length != 0)) {
            CsrUint32 datalen = bulkdata->d[i].data_length;

            /* Make sure data will fit in a bulk data slot */
            if (bulkdata->d[i].os_data_ptr == NULL) {
                unifi_error(card->ospriv, "send_signal - NULL bulkdata[%d]\n", i);
                debug_print++;
                csptr->bulkdata[i].data_length = 0;
            } else {
                if (datalen > data_slot_size) {
                    unifi_error(card->ospriv,
                                "send_signal - Invalid data length %u (@%p), "
                                "truncating\n",
                                datalen, bulkdata->d[i].os_data_ptr);
                    datalen = data_slot_size;
                    debug_print++;
                }
                /* Store the bulk data info in the soft queue. */
                csptr->bulkdata[i].os_data_ptr = (CsrUint8*)bulkdata->d[i].os_data_ptr;
                csptr->bulkdata[i].os_net_buf_ptr = (CsrUint8*)bulkdata->d[i].os_net_buf_ptr;
                csptr->bulkdata[i].net_buf_length = bulkdata->d[i].net_buf_length;
                csptr->bulkdata[i].data_length = datalen;
            }
        } else {
            unifi_init_bulk_data(&csptr->bulkdata[i]);
        }
    }

    if (debug_print) {
        const CsrUint8 *sig = sigptr;
        
        unifi_error(card->ospriv, "Signal(%d): %02x %02x %02x %02x %02x %02x %02x %02x"
                    " %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    siglen,
                    sig[0], sig[1], sig[2], sig[3],
                    sig[4], sig[5], sig[6], sig[7],
                    sig[8], sig[9], sig[10], sig[11],
                    sig[12], sig[13], sig[14], sig[15]);
        unifi_error(card->ospriv, "Bulkdata pointer %p(%d), %p(%d)\n",
                    bulkdata != NULL ? bulkdata->d[0].os_data_ptr : NULL,
                    bulkdata != NULL ? bulkdata->d[0].data_length : 0,
                    bulkdata != NULL ? bulkdata->d[1].os_data_ptr : NULL,
                    bulkdata != NULL ? bulkdata->d[1].data_length : 0);
    }

    /* Advance the written count to say there is a new entry */
    q_inc_w(sigq);

    /* 
     * Set the flag to say reason for waking was a host request.
     * Then ask the OS layer to run the unifi_bh.
     */
    if (run_bh == 1) {
        card->bh_reason_host = 1;
        r = unifi_run_bh(card->ospriv);
        if (r) {
            unifi_error(card->ospriv, "failed to run bh.\n");
            card->bh_reason_host = 0;

            /*
             * The bulk data buffer will be freed by the caller.
             * We need to invalidate the description of the bulk data in our
             * soft queue, to prevent the core freeing the bulk data again later.
             */
            for (i = 0; i < UNIFI_MAX_DATA_REFERENCES; ++i) {
                if (csptr->bulkdata[i].data_length != 0) {
                    csptr->bulkdata[i].os_data_ptr = csptr->bulkdata[i].os_net_buf_ptr = NULL;
                    csptr->bulkdata[i].net_buf_length = csptr->bulkdata[i].data_length = 0;
                }
            }
            return r;
        }
    } else {
        unifi_error(card->ospriv, "run_bh=%d, bh not called.\n", run_bh);
    }

    /* 
     * Have we used up all the fh signal list entries?
     */
    if (q_slots_free(sigq) == 0) {
        /* We have filled the queue, so stop the upper layer. The command queue
         * is an exception, as suspending due to that being full could delay
         * resume/retry until new commands or data are received.
         */
        if (sigq != &card->fh_command_queue) {
            /*
             * Call unifi_pause_xmit() before seting the paused flag.
             * This means that if this thread is interrupted by the bh thread
             * unifi_restart_xmit() may be called twice.
             * Also, set the paused_queue before the paused flag
             * to avoid races using the wrong queue.
             */
            unifi_pause_xmit(card->ospriv);
            card->paused_queue = priority_q;
            card->paused = 1;
        } else {
            unifi_trace(card->ospriv, UDBG1,
                        "send_signal: fh_cmd_q full, not pausing (run_bh=%d)\n",
                        run_bh);
        }
    }

    func_exit();

    return 0;
} /*  send_signal() */


/*
 * ---------------------------------------------------------------------------
 *  unifi_send_signal
 *
 *    Invokes send_signal() to queue a signal in the command or traffic queue
 *    If sigptr pointer is NULL, it pokes the bh to check if UniFi is responsive.
 * 
 *  Arguments:
 *      card        Pointer to card context struct
 *      sigptr      Pointer to signal from card.
 *      siglen      Size of the signal
 *      bulkdata    Pointer to the bulk data of the signal
 *
 *  Returns:
 *      0 on success
 *      -CSR_ENOSPC if there were insufficient data slots or no free signal queue entry
 *
 *  Notes:
 *      unifi_send_signal() is used to queue signals, created by the driver,
 *      to the device. Signals are constructed using the UniFi packed structures.
 * ---------------------------------------------------------------------------
 */
CsrInt32
unifi_send_signal(card_t *card, const CsrUint8 *sigptr, CsrUint32 siglen,
                  const bulk_data_param_t *bulkdata)
{
    q_t *sig_soft_q;
    CsrUint16 signal_id;
    CsrInt32 r;
    CsrUint32 run_bh;
    CsrUint32 priority_q;

    /* A NULL signal pointer is a request to check if UniFi is responsive */
    if (sigptr == NULL) {
        card->bh_reason_host = 1;
        return unifi_run_bh(card->ospriv);
    }

    priority_q = 0;
    run_bh = 1;
    signal_id = GET_SIGNAL_ID(sigptr);
    /*
     * If the signal is a CSR_MA_UNITDATA_REQUEST or a CSR_DS_UNITDATA_REQUEST,
     * we send it using the traffic soft queue. Else we use the command soft queue.
     */
    if (signal_id == CSR_MA_UNITDATA_REQUEST_ID) {

        CsrUint32 frame_priority;

        if (card->periodic_wake_mode == UNIFI_PERIODIC_WAKE_HOST_ENABLED) {
            run_bh = 0;
        }

        /* Map the frame priority to a traffic queue index. */
        frame_priority = GET_PACKED_MA_UNIDATA_REQUEST_FRAME_PRIORITY(sigptr);
        priority_q = unifi_frame_priority_to_queue(frame_priority);

        sig_soft_q = &card->fh_traffic_queue[priority_q];
    } else if (signal_id == CSR_MA_PACKET_REQUEST_ID) {

        CsrUint32 frame_priority;

        if (card->periodic_wake_mode == UNIFI_PERIODIC_WAKE_HOST_ENABLED) {
            run_bh = 0;
        }

        /* Map the frame priority to a traffic queue index. */
        frame_priority = GET_PACKED_MA_PACKET_REQUEST_FRAME_PRIORITY(sigptr);
        priority_q = unifi_frame_priority_to_queue(frame_priority);

        sig_soft_q = &card->fh_traffic_queue[priority_q];

    } else if (signal_id == CSR_DS_UNITDATA_REQUEST_ID) {
        sig_soft_q = &card->fh_traffic_queue[0];
    } else {
        sig_soft_q = &card->fh_command_queue;
    }

    r = send_signal(card, sigptr, siglen, bulkdata, sig_soft_q, priority_q, run_bh);
    /* On error, the caller must free or requeue bulkdata buffers */

    return r;
} /* unifi_send_signal() */


/*
 * ---------------------------------------------------------------------------
 *  unifi_send_resources_available
 *
 *      Examines whether there is available space to queue 
 *      a signal in the command or traffic queue
 * 
 *  Arguments:
 *      card        Pointer to card context struct
 *      sigptr      Pointer to signal.
 *
 *  Returns:
 *      0 on success
 *      -CSR_ENOSPC if there was no free signal queue entry
 *
 *  Notes:
 * ---------------------------------------------------------------------------
 */
CsrInt32
unifi_send_resources_available(card_t *card, const CsrUint8 *sigptr)
{
    q_t *sig_soft_q;
    CsrUint16 signal_id = GET_SIGNAL_ID(sigptr);

    /*
     * If the signal is a CSR_MA_UNITDATA_REQUEST or a CSR_DS_UNITDATA_REQUEST,
     * we send it using the traffic soft queue. Else we use the command soft queue.
     */
    
    if (signal_id == CSR_MA_UNITDATA_REQUEST_ID) {
        CsrUint32 frame_priority;
        CsrUint32 priority_q;

        /* Map the frame priority to a traffic queue index. */
        frame_priority = GET_PACKED_MA_UNIDATA_REQUEST_FRAME_PRIORITY(sigptr);
        priority_q = unifi_frame_priority_to_queue(frame_priority);

        sig_soft_q = &card->fh_traffic_queue[priority_q];
    } else if (signal_id == CSR_MA_PACKET_REQUEST_ID) {
        CsrUint32 frame_priority;
        CsrUint32 priority_q;

        /* Map the frame priority to a traffic queue index. */
        frame_priority = GET_PACKED_MA_PACKET_REQUEST_FRAME_PRIORITY(sigptr);
        priority_q = unifi_frame_priority_to_queue(frame_priority);

        sig_soft_q = &card->fh_traffic_queue[priority_q];
    } else if (signal_id == CSR_DS_UNITDATA_REQUEST_ID) {
        sig_soft_q = &card->fh_traffic_queue[0];
    } else {
        sig_soft_q = &card->fh_command_queue;
    }

    /* Check that the fh_data_queue has a free slot */
    if (!q_slots_free(sig_soft_q)) {
        unifi_notice(card->ospriv, "unifi_send_resources_available: %s full\n",
                     sig_soft_q->name);
        return -CSR_ENOSPC;
    }

    return 0;
} /* unifi_send_resources_available() */

