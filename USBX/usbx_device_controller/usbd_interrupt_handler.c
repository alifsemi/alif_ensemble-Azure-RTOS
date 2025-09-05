/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     usbd_interrupt_handler.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function is the interrupt handler for usb controller.
 *           The controller will trigger an interrupt when something happens on
 *           an endpoint whose mask has been set in the interrupt enable
 *           register, or when a bus reset is detected.   .
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "usbd_spec.h"

static void usbd_event_buffer_handler(USB_DRIVER *drv, USBD_EVENT_BUFFER *event_buffer)
{
    uint32_t reg;

    RTSS_InvalidateDCache_by_Addr(event_buffer->buf, USB_EVENT_BUFFER_SIZE);
    while (event_buffer->count) {
        reg = ((uint32_t) (*((uint32_t *) ((uint8_t *) event_buffer->buf + event_buffer->lpos))));
        /* Check type of event */
        if ((reg & USB_EVENT_TYPE_CHECK) == USB_DEV_EVENT_TYPE) {
            /* process device specific events  */
            usbd_devt_handler(drv, reg);
        } else if ((reg & USB_EVENT_TYPE_CHECK) == USB_EP_EVENT_TYPE) {
            /* process  endpoint specific events  */
            usbd_depevt_handler(drv, reg);
        } else {
#ifdef DEBUG
            printf("Unknown events\n");
#endif
        }
        event_buffer->lpos      = (event_buffer->lpos + USB_EVENT_CNT_SIZE) % USB_EVENT_BUFFER_SIZE;
        event_buffer->count    -= USB_EVENT_CNT_SIZE;
        drv->regs->GEVNTCOUNT0  = USB_EVENT_CNT_SIZE;
    }
    /* Unmask interrupt */
    event_buffer->count = 0;
}

/**
 *\fn           usbd_interrupt_handler
 *\brief        service the usb interrupts.
 *\return       none
 **/
void usbd_interrupt_handler(USB_DRIVER *drv)
{
    uint32_t           pending_interrupt;
    uint32_t           mask_interrupt;
    USBD_EVENT_BUFFER *event_buf;

    /* Get event pointer ...*/
    event_buf          = drv->event_buf;
    pending_interrupt  = drv->regs->GEVNTCOUNT0;
    pending_interrupt &= USB_GEVNTCOUNT_MASK;
    if (!pending_interrupt) {
#ifdef DEBUG
        printf("no pending irq\n");
#endif
        return;
    }
    event_buf->count = pending_interrupt;
    /* Set the Event Interrupt Mask */
    mask_interrupt   = drv->regs->GEVNTSIZ0;
    SET_BIT(mask_interrupt, USB_GEVNTSIZ_INTMASK);
    drv->regs->GEVNTSIZ0 = mask_interrupt;
    NVIC_DisableIRQ(USB_IRQ_IRQn);
    /* Processes events in an Event Buffer */
    usbd_event_buffer_handler(drv, event_buf);
    NVIC_EnableIRQ(USB_IRQ_IRQn);
    /* Clear the Event Interrupt Mask */
    mask_interrupt = drv->regs->GEVNTSIZ0;
    CLEAR_BIT(mask_interrupt, USB_GEVNTSIZ_INTMASK);
    drv->regs->GEVNTSIZ0 = mask_interrupt;
}

/**
 *\fn           usbd_prepare_setup
 *\brief        Prepare the setup TRB to recv the setup pkt from the host
                and send the start transfer command.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

void usbd_prepare_setup(USB_DRIVER *drv)
{
    USBD_EP_PARAMS params = {0};
    USBD_TRB      *trb_ptr;
    USBD_EP       *ept;
    uint32_t       ret;

    /* Setup packet always on EP0 */
    ept     = &drv->eps[USB_CONTROL_EP];
    trb_ptr = &drv->ep0_trb;
    RTSS_CleanDCache_by_Addr(&drv->setup_data, USB_SETUP_PKT_SIZE);

    trb_ptr->buf_ptr_low  = LOWER_32_BITS(LocalToGlobal(&drv->setup_data));
    trb_ptr->buf_ptr_high = 0;
    trb_ptr->size         = USB_TRB_SIZE_LENGTH(USB_SETUP_PKT_SIZE);
    trb_ptr->ctrl         = USB_TRBCTL_CONTROL_SETUP;
    SET_BIT(trb_ptr->ctrl,
            USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI);

    RTSS_CleanDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    params.param1  = (uint32_t) trb_ptr;
    drv->ep0_state = EP0_SETUP_PHASE;
    /* Issue the command to the hardware */
    ret            = usbd_send_ep_cmd(drv, 0U, USB_DEPCMD_STARTTRANSFER, params);
    if (ret) {
#ifdef DEBUG
        printf("Failed to send the command for setup pkt:%" PRIu32 "\n", ret);
#endif
    }
    SET_BIT(ept->ep_status, USB_EP_BUSY);
    ept->ep_resource_index = usbd_get_ep_transfer_resource_index(drv, ept->ep_index, ept->ep_dir);
}

/**
 *\fn           usbd_get_ep_transfer_resource_index
 *\brief        Get the transfer resource index for the endpoint.
                and send the start transfer command.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint num
 *\param[in]    endpoint direction
 *\return       transfer resource index.
 **/

uint32_t usbd_get_ep_transfer_resource_index(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    uint8_t  phy_ep;
    uint32_t resource_index;

    phy_ep         = USB_GET_PHYSICAL_EP(ep_num, dir);
    /* [22:16]: Transfer Resource Index (XferRscIdx). The hardware-assigned
     * transfer resource index for the transfer
     */
    resource_index = drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMD;

    return USB_DEPCMD_GET_RSC_IDX(resource_index);
}

/**
 *\fn           usbd_clear_stall_all_eps
 *\brief        Clears the stall for all endponts.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

void usbd_clear_stall_all_eps(USB_DRIVER *drv)
{
    uint8_t        phy_ep;
    USBD_EP       *ept;
    int32_t        ret;
    USBD_EP_PARAMS params = {0};

    for (phy_ep = 1U; phy_ep < (drv->out_eps + drv->in_eps); phy_ep++) {
        ept = &drv->eps[phy_ep];
        /* Skip if endpoint is not enabled */
        if ((ept->ep_status & USB_EP_ENABLED) == 0U) {
            continue;
        }
        /* Skip if endpoint is not stalled */
        if ((ept->ep_status & USB_EP_STALL) == 0U) {
            continue;
        }
        /* Send the command to the hardware */
        ret = usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
        if (ret < 0) {
#ifdef DEBUG
            printf("Failed to send the for clearstall\n");
#endif
            return;
        }
        CLEAR_BIT(ept->ep_status, USB_EP_STALL);
    }
}

/**
 *\fn           usbd_stop_transfer
 *\brief        stop the current transfers on endpoint.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint num
 *\param[in]    endpoint direction
 *\param[in]    force rm
 *\return       On success 0, failure case returns error.
 **/

int32_t usbd_stop_transfer(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint32_t force_rm)
{
    USBD_EP       *ept;
    USBD_TRB      *trb_ptr;
    USBD_EP_PARAMS params = {0};
    uint8_t        phy_ep;
    uint32_t       cmd;
    uint32_t       ret;

    phy_ep  = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept     = &drv->eps[phy_ep];
    trb_ptr = &ept->ep_trb[ept->trb_enqueue];
    if (trb_ptr->ctrl) {
        ept->trb_enqueue = 0;
        trb_ptr->ctrl    = 0;
    }
    /* check the endpoint stall condition */
    if (ept->ep_status & USB_EP_STALL) {
        ret = usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
        if (ret < 0) {
#ifdef DEBUG
            printf("Failed to send command CLEARSTALL Ep\n");
#endif
            return ret;
        }
        CLEAR_BIT(ept->ep_status, USB_EP_STALL);
    }

    if (ept->ep_resource_index == 0U) {
        return USB_EP_RESOURCE_INDEX_INVALID;
    }

    /* Data book says for end transfer  HW needs some
     * extra time to synchronize with the interconnect
     * - Issue EndTransfer WITH CMDIOC bit set
     * - Wait 100us
     */
    cmd  = USB_DEPCMD_ENDTRANSFER;
    cmd |= (force_rm == 1) ? USB_DEPCMD_HIPRI_FORCERM : 0U;
    SET_BIT(cmd, USB_DEPCMD_CMDIOC);
    cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);

    ret  = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
    if (ret < 0) {
#ifdef DEBUG
        printf("Failed to send command at END transfer\n");
#endif
        return ret;
    }
    if (force_rm == 1) {
        ept->ep_resource_index = 0U;
    }
    CLEAR_BIT(ept->ep_status, USB_EP_BUSY);
    sys_busy_loop_us(100);

    return ret;
}

/**
 *\fn           usbd_ep0_end_control_data
 *\brief        stop sending control ep data using ENDTRANSFER command.
 *\param[in]    pointer to the controller context structure
 *\param[in]    pointer to the endpoint structure
 *\return       none.
 **/

void usbd_ep0_end_control_data(USB_DRIVER *drv, USBD_EP *ept)
{
    USBD_EP_PARAMS params = {0};
    uint32_t       cmd;
    uint32_t       ret;
    uint8_t        phy_ep;

    if (ept->ep_resource_index == 0U) {
        return;
    }
    phy_ep = USB_GET_PHYSICAL_EP(ept->ep_index, ept->ep_dir);
    cmd    = USB_DEPCMD_ENDTRANSFER;
    SET_BIT(cmd, USB_DEPCMD_CMDIOC);
    cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
    ret  = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
    if (ret < 0) {
#ifdef DEBUG
        printf("Failed to send command at Endcontrol data\n");
#endif
        return;
    }
    ept->ep_resource_index = 0U;
}

/**
 *\fn           usbd_ep0_stall_restart
 *\brief        stall the control ep and restart.
                stall is always issued on EP0 not EP1
                EP0-> control ep OUT, EP1-> control ep IN
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint num
 *\param[in]    endpoint direction.
 *\return       On success 0, failure case returns error..
 **/

int32_t usbd_ep0_stall_restart(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    uint8_t        phy_ep;
    USBD_EP       *ept    = NULL;
    USBD_EP_PARAMS params = {0};
    uint32_t       ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept    = &drv->eps[phy_ep];
    /* Issue the command to the hardware */
    ret    = usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETSTALL, params);
    if (ret < 0) {
#ifdef DEBUG
        printf("failed to send STALL command\n");
#endif
        return ret;
    }
    ept->ep_status |= USB_EP_STALL;
    /* when we issued stall on EP0 OUT after immediately
     * software has to schedule the setup TRB for next setup packet.
     */
    usbd_prepare_setup(drv);
    return ret;
}

/**
 *\fn           usbd_ep0_data_done
 *\brief        This function is called when Xfercomplete event being generated for control
                data transfer, it will check the TRB status of data transfer and read the
                number transferred bytes from the TRB.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint number
 *\return       none.
 **/

void usbd_ep0_data_done(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_EP  *ept;
    USBD_TRB *trb_ptr;
    uint32_t  trb_status;
    uint8_t   length;

    ept     = &drv->eps[endp_number];
    trb_ptr = &drv->ep0_trb;
    RTSS_InvalidateDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    trb_status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);
    if (trb_status == USB_TRBSTS_SETUP_PENDING) {
        drv->setup_packet_pending = true;
#ifdef DEBUG
        printf("TRB transmission pending in control DATA_PHASE\n");
#endif
        return;
    }
    length          = trb_ptr->size & USB_TRB_SIZE_MASK;
    ept->bytes_txed = ept->ep_requested_bytes - length;
}

/**
 *\fn           usbd_ep0_start_status
 *\brief        This function is called when xferNotReady(status) event generated for control
                endpoint, prepares the status TRB for two stage and three stage control transfers
                and issue the start transfer command.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint number
 *\return       On success 0, failure case returns error..
 **/

int32_t usbd_ep0_start_status(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_EP       *ept;
    USBD_TRB      *trb_ptr;
    USBD_EP_PARAMS params = {0};
    uint32_t       type;
    uint32_t       ret;
    uint8_t        dir;

    ept = &drv->eps[endp_number];

    if ((ept->ep_status & USB_EP_BUSY) != 0U) {
#ifdef DEBUG
        printf("Ep is busy\n");
#endif
        return USB_EP_BUSY_ERROR;
    }

    type = (drv->three_stage_setup != false) ? USB_TRBCTL_CONTROL_STATUS3 :
            USB_TRBCTL_CONTROL_STATUS2;
    trb_ptr = &drv->ep0_trb;

    RTSS_CleanDCache_by_Addr(&drv->setup_data, USB_SETUP_PKT_SIZE);
    /* we use same trb_ptr for setup packet and status */
    trb_ptr->buf_ptr_low  = LOWER_32_BITS(LocalToGlobal(&drv->setup_data));
    trb_ptr->buf_ptr_high = 0;
    trb_ptr->size         = USB_TRB_SIZE_LENGTH(0U);
    trb_ptr->ctrl         = type;
    SET_BIT(trb_ptr->ctrl,
            USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI);

    RTSS_CleanDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    params.param1  = (uint32_t) trb_ptr;
    drv->ep0_state = EP0_STATUS_PHASE;
    /*
     * Control OUT transfer - Status stage happens on EP0 IN - EP1
     * Control IN transfer - Status stage happens on EP0 OUT - EP0
     */
    dir            = !drv->ep0_expect_in;
    ret            = usbd_send_ep_cmd(drv, 0U | dir, USB_DEPCMD_STARTTRANSFER, params);

    if (ret < USB_SUCCESS) {
#ifdef DEBUG
        printf("failed to execute the command at control status\n");
#endif
        return ret;
    }

    SET_BIT(ept->ep_status, USB_EP_BUSY);
    ept->ep_resource_index = usbd_get_ep_transfer_resource_index(drv, ept->ep_index, ept->ep_dir);

    return ret;
}

/**
 *\fn           usbd_ep0_status_done
 *\brief        This function is called when status send/recv completed for control endpoint
                it will check the status of TRB and prepares the setup TRB for next setup pkt
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

void usbd_ep0_status_done(USB_DRIVER *drv)
{
    USBD_TRB *trb_ptr;
    uint32_t  trb_status;

    trb_ptr = &drv->ep0_trb;
    RTSS_InvalidateDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    trb_status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);
    if (trb_status == USB_TRBSTS_SETUP_PENDING) {
        drv->setup_packet_pending = true;
#ifdef DEBUG
        printf("status pending at status done\n");
#endif
    }
    drv->ep0_state = EP0_SETUP_PHASE;
    usbd_prepare_setup(drv);
}

/**
 *\fn           usbd_ep_xfercomplete
 *\brief        This function is called when xfer_progress event being generated for non control
                endpoint transfers completes, it will check the TRB status and read the number
                of transferred bytes from the TRB.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint number
 *\return       none.
 **/

void usbd_ep_xfercomplete(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_TRB *trb_ptr;
    USBD_EP  *ept;
    uint32_t  length;
    uint8_t   dir;
    uint32_t  status;

    ept     = &drv->eps[endp_number];
    dir     = ept->ep_dir;
    trb_ptr = &ept->ep_trb[ept->trb_dequeue];
    RTSS_InvalidateDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));

    status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);
    /* only for Isochronous endpoint */
    if (status == USB_TRBSTS_MISSED_ISOC) {
#ifdef DEBUG
        printf("Missed Ischronous transfer\n");
#endif
        return;
    }
    if (status == USB_TRBSTS_SETUP_PENDING) {
        drv->setup_packet_pending = true;
#ifdef DEBUG
        printf("TRB transmission pending at data stage\n");
#endif
        return;
    }
    ept->trb_dequeue++;
    if (ept->trb_dequeue == NO_OF_TRB_PER_EP) {
        ept->trb_dequeue = 0U;
    }
    length = trb_ptr->size & USB_TRB_SIZE_MASK;
    if (length == 0U) {
        ept->bytes_txed = ept->ep_requested_bytes;
    } else {
        if (dir == USB_DIR_IN) {
            ept->bytes_txed = ept->ep_requested_bytes - length;
        } else {
            if (ept->unaligned_txed == 1U) {
                ept->bytes_txed      = ROUND_UP(ept->ep_requested_bytes, ept->ep_maxpacket);
                ept->bytes_txed     -= length;
                ept->unaligned_txed  = 0U;
            } else {
                /* Get the actual number of bytes transmitted by host */
                ept->bytes_txed = ept->ep_requested_bytes - length;
            }
        }
    }
    drv->num_bytes = ept->bytes_txed;
}

/**
 *\fn           usbd_clear_trb
 *\brief        clears the TRB HW0 bit.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint number
 *\return       none.
 **/

void usbd_clear_trb(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_TRB *trb_ptr;
    USBD_EP  *ept;

    ept     = &drv->eps[endp_number];
    trb_ptr = &ept->ep_trb[ept->trb_dequeue];
    if (trb_ptr->ctrl & USB_TRB_CTRL_HWO) {
        CLEAR_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO);
    }
}

/**
 *\fn           usbd_devt_handler
 *\brief        process the Device specific events.
 *\param[in]    pointer to the controller context structure
 *\param[in]    Event buffer reg value.
 *\return       none.
 **/

void usbd_devt_handler(USB_DRIVER *drv, uint32_t reg)
{
    uint32_t event_type;

    /* Device specific events */
    event_type = USB_DEVT_TYPE(reg);
    switch (event_type) {
    case USB_EVENT_WAKEUP:
        break;
    case USB_EVENT_DISCONNECT:
        if (drv->ux_dcd_disconnect_cb != NULL) {
            drv->ux_dcd_disconnect_cb();
        }
        break;
    case USB_EVENT_EOPF:
        break;
    case USB_EVENT_RESET:
        usbd_reset_event(drv);
        if (drv->ux_dcd_reset_cb != NULL) {
            drv->ux_dcd_reset_cb(drv);
        }
        break;
    case USB_EVENT_CONNECT_DONE:
        usbd_connectiondone_event(drv);
        if (drv->ux_dcd_connect_cb != NULL) {
            drv->ux_dcd_connect_cb();
        }
        break;
    case USB_EVENT_LINK_STATUS_CHANGE:
        break;
    case USB_EVENT_HIBER_REQ:
        break;
    default:
        break;
    }
}

/**
 *\fn           usbd_reset_event
 *\brief        This function is called when a usb reset event detected on bus
                clears the stall for all ep's and reset the device address to zero.
 *\param[in]    pointer to the controller context structure
 *\return       none.
 **/

void usbd_reset_event(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t ep_num;

    reg              = drv->regs->DCTL;
    reg             &= ~USB_DCTL_TSTCTRL_MASK;
    drv->regs->DCTL  = reg;
    /* Clear STALL on all endpoints */
    usbd_clear_stall_all_eps(drv);
    for (ep_num = 0U; ep_num < (drv->in_eps + drv->out_eps); ep_num++) {
        drv->eps[ep_num].ep_status = 0U;
    }
    /* Reset device address to zero */
    reg              = drv->regs->DCFG;
    reg             &= ~(USB_DCFG_DEVADDR_MASK);
    drv->regs->DCFG  = reg;
}

/**
 *\fn           usbd_connectiondone_event
 *\brief        This function is called when the USB device has successfully completed
                the connection handshake with the host (i.e., pull-up has been detected
                and the device is now considered connected by the host
 *\param[in]    pointer to the controller context structure
 *\return       none.
 **/
void usbd_connectiondone_event(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t speed;

    reg   = drv->regs->DSTS;
    /* if speed value is 0 then it's HIGH SPEED */
    speed = reg & USB_DSTS_CONNECTSPD;
    if (speed == USB_DSTS_HIGHSPEED) {
        drv->speed = USB_HIGH_SPEED_DEVICE;
    }
    /* Enable USB2 LPM Capability */
    reg              = drv->regs->DCFG;
    reg             |= USB_DCFG_LPM_CAP;
    drv->regs->DCFG  = reg;
    reg              = drv->regs->DCTL;
    reg             |= USB_DCTL_HIRD_THRES_MASK;
    drv->regs->DCTL  = reg;
}

void usbd_isoc_micro_frame_update(USB_DRIVER *drv)
{
    uint32_t  microframe_number;
    uint32_t reg = drv->regs->DSTS;

    /* Read the frame number from DSTS register */
    microframe_number = USB_DSTS_SOFFN(reg);
    /* Compare the microframe_number with 14 bit value microframe number */
    bool rollover = microframe_number < (drv->micro_frame_number & 0x3FFF);
    drv->micro_frame_number = (drv->micro_frame_number & ~0x3FFF) | microframe_number;
    if (rollover) {
        drv->micro_frame_number += BIT(14);
    }
}

/* Endpoint specific events */
void usbd_depevt_handler(USB_DRIVER *drv, uint32_t reg)
{
    USBD_EP *ept;
    uint8_t  endp_number;
    uint32_t event_status;
    uint32_t event_type;

    /* Event status from bit[15:12]  */
    event_status     = USB_GET_EP_EVENT_STATUS(reg);
    endp_number      = USB_GET_DEPEVT_EP_NUM(reg);
    drv->endp_number = endp_number;
    ept              = &drv->eps[endp_number];
    if (!(ept->ep_status & USB_EP_ENABLED)) {
#ifdef DEBUG
        printf("endpoint has not enabled\n");
#endif
        return;
    }
    /*  Get the event type  */
    event_type      = USB_GET_DEPEVT_TYPE(reg);
    /* Event type can be used for Debugging purpose */
    drv->event_type = event_type;
    if (endp_number == USB_CTRL_PHY_EP0 || endp_number == USB_CTRL_PHY_EP1) {
        RTSS_InvalidateDCache_by_Addr(&drv->setup_data, USB_SETUP_PKT_SIZE);
        RTSS_InvalidateDCache_by_Addr(&drv->ep0_trb, sizeof(drv->ep0_trb));
        /* Get the physical endpoint associated with this endpoint.  */
        ept                     = &drv->eps[endp_number];
        /* Reset the endpoint transfer status. */
        ept->ep_transfer_status = USB_EP_TRANSFER_IDLE;
        /* Process the ep0 interrupts bases on event_type. */
        if (event_type == USB_DEPEVT_XFERNOTREADY) {
            if (event_status == USB_DEPEVT_STATUS_CONTROL_DATA) {
                /* if we receive a XferNotReady(DATA) we will ignore it, unless
                 * it's for the wrong direction.
                 */
                if (endp_number != drv->ep0_expect_in) {
#ifdef DEBUG
                    printf("unexpected direction for the data phase\n");
#endif
                    usbd_ep0_end_control_data(drv, ept);
                    usbd_ep0_stall_restart(drv, 0, 0);
                }
            } else if (event_status == USB_DEPEVT_STATUS_CONTROL_STATUS) {
                drv->ep0_state = EP0_STATUS_PHASE;
                usbd_ep0_start_status(drv, endp_number);
            } else {
                /* Do nothing  */
            }
        } else if (event_type == USB_DEPEVT_XFERCOMPLETE) {
            USBD_CTRL_REQUEST *setup;

            setup = &drv->setup_data;
            ept   = &drv->eps[endp_number];
            if (endp_number == 0) {
                CLEAR_BIT(ept->ep_status, USB_EP_STALL);
            }
            CLEAR_BIT(ept->ep_status, USB_EP_BUSY);
            ept->ep_resource_index    = 0U;
            drv->setup_packet_pending = false;
            switch (drv->ep0_state) {
            case EP0_SETUP_PHASE:
                ept->ep_transfer_status = USB_EP_TRANSFER_SETUP;
                if (setup->wLength == 0U) {
                    drv->three_stage_setup = false;
                    drv->ep0_expect_in     = false;
                } else {
                    drv->three_stage_setup = true;
                    drv->ep0_expect_in     = !!(setup->bRequestType & USB_REQUEST_IN);
                }
                if (drv->ux_dcd_setup_stage_cb != NULL) {
                    drv->ux_dcd_setup_stage_cb(drv, 0);
                }
                break;
            case EP0_DATA_PHASE:
                ept->ep_transfer_status = USB_EP_TRANSFER_DATA_COMPLETION;
                usbd_ep0_data_done(drv, endp_number);
                if (drv->ux_dcd_ep0_data_stage_cb != NULL) {
                    drv->ux_dcd_ep0_data_stage_cb(drv, endp_number);
                }
                break;
            case EP0_STATUS_PHASE:
                ept->ep_transfer_status = USB_EP_TRANSFER_STATUS_COMPLETION;
                usbd_ep0_status_done(drv);
                break;
            default:
                break;
            }
        } else {
#ifdef DEBUG
            printf("some other events\n");
#endif
        }
    } else {
        /* non control ep events */
        uint32_t cmd_param;

        switch (event_type) {
        case USB_DEPEVT_XFERINPROGRESS:
            usbd_ep_xfercomplete(drv, endp_number);
            if (drv->ux_dcd_data_stage_cb != NULL) {
                drv->ux_dcd_data_stage_cb(drv, endp_number);
            }
            break;
        case USB_DEPEVT_XFERCOMPLETE:
            break;
        case USB_DEPEVT_XFERNOTREADY:
            /* for Isochronous only */
            /* Get the frame from event param BIT[31:16] */
            cmd_param = USB_GET_EVENT_CMD_PARAM(reg);
            drv->micro_frame_number = cmd_param;
            usbd_isoc_micro_frame_update(drv);
            if (drv->ux_dcd_xfernotready_data != NULL) {
                drv->ux_dcd_data_stage_cb(drv, endp_number);
            }
            break;
        default:
            break;
        }
    }
}

/**
 *\fn           usbd_usb2phy_config_check
 *\brief        Save the current phy state of GUSB2PHYCFG0 and disable the lpm and suspend bits
 *\param[in]    pointer to the controller context structure.
 *\return       none.
 **/

void usbd_usb2phy_config_check(USB_DRIVER *drv)
{
    uint32_t reg;

    drv->usb2_phy_config = 0;
    reg             = drv->regs->GUSB2PHYCFG0;

    if (reg & USB_GUSB2PHYCFG_SUSPHY) {
        SET_BIT(drv->usb2_phy_config, USB_GUSB2PHYCFG_SUSPHY);
        CLEAR_BIT(reg, USB_GUSB2PHYCFG_SUSPHY);
    }

    if (reg & USB_GUSB2PHYCFG_ENBLSLPM) {
        SET_BIT(drv->usb2_phy_config, USB_GUSB2PHYCFG_ENBLSLPM);
        CLEAR_BIT(reg, USB_GUSB2PHYCFG_ENBLSLPM);
    }

    if (drv->usb2_phy_config) {
        drv->regs->GUSB2PHYCFG0 = reg;
    }
}

/**
 *\fn           usbd_usb2phy_config_reset
 *\brief        Restore the phy state into GUSB2PHYCFG0 reg.
 *\param[in]    pointer to the controller context structure.
 *\return       none.
 **/
/* Restore the phy state */
void usbd_usb2phy_config_reset(USB_DRIVER *drv)
{
    uint32_t reg;

    if (drv->usb2_phy_config) {
        reg                      = drv->regs->GUSB2PHYCFG0;
        reg                     |= drv->usb2_phy_config;
        drv->regs->GUSB2PHYCFG0  = reg;
    }
}

/**
 *\fn            usbd_set_device_address
 *\brief        This function is used to set device address
 *\param[in]    pointer to controller context structure
 *\param[in]    Address to be set
 *\return       On success 0, failure cases error
 */
uint32_t usbd_set_device_address(USB_DRIVER *drv, uint8_t addr)
{
    uint32_t reg;

    if (addr > USB_DEVICE_MAX_ADDRESS) {
#ifdef DEBUG
        printf("Invalid address\n");
#endif
        return USB_DEVICE_SET_ADDRESS_INVALID;
    }
    if (drv->config_state == USBD_STATE_CONFIGURED) {
#ifdef DEBUG
        printf("address can't set from Configured State\n");
#endif
        return USB_DEVICE_ALREADY_CONFIGURED;
    }
    reg              = drv->regs->DCFG;
    reg             &= ~(USB_DCFG_DEVADDR_MASK);
    reg             |= USB_DCFG_DEVADDR(addr);
    drv->regs->DCFG  = reg;

    if (addr > USB_DEVICE_DEFAULT_ADDRESS) {
        drv->config_state = USBD_STATE_ADDRESS;
    } else {
        drv->config_state = USBD_STATE_DEFAULT;
    }

    return USB_SUCCESS;
}

uint32_t usbd_state_change(USB_DRIVER *drv, uint32_t state)
{
    drv->config_state = state;
    return USB_SUCCESS;
}
