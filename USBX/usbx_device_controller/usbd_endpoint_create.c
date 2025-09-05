/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     usbd_endpoint_create.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will create all types of endpoints.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "usbd_spec.h"

uint32_t usbd_ep_enable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t ep_type,
                        uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP *ept;
    uint32_t reg;
    uint8_t  phy_ep;
    uint32_t ret;

    phy_ep            = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept               = &drv->eps[phy_ep];
    ept->ep_index     = ep_num;
    ept->ep_dir       = dir;
    ept->ep_maxpacket = ep_max_packet_size;
    ept->phy_ep       = phy_ep;
    if (!(ept->ep_status & USB_EP_ENABLED)) {
        ret = usbd_start_endpoint_config(drv, ep_num, dir);
        if (ret) {
#ifdef DEBUG
            printf("endpoint config failed\n");
#endif
            return ret;
        }
    }
    ret = usbd_configure_endpoint_parameters(drv, ep_num, dir, ep_type, ep_max_packet_size,
            ep_interval);
    if (ret) {
#ifdef DEBUG
        printf("SetEP failed\n");
#endif
        return ret;
    }

    if (!(ept->ep_status & USB_EP_ENABLED)) {
        SET_BIT(ept->ep_status, USB_EP_ENABLED);

        reg                  = drv->regs->DALEPENA;
        reg                 |= USB_DALEPENA_EP(ept->phy_ep);
        drv->regs->DALEPENA  = reg;
        if (phy_ep > 1) {
            USBD_TRB *trb_ptr, *trb_link;
            /* Initialize TRB ring   */
            ept->trb_enqueue = 0;
            ept->trb_dequeue = 0;
            trb_ptr          = &ept->ep_trb[0U];
            /* Link TRB. The HWO bit is never reset */
            trb_link         = &ept->ep_trb[NO_OF_TRB_PER_EP];
            memset(trb_link, 0x0, sizeof(USBD_TRB));

            trb_link->buf_ptr_low   = LOWER_32_BITS(LocalToGlobal(trb_ptr));
            trb_link->buf_ptr_high  = 0;
            trb_link->ctrl         |= USB_TRBCTL_LINK_TRB;
            SET_BIT(trb_link->ctrl, USB_TRB_CTRL_HWO);

            return USB_SUCCESS;
        }

        return USB_SUCCESS;
    }

    return USB_SUCCESS;
}

/**
 *\fn           usbd_configure_endpoint_parameters
 *\brief        This function configures the endpoint parameters.
 *\param[in]    drv  pointer to the USB driver context structure
 *\param[in]    ep_num  endpoint number
 *\param[in]    dir  endpoint direction (USB_DIR_IN/USB_DIR_OUT)
 *\param[in]    ep_type  endpoint type
 *\param[in]    ep_max_packet_size  maximum packet size for the endpoint
 *\param[in]    ep_interval  polling interval for interrupt/isochronous endpoints
 *\return       On success returns 0, on failure returns error code
 **/

uint32_t usbd_configure_endpoint_parameters(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir,
        uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP_PARAMS params = {0};
    uint8_t        phy_ep;

    phy_ep         = USB_GET_PHYSICAL_EP(ep_num, dir);

    params.param0  = USB_DEPCFG_EP_TYPE(ep_type) | USB_DEPCFG_MAX_PACKET_SIZE(ep_max_packet_size);

    params.param0 |= USB_DEPCFG_ACTION_INIT;
    SET_BIT(params.param1, USB_DEPCFG_XFER_COMPLETE_EN | USB_DEPCFG_XFER_NOT_READY_EN);

    /*
     * We are doing 1:1 mapping for endpoints, meaning
     * Physical Endpoints 2 maps to Logical Endpoint 2 and
     * so on. We consider the direction bit as part of the physical
     * endpoint number. So USB endpoint 0x81 is 0x03.
     */
    params.param1 |= USB_DEPCFG_EP_NUMBER(phy_ep);

    if (dir != USB_DIR_OUT) {
        params.param0 |= USB_DEPCFG_FIFO_NUMBER(phy_ep >> 1U);
    }
    if (ep_type != 0) {
        SET_BIT(params.param1, USB_DEPCFG_XFER_IN_PROGRESS_EN);
    }

    if (ep_interval) {
        params.param1 |= USB_DEPCFG_BINTERVAL_M1(ep_interval - 1);
    }

    return usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETEPCONFIG, params);
}

/**
 *\fn           usbd_start_endpoint_config
 *\brief        This function configure the control ep and set the transfer resource
                index to all physical endpoints.
 *\param[in]    drv pointer to the controller context structure
 *\param[in]    endpoint num
 *\param[in]    endpoint direction
 *\return       On success returns 0, failure case returns error.
 **/

uint32_t usbd_start_endpoint_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    USBD_EP_PARAMS params = {0};
    uint32_t       cmd;
    uint8_t        phy_ep;
    uint8_t        ep_index;
    uint32_t       ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    if (phy_ep == 0) {
        cmd = USB_DEPCMD_DEPSTARTCFG;
        /* Issue the command to the hardware */
        ret = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
        if (ret) {
#ifdef DEBUG
            printf("USB_DEPCMD_DEPSTARTCFG cmd failed\n");
#endif
            return ret;
        }
        for (ep_index = 0; ep_index < (drv->in_eps + drv->out_eps); ep_index++) {

            ret = usbd_set_xfer_resource(drv, ep_index);
            if (ret) {
#ifdef DEBUG
                printf("Set XferResource cmd failed\n");
#endif
                return ret;
            }
        }
    }

    return USB_SUCCESS;
}

/**
 *\fn           usbd_set_xfer_resource
 *\brief        This function sends the transfer resource index command.
 *\param[in]    drv pointer to the controller context structure
 *\param[in]    phy_ep physical endpoint
 *\return       On success 0, failure case returns error.
 **/

uint32_t usbd_set_xfer_resource(USB_DRIVER *drv, uint8_t phy_ep)
{
    USBD_EP_PARAMS params = {0};
    /* Set Endpoint Transfer Resource configuration parameter
     */
    params.param0         = USB_DEPXFERCFG_NUM_XFER_RES(1U);
    return usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETTRANSFRESOURCE, params);
}

int32_t usbd_ep_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    uint8_t phy_ep;
    int32_t ret;
    USBD_EP_PARAMS params = {0};

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    /* Handle control endpoint (EP0) */
    if (ep_num == 0) {
        USBD_TRB *trb_ptr = &drv->ep0_trb;

        /* Check if hardware owns the TRB */
        if (trb_ptr->ctrl & USB_TRB_CTRL_HWO) {
            return USB_SUCCESS;
        }
        /* Issue the stall on control endpoint and restart */
        return usbd_ep0_stall_restart(drv, 0, 0);
    }
    /* Handle non-control endpoints */
    USBD_EP *ept = &drv->eps[phy_ep];

    ret = usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETSTALL, params);
    if (ret < USB_SUCCESS) {
#ifdef DEBUG
        printf("failed to send STALL command\n");
#endif
        return ret;
    }
    SET_BIT(ept->ep_status, USB_EP_STALL);

    return ret;
}

int32_t usbd_ep_clear_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    uint8_t        phy_ep;
    USBD_EP       *ept;
    USBD_EP_PARAMS params = {0};
    uint32_t       ret;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept    = &drv->eps[phy_ep];
    ret    = usbd_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
    if (ret < 0) {
#ifdef DEBUG
        printf("Failed to send command at STALL Ep\n");
#endif
        return ret;
    }
    CLEAR_BIT(ept->ep_status, USB_EP_STALL);

    return ret;
}
int32_t usbd_ep_disable(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    USBD_EP *ept;
    uint32_t reg;
    uint8_t  phy_ep;

    phy_ep               = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept                  = &drv->eps[phy_ep];
    reg                  = drv->regs->DALEPENA;
    reg                 &= ~USB_DALEPENA_EP(phy_ep);
    drv->regs->DALEPENA  = reg;
    CLEAR_BIT(ept->ep_status, USB_EP_ENABLED);

    return USB_SUCCESS;
}

/**
 *\fn           usbd_ep_transfer_abort
 *\brief        This function abort the transfers on endpoint.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint num
 *\param[in]    endpoint direction
 *\return       On success 0, failure case returns error.
 **/

int32_t usbd_ep_transfer_abort(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir)
{
    /* As per the data sheet ENDTRANSFER COMMAND,
     * Software issues this command requesting DMA to stop for the endpoint/stream specifying
     * the transfer resource index of the TRB and the ForceRM parameter to be set to 1 in
     *  the DEPCMD register.
     */

    /* When issuing an End Transfer command, software must set the CmdIOC bit (field 8)
     * so that an Endpoint Command Complete event is generated after the transfer ends.
     */
    return usbd_stop_transfer(drv, ep_num, dir, USB_DEPCMD_FORCERM);

}
