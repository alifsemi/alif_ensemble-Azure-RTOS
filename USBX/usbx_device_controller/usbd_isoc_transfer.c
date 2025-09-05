/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     usbd_isoc_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     13-Aug-2024
 * @brief    This file will transfers the Isochronous data.
 * @bug      None
 * @Note     None
 ******************************************************************************/

#include "usbd_spec.h"

/**
 *\fn           usbd_isoc_send
 *\brief        This function will prepare the TRB for sending isoc data to the host.
 *\param[in]    pointer to the controller context structure
 *\param[in]    endpoint number
 *\param[in]    endpoint direction
 *\param[in]    buffer pointer
 *\param[in]    buffer length
 *\return       On success 0, else error.
 **/
int32_t usbd_isoc_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir, uint8_t *bufferptr,
                       uint32_t buf_len)
{
    USBD_EP       *ept;
    USBD_TRB      *trb_ptr;
    USBD_EP_PARAMS params = {0};
    uint8_t        phy_ep;
    int32_t        ret;
    uint32_t       cmd;
    uint32_t       mult;

    phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
    ept    = &drv->eps[phy_ep];
    if (ept->ep_dir != USB_DIR_IN) {
#ifdef DEBUG
        printf("Direction is wrong returning\n");
#endif
        return USB_EP_DIRECTION_WRONG;
    }
    ept->bytes_txed         = 0U;
    ept->ep_requested_bytes = buf_len;

    trb_ptr                 = &ept->ep_trb[ept->trb_enqueue];
    ept->trb_enqueue++;
    if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
        ept->trb_enqueue = 0U;
    }
    RTSS_CleanDCache_by_Addr(bufferptr, buf_len);
    trb_ptr->buf_ptr_low  = LOWER_32_BITS(LocalToGlobal((uint32_t *) bufferptr));
    trb_ptr->buf_ptr_high = 0;
    trb_ptr->size         = USB_TRB_SIZE_LENGTH(buf_len);
    /* For Isochronous transfers use First TRB as USB_TRBCTL_ISOCHRONOUS_FIRST
     * and other TRB's as USB_TRBCTL_ISOCHRONOUS
     */
    trb_ptr->ctrl         = USB_TRBCTL_ISOCHRONOUS_FIRST;

    /* For High-Speed, High-Bandwidth IN endpoints, a maximum of three packets can be sent during an
     * interval. The PktCntM1 field ([25:24] of the third DWORD) must be set to the (number of
     * packets in the Buffer Descriptor - 1 "example :If there is only a single transaction in the
     * microframe, only a DATA0 data packet PID is used.  If there are two transactions per
     * microframe, DATA1 is used for the first transaction data packet and DATA0 is used for the
     * second transaction data packet.  If there are three transactions per microframe, DATA2 is
     * used for the first transaction data packet, DATA1 is used for the second, and DATA0 is used
     * for the third".
     *
     */
    /* If there are three transactions per microframe mult should be 2 */

    /* 1) length <= maxpacket
     *  - DATA0
     *
     * 2) maxpacket < length <= (2 * maxpacket)
     *  - DATA1, DATA0
     *
     * 3) (2 * maxpacket) < length <= (3 * maxpacket)
     *  - DATA2, DATA1, DATA0
     */
    mult                  = USB_ISOC_MAX_MULT_VALUE;
    if (buf_len <= (2 * (ept->ep_maxpacket))) {
        mult--;
    }
    if (buf_len <= ept->ep_maxpacket) {
        mult--;
    }
    trb_ptr->size |= USB_TRB_PCM1(mult);
    /* For Isochronous always enables the Interrupt on Missed ISOC  */
    SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI);

    RTSS_CleanDCache_by_Addr(trb_ptr, sizeof(*trb_ptr));
    params.param1 = (uint32_t) trb_ptr;
    if ((ept->ep_status & USB_EP_BUSY) != 0U) {
        cmd  = USB_DEPCMD_UPDATETRANSFER;
        cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
    } else {
        cmd  = USB_DEPCMD_STARTTRANSFER;
        cmd |= USB_DEPCMD_PARAM(drv->micro_frame_number);
    }
    /* Issue the command to hardware */
    ret = usbd_send_ep_cmd(drv, phy_ep, cmd, params);
    if (ret < USB_SUCCESS) {
#ifdef DEBUG
        printf("failed to send the command\n");
#endif
        return ret;
    }
    if ((ept->ep_status & USB_EP_BUSY) == 0U) {
        ept->ep_resource_index =
            usbd_get_ep_transfer_resource_index(drv, ept->ep_index, ept->ep_dir);

        SET_BIT(ept->ep_status, USB_EP_BUSY);
    }

    return ret;
}
