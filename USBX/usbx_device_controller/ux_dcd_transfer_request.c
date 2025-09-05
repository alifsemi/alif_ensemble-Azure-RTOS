/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     ux_dcd_transfer_request.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This file will have,initiate the all types of endpoint transfers to the driver
 *           .
 * @bug      None.
 * @Note     None.
 *******************************************************************************/

#include "ux_dcd.h"

extern TX_EVENT_FLAGS_GROUP DATA_IN_OUT_EVENT_FLAG;

/**
 *\fn           ux_dcd_control_transfer
 *\brief        This function transfers the control endpoint data to the driver.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to transfer structure
 *\return       0 on success, else error.
 **/

static int32_t ux_dcd_control_transfer(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir,
                                       UX_SLAVE_TRANSFER *transfer_request)
{
    int32_t status = UX_ERROR;

    if (transfer_request->ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT) {
        status = usbd_ep0_send(drv,
                               ep_num,
                               dir,
                               transfer_request->ux_slave_transfer_request_current_data_pointer,
                               transfer_request->ux_slave_transfer_request_requested_length);
        if (status != UX_SUCCESS) {
#ifdef DEBUG
            printf("control transfer failed with error: %d\n", status);
#endif
            return status;
        } else {
            transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
        }
    } else {
        /* do nothing  */
    }

    return status;
}

/**
 *\fn           ux_dcd_bulk_transfer
 *\brief        This function will schedule the bulk transfers.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the transfer structure.
 *\return       On success 0, else error.
 **/

static int32_t ux_dcd_bulk_transfer(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir,
                                    UX_SLAVE_TRANSFER *transfer_request)
{
    int32_t  status;
    uint32_t events;

    if (transfer_request->ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT) {
        status = usbd_bulk_send(drv,
                                ep_num,
                                dir,
                                transfer_request->ux_slave_transfer_request_current_data_pointer,
                                transfer_request->ux_slave_transfer_request_requested_length);
        if (status != UX_SUCCESS) {
            return status;
        }
        status = _ux_utility_event_flags_get(&DATA_IN_OUT_EVENT_FLAG,
                                             UX_DATA_IN_EVENT | UX_DISCONNECT_EVENT,
                                             UX_OR_CLEAR,
                                             (ULONG *) &events,
                                             transfer_request->ux_slave_transfer_request_timeout);
        if (status != UX_SUCCESS) {
            return status;
        } else {
            transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            return status;
        }
    } else if (transfer_request->ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_IN) {
        status = usbd_bulk_recv(drv,
                                ep_num,
                                dir,
                                transfer_request->ux_slave_transfer_request_current_data_pointer,
                                transfer_request->ux_slave_transfer_request_requested_length);
        if (status != UX_SUCCESS) {
            return status;
        }
        status = _ux_utility_event_flags_get(&DATA_IN_OUT_EVENT_FLAG,
                                             UX_DATA_OUT_EVENT | UX_DISCONNECT_EVENT,
                                             UX_OR_CLEAR,
                                             (ULONG *) &events,
                                             transfer_request->ux_slave_transfer_request_timeout);
        if (status != UX_SUCCESS) {
            return status;
        } else {
            transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            return status;
        }
    } else {
        /* Do nothing  */
    }

    return UX_SUCCESS;
}

/**
 *\fn           ux_dcd_isoc_transfer
 *\brief        This function will schedule the isoc transfer.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the transfer structure.
 *\return       On success 0, else error.
 **/

static int32_t ux_dcd_isoc_transfer(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir,
                                    UX_SLAVE_TRANSFER *transfer_request)
{
    int32_t  status;
    uint32_t events;

    if (transfer_request->ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT) {
        status = usbd_isoc_send(drv,
                                ep_num,
                                dir,
                                transfer_request->ux_slave_transfer_request_current_data_pointer,
                                transfer_request->ux_slave_transfer_request_requested_length);
        if (status != UX_SUCCESS) {
            return status;
        }
        status = _ux_utility_event_flags_get(&DATA_IN_OUT_EVENT_FLAG,
                                             UX_DATA_IN_EVENT | UX_DISCONNECT_EVENT,
                                             UX_OR_CLEAR,
                                             (ULONG *) &events,
                                             transfer_request->ux_slave_transfer_request_timeout);
        if (status != UX_SUCCESS) {
            return status;
        } else {
            transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
            return status;
        }
    } else {
        /* Do nothing  */
    }

    return UX_SUCCESS;
}

/**
 *\fn           ux_dcd_transfer_request
 *\brief        This function will call the endpoint specific transfers.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to transfer structure
 *\return       On success 0, else error.
 **/

int32_t ux_dcd_transfer_request(USB_DRIVER *drv, UX_SLAVE_TRANSFER *transfer_request)
{
    UX_SLAVE_ENDPOINT *endpoint;
    int32_t            status = UX_ERROR;
    uint8_t            ep_num;
    uint8_t            dir;

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint = transfer_request->ux_slave_transfer_request_endpoint;
    ep_num   = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    dir      = (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION)
                   ? USB_DIR_IN
                   : USB_DIR_OUT;
    switch (ux_endpoint_type(&endpoint->ux_slave_endpoint_descriptor)) {
    case UX_CONTROL_ENDPOINT:
        /* Control Transfers */
        status = ux_dcd_control_transfer(drv, ep_num, dir, transfer_request);
        break;
    case UX_BULK_ENDPOINT:
        /* BULK Transfers  */
        status = ux_dcd_bulk_transfer(drv, ep_num, dir, transfer_request);
        break;
    case UX_INTERRUPT_ENDPOINT:
        /* Interrupt Transfers */
        /* Not Yet Implemented */
        break;
    case UX_ISOCHRONOUS_ENDPOINT:
        /* Isochronous transfers  */
        status = ux_dcd_isoc_transfer(drv, ep_num, dir, transfer_request);
        break;
    default:
        break;
    }

    return status;
}

/**
 *\fn           ux_dcd_endpoint_create
 *\brief        This function will configure and enables the endpoints.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the endpoint structure
 *\return       On success 0, else error.
 **/

uint32_t ux_dcd_endpoint_create(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint)
{
    uint32_t status;
    USBD_EP *ept;
    uint8_t  ep_num = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    uint8_t  dir = (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION)
                       ? USB_DIR_IN
                       : USB_DIR_OUT;
    uint8_t  ep_type         = ux_endpoint_type(&endpoint->ux_slave_endpoint_descriptor);
    uint16_t ep_max_pkt_size = ux_endpoint_maxp(&endpoint->ux_slave_endpoint_descriptor);
    uint8_t  ep_interval     = endpoint->ux_slave_endpoint_descriptor.bInterval;
    uint32_t phy_ep          = USB_GET_PHYSICAL_EP(ep_num, dir);

    ept                      = &drv->eps[phy_ep];
    ept->endpoint            = endpoint;

    switch (ep_type) {

    case UX_CONTROL_ENDPOINT:
        /* Enable the control OUT endpoint  */
        status = usbd_ep_enable(drv, ep_num, USB_DIR_OUT, ep_type, ep_max_pkt_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable control ep num : %d dir: %d\n", ep_num, USB_DIR_OUT);
#endif
            return status;
        }
        /* Enable the control IN endpoint  */
        status = usbd_ep_enable(drv, ep_num, USB_DIR_IN, ep_type, ep_max_pkt_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable control ep num : %d dir: %d\n", ep_num, USB_DIR_IN);
#endif
            return status;
        }
        break;
    case UX_BULK_ENDPOINT:
        /* Enable the bulk endpoint  */
        status = usbd_ep_enable(drv, ep_num, dir, ep_type, ep_max_pkt_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable bulk ep num %d: dir: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    case UX_ISOCHRONOUS_ENDPOINT:
        /* Enable the Isoc endpoint  */
        status = usbd_ep_enable(drv, ep_num, dir, ep_type, ep_max_pkt_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("Failed to enable isoc ep num %d: dir: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    case UX_INTERRUPT_ENDPOINT:
        /* Enable the Interrupt endpoint  */
        status = usbd_ep_enable(drv, ep_num, dir, ep_type, ep_max_pkt_size, ep_interval);
        if (status) {
#ifdef DEBUG
            printf("failed to enable interrupt ep num %d: dir: %d\n", ep_num, dir);
#endif
            return status;
        }
        break;
    default:
        return UX_ERROR;
    }

    return status;
}

/**
 *\fn           ux_dcd_endpoint_stall
 *\brief        This function will stall the  endpoint.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the endpoint structure
 *\return       On success 0, else error.
 **/

uint32_t ux_dcd_endpoint_stall(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint)
{
    uint8_t ep_num = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    uint8_t dir = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    return usbd_ep_stall(drv, ep_num, dir);
}

/**
 *\fn           ux_dcd_endpoint_reset
 *\brief        This function will clears the stalled endpoint.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the endpoint structure
 *\return       On success 0, else error.
 **/

uint32_t ux_dcd_endpoint_reset(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint)
{
    uint8_t ep_num = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    uint8_t dir = endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

    return usbd_ep_clear_stall(drv, ep_num, dir);
}

/**
 *\fn           ux_dcd_endpoint_destroy
 *\brief        This function will disables endpoint.
 *\param[in]    pointer to our controller context structure
 *\param[in]    pointer to the endpoint structure
 *\return       On success 0, else error.
 **/

uint32_t ux_dcd_endpoint_destroy(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint)
{
    uint8_t ep_num = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    uint8_t dir    = usb_endpoint_dir(&endpoint->ux_slave_endpoint_descriptor);

    usbd_stop_transfer(drv, ep_num, dir, true);
    return usbd_ep_disable(drv, ep_num, dir);
}

uint32_t ux_dcd_transfer_request_abort(USB_DRIVER *drv, UX_SLAVE_TRANSFER *transfer_request)
{
    uint8_t ep_num;
    uint8_t dir;
    UX_SLAVE_ENDPOINT *endpoint;

    endpoint = transfer_request->ux_slave_transfer_request_endpoint;
    ep_num   = ux_endpoint_num(&endpoint->ux_slave_endpoint_descriptor);
    dir      = usb_endpoint_dir(&endpoint->ux_slave_endpoint_descriptor);
    return usbd_ep_transfer_abort(drv, ep_num, dir);
}
