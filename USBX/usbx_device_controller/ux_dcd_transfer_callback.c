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
 * @file     ux_dcd_transfer_callback.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     03-Sep-2025
 * @brief    This file a wrapper between usbx component and low level driver.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "ux_dcd.h"

extern TX_EVENT_FLAGS_GROUP DATA_IN_OUT_EVENT_FLAG;

/**
 *\fn           ux_dcd_disconnect_cb
 *\brief        This function will inform the disconnect event to usbx component.
 *\param[in]    none
 *\return       none
 **/

void ux_dcd_disconnect_cb(void)
{
    uint32_t ret;

    if (_ux_system_slave->ux_system_slave_device.ux_slave_device_state != UX_DEVICE_RESET) {
        /* Disconnect the device */
        ret = _ux_device_stack_disconnect();
        if (ret != UX_SUCCESS) {
#ifdef DEBUG
            printf("Device disconnection failed\n");
#endif
        }
    }
}

/**
 *\fn           ux_dcd_reset_cb
 *\brief        This function will inform the reset event to usbx component.
 *\param[in]    pointer to the controller context structure
 *\return       none
 **/

void ux_dcd_reset_cb(USB_DRIVER *drv)
{
    uint32_t ret;

    if (_ux_system_slave->ux_system_slave_device.ux_slave_device_state == UX_DEVICE_CONFIGURED) {
        /* Device is reset, the behavior is the same as disconnection.  */
        if ((drv->endp_number != 0) && (drv->endp_number != 1)) {
            usbd_clear_trb(drv, drv->endp_number);
        }
        ret = _ux_utility_event_flags_set(&DATA_IN_OUT_EVENT_FLAG, UX_DISCONNECT_EVENT, UX_OR);
        if (ret != UX_SUCCESS) {
#ifdef DEBUG
            printf("Failed to set the disconnect event\n");
#endif
        }
        /* Disconnect the device */
        ret = _ux_device_stack_disconnect();
        if (ret != UX_SUCCESS) {
#ifdef DEBUG
            printf("Device disconnection failed\n");
#endif
        }
    } else {
        _ux_system_slave->ux_system_slave_device.ux_slave_device_state = UX_DEVICE_RESET;
    }
}

/**
 *\fn           ux_dcd_connect_cb
 *\brief        This function will inform the connectiondone event to usbx component.
 *\param[in]    none
 *\return       none
 **/

void ux_dcd_connect_cb(void)
{

    if (_ux_system_slave->ux_system_slave_device.ux_slave_device_state != UX_DEVICE_ATTACHED &&
        _ux_system_slave->ux_system_slave_device.ux_slave_device_state != UX_DEVICE_CONFIGURED) {
        /* We are connected at high speed.  */
        _ux_system_slave->ux_system_slave_speed = UX_HIGH_SPEED_DEVICE;
        /* Complete the device initialization.  */
        ux_dcd_initialize_complete();
        /* Mark the device as attached now.  */
        _ux_system_slave->ux_system_slave_device.ux_slave_device_state = UX_DEVICE_ATTACHED;
    }
}

/**
 *\fn           ux_dcd_ep0_events
 *\brief        This function will process the setup packets to the USBX component.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint
 *\param[in]    pointer to the transfer request
 *\return       On success 0, else error
 **/

uint32_t ux_dcd_ep0_events(USB_DRIVER *drv, uint8_t phy_ep, UX_SLAVE_TRANSFER *transfer_request)
{
    UX_SLAVE_ENDPOINT *endpoint;
    USBD_EP           *ept;
    uint32_t           status;

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint = transfer_request->ux_slave_transfer_request_endpoint;
    /* Get the physical endpoint.  */
    ept      = &drv->eps[phy_ep];
    /* Check if we have received a SETUP command. */
    if (ept->ep_transfer_status == USB_EP_TRANSFER_SETUP) {
        /* Check if the transaction is IN.  */
        if (*transfer_request->ux_slave_transfer_request_setup & UX_REQUEST_IN) {
            ept->ep_dir = UX_ENDPOINT_IN;
            /* Call the Control Transfer dispatcher.as usbx stack API  */
            status      = _ux_device_stack_control_request_process(transfer_request);
            if (status != UX_SUCCESS) {
#ifdef DEBUG
                printf("failed to process the setu : %d\n", status);
#endif
            }
            /* Set the state to TX  */
            ept->ep_state = UX_DCD_EP_STATE_DATA_TX;
        } else {
            /* We are in a OUT transaction. Check if there is a data payload. If so, wait for the
             * payload to be delivered.
             */
            ept->ep_dir = UX_ENDPOINT_OUT;
            if ((*(transfer_request->ux_slave_transfer_request_setup + 6U) == 0U) &&
                (*(transfer_request->ux_slave_transfer_request_setup + 7U) == 0U)) {
                /* Call the Control Transfer dispatcher.  */
                status = _ux_device_stack_control_request_process(transfer_request);
            } else {
                /* Get the pointer to the logical endpoint from the transfer request.  */
                endpoint = transfer_request->ux_slave_transfer_request_endpoint;
                /* Get the length we expect from the SETUP packet.  */
                transfer_request->ux_slave_transfer_request_requested_length =
                    _ux_utility_short_get(transfer_request->ux_slave_transfer_request_setup + 6);
                /* Reset what we have received so far.  */
                transfer_request->ux_slave_transfer_request_actual_length = 0;
                /* And reprogram the current buffer address to the beginning of the buffer.  */
                transfer_request->ux_slave_transfer_request_current_data_pointer =
                    transfer_request->ux_slave_transfer_request_data_pointer;
                /* Requesting control OUT data and data packet length as  max packet size of control
                 * endpoint
                 */
                status =
                    usbd_ep0_recv(drv,
                                  0,
                                  0,
                                  transfer_request->ux_slave_transfer_request_current_data_pointer,
                                  endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize);
                ept->ep_state = UX_DCD_EP_STATE_DATA_RX;
            }
        }
    }

    return status;
}

/**
 *\fn           ux_dcd_setupstage_cb
 *\brief        This function will copy the setup packet to the transfer request.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint
 *\return       none
 **/

void ux_dcd_setupstage_cb(USB_DRIVER *drv, uint8_t phy_ep)
{
    UX_SLAVE_ENDPOINT *endpoint;
    UX_SLAVE_TRANSFER *transfer_request;
    USBD_EP           *ed;
    uint32_t           ret;

    ed               = &drv->eps[phy_ep];
    endpoint         = ed->endpoint;
    /* Get the pointer to the transfer request  */
    transfer_request = &endpoint->ux_slave_endpoint_transfer_request;
    /* Copy the setup pkt to the transfer request */
    _ux_utility_memory_copy(transfer_request->ux_slave_transfer_request_setup,
                            &drv->setup_data,
                            UX_SETUP_SIZE);
    /* Mark the phase as SETUP  */
    transfer_request->ux_slave_transfer_request_type            = UX_TRANSFER_PHASE_SETUP;
    /* Mark the transfer as successful.  */
    transfer_request->ux_slave_transfer_request_completion_code = UX_SUCCESS;
    /* Process the ep0 interrupts as standard setup packets */
    ret = ux_dcd_ep0_events(drv, phy_ep, transfer_request);
    if (ret != UX_SUCCESS) {
#ifdef DEBUG
        printf("failed to process setup pkts\n");
#endif
    }
}

/**
 *\fn           ux_dcd_ep0_data_stage_cb
 *\brief        This function will get called when data transfers completed on EPO & EP1
                and update the transfer request.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint
 *\return       none
 **/

void ux_dcd_ep0_data_stage_cb(USB_DRIVER *drv, uint8_t phy_ep)
{
    UX_SLAVE_ENDPOINT *endpoint;
    UX_SLAVE_TRANSFER *transfer_request;
    USBD_EP           *ept;
    uint32_t           ret;

    ept              = &drv->eps[phy_ep];
    endpoint         = ept->endpoint;
    transfer_request = &endpoint->ux_slave_endpoint_transfer_request;
    RTSS_InvalidateDCache_by_Addr(transfer_request->ux_slave_transfer_request_data_pointer,
                                  ept->bytes_txed);
    transfer_request->ux_slave_transfer_request_actual_length = ept->bytes_txed;
    if (phy_ep == 0) {
        /* Get the pointer to the logical endpoint from the transfer request.  */
        if (ept->ep_state == UX_DCD_EP_STATE_DATA_RX) {
            /* We are using a Control endpoint on a OUT transaction and there was a payload.  */
            if (transfer_request->ux_slave_transfer_request_actual_length ==
                transfer_request->ux_slave_transfer_request_requested_length) {
                ept->ep_dir = UX_ENDPOINT_IN;
                ret         = _ux_device_stack_control_request_process(transfer_request);
                if (ret != UX_SUCCESS) {
#ifdef DEBUG
                    printf("there was issue to process control out data\n");
#endif
                }
            }
        }
    }
}

/**
 *\fn           ux_dcd_data_stage_cb
 *\brief        This function will get called when data transfer completed on non control endpoints
                and update the transfer request.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint
 *\return       none
 **/

void ux_dcd_data_stage_cb(USB_DRIVER *drv, uint8_t phy_ep)
{
    UX_SLAVE_ENDPOINT *endpoint;
    UX_SLAVE_TRANSFER *transfer_request;
    USBD_EP           *ept;
    uint32_t           ret;
    uint32_t           event;

    ept              = &drv->eps[phy_ep];
    endpoint         = ept->endpoint;
    transfer_request = &endpoint->ux_slave_endpoint_transfer_request;
    RTSS_InvalidateDCache_by_Addr(transfer_request->ux_slave_transfer_request_data_pointer,
                                  ept->bytes_txed);
    drv->num_bytes                                            = ept->bytes_txed;
    transfer_request->ux_slave_transfer_request_actual_length = drv->num_bytes;
    if (ept->ep_dir) {
        event = UX_DATA_IN_EVENT;
    } else {
        event = UX_DATA_OUT_EVENT;
    }
    ret = _ux_utility_event_flags_set(&DATA_IN_OUT_EVENT_FLAG, event, UX_OR);
    if (ret != UX_SUCCESS) {
#ifdef DEBUG
        printf(" failed to set the event flag %d\n", event);
#endif
        return;
    }
}

/**
 *\fn           ux_dcd_xfernotready_data
 *\brief        This function will get called when controller generates xfernotready event
                for Isochronous endpoint and set the xfernotready event flag.
 *\param[in]    pointer to the controller context structure
 *\param[in]    physical endpoint
 *\return       none
 **/

void ux_dcd_xfernotready_data(USB_DRIVER *drv, uint8_t phy_ep)
{
    UX_SLAVE_ENDPOINT *endpoint;
    USBD_EP           *ept;
    uint32_t           ret;

    ept              = &drv->eps[phy_ep];
    endpoint         = ept->endpoint;
    if (ux_endpoint_xfer_isoc(&endpoint->ux_slave_endpoint_descriptor)) {
        ret = _ux_utility_event_flags_set(&DATA_IN_OUT_EVENT_FLAG, UX_XFERNOTREADY_EVENT, UX_OR);
        if (ret != UX_SUCCESS) {
#ifdef DEBUG
            printf(" failed to set the xfernotready event flag %d\n", event);
#endif
            return;
        }
    }

}
