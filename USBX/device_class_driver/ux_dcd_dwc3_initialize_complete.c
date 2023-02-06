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
 * @file     ux_dcd_dwc3_initialize_complete.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function completes the initialization of the USB slave
 *           controller for the chip.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"


UINT  _ux_dcd_dwc3_initialize_complete(VOID)
{

UX_SLAVE_DCD            *dcd;
UX_DCD_DWC3             *dcd_dwc3;
UX_SLAVE_DEVICE         *device;
UCHAR                     *device_framework;
UX_SLAVE_TRANSFER       *transfer_request;
ULONG                    dwc3_register;

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the DWC3 DCD.  */
    dcd_dwc3 = (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

        /* Check the speed and set the correct descriptor.  */
        if (_ux_system_slave -> ux_system_slave_speed ==  UX_FULL_SPEED_DEVICE)
        {

            /* The device is operating at full speed.  */
            _ux_system_slave -> ux_system_slave_device_framework =  _ux_system_slave -> ux_system_slave_device_framework_full_speed;
            _ux_system_slave -> ux_system_slave_device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length_full_speed;
        }
        else
        {

            /* The device is operating at high speed.  */
            _ux_system_slave -> ux_system_slave_device_framework =  _ux_system_slave -> ux_system_slave_device_framework_high_speed;
            _ux_system_slave -> ux_system_slave_device_framework_length =  _ux_system_slave -> ux_system_slave_device_framework_length_high_speed;

	    }

         /* Get the device framework pointer.  */
    device_framework =  _ux_system_slave -> ux_system_slave_device_framework;

    /* And create the decompressed device descriptor structure.  */
    _ux_utility_descriptor_parse(device_framework,
                                _ux_system_device_descriptor_structure,
                                UX_DEVICE_DESCRIPTOR_ENTRIES,
                                (UCHAR *) &device -> ux_slave_device_descriptor);

    /* Now we create a transfer request to accept the first SETUP packet
       and get the ball running. First get the address of the endpoint
       transfer request container.  */
    transfer_request =  &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Adjust the current data pointer as well.  */
    transfer_request -> ux_slave_transfer_request_current_data_pointer =
                            transfer_request -> ux_slave_transfer_request_data_pointer;

    /* Update the transfer request endpoint pointer with the default endpoint.  */
    transfer_request -> ux_slave_transfer_request_endpoint =  &device -> ux_slave_device_control_endpoint;

    /* The control endpoint max packet size needs to be filled manually in its descriptor.  */
    transfer_request -> ux_slave_transfer_request_endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize = 64;

    device -> ux_slave_device_descriptor.bMaxPacketSize0 = 64;

    /* On the control endpoint, always expect the maximum.  */
    transfer_request -> ux_slave_transfer_request_requested_length =
                                device -> ux_slave_device_descriptor.bMaxPacketSize0;

    /* Attach the control endpoint to the transfer request.  */
    transfer_request -> ux_slave_transfer_request_endpoint =  &device -> ux_slave_device_control_endpoint;

    /* Get total number of endpoints supported by usb controller */
    /* Create the default control endpoint attached to the device.
       Once this endpoint is enabled, the host can then send a setup packet
       The device controller will receive it and will call the setup function
       module.  */
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_CREATE_ENDPOINT,
                                    (VOID *) &device -> ux_slave_device_control_endpoint);

    /* Ensure the control endpoint is properly reset.  */
    device -> ux_slave_device_control_endpoint.ux_slave_endpoint_state = UX_ENDPOINT_RESET;

    /* Mark the phase as SETUP.  */
    transfer_request -> ux_slave_transfer_request_type =  UX_TRANSFER_PHASE_SETUP;

    /* Mark for three stage setup for usb controller */
    dcd_dwc3 -> three_stage_setup = 0;
    dcd_dwc3 -> ep0state = EP0_SETUP_PHASE;

    /* Mark this transfer request as pending.  */
    transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_PENDING;

    /* Ask for 8 bytes of the SETUP packet.  */
    transfer_request -> ux_slave_transfer_request_requested_length =    UX_SETUP_SIZE;
    transfer_request -> ux_slave_transfer_request_in_transfer_length =  UX_SETUP_SIZE;

    /* Reset the number of bytes sent/received.  */
    transfer_request -> ux_slave_transfer_request_actual_length =  0;

    /* Call the DCD driver transfer function.   */
    dcd -> ux_slave_dcd_function(dcd, UX_DCD_TRANSFER_REQUEST, transfer_request);

    /* Check the status change callback.  */
    if(_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
    {

        /* Inform the application if a callback function was programmed.  */
        _ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_ATTACHED);
    }

    /* We are now ready for the USB device to accept the first packet when connected.  */
    return(UX_SUCCESS);
}
