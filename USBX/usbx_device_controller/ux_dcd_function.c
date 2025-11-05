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
 * @file     ux_dcd_function.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function dispatches the DCD function internally to the USB
 *           controller.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "ux_dcd.h"

uint32_t ux_dcd_function(UX_SLAVE_DCD *dcd, uint32_t function, VOID *parameter)
{
    uint32_t    status;
    USB_DRIVER *drv;

    /* Check the status of the controller.  */
    if (dcd->ux_slave_dcd_status == UX_UNUSED) {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD,
                                 UX_SYSTEM_CONTEXT_DCD,
                                 UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR,
                                UX_CONTROLLER_UNKNOWN,
                                0,
                                0,
                                0,
                                UX_TRACE_ERRORS,
                                0,
                                0)
        return UX_CONTROLLER_UNKNOWN;
    }
    drv = (USB_DRIVER *) dcd->ux_slave_dcd_controller_hardware;
    if (drv == UX_NULL) {
        return UX_CONTROLLER_UNKNOWN;
    }

    /* Look at the function and route it.  */
    switch (function) {
    case UX_DCD_GET_FRAME_NUMBER:
        break;
    case UX_DCD_TRANSFER_REQUEST:
        status = ux_dcd_transfer_request(drv, (UX_SLAVE_TRANSFER *) parameter);
        break;
    case UX_DCD_TRANSFER_ABORT:
        status = ux_dcd_transfer_request_abort(drv, (UX_SLAVE_TRANSFER *) parameter);
        break;
    case UX_DCD_CREATE_ENDPOINT:
        status = ux_dcd_endpoint_create(drv, parameter);
        break;
    case UX_DCD_DESTROY_ENDPOINT:
        status = ux_dcd_endpoint_destroy(drv, parameter);
        break;
    case UX_DCD_RESET_ENDPOINT:
        status = ux_dcd_endpoint_reset(drv, parameter);
        break;
    case UX_DCD_STALL_ENDPOINT:
        status = ux_dcd_endpoint_stall(drv, parameter);
        break;
    case UX_DCD_SET_DEVICE_ADDRESS:
        status = usbd_set_device_address(drv, (uint32_t) parameter);
        break;
    case UX_DCD_CHANGE_STATE:
        status = usbd_state_change(drv, (uint32_t) parameter);
        break;
    case UX_DCD_ENDPOINT_STATUS:
        break;
    default:
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD,
                                 UX_SYSTEM_CONTEXT_DCD,
                                 UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR,
                                UX_FUNCTION_NOT_SUPPORTED,
                                0,
                                0,
                                0,
                                UX_TRACE_ERRORS,
                                0,
                                0)

        status = UX_FUNCTION_NOT_SUPPORTED;
        break;
    }
    /* Return completion status.  */
    return status;
}
