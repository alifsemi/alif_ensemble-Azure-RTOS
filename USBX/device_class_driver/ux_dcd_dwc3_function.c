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
 * @file     ux_dcd_dwc3_function.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function dispatches the DCD function internally to the dwc3
 *           controller.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"


UINT  _ux_dcd_dwc3_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter)
{

UINT            status;
UX_DCD_DWC3     *dcd_dwc3;


    /* Check the status of the controller.  */
    if (dcd -> ux_slave_dcd_status == UX_UNUSED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }

    /* Get the pointer to the DWC3 DCD.  */
    dcd_dwc3 =  (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {

        case UX_DCD_GET_FRAME_NUMBER:

            break;

        case UX_DCD_TRANSFER_REQUEST:

            status =  _ux_dcd_dwc3_transfer_request(dcd_dwc3, (UX_SLAVE_TRANSFER *) parameter);
            break;

        case UX_DCD_CREATE_ENDPOINT:

            status =  _ux_dcd_dwc3_endpoint_create(dcd_dwc3, parameter);
            break;

        case UX_DCD_DESTROY_ENDPOINT:

            status = 0;
            break;

        case UX_DCD_RESET_ENDPOINT:

            break;

        case UX_DCD_STALL_ENDPOINT:

            break;

        case UX_DCD_SET_DEVICE_ADDRESS:

	        status = _ux_dcd_SetDeviceAddress(dcd_dwc3, (ULONG) parameter);
	        EpBufferSend(dcd_dwc3, 0, NULL, 0);
            break;

        case UX_DCD_CHANGE_STATE:

            status =  _ux_dcd_dwc3_state_change(dcd_dwc3, (ULONG) parameter);
            break;

        case UX_DCD_ENDPOINT_STATUS:

            break;

        case UX_SET_CONFIGURATION:

            status =  _ux_dcd_dwc3_SetConfiguration(dcd_dwc3, (ULONG) parameter);
            break;

        default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }

    /* Return completion status.  */
    return(status);
}
