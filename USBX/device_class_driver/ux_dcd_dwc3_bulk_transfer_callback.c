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
 * @file     ux_dcd_dwc3_bulk_transfer_callback.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function is invoked under ISR when an event happens on a
 *           specific endpoint.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"
#include "ux_utility.h"


UINT  _ux_dcd_dwc3_bulk_transfer_callback(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request)
{

ULONG                   dwc3_register;
UX_SLAVE_ENDPOINT       *endpoint;
UX_DCD_DWC3_ED		*ed;
UCHAR *                 data_pointer;
ULONG                   fifo_length;

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the DWC3 endpoint.  */
    ed =  (UX_DCD_DWC3_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Get the pointer to the data buffer of the transfer request.  */
    data_pointer =  transfer_request -> ux_slave_transfer_request_data_pointer;

	/* Read the fifo length for the endpoint.  */
    fifo_length =  ed -> ux_dcd_dwc3_ed_payload_length;

    /* Set the completion code to no error.  */
    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

    /* The transfer is completed.  */
    transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

    /* Non control endpoint operation, use semaphore.  */
    _ux_utility_semaphore_put(&transfer_request -> ux_slave_transfer_request_semaphore);

    /* We are done.  */
    return(UX_SUCCESS);
}
