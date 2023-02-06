/**************************************************************************/
/*                                                                        */
/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************
 * @file     ux_dcd_dwc3_transfer_callback.c
 * @author   anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    The function _ux_dcd_dwc3_transfer_callback() is invoked under ISR
 *           when an event happens on a specific endpoint.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"
#include "ux_utility.h"

UINT  _ux_dcd_dwc3_transfer_callback(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request)
{

    UX_SLAVE_ENDPOINT  *endpoint;
    UX_DCD_DWC3_ED     *ed;
    UCHAR *            data_pointer;
    ULONG              fifo_length;
    ULONG              dwc3_endpoint_index;
    ULONG                    endpoint_size;
    ULONG              retval;
    ULONG	       status_cmd;
    ULONG	       LineCoding;
    UCHAR	       *Null_Ptr = NULL;

    _ux_dcd_dwc3_fifo_read(dcd_dwc3, transfer_request -> ux_slave_transfer_request_setup, UX_SETUP_SIZE);

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the endpoint index.  */
    dwc3_endpoint_index =  endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;
    /* Get the DWC3 endpoint.  */
    ed =  (UX_DCD_DWC3_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Get the pointer to the data buffer of the transfer request.  */
    data_pointer =  transfer_request -> ux_slave_transfer_request_data_pointer;


    /* Endpoint 0 is different.  */
    if (dwc3_endpoint_index == 0)
    {

        /* Check if we have received a SETUP command. */

        if (ed -> ux_dcd_dwc3_ed_transfer_status == UX_DCD_DWC3_ED_TRANSFER_STATUS_SETUP)
        {
            /* Clear the length of the data received.  */
            transfer_request -> ux_slave_transfer_request_actual_length =  0;

            /* Mark the phase as SETUP.  */
            transfer_request -> ux_slave_transfer_request_type =  UX_TRANSFER_PHASE_SETUP;

            /* Mark the transfer as successful.  */
            transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

            /* Set the status of the endpoint to not stalled.  */
            ed -> ux_dcd_dwc3_ed_status &= ~UX_DCD_DWC3_ED_STATUS_STALLED;

            /* Check if the transaction is IN.  */
            if (*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) /* commented because TCM dereference has zero*/
            {
                /* The endpoint is IN.  This is important to memorize the direction for the control endpoint
                   in case of a STALL. */
                ed -> ux_dcd_dwc3_ed_direction  = UX_ENDPOINT_IN;

                /* Call the Control Transfer dispatcher.  */
                _ux_device_stack_control_request_process(transfer_request);
            }
            else
            {
                /* The endpoint is OUT.  This is important to memorize the direction for the control endpoint
                   in case of a STALL. */
                ed -> ux_dcd_dwc3_ed_direction  = UX_ENDPOINT_OUT;

                /* We are in a OUT transaction. Check if there is a data payload. If so, wait for the payload
                   to be delivered.  */
                if (*(transfer_request -> ux_slave_transfer_request_setup + 6) == 0 &&
                    *(transfer_request -> ux_slave_transfer_request_setup + 7) == 0)

                    /* Call the Control Transfer dispatcher.  */
                    _ux_device_stack_control_request_process(transfer_request);
            }

            /* Check if the transaction is OUT and there is no data payload.  */
            if (((*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0) &&
                    *(transfer_request -> ux_slave_transfer_request_setup + 6) == 0 &&
                    *(transfer_request -> ux_slave_transfer_request_setup + 7) == 0)
            {
                /* In the case of a SETUP followed by NO data payload, we let the controller reply to the next
                   zero length IN packet. Reset the length to transfer. */
                transfer_request -> ux_slave_transfer_request_in_transfer_length =  0;
                transfer_request -> ux_slave_transfer_request_requested_length =    0;

                /* Set the phase of the transfer to data OUT for status.  */

		        if(*(transfer_request -> ux_slave_transfer_request_setup + 2) != 0)

                    _ux_device_stack_control_request_process(transfer_request);

                /* Set the state to STATUS RX.  */
                ed -> ux_dcd_dwc3_ed_state =  UX_DCD_DWC3_ED_STATE_STATUS_RX;

	        }

            if ((*transfer_request -> ux_slave_transfer_request_setup & UX_REQUEST_IN) == 0 &&
				(*(transfer_request -> ux_slave_transfer_request_setup + 6) != 0 ||
				*(transfer_request -> ux_slave_transfer_request_setup + 7) != 0))
            {

			/* Get the length we expect from the SETUP packet.  */
                    transfer_request -> ux_slave_transfer_request_requested_length =
			    _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + 6);

                _ux_utility_memory_set(dcd_dwc3->BufferPtr, 0, sizeof(dcd_dwc3->BufferPtr));

                /* Reset what we have received so far.  */
                transfer_request -> ux_slave_transfer_request_actual_length =  0;

                /* And reprogram the current buffer address to the beginning of the buffer.  */

                    _ux_dcd_RecvLineCoding(dcd_dwc3);
		         /* Set the state to RX.  */
                ed -> ux_dcd_dwc3_ed_state =  UX_DCD_DWC3_ED_STATE_DATA_RX;
		        LineCoding = 1;
            }

		    if((*(transfer_request -> ux_slave_transfer_request_setup) == 0x21)
				&& (*(transfer_request -> ux_slave_transfer_request_setup + 1) == 0x22)
				&& (*(transfer_request -> ux_slave_transfer_request_setup + 2) != 0))
		    {
			    _ux_device_stack_control_request_process(transfer_request);

		        if(LineCoding == 1)
		        {
			        status_cmd = _ux_dcd_SendLineCoding(dcd_dwc3);
			        EpBufferSend(dcd_dwc3, 0, (ULONG *)Null_Ptr, 0);
			        LineCoding = 0;
		        }
		        /* Set the state to RX.  */
                ed -> ux_dcd_dwc3_ed_state =  UX_DCD_DWC3_ED_STATE_DATA_RX;
		    }

		    if((*(transfer_request -> ux_slave_transfer_request_setup) == 0x21)
				&& (*(transfer_request -> ux_slave_transfer_request_setup + 1) == 0x22)
				&& (*(transfer_request -> ux_slave_transfer_request_setup + 2) == 0))
		    {
			    EpBufferSend(dcd_dwc3, 0, (ULONG *)Null_Ptr, 0);
		    }
	}
        else
        {
	    /* Check if we have received something on endpoint 0 during data phase .  */
            if (ed -> ux_dcd_dwc3_ed_state == UX_DCD_DWC3_ED_STATE_DATA_RX)
            {

	        /* Read the fifo length for the Control endpoint.  */
                fifo_length = ed -> ux_dcd_dwc3_ed_payload_length;


                /* Obtain the current data buffer address.  */
                data_pointer =  transfer_request -> ux_slave_transfer_request_current_data_pointer;

                /* Can we accept this much?  */
                if (transfer_request -> ux_slave_transfer_request_actual_length <=
                        transfer_request -> ux_slave_transfer_request_requested_length)
                {

                    /* Are we done with this transfer ? */
                    if ((transfer_request -> ux_slave_transfer_request_actual_length ==
                            transfer_request -> ux_slave_transfer_request_requested_length) ||
                            (fifo_length != endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize))
                    {

                        /* Set the completion code to no error.  */
                        transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

			            printf("Done with transfer calling stack_request_process for ZERO trasfer\n");
                        /* We are using a Control endpoint on a OUT transaction and there was a payload.  */
                        _ux_device_stack_control_request_process(transfer_request);

                        /* Set the state to STATUS phase TX.  */
                        ed -> ux_dcd_dwc3_ed_state =  UX_DCD_DWC3_ED_STATE_STATUS_TX;

                        /* Reset the length to transfer. */
                        transfer_request -> ux_slave_transfer_request_in_transfer_length =  0;
                        transfer_request -> ux_slave_transfer_request_requested_length =    0;

                        /* Set the phase of the transfer to data OUT for status.  */
                        transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;

                    }
                    else
                    {

                        /* Rearm the OUT control endpoint. */
                    }
                }
                else
                {
			        printf("Some bufferoverflow sequence\n");

                    /*  We have an overflow situation. Set the completion code to overflow.  */
                    transfer_request -> ux_slave_transfer_request_completion_code =  UX_TRANSFER_BUFFER_OVERFLOW;

                    /* If trace is enabled, insert this event into the trace buffer.  */
                    UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_BUFFER_OVERFLOW, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                    /* We are using a Control endpoint, if there is a callback, invoke it. We are still under ISR.  */
                    if (transfer_request -> ux_slave_transfer_request_completion_function)
                        transfer_request -> ux_slave_transfer_request_completion_function (transfer_request) ;
                }

            }
            else
            {
#ifdef DEBUG
		        printf("ed->ux_dcd_dwc3_ed_state [%x]\n", ed->ux_dcd_dwc3_ed_state);
#endif

                /* Check if we have received something on endpoint 0 during status phase .  */
                if (ed -> ux_dcd_dwc3_ed_state == UX_DCD_DWC3_ED_STATE_STATUS_RX)
                {

                    /* Next phase is a SETUP.  */
                    ed -> ux_dcd_dwc3_ed_state =  UX_DCD_DWC3_ED_STATE_IDLE;


                }
                else
                {

                    /* Check if we need to send data again on control endpoint. */
                    if (ed -> ux_dcd_dwc3_ed_state == UX_DCD_DWC3_ED_STATE_DATA_TX)
                    {
			            printf("Do we need to send data again in zlp\n");

                        /* Check if we have data to send.  */
                        if (transfer_request -> ux_slave_transfer_request_in_transfer_length == 0)
                        {

                            /* There is no data to send but we may need to send a Zero Length Packet.  */
                            if (transfer_request -> ux_slave_transfer_request_force_zlp ==  UX_TRUE)
                            {


                                /* Arm a ZLP packet on IN.  */
                                /* Write the size of the FIFO.  1 packet, 0 XFERSIZ. */

			        printf("ZLP == TRUE CASE, As per USB standard for < wlength no need of zlp\n");
                                /* Reset the ZLP condition.  */
                                transfer_request -> ux_slave_transfer_request_force_zlp =  UX_FALSE;

                            }
                            else
                            {

                                /* Set the completion code to no error.  */
                                transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;

                                /* The transfer is completed.  */
                                transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;

                                /* We are using a Control endpoint, if there is a callback, invoke it. We are still under ISR.  */
                                if (transfer_request -> ux_slave_transfer_request_completion_function)
                                    transfer_request -> ux_slave_transfer_request_completion_function (transfer_request) ;

                                /* State is now STATUS RX.  */
                                ed -> ux_dcd_dwc3_ed_state = UX_DCD_DWC3_ED_STATE_STATUS_RX;

                            }
                        }
                        else
                        {

                            /* Get the size of the transfer, used for a IN transaction only.  */
                            fifo_length =  transfer_request -> ux_slave_transfer_request_in_transfer_length;

                            /* Check if the endpoint size is bigger that data requested. */
                            if (fifo_length > endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
                            {

                                /* Adjust the transfer size.  */
                                fifo_length =  endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;
                            }

                            /* Keep the FIFO length in the endpoint.  */
                            ed -> ux_dcd_dwc3_ed_payload_length =  fifo_length;

                            /* Program the transfer size.  */
                            endpoint_size = fifo_length;


                            /* Adjust the data pointer.  */
                            transfer_request -> ux_slave_transfer_request_current_data_pointer += fifo_length;

                            /* Adjust the transfer length remaining.  */
                            transfer_request -> ux_slave_transfer_request_in_transfer_length -= fifo_length;

                            /* If this is the last packet, set data end as well.  */
                            if (transfer_request -> ux_slave_transfer_request_in_transfer_length == 0)
			                {
#ifdef  DEBUG
			                    /* Arm a ZLP packet on IN.  */
				                printf("request_in_transfer_length == 0\n");
#endif
			                }
                            else
                            {
                                /* Write to the Fifo.  More packets to come. */
#ifdef	DEBUG
				printf("More packets to come in fifo\n");
#endif
                            }

                        }
                    }
                }
            }
        }
    }
    else
    {
#ifdef	DEBUG
	    printf("Non 0 endpoints sequence started\n");
#endif
        /* We treat non 0 endpoints here.  Look at the direction and determine if this an OUT or IN endpoint. */
        if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
        {
#ifdef	DEBUG
		printf("Data from Host\n");
#endif
		    /* Read the fifo length for the endpoint.  */
            fifo_length =  ed -> ux_dcd_dwc3_ed_payload_length;

            /* Obtain the current data buffer address.  */
            data_pointer =  transfer_request -> ux_slave_transfer_request_current_data_pointer;

            /* Can we accept this much?  */
            if (transfer_request -> ux_slave_transfer_request_actual_length <=
                        transfer_request -> ux_slave_transfer_request_requested_length)
            {

                /* Are we done with this transfer ? */
                if ((transfer_request -> ux_slave_transfer_request_actual_length ==
                            transfer_request -> ux_slave_transfer_request_requested_length) ||
                            (fifo_length != endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize))
                {
                    /* Set the completion code to no error.  */
                    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
                    /* The transfer is completed.  */
                    transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;
                    /* Non control endpoint operation, use semaphore.  */

                }
                else
                {
#ifdef	DEBUG
		    printf("More packet loop will be covered later\n");
#endif
		    /* We need to accept more packets.  */
                    fifo_length =  transfer_request -> ux_slave_transfer_request_requested_length;
                }

            }
            else
            {

                /*  We have an overflow situation. Set the completion code to overflow.  */
                transfer_request -> ux_slave_transfer_request_completion_code =  UX_TRANSFER_BUFFER_OVERFLOW;
                /* If trace is enabled, insert this event into the trace buffer.  */
                UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_TRANSFER_BUFFER_OVERFLOW, transfer_request, 0, 0, UX_TRACE_ERRORS, 0, 0)

                /* The transfer is completed.  */
                transfer_request -> ux_slave_transfer_request_status =  UX_TRANSFER_STATUS_COMPLETED;
                /* Non control endpoint operation, use semaphore.  */

            }
        }
        else
        {
#ifdef	DEBUG
            printf("IN TRANSACTION taken care later\n");
#endif
        }
    }

    /* We are done.  */
    return(UX_SUCCESS);
}
