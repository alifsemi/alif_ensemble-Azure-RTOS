/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** USBX Component                                                        */ 
/**                                                                       */
/**   Device Stack                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_stack.h"

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_stack_control_request_process            PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is called by the DCD when the device has received a   */
/*    SETUP packet.                                                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_class_entry_function)       Device class entry function   */ 
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
/*    _ux_device_stack_transfer_request     Transfer request              */
/*    _ux_device_stack_endpoint_stall       Stall endpoint                */
/*    _ux_device_stack_alternate_setting_get                              */
/*                                          Get alternate settings        */ 
/*    _ux_device_stack_alternate_setting_set                              */
/*                                          Set alternate settings        */ 
/*    _ux_device_stack_clear_feature        Clear feature                 */ 
/*    _ux_device_stack_configuration_get    Get configuration             */ 
/*    _ux_device_stack_configuration_set    Set configuration             */ 
/*    _ux_device_stack_descriptor_send      Send descriptor               */ 
/*    _ux_device_stack_get_status           Get status                    */ 
/*    _ux_device_stack_set_feature          Set feature                   */ 
/*    _ux_utility_short_get                 Get short value               */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Device Stack                                                        */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_control_request_process(UX_SLAVE_TRANSFER *transfer_request)
{

UX_SLAVE_DCD                *dcd;
UX_SLAVE_DEVICE             *device;
UX_SLAVE_CLASS              *class;
UX_SLAVE_CLASS_COMMAND      class_command;
ULONG                       request_type;
ULONG                       request;
ULONG                       request_value;
ULONG                       request_index;
ULONG                       request_length;
ULONG                       class_index;
UINT                        status =  UX_ERROR;
UX_SLAVE_ENDPOINT           *endpoint;
ULONG                       application_data_length;

//ULONG			    temp;

    //printf("ux_device_stack_control_request_process invoked\n");
  //  temp = (ULONG)TRB_ADDR_SRAM0;
    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    //printf("Host request type [%x]\n", transfer_request -> ux_slave_transfer_request_setup);

    /* Ensure that the Setup request has been received correctly.  */
    if (transfer_request -> ux_slave_transfer_request_completion_code == UX_SUCCESS)
    {

        /* Seems so far, the Setup request is valid. Extract all fields of
           the request.  */

        request_type   =   *transfer_request -> ux_slave_transfer_request_setup;
        request        =   *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);
        request_value  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_VALUE);
        request_index  =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_INDEX);
        request_length =   _ux_utility_short_get(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

	 /* printf("request %x request_value %x request_length %x request_type %x\n", request, request_value ,
						request_length,request_type); */
        /* Filter for GET_DESCRIPTOR/SET_DESCRIPTOR commands. If the descriptor to be returned is not a standard descriptor,
           treat the command as a CLASS command.  */
        if ((request == UX_GET_DESCRIPTOR || request == UX_SET_DESCRIPTOR) && (((request_value >> 8) & UX_REQUEST_TYPE) != UX_REQUEST_TYPE_STANDARD))
        {        

            /* This request is to be handled by the class layer.  */
            request_type &=  (UINT)~UX_REQUEST_TYPE;
            request_type |= UX_REQUEST_TYPE_CLASS;
	       // printf("request_type [%x]\n", request_type);
	}                   

        /* Check if there is a vendor registered function at the application layer.  If the request
           is VENDOR and the request match, pass the request to the application.  */
        if ((request_type & UX_REQUEST_TYPE) == UX_REQUEST_TYPE_VENDOR)
        {
		
		    //printf("UX_REQUEST_TYPE_VENDOR\n");
            /* Check the request demanded and compare it to the application registered one.  */
            if (request == _ux_system_slave -> ux_system_slave_device_vendor_request)
            {
		        //printf("Calling vendor_request_function\n");

                /* This is a Microsoft extended function. It happens before the device is configured. 
                   The request is passed to the application directly.  */
                status = _ux_system_slave -> ux_system_slave_device_vendor_request_function(request, request_value, 
                                                                                            request_index, request_length, 
                                                                                            transfer_request -> ux_slave_transfer_request_data_pointer,
                                                                                            &application_data_length);
                //printf("STATUS :%d\n",status);

                /* Check the status from the application.  */
                if (status == UX_SUCCESS)
                {
                
                    /* Get the control endpoint associated with the device.  */
                    endpoint =  &device -> ux_slave_device_control_endpoint;
    
                    /* Get the pointer to the transfer request associated with the control endpoint.  */
                    transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
    
                    /* Set the direction to OUT.  */
                    transfer_request -> ux_slave_transfer_request_phase =  UX_TRANSFER_PHASE_DATA_OUT;
    
                    /* Perform the data transfer.  */
                    _ux_device_stack_transfer_request(transfer_request, request_length, application_data_length);

                    /* We are done here.  */
                    return(UX_SUCCESS);
                }
                else
                {

                    /* The application did not like the vendor command format, stall the control endpoint.  */
                    _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
                    
                    /* We are done here.  */
                    return(UX_SUCCESS);
                }
            }
        }

        /* Check the destination of the request. If the request is of type CLASS or VENDOR_SPECIFIC,
           the function has to be passed to the class layer.  */
        if (((request_type & UX_REQUEST_TYPE) == UX_REQUEST_TYPE_CLASS) ||
            ((request_type & UX_REQUEST_TYPE) == UX_REQUEST_TYPE_VENDOR))
        {

		    //printf("request is of type CLASS\n");
            /* Build all the fields of the Class Command.  */
            class_command.ux_slave_class_command_request =  UX_SLAVE_CLASS_COMMAND_REQUEST;

            /* We need to find which class this request is for.  */
            for (class_index = 0; class_index < UX_MAX_SLAVE_INTERFACES; class_index ++)
            {

                /* Is the request target to an interface?  */
                if ((request_type & UX_REQUEST_TARGET) == UX_REQUEST_TARGET_INTERFACE)
                {

                    /* Yes, so the request index contains the index of the interface 
                       the request is for. So if the current index does not match 
                       the request index, we should go to the next one.  */
                    if ((request_index & 0xFF) != class_index)
                    continue;
                }

                /* Get the class for the interface.  */
                class =  _ux_system_slave -> ux_system_slave_interface_class_array[class_index];

                /* If class is not ready, try next.  */
                if (class == UX_NULL)
                    continue;

                /* Memorize the class in the command.  */
                class_command.ux_slave_class_command_class_ptr = class;

		        //printf("Calling ux_slave_class_entry_function\n");
                /* We have found a potential candidate. Call this registered class entry function.  */
                status = class -> ux_slave_class_entry_function(&class_command);

                /* The status simply tells us if the registered class handled the 
                   command - if there was an issue processing the command, it would've 
                   stalled the control endpoint, notifying the host (and not us).  */
                if (status == UX_SUCCESS)

                    /* We are done, break the loop!  */
                    break;

                /* Not handled, try next.  */
            }

            /* If no class handled the command, then we have an error here.  */
            if (status != UX_SUCCESS)

	    {
                /* We stall the command (request not supported).  */
                _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
	    }

            /* We are done for class/vendor request.  */
            return(status);
        }

//	printf("Request to handle [%x] request_type [%x]\n", 
//			request, request_type);

        /* At this point, the request must be a standard request that the device stack should handle.  */
        switch (request)
        {

        case UX_GET_STATUS:
            //printf("UX_GET_STATUS\n");
            status =  _ux_device_stack_get_status(request_type, request_index, request_length);
            break;

        case UX_CLEAR_FEATURE:
        	//printf("UX_CLEAR_FEATURE\n");
            status =  _ux_device_stack_clear_feature(request_type, request_value, request_index);
            break;

        case UX_SET_FEATURE:
        	//printf("UX_SET_FEATURE\n");
            status =  _ux_device_stack_set_feature(request_type, request_value, request_index);
            break;

        case UX_SET_ADDRESS:
        	//printf("UX_SET_ADDRESS\n");
            /* Memorize the address. Some controllers memorize the address here. Some don't.  */
            dcd -> ux_slave_dcd_device_address =  request_value;
            //printf("requested value: %x\n",request_value);
            /* Force the new address.  */
	    status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_SET_DEVICE_ADDRESS, (VOID *) (ALIGN_TYPE) request_value);
            break;

        case UX_GET_DESCRIPTOR:
        	//printf("UX_GET_DESCRIPTOR\n");
            status =  _ux_device_stack_descriptor_send(request_value, request_index, request_length);
            break;

        case UX_SET_DESCRIPTOR:
        	//printf("UX_SET_DESCRIPTOR\n");
            status = UX_FUNCTION_NOT_SUPPORTED;
            break;

        case UX_GET_CONFIGURATION:
        	//printf("UX_GET_CONFIGURATION\n");
            status =  _ux_device_stack_configuration_get();
            break;

        case UX_SET_CONFIGURATION:
        	//printf("UX_SET_CONFIGURATION\n");
            status =  _ux_device_stack_configuration_set(request_value);
            break;

        case UX_GET_INTERFACE:
        	//printf("UX_GET_INTERFACE\n");
            status =  _ux_device_stack_alternate_setting_get(request_index);
            break;
                
        case UX_SET_INTERFACE:
        	//printf("UX_SET_INTERFACE\n");
            status =  _ux_device_stack_alternate_setting_set(request_index,request_value);
            break;
                

        case UX_SYNCH_FRAME:
        	//printf("UX_SYNCH_FRAME\n");
            status = UX_SUCCESS;
            break;

        default :

            status = UX_FUNCTION_NOT_SUPPORTED;
            break;
        }

        if (status != UX_SUCCESS)

            /* Stall the control endpoint to issue protocol error. */
            _ux_device_stack_endpoint_stall(&device -> ux_slave_device_control_endpoint);
    }

    /* Return the function status.  */
    return(status);
}

