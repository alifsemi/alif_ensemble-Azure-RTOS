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
/*    _ux_device_stack_interface_set                      PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets one alternate setting of one interface and       */
/*    enable all endpoints associated with this alternate setting.        */
/*    configuration.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    device_framework                      Address in device framework   */ 
/*                                          for selected alternate setting*/
/*    device_framework_length               Length of device framework    */ 
/*    alternate_setting_value               Alternate setting             */ 
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */
/*    (ux_slave_dcd_function)               DCD dispatch function         */ 
/*    _ux_device_stack_interface_start      Start interface               */ 
/*    _ux_utility_descriptor_parse          Parse descriptor              */ 
/*                                                                        */ 
/*  CALLED BY                                                             */ 
/*                                                                        */ 
/*    Application                                                         */ 
/*    Device Stack                                                        */
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            optimized based on compile  */
/*                                            definitions,                */
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_interface_set(UCHAR * device_framework, ULONG device_framework_length,
                                                    ULONG alternate_setting_value)
{

UX_SLAVE_DCD            *dcd;
UX_SLAVE_DEVICE         *device;
UX_SLAVE_TRANSFER       *transfer_request;
UX_SLAVE_INTERFACE      *interface;
#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
UX_SLAVE_INTERFACE      *interface_link;
ULONG                   interfaces_pool_number;
#endif
UX_SLAVE_ENDPOINT       *endpoint;
UX_SLAVE_ENDPOINT       *endpoint_link;
ULONG                   descriptor_length;
UCHAR                   descriptor_type;
ULONG                   endpoints_pool_number;
UINT                    status;

    UX_PARAMETER_NOT_USED(alternate_setting_value);

    /* If trace is enabled, insert this event into the trace buffer.  */
    UX_TRACE_IN_LINE_INSERT(UX_TRACE_DEVICE_STACK_INTERFACE_SET, alternate_setting_value, 0, 0, 0, UX_TRACE_DEVICE_STACK_EVENTS, 0, 0)

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Find a free interface in the pool and hook it to the 
       existing interface.  */
    interface = device -> ux_slave_device_interfaces_pool;

    //printf("interface [%x] from free pool pool_number [%x]\n",
//		    interface, device -> ux_slave_device_interfaces_pool_number);

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1
    interfaces_pool_number = device -> ux_slave_device_interfaces_pool_number;

    while (interfaces_pool_number != 0)
    {
        /* Check if this interface is free.  */
        if (interface -> ux_slave_interface_status == UX_UNUSED)
            break;
    
        /* Try the next interface.  */
        interface++;
        
        /* Decrement the number of interfaces left to scan in the pool.  */
        interfaces_pool_number--;
    }

    /* Did we find a free interface ?  */
    if (interfaces_pool_number == 0)
        return(UX_MEMORY_INSUFFICIENT);
#else

    /* Check if this interface is free.  */
    if (interface -> ux_slave_interface_status != UX_UNUSED)
        return(UX_MEMORY_INSUFFICIENT);
    
#endif

    /* Mark this interface as used now.  */
    interface -> ux_slave_interface_status = UX_USED;

    /* If trace is enabled, register this object.  */
    UX_TRACE_OBJECT_REGISTER(UX_TRACE_DEVICE_OBJECT_TYPE_INTERFACE, interface, 0, 0, 0)

    /* Parse the descriptor in something more readable.  */
    _ux_utility_descriptor_parse(device_framework,
                _ux_system_interface_descriptor_structure,
                UX_INTERFACE_DESCRIPTOR_ENTRIES,
                (UCHAR *) &interface -> ux_slave_interface_descriptor);

#if !defined(UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE) || UX_MAX_DEVICE_INTERFACES > 1

    /* Attach this interface to the end of the interface chain.  */
    if (device -> ux_slave_device_first_interface == UX_NULL)
    {

        device -> ux_slave_device_first_interface =  interface;
    }
    else
    {
        /* Multiple interfaces exist, so find the end of the chain.  */
        interface_link =  device -> ux_slave_device_first_interface;
        while (interface_link -> ux_slave_interface_next_interface != UX_NULL)
            interface_link =  interface_link -> ux_slave_interface_next_interface;
        interface_link -> ux_slave_interface_next_interface =  interface;
    }
#else


    /* It must be very first one.  */
    device -> ux_slave_device_first_interface = interface;
#endif

//    printf("device_framework [%x]\n", *device_framework);
    /* Point beyond the interface descriptor.  */
    device_framework_length -=  (ULONG) *device_framework;
    device_framework +=  (ULONG) *device_framework;

//    printf("interface [%x] can be passed framework_len [%x] device_framework [%x] [%x]\n",
//		    interface, device_framework_length, device_framework, *device_framework);
    /* Parse the device framework and locate endpoint descriptor(s).  */
    while (device_framework_length != 0)
    {

        /* Get the length of the current descriptor.  */
        descriptor_length =  (ULONG) *device_framework;

        /* And its type.  */
        descriptor_type =  *(device_framework + 1);
   
//	printf("device_framework_length [%x] descriptor_length [%x] descriptor_type [%x]\n",
//			device_framework_length,
//			descriptor_length, descriptor_type);

        /* Check if this is an endpoint descriptor.  */
        switch(descriptor_type)
        {

        case UX_ENDPOINT_DESCRIPTOR_ITEM:

            /* Find a free endpoint in the pool and hook it to the 
               existing interface after it's created by DCD.  */
        	//printf("END Point descriptor\n");
            endpoint = device -> ux_slave_device_endpoints_pool;
            endpoints_pool_number = device -> ux_slave_device_endpoints_pool_number;
            while (endpoints_pool_number != 0)
            {
                /* Check if this endpoint is free.  */
                if (endpoint ->    ux_slave_endpoint_status == UX_UNUSED)
                {
                    /* Mark this endpoint as used now.  */
                    endpoint ->    ux_slave_endpoint_status = UX_USED;
                    break;
                }
            
                /* Try the next endpoint.  */
                endpoint++;
                
                /* Decrement the number of endpoints to scan from the pool.  */
               endpoints_pool_number--; 
            }

            /* Did we find a free endpoint ?  */
            if (endpoints_pool_number == 0)
                return(UX_MEMORY_INSUFFICIENT);

            /* Parse the descriptor in something more readable.  */
            _ux_utility_descriptor_parse(device_framework,
                            _ux_system_endpoint_descriptor_structure,
                            UX_ENDPOINT_DESCRIPTOR_ENTRIES,
                            (UCHAR *) &endpoint -> ux_slave_endpoint_descriptor);

            /* Now we create a transfer request to accept transfer on this endpoint.  */
            transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;
                
            /* We store the endpoint in the transfer request as well.  */
            transfer_request -> ux_slave_transfer_request_endpoint =  endpoint;
                
            /* By default the timeout is infinite on request.  */
            transfer_request -> ux_slave_transfer_request_timeout = UX_WAIT_FOREVER;
            
            /* Attach the interface to the endpoint.  */
            endpoint -> ux_slave_endpoint_interface =  interface;

	//	printf("Interface pointer is [%x]\n",
	//	    interface);

            /* Attach the device to the endpoint.  */
            endpoint -> ux_slave_endpoint_device =  device;
               
            /* Create the endpoint at the DCD level.  */
            status =  dcd -> ux_slave_dcd_function(dcd, UX_DCD_CREATE_ENDPOINT, (VOID *) endpoint); 
            
            /* Do a sanity check on endpoint creation.  */
            if (status != UX_SUCCESS)
            {

                /* Error was returned, endpoint cannot be created.  */
                endpoint -> ux_slave_endpoint_status = UX_UNUSED;
                return(status);
            }

            /* Attach this endpoint to the end of the endpoint chain.  */
            if (interface -> ux_slave_interface_first_endpoint == UX_NULL)
            {

                interface -> ux_slave_interface_first_endpoint =  endpoint;
            }
            else
            {
                /* Multiple endpoints exist, so find the end of the chain.  */
                endpoint_link =  interface -> ux_slave_interface_first_endpoint;
                while (endpoint_link -> ux_slave_endpoint_next_endpoint != UX_NULL)
                    endpoint_link =  endpoint_link -> ux_slave_endpoint_next_endpoint;
                endpoint_link -> ux_slave_endpoint_next_endpoint =  endpoint;
            }
	//	printf("interface_first_endpoint [%x]\n",
	//			interface -> ux_slave_interface_first_endpoint);
	    break;

        case UX_CONFIGURATION_DESCRIPTOR_ITEM:
        case UX_INTERFACE_DESCRIPTOR_ITEM:

            /* If the descriptor is a configuration or interface,
               we have parsed and mounted all endpoints. 
               The interface attached to this configuration must be started at the class level.  */
	        //printf("Interface pointer passed in start [%x]\n",interface);
        	//printf("UX_INTERFACE_DESCRIPTOR_ITEM &UX_CONFIGURATION_DESCRIPTOR_ITEM\n");

	    status =  _ux_device_stack_interface_start(interface);

            /* Return the status to the caller.  */
            return(status);

        default:
            break;
        }

        /* Adjust what is left of the device framework.  */
        device_framework_length -=  descriptor_length;

        /* Point to the next descriptor.  */
        device_framework +=  descriptor_length;
    }

    /* The interface attached to this configuration must be started at the class
       level.  */
    status =  _ux_device_stack_interface_start(interface);

    /* Return the status to the caller.  */
    return(status);
}

