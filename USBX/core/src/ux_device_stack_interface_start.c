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
/*    _ux_device_stack_interface_start                    PORTABLE C      */
/*                                                           6.1          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function starts an interface associated with the enabled       */
/*    configuration.                                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    interface                             Pointer to interface          */ 
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */ 
/*                                                                        */
/*  CALLS                                                                 */ 
/*                                                                        */ 
/*    (ux_slave_class_entry_function)       Device class entry function   */ 
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
/*                                            resulting in version 6.1    */
/*                                                                        */
/**************************************************************************/
UINT  _ux_device_stack_interface_start(UX_SLAVE_INTERFACE *interface)
{

UX_SLAVE_DEVICE             *device;
UX_SLAVE_CLASS              *class;
UINT                        status;
UX_SLAVE_CLASS_COMMAND      class_command;


    /* Get the class for the interface.  */
  //  printf("_ux_device_stack_interface_start\n");
    class =  _ux_system_slave -> ux_system_slave_interface_class_array[interface -> ux_slave_interface_descriptor.bInterfaceNumber];

    /* Check if class driver is available. */
    if (class == UX_NULL)

        /* There is no class driver supported. */
        return (UX_NO_CLASS_MATCH);

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Build all the fields of the Class Command.  */
    class_command.ux_slave_class_command_request   =    UX_SLAVE_CLASS_COMMAND_QUERY;
    class_command.ux_slave_class_command_interface =   (VOID *)interface;
    class_command.ux_slave_class_command_class     =   interface -> ux_slave_interface_descriptor.bInterfaceClass;
    //printf("Class value:%d\n",class_command.ux_slave_class_command_class);
    class_command.ux_slave_class_command_subclass  =   interface -> ux_slave_interface_descriptor.bInterfaceSubClass;
    //printf("Sub Class value:%d\n",interface -> ux_slave_interface_descriptor.bInterfaceSubClass);
    class_command.ux_slave_class_command_protocol  =   interface -> ux_slave_interface_descriptor.bInterfaceProtocol;
    class_command.ux_slave_class_command_vid       =   device -> ux_slave_device_descriptor.idVendor;
    class_command.ux_slave_class_command_pid       =   device -> ux_slave_device_descriptor.idProduct;

    /* We can now memorize the interface pointer associated with this class.  */
    class -> ux_slave_class_interface = interface;

    /* We have found a potential candidate. Call this registered class entry function.  */
    status = class -> ux_slave_class_entry_function(&class_command);
   // printf("STATUS:%d\n",status);

    /* The status tells us if the registered class wants to own this class.  */
    if (status == UX_SUCCESS)
    {
        //printf("ENTER into CLASS COMMAND ACTIVATE\n");
        /* Store the class container. */
        class_command.ux_slave_class_command_class_ptr =  class;
        
        /* Store the command.  */
        class_command.ux_slave_class_command_request =  UX_SLAVE_CLASS_COMMAND_ACTIVATE;
        
        /* Activate the class.  */
        status = class -> ux_slave_class_entry_function(&class_command);

        /* If the class was successfully activated, set the class for the interface.  */
        if(status == UX_SUCCESS)
            interface -> ux_slave_interface_class =  class;

        return(status); 
    }

    /* There is no driver who want to own this class!  */
    return(UX_NO_CLASS_MATCH);
}

