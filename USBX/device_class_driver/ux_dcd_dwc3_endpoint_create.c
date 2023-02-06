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
 * @file     ux_dcd_dwc3_endpoint_create.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will create a physical endpoint.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"


UINT  _ux_dcd_dwc3_endpoint_create(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_ENDPOINT *endpoint)
{

UX_DCD_DWC3_ED      *ed;
ULONG               dwc3_endpoint_index;
ULONG               dwc3_endpoint_mask;
ULONG               dwc3_endpoint_address;
UX_SLAVE_DCD        *dcd;
UX_DCD_DWC3         *dcd_dwc332;
UX_SLAVE_DEVICE     *device;
ULONG		    Status;
ULONG PhyEpNum;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the STM32 DCD.  */
    dcd_dwc332 = (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

   /* The endpoint index in the array of the DWC3 must match the endpoint number.
       The DWC3 has 4 endpoints maximum. Each can be IN/OUT.  */
    dwc3_endpoint_index =  endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;

    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
    {

        PhyEpNum = UX_DCD_PhysicalEp(dwc3_endpoint_index, 1);
    }
    else
    {
	PhyEpNum = UX_DCD_PhysicalEp(dwc3_endpoint_index, 0);
    }
    /* Fetch the address of the physical endpoint.  */
    ed =  &dcd_dwc3 -> ux_dcd_dwc3_ed[PhyEpNum];


    /* Check the endpoint status, if it is free, reserve it. If not reject this endpoint.  */
    if ((ed -> ux_dcd_dwc3_ed_status & UX_DCD_DWC3_ED_STATUS_USED) == 0)
    {
        /* We can use this endpoint.  */

        /* Keep the physical endpoint address in the endpoint container.  */
        endpoint -> ux_slave_endpoint_ed =  (VOID *) ed;

        /* And its mask.  */
        ed -> ux_dcd_dwc3_ed_index =  dwc3_endpoint_index;

        /* Save the endpoint pointer.  */
        ed -> ux_dcd_dwc3_ed_endpoint =  endpoint;

	    /* For IN endpoint, the FIFO number is stored in the DALEPENA register.  */

            /* Build the endpoint mask from the endpoint descriptor.  */
         dwc3_endpoint_mask =  endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE;

            /* Build the endpoint DIEP or DOEP register. */
            switch (dwc3_endpoint_mask)
            {

                case UX_CONTROL_ENDPOINT:

                    /* Set the control endpoint for both IN and OUT.  */
                    ed -> ux_dcd_dwc3_ed_type  = UX_CONTROL_ENDPOINT;

	                for (dwc3_endpoint_index = 0; dwc3_endpoint_index < 8; dwc3_endpoint_index++)
	                {
		                dwc3_endpoint_address = dwc3_endpoint_index >> 1;

		                if(dwc3_endpoint_address == 0)
		                {
			                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_type  = UX_CONTROL_ENDPOINT;
			                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_maxpacket  = 64;
		                }
		                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_address =
		                dcd_dwc3 -> ux_dcd_dwc3_base + UX_DCD_DWC3_DEP_BASE(dwc3_endpoint_index);
		                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_index = dwc3_endpoint_index;
		                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_direction = dwc3_endpoint_index & 1;
		                dcd_dwc3 -> eps[dwc3_endpoint_index].ux_dcd_dwc3_ed_resource_index = 0U;
		                dcd_dwc3 -> eps[dwc3_endpoint_index].trb_enqueue = 0;
		                dcd_dwc3 -> eps[dwc3_endpoint_index].trb_dequeue = 0;

	                }
                    break;

                case UX_BULK_ENDPOINT:

                    /* Store the endpoint type.  */
                    ed -> ux_dcd_dwc3_ed_type  = UX_BULK_ENDPOINT;
                    ed -> ux_dcd_dwc3_ed_maxpacket  = 512;

                    if((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
                    {

                         Status = _ux_dcd_EpEnable(dcd_dwc3, UX_BULK_ENDPOINT, 1, 512, UX_DCD_ENDPOINT_XFER_BULK, 0);
	                     if(!Status)
	                     {
		                     printf("UX_BULK_ENDPOINT IN configuration failed\n");
	                     }
                             ed -> ux_dcd_dwc3_ed_status |=  UX_DCD_DWC3_ED_STATUS_USED;
                    }
                    else
                    {
			 Status = _ux_dcd_EpEnable(dcd_dwc3, UX_BULK_ENDPOINT, 0, 512, UX_DCD_ENDPOINT_XFER_BULK, 0);
                         if(!Status)
			 {
			      printf("UX_BULK_ENDPOINT OUT configuration failed\n");
			 }
                         ed -> ux_dcd_dwc3_ed_status |=  UX_DCD_DWC3_ED_STATUS_USED;
                    }


		            dcd_dwc3->test_mode = 0;
		            dcd_dwc3->IsocTrans = 0;
		            dcd_dwc3 -> total_trb = 0;

	                break;

	            case UX_ISOCHRONOUS_ENDPOINT:

                    /* Store the endpoint type.  */
                    ed -> ux_dcd_dwc3_ed_type  = UX_ISOCHRONOUS_ENDPOINT;
                    ed -> ux_dcd_dwc3_ed_maxpacket  = 1024;

                    break;

                case UX_INTERRUPT_ENDPOINT:

                    /* Store the endpoint type.  */
                    ed -> ux_dcd_dwc3_ed_type  = UX_INTERRUPT_ENDPOINT;
                    ed -> ux_dcd_dwc3_ed_maxpacket  = 8;
                    if((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
                    {

                          Status = _ux_dcd_EpEnable(dcd_dwc3, UX_INTERRUPT_ENDPOINT, 1, 8, UX_DCD_ENDPOINT_XFER_INT, 0);
	                      if(!Status)
	                      {
		                        printf("UX_INT_ENDPOINT IN configuration failed\n");
	                      }
                              ed -> ux_dcd_dwc3_ed_status |=  UX_DCD_DWC3_ED_STATUS_USED;
                    }
                    else
                    {

                          Status = _ux_dcd_EpEnable(dcd_dwc3, UX_INTERRUPT_ENDPOINT, 0, 8, UX_DCD_ENDPOINT_XFER_INT, 0);
	                      if(!Status)
	                      {
		                      printf("UX_INT_ENDPOINT IN configuration failed\n");
	                      }
                              ed -> ux_dcd_dwc3_ed_status |=  UX_DCD_DWC3_ED_STATUS_USED;
                    }
                    break;
                default:

            return(UX_ERROR);
        }

	        /* Continue initialization for non control endpoints.  */

        if (dwc3_endpoint_mask != UX_CONTROL_ENDPOINT)
        {

            /* Set the endpoint direction.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            {

                /* The endpoint is IN.  */
                ed -> ux_dcd_dwc3_ed_direction  = UX_ENDPOINT_IN;

            }
            else
            {

                /* The endpoint is OUT.  */
                ed -> ux_dcd_dwc3_ed_direction   = UX_ENDPOINT_OUT;

            }

        }

        /* Return successful completion.  */
        return(UX_SUCCESS);
    }

    /* Return an error.  */
    return(UX_NO_ED_AVAILABLE);
}
