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
 * @file     ux_dcd_dwc3_devt_check.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function returns the Device specific event type by reading a register from the dwc3 controller.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"


UINT _ux_dcd_dwc3_check_devt_event_type(ULONG dwc3_register)
{
    UINT event_type;
    dwc3_register = (dwc3_register & 0x1f00) >> 8;
    if(dwc3_register == 0)
    {
    	return dwc3_register;
    }
    if((dwc3_register & 0x1) == 0x1)
    {
        event_type = 1;
    }
    else if((dwc3_register & 0x2) == 0x2)
    {
        event_type = 2;
    }
    else if((dwc3_register & 0x3) == 0x3)
    {
        event_type = 3;
    }
    else if((dwc3_register & 0x4) == 0x4)
    {
        event_type = 4;
    }
    else if((dwc3_register & 0x5) == 0x5)
    {
        event_type = 5;
    }
    else if((dwc3_register & 0x6) == 0x6)
    {
        event_type = 6;
    }
    else
    {
       event_type =  dwc3_register;
    }
	return event_type;
}
