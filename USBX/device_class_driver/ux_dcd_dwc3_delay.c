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
 * @file     ux_dcd_dwc3_delay.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function performs a wait of usec on the DWC3 platform.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

VOID  _ux_dcd_dwc3_delay(ULONG usec)
{
volatile ULONG     utime;
volatile ULONG     ucount = 0;
ULONG              ucount_local;
ULONG              utime_local;

    /* Calculate the time to wait in cycles.  */
    utime = UX_DCD_DWC3_CONTROLLER_DELAY * usec;

    /* Now loop to wait.  */
    do
    {
        /* Check the count.  Place volatile variables in non-volatile to
           avoid compiler confusion regarding the order of volatile
           comparisons.  */
        ucount_local =  ++ucount;
        utime_local =  utime;
        if (ucount_local > utime_local)

            /* Done.  */
            return;

    } while(1);
}
