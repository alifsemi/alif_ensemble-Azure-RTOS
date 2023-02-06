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
 * @file     ux_dcd_dwc3_fifo_read.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will read from the FIFO of a particular endpoint.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_utility.h"
#include "ux_device_stack.h"


UINT  _ux_dcd_dwc3_fifo_read(UX_DCD_DWC3 *dcd_dwc3, UCHAR *data_pointer, ULONG fifo_length)
{

TX_INTERRUPT_SAVE_AREA
ULONG    fifo_value;
ULONG	offset = 0x0;

ULONG Ctrl;
Ctrl = (ULONG)&dcd_dwc3->SetupData;

    /* Number of bytes to read is based on DWORDS.  */
    fifo_length = (fifo_length + 3) / sizeof(ULONG);
    /* Lockout interrupts.  */
    TX_DISABLE

    /* Read one DWORD at a time.  */
    while (fifo_length--)
    {

        /* Read from FIFO.  */
	    fifo_value = ((ULONG)(*((ULONG *) (Ctrl + offset))));

	    /* Store this value in a endian agnostic way.  */
        _ux_utility_long_put(data_pointer, fifo_value);

        /* Increment the data pointer buffer address.  */
        data_pointer += sizeof(ULONG);
	    offset += sizeof(ULONG);
    }

    /* Restore interrupts.  */
    TX_RESTORE

    /* Return successful completion.  */
    return(UX_SUCCESS);
}
