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
 * @file     ux_dcd_dwc3_clock_set.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will set the clock for the dwc3 controller.
 *           value
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

VOID _ux_dcd_dwc3_clock_set(ULONG clock_base, ULONG clock_offset, ULONG value)
{

    *((ULONG *) (volatile ULONG *) (clock_base + clock_offset)) |=  value;
}
