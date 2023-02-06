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
 * @file     ux_dcd_dwc3_register_clear.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function clears a bit in a register of the DWC3 controller.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE

/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

VOID  _ux_dcd_dwc3_register_clear(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG value)
{
    *((ULONG *) (volatile ULONG *)(dcd_dwc3 -> ux_dcd_dwc3_base + dwc3_register)) &=~ value;
    return;
}
