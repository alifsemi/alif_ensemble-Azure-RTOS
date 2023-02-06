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
 * @file     ux_dcd_dwc3_rxfifo_nump.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will read register and update value in register.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#define UX_SOURCE_CODE


/* System Includes  */
#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"


VOID  _ux_dcd_dwc3_gadget_setup_nump(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG ram2_depth;
    ULONG mdwidth;
    ULONG nump;
    ULONG reg;

    reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GHWPARAMS7);
    ram2_depth = UX_DCD_DWC3_GHWPARAMS7_RAM2_DEPTH(reg);
    reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GHWPARAMS0);
    mdwidth = DWC3_GHWPARAMS0_MDWIDTH(reg);

    nump = ((ram2_depth * mdwidth / 8) - 24 - 16) / 1024;
    nump = min_t(ULONG, nump, 16);

    /* update NumP */
    reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCFG);
    reg &= ~UX_DCD_DWC3_DCFG_NUMP_MASK;
    reg |= nump << UX_DCD_DWC3_DCFG_NUMP_SHIFT;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCFG, reg);

}
