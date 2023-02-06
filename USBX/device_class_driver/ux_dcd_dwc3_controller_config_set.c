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
 * @file     ux_dcd_dwc3_controller_config_set.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will set the state of the controller to the desired value
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

VOID  _ux_dcd_dwc3_controller_config_check(UX_DCD_DWC3 *dcd_dwc3)
{
ULONG dwc3_register;

	dcd_dwc3 -> endp_config = 0;
	dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0));

	if (dwc3_register & UX_DCD_GUSB2PHYCFG_SUSPHY) {
		dcd_dwc3 -> endp_config |= UX_DCD_GUSB2PHYCFG_SUSPHY;
		dwc3_register &= ~UX_DCD_GUSB2PHYCFG_SUSPHY;
	}

	if (dwc3_register & UX_DCD_GUSB2PHYCFG_ENBLSLPM) {
		dcd_dwc3 -> endp_config |= UX_DCD_GUSB2PHYCFG_ENBLSLPM;
		dwc3_register &= ~UX_DCD_GUSB2PHYCFG_ENBLSLPM;
	}

	if (dcd_dwc3 -> endp_config)
		_ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0), dwc3_register);
}

VOID  _ux_dcd_dwc3_controller_config_reset(UX_DCD_DWC3 *dcd_dwc3)
{
ULONG dwc3_register;

	if (dcd_dwc3 -> endp_config) {
	dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0));
	dwc3_register |= dcd_dwc3 -> endp_config;
	_ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0), dwc3_register);
	}
}
