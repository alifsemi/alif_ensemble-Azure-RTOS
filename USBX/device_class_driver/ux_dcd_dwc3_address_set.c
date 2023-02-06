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
 * @file     ux_dcd_dwc3_address_set.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will set the address of the device after we have
 *		 received a SET_ADDRESS command from the host.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

/**
  \fn          UINT _ux_dcd_SetDeviceAddress(UX_DCD_DWC3 *dcd_dwc3, ULONG  Addr)
  \brief       This function is used to set device address
  \param[in]   dcd_dwc3: dw3 structure address
  \param[in]   Addr: Address to be set
  \return      \ref returns 0 or 1
*/
UINT _ux_dcd_SetDeviceAddress(UX_DCD_DWC3 *dcd_dwc3, ULONG  Addr)
{
ULONG RegVal;

    if (dcd_dwc3->config_state == USB_STATE_CONFIGURED) {
        return 0;
    }

    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCFG);
    RegVal &= ~(UX_DCD_DWC3_DCFG_DEVADDR_MASK);
    RegVal |= UX_DCD_DWC3_DCFG_DEVADDR_SET(Addr);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCFG, RegVal);

    if (Addr > 0U) {
        dcd_dwc3 -> config_state = USB_STATE_ADDRESS;
    } else {
        dcd_dwc3 -> config_state = USB_STATE_DEFAULT;
    }

    return 1;
}
