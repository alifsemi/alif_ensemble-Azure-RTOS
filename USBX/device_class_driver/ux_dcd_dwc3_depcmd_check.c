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
 * @file     ux_dcd_dwc3_depcmd_check.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function reads UX_DCD_DWC3_DEPCMD register from the dwc3 controller.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

UINT _ux_dcd_dwc3_depcmd_check(UX_DCD_DWC3 *dcd_dwc3)
{
ULONG timeout = 1000;
ULONG dwc3_register;
ULONG cmd_status;
ULONG ret;

    do {
            /* Read the device endpoint Command register.  */
	    dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) +
                                                    UX_DCD_DWC3_DEPCMD);
            if (!(dwc3_register & UX_DCD_DEPCMD_CMDACT)) {
                cmd_status = UX_DCD_DEPCMD_STATUS(dwc3_register);

                switch (cmd_status) {
                    case 0:
                        ret = 0;
                        break;
                    case DEPEVT_TRANSFER_NO_RESOURCE:
                        ret = -1;
                        break;
                    case DEPEVT_TRANSFER_BUS_EXPIRY:
                    /*
                     * SW issues START TRANSFER command to
                     * isochronous ep with future frame interval. If
                     * future interval time has already passed when
                     * core receives the command, it will respond
                     * with an error status of 'Bus Expiry'.
                     *
                     * Instead of always returning -EINVAL, let's
                     * give a hint to the gadget driver that this is
                     * the case by returning -EAGAIN.
                     */
                        ret = -2;
                        break;
                    default:
                        printf("UNKNOWN cmd status\n");
                }
                break;
            }
        } while (--timeout);

	if (timeout == 0) {
            ret = -3;
            cmd_status = -3;
        }

	return ret;
}
