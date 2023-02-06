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
 * @file     ux_dcd_dwc3_set_configuration.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    The function _ux_dcd_dwc3_SetConfiguration() will print
 *			 requirement to send configuration
 *			 returns 1 always
 *			 host.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE

/* System Includes  */
#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"

UINT _ux_dcd_dwc3_SetConfiguration(UX_DCD_DWC3 *dcd_dwc3, ULONG Parameter)
{
#ifdef DEBUG_SET_CONFIG
    printf("Need to send configuration\n");
#endif
    return 1;
}
