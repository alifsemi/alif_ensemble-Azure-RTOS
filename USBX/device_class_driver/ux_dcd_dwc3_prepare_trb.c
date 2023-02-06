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
 * @file     ux_dcd_dwc3_prepare_trb.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function is invoked under ISR when an event happens on a
 *		     specific endpoint.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"
#include "ux_utility.h"


#if 0
VOID  _ux_dcd_dwc3_ep0_prepare_one_trb(UX_DCD_DWC3 *dcd_dwc3, UX_DCD_DWC3_ED *ep0, ULONG ep0_trb, ULONG len,
                        ULONG flag, ULONG chain)
{
ULONG                   dwc3_register;
UX_SLAVE_ENDPOINT       *endpoint;
UX_DCD_DWC3_ED         *ed;
UCHAR *                 data_pointer;
ULONG                   fifo_length;
ULONG                   dwc3_endpoint_index;
ULONG                    endpoint_control_address;
ULONG                    endpoint_size_address;
ULONG                    endpoint_control;
ULONG                    endpoint_size;
UX_DC_EP0_TRB                 *trb;
UX_DCD_DWC3		*temp_dcd;

	trb = &dcd_dwc3 -> ep0_trb[dcd_dwc3 -> trb_enqueue];
	printf("trb address [%x]\n", trb);

        if (chain)
                dcd_dwc3->trb_enqueue++;

        trb->bpl = lower_32_bits(ep0_trb);
        trb->bph = upper_32_bits(ep0_trb);
        trb->size = len;
        trb->ctrl = flag;

        trb->ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                        | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);

        if (chain)
                trb->ctrl |= UX_DCD_DWC3_TRB_CTRL_CHN;
        else
                trb->ctrl |= (UX_DCD_DWC3_TRB_CTRL_IOC
                                | UX_DCD_DWC3_TRB_CTRL_LST);

}
#endif
