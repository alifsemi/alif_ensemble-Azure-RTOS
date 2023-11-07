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
 * @file     ux_dcd_dwc3_initialize.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function initializes the USB slave controller of the dwc3
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"
#include "RTE_Device.h"
#include "RTE_Components.h"
#include "sys_ctrl_usb.h"
#include "power.h"
#include CMSIS_device_header



TX_EVENT_FLAGS_GROUP BULKIN_BULKOUT_FLAG;

UINT  _ux_dcd_dwc3_initialize()
{

    UX_SLAVE_DCD    *dcd;
    UX_DCD_DWC3     *dcd_dwc3;
    UX_DCD_EVENT    *dcd_evt;
    ULONG           dwc3_register;
    ULONG           ScratchBuff;
    ULONG           *ScratchBuff_ptr;
    ULONG           Ret_Scratch;
    UINT            status;
    LONG            retries;

    /* Enable peripheral clk for USB  */
    enable_usb_periph_clk();

    /* USB phy reset */
    usb_ctrl2_phy_power_on_reset_clear();

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of DWC3 type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_DWC3_SLAVE_CONTROLLER;

    /* Allocate memory for this DWC3 DCD instance.  */
    dcd_dwc3 =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_DWC3));

    /* Check if memory was properly allocated.  */
    if(dcd_dwc3 == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    status = _ux_utility_event_flags_create(&BULKIN_BULKOUT_FLAG,"USB_BULKIN_BULKOUT_EVENT_FLAG");
    if (status != UX_SUCCESS)
    {
        return UX_EVENT_ERROR;
    }
    /* Set the pointer to the DWC3 DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_dwc3;

    /* Save the base address of the controller.  */
    dcd -> ux_slave_dcd_io =     USB_BASE;
    dcd_dwc3 -> ux_dcd_dwc3_base =  USB_BASE;

    /* Set the generic DCD owner for the DWC3 DCD.  */
    dcd_dwc3 -> ux_dcd_dwc3_dcd_owner =  dcd;

    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_dwc3_function;

    /* Allocate memory for event buffer.  */
    dcd_evt =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_EVENT));

     /* Check if memory was properly allocated.  */
    if(dcd_evt == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    dcd_evt -> dcd_dwc3        = dcd_dwc3;
    dcd_evt ->length		   = UX_DCD_DWC3_EVENT_BUFFER_SIZE;
    dcd_evt -> cache = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_DCD_DWC3_EVENT_BUFFER_SIZE);

    if(dcd_evt -> cache == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    dcd_evt -> buf = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_DCD_DWC3_EVENT_BUFFER_SIZE);

    if(dcd_evt -> buf == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    memset(dcd_evt -> buf , 0, DCD_DWC3_EVT_BUFF);

    dcd_dwc3 -> ux_dcd_dwc3_event = dcd_evt;

    /* Initialize dcd_dwc3 -> ep0_trb. May depreciated in future */
   dcd_dwc3 -> ep0_trb = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DC_EP0_TRB) * 2);

   if(dcd_dwc3 -> ep0_trb == UX_NULL)
       return(UX_MEMORY_INSUFFICIENT);

   ScratchBuff_ptr = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, UX_DCD_DWC3_SCRATCH_BUFF_SIZE);

   if(ScratchBuff_ptr == UX_NULL)
          return(UX_MEMORY_INSUFFICIENT);
    ScratchBuff = (ULONG)ScratchBuff_ptr;
    /* Perform the core soft reset.  */
    dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);

    dwc3_register |= UX_DCD_DWC3_FS_DCTL_CSRST;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, dwc3_register);

    retries = 1000;

    /* Wait for Soft Reset to be completed.  */
    do
    {
        /* Read the RST Control register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);

        if(!(dwc3_register & UX_DCD_DWC3_FS_DCTL_CSRST))
             goto done;

    } while(-- retries);

    if(retries == 0)
        return UX_ERROR;

done:
    /* Spec says wait for 50 cycles.  */
    _ux_dcd_dwc3_delay(50);

    _ux_dcd_phy_reset(dcd_dwc3);

       /* setup global control registers */
    dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GCTL);
    dwc3_register &= ~UX_DCD_GCTL_SCALEDOWN_MASK;
    dwc3_register &= ~UX_DCD_GCTL_DISSCRAMBLE;
    dwc3_register &= ~UX_DCD_GCTL_DSBLCLKGTNG;
    dwc3_register |= UX_DCD_GCTL_U2EXIT_LFPS;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GCTL, dwc3_register);

    _ux_dcd_dwc3_delay(50);

     /*Configure Global USB2 PHY Configuration Register */

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0), UX_DCD_DWC3_GUSB2PHYCFG_CFG);

    /* Initialize DCD USB event buffer.  */
    dcd_evt -> lpos = 0;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTADRLO(0), lower_32_bits(LocalToGlobal(dcd_evt -> buf)));
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTADRHI(0), upper_32_bits(LocalToGlobal(dcd_evt -> buf)));

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTSIZ(0), dcd_evt -> length);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTCOUNT(0), UX_DCD_DWC3_GEVENT_COUNT);

    ULONG reg_val = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GHWPARAMS3);
    dcd_dwc3->NumInEps = UX_DCD_DWC3_IN_EPS(reg_val);
    dcd_dwc3->NumOutEps = (UX_DCD_DWC3_NUM_EPS(reg_val) -
                        dcd_dwc3->NumInEps);

    _ux_dcd_InitializeEps(dcd_dwc3);

    Ret_Scratch = _ux_dcd_SetupScratchpad(dcd_dwc3, ScratchBuff);

    if(Ret_Scratch != 0)
         return Ret_Scratch;

    /* Set speed on Controller */
    _ux_dcd_set_speed(dcd_dwc3);

    /* Define incr in global soc bus configuration.  */
    _ux_dcd_dwc3_register_set(dcd_dwc3, UX_DCD_DWC3_GSBUSCFG0, UX_DCD_DWC3_GSBUSCFG);

    /* Update Global User control register */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUCTL, UX_DCD_DWC3_GUCTL_CFG);


    dcd_dwc3 -> ConnectionDone = 0;

    /* BULK endpoint transfers event handler registration */
    _ux_dcd_SetEpHandler(dcd_dwc3, 2, 0,
            BulkOutHandler);
    _ux_dcd_SetEpHandler(dcd_dwc3, 2, 1,
                        BulkInHandler);

    /* Starting controller . */
    dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);

    dwc3_register |= UX_DCD_DWC3_DCTL_START;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, dwc3_register);

    retries = 500;
    do{

        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DSTS);

        if(!(dwc3_register & UX_DCD_DSTS_DEVCTRLHLT))
              goto next;
      }while (-- retries);
      if(retries == 0)
         return UX_ERROR;

next:

    /* enable USB Reset, Connection Done, and USB/Link State Change events */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEVTEN, UX_DCD_DWC3_DEV_INT_EN);

    NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
    NVIC_EnableIRQ(USB_IRQ_IRQn);

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;
    /* Return successful completion.  */
    return(UX_SUCCESS);
}

/**
  \fn          void  USBx_IRQHandler (void)
  \brief       Run the Interrupt Handler for USBx
*/
void USB_IRQHandler(void)
{
    _ux_dcd_dwc3_interrupt_handler();
}

VOID _ux_dcd_InitializeEps(UX_DCD_DWC3 *dcd_dwc3)
{
ULONG  i;
ULONG Epnum;

    for (i = 0U; i < dcd_dwc3->NumOutEps; i++) {
        Epnum = (i << 1U) | UX_DCD_EP_DIR_OUT;
        dcd_dwc3->eps[Epnum].Phy_EPNum = Epnum;
        dcd_dwc3->eps[Epnum].ux_dcd_dwc3_ed_direction = UX_DCD_EP_DIR_OUT;
        dcd_dwc3->eps[Epnum].ux_dcd_dwc3_ed_resource_index = 0U;
        }
        for (i = 0U; i < dcd_dwc3->NumInEps; i++) {
            Epnum = (i << 1U) | UX_DCD_EP_DIR_IN;
            dcd_dwc3->eps[Epnum].Phy_EPNum = Epnum;
            dcd_dwc3->eps[Epnum].ux_dcd_dwc3_ed_direction = UX_DCD_EP_DIR_IN;
            dcd_dwc3->eps[Epnum].ux_dcd_dwc3_ed_resource_index = 0U;
        }

}

VOID _ux_dcd_phy_reset(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG             RegVal;

    /* Before Resetting PHY, put Core in Reset */
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GCTL);
    RegVal |= UX_DCD_GCTL_CORESOFTRESET;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GCTL, RegVal);

    /* Assert USB3 PHY reset */
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB3PIPECTL(0U));
    RegVal |= UX_DCD_GUSB3PIPECTL_PHYSOFTRST;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB3PIPECTL(0U), RegVal);

    /* Assert USB2 PHY reset */
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0U));
    RegVal |= UX_DCD_GUSB2PHYCFG_PHYSOFTRST;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0U), RegVal);

    _ux_dcd_dwc3_delay(50);

    /* Clear USB3 PHY reset */
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB3PIPECTL(0U));
    RegVal &= ~UX_DCD_GUSB3PIPECTL_PHYSOFTRST;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB3PIPECTL(0U), RegVal);


    /* Clear USB2 PHY reset */
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0U));
    RegVal &= ~UX_DCD_GUSB2PHYCFG_PHYSOFTRST;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0U), RegVal);

    _ux_dcd_dwc3_delay(50);

    /* Take Core out of reset state after PHYS are stable*/
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GCTL);
    RegVal &= ~UX_DCD_GCTL_CORESOFTRESET;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GCTL, RegVal);

}


VOID _ux_dcd_set_speed(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG  RegVal;
    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCFG);
    RegVal &= ~(UX_DCD_DCFG_SPEED_MASK);
    RegVal |= UX_DCD_DCFG_HIGHSPEED;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCFG, RegVal);

}

UINT _ux_dcd_SetupScratchpad(UX_DCD_DWC3 *dcd_dwc3, ULONG ScratchBuff)
{
    ULONG Ret;
    Ret = _ux_dcd_SendGadgetGenericCmd(dcd_dwc3,
                        UX_DCD_DGCMD_SET_SCRATCHPAD_ADDR_LO,
                        (ULONG)(ScratchBuff & 0xFFFFFFFFU));
    if (Ret !=  0) {
#ifdef	DEBUG
        printf("Failed to set scratchpad low addr: %d\n", Ret);
#endif
        return Ret;
    }

    Ret = _ux_dcd_SendGadgetGenericCmd(dcd_dwc3,
                UX_DCD_DGCMD_SET_SCRATCHPAD_ADDR_HI,
                        (ULONG)((ScratchBuff >> 16U) >> 16U));
    if (Ret != 0) {
#ifdef	DEBUG
        printf("Failed to set scratchpad high addr: %d\n", Ret);
#endif
        return Ret;
    }

    return Ret;

}

UINT _ux_dcd_SendGadgetGenericCmd(UX_DCD_DWC3 *dcd_dwc3, ULONG cmd,
                        ULONG param)
{
    ULONG RegVal, retry = 500U;
    LONG  status = 0;
    LONG  ret = 0;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DGCMDPAR, param);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DGCMD,
                                cmd | UX_DCD_DGCMD_CMDACT);
     do
     {
        RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DGCMD);
        if(!(RegVal & UX_DCD_DGCMD_CMDACT))
        {
            status = UX_DCD_DGCMD_STATUS(RegVal);
            if(status)
                ret =  UX_ERROR;
             break;
        }
     }while (-- retry);
     if(!retry)
     {
         ret = UX_ERROR;
         status = UX_ERROR;
     }

     return ret;
}

VOID _ux_dcd_SetEpHandler(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum,
                        ULONG Dir, VOID (*Handler)(VOID *, ULONG, ULONG, UCHAR *, ULONG))
{
        ULONG PhyEpNum;
	UX_DCD_DWC3_ED *Ept;

        PhyEpNum = UX_DCD_PhysicalEp(Epnum, Dir);
        Ept = &dcd_dwc3->eps[PhyEpNum];
        Ept->Handler = (VOID (*)(VOID *, ULONG, ULONG, UCHAR *, ULONG))Handler;
}
