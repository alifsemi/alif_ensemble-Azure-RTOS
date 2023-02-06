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
#include CMSIS_device_header



TX_EVENT_FLAGS_GROUP BULKIN_BULKOUT_FLAG;

UINT  _ux_dcd_dwc3_initialize()
{

UX_SLAVE_DCD    *dcd;
UX_DCD_DWC3	*dcd_dwc3;
UX_DCD_EVENT	*dcd_evt;
ULONG           dwc3_register;
ULONG		ScratchBuff;
ULONG		*ScratchBuff_ptr;
ULONG		Ret_Scratch;
UINT            semaphore_create_status;
UINT            status;

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
    dcd -> ux_slave_dcd_io =     USB0_BASE;
    dcd_dwc3 -> ux_dcd_dwc3_base =  USB0_BASE;

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
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, UX_DCD_DWC3_FS_DCTL_CSRST);

    /* Wait for Soft Reset to be completed.  */
    do
    {
        /* Read the RST Control register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);

    } while (dwc3_register & UX_DCD_DWC3_FS_DCTL_CSRST_STS);

    /* Spec says wait for 50 cycles.  */
    _ux_dcd_dwc3_delay(50);

    _ux_dcd_phy_reset(dcd_dwc3);

    /************ Checking Hibernate ***********************/
        dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GCTL);
        dwc3_register &= ~UX_DCD_GCTL_SCALEDOWN_MASK;
        dwc3_register &= ~UX_DCD_GCTL_DISSCRAMBLE;
        dwc3_register &= ~UX_DCD_GCTL_DSBLCLKGTNG;
        dwc3_register |= UX_DCD_GCTL_U2EXIT_LFPS;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GCTL, dwc3_register);

    /*********************************************************/

    _ux_dcd_dwc3_delay(50);

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUSB2PHYCFG(0), UX_DCD_DWC3_GUSB2PHYCFG_CFG);
    _ux_dcd_dwc3_delay(50);

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

    /* Set speed on Controller */
    _ux_dcd_set_speed(dcd_dwc3);

    /* Define incr in global soc bus configuration.  */
    _ux_dcd_dwc3_register_set(dcd_dwc3, UX_DCD_DWC3_GSBUSCFG0, UX_DCD_DWC3_GSBUSCFG);

    /* Update Global User control register */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GUCTL, UX_DCD_DWC3_GUCTL_CFG);


   /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
    /* Issuing DEPSTARTCFG Command to ep_resource[0] (for EP 0/CONTROL/) */
   _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_SCFG);
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) +
								UX_DCD_DWC3_DEPCMD);
    }while ((dwc3_register & 0x400) != 0);
    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);


    /* Initializing endpoints here ...... */

    /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
    /* Issuing DEPCFG Command to ep_resource[0] (for EP 0 OUT/CONTROL/) */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMDPAR1, UX_DCD_DWC3_DEPCMD_EP0_OUT_PAR1);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP0_PAR0);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMDPAR2, UX_DCD_DWC3_DEPCMD_EP0_PAR2);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_COMMON_SCFG);
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) +
								UX_DCD_DWC3_DEPCMD);
    }while ((dwc3_register & 0x400) != 0);
    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);


    /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
   /* Issuing DEPCFG Command to ep_resource[1] (for EP 0 IN/CONTROL/) */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMDPAR1, UX_DCD_DWC3_DEPCMD_EP0_IN_PAR1);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP0_PAR0);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMDPAR2, UX_DCD_DWC3_DEPCMD_EP0_PAR2);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_COMMON_SCFG);
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) +
								UX_DCD_DWC3_DEPCMD);
       // printf("EP0 IN register value:%x\n",dwc3_register);
    }while ((dwc3_register & 0x400) != 0);
    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);


    /* Check controller config before issuing DEPCMD always  */
        _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
        /* Issuing DEPCFG Command to ep_resource[0] (for EP 2 BULK OUT/) */
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(4) + UX_DCD_DWC3_DEPCMDPAR1, UX_DCD_DWC3_DEPCMD_EP2_OUT_PAR1);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(4) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP2_OUT_PAR0);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(4) + UX_DCD_DWC3_DEPCMDPAR2, UX_DCD_DWC3_DEPCMD_EP2_PAR2);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(4) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_COMMON_SCFG);
        do{
            /* Read the device endpoint Command register.  */
            dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(4) +
								UX_DCD_DWC3_DEPCMD);
        }while ((dwc3_register & 0x400) != 0);
        _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);


        /* Check controller config before issuing DEPCMD always */
        _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
       /* Issuing DEPCFG Command to ep_resource[1] (for EP 2 BULK IN/) */
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(5) + UX_DCD_DWC3_DEPCMDPAR1, UX_DCD_DWC3_DEPCMD_EP2_IN_PAR1);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(5) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP2_IN_PAR0);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(5) + UX_DCD_DWC3_DEPCMDPAR2, UX_DCD_DWC3_DEPCMD_EP2_PAR2);
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(5) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_COMMON_SCFG);
        do{
            /* Read the device endpoint Command register.  */
            dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(5) +
								UX_DCD_DWC3_DEPCMD);
        }while ((dwc3_register & 0x400) != 0);
        _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);



    /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
   /* Issuing Transfer Resource command for each initialized endpoint */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP_XFERCFG_PAR0);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_EP_XFERCFG);
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(0) +
								UX_DCD_DWC3_DEPCMD);
    }while ((dwc3_register & 0x400) != 0);
    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);

    /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMDPAR0, UX_DCD_DWC3_DEPCMD_EP_XFERCFG_PAR0);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) + UX_DCD_DWC3_DEPCMD, UX_DCD_DWC3_DEPCMD_EP_XFERCFG);
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(1) +
								UX_DCD_DWC3_DEPCMD);
    }while ((dwc3_register & 0x400) != 0);
    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);

    dcd_dwc3 -> ConnectionDone = 0;

    /* BULK endpoint transfers event hadler registration */
    _ux_dcd_SetEpHandler(dcd_dwc3, 2, 0,
			BulkOutHandler);
    _ux_dcd_SetEpHandler(dcd_dwc3, 2, 1,
                        BulkInHandler);
    semaphore_create_status =  _ux_utility_semaphore_create
                                (&dcd_dwc3->ux_dcd_dwc3_ep_slave_transfer_request_semaphore,
                                "bulk endpoint semaphore", 1);

    if (semaphore_create_status != (UINT)UX_SUCCESS)
    {
        printf("Semaphore creation fail\n");
    }


    /* Starting controller . */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, UX_DCD_DWC3_DCTL_START); //Eearlier this was but changed to below
    do{
        /* Read the device endpoint Command register.  */
        dwc3_register =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);
    }while ((dwc3_register & 0x4000000) != 0);

    /* enable USB Reset, Connection Done, and USB/Link State Change events */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEVTEN, UX_DCD_DWC3_DEV_INT_EN);

    NVIC_ClearPendingIRQ(USB0_IRQ);
    NVIC_EnableIRQ(USB0_IRQ);

    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;
    /* Return successful completion.  */
    return(UX_SUCCESS);
}

/**
  \fn          void  USBx_IRQHandler (void)
  \brief       Run the Interrupt Handler for USBx
*/
void USB0_IRQHandler (void)
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
    ULONG     RegVal;

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
    if (Ret == 0) {
#ifdef	DEBUG
        printf("Failed to set scratchpad low addr: %d\n", Ret);
#endif
        return Ret;
    }

    Ret = _ux_dcd_SendGadgetGenericCmd(dcd_dwc3,
                UX_DCD_DGCMD_SET_SCRATCHPAD_ADDR_HI,
                        (ULONG)((ScratchBuff >> 16U) >> 16U));
    if (Ret == 0) {
#ifdef	DEBUG
        printf("Failed to set scratchpad high addr: %d\n", Ret);
#endif
        return Ret;
    }

    return 1;

}

UINT _ux_dcd_SendGadgetGenericCmd(UX_DCD_DWC3 *dcd_dwc3, ULONG cmd,
                        ULONG param)
{
    ULONG             RegVal, retry = 500U;

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DGCMDPAR, param);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DGCMD,
                                cmd | UX_DCD_DGCMD_CMDACT);

    while (retry > 0U) {
        RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DGCMD);
            if ((RegVal & UX_DCD_DGCMD_CMDACT) == 0U) {
                if (UX_DCD_DGCMD_STATUS(RegVal) != 0U) {
                    return -1;
                    }
                return 1;
            }
            retry = retry - 1U;
        }

        return 0;
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
