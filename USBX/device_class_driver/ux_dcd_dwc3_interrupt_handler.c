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
 * @file     ux_dcd_dwc3_interrupt_handler.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function is the interrupt handler for dwc3 controller.
 *		 The controller will trigger an interrupt when something happens on
 *           an endpoint whose mask has been set in the interrupt enable
 *		 register, or when a bus reset is detected.   .
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_device_stack.h"
#include "ux_system.h"
#include "ux_device_class_cdc_acm.h"
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header
extern TX_EVENT_FLAGS_GROUP  BULKIN_BULKOUT_FLAG;

VOID  _ux_dcd_dwc3_interrupt_handler(VOID)
{
    ULONG              dwc3_pending_interrupt;
    ULONG              dwc3_masked_interrupt;
    UX_SLAVE_DCD       *dcd;
    UX_DCD_DWC3	       *dcd_dwc3;
    UX_SLAVE_DEVICE    *device;
    UX_DCD_EVENT       *event_buffer;

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the DWC3 DCD.  */
    dcd_dwc3 = (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get event pointer ...*/
    event_buffer = (UX_DCD_EVENT *)dcd_dwc3 -> ux_dcd_dwc3_event;

    if ((dcd_dwc3 -> event_flag & UX_DCD_DWC3_EVENT_PENDING))
                return;

    /* Read the interrupt status register from the controller.  */
    dwc3_pending_interrupt =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GEVNTCOUNT(0));
    dwc3_pending_interrupt &= UX_DCD_GEVNTCOUNT_MASK;
    if(!dwc3_pending_interrupt)
    {
       return;
    }

    event_buffer -> count = dwc3_pending_interrupt;

    dcd_dwc3 -> event_flag |= UX_DCD_DWC3_EVENT_PENDING;

    /* Mask only with the interrupts we have programmed.  */
    dwc3_masked_interrupt = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GEVNTSIZ(0));

    dwc3_masked_interrupt |= UX_DCD_GEVNTSIZ_INTMASK;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTSIZ(0), dwc3_masked_interrupt);

     /* Processes events in an Event Buffer */
     _ux_dcd_EventBufferHandler(dcd_dwc3);

}

VOID _ux_dcd_EventBufferHandler(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG               transfer_length;
    ULONG               dwc3_register;
    ULONG               endp_number;
    UX_SLAVE_TRANSFER   *transfer_request;
    UX_DCD_DWC3_ED      *ed;
    UX_SLAVE_ENDPOINT   *endpoint;
    UX_SLAVE_DCD        *dcd;
    UX_SLAVE_DEVICE     *device;
    UX_DCD_EVENT        *event_buffer;
    ULONG               event_info;
    ULONG            status;

     /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the DWC3 DCD.  */
    dcd_dwc3 = (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    dcd_dwc3->test_mode = 0;

    /* Get event pointer ...*/
    event_buffer = (UX_DCD_EVENT *)dcd_dwc3 -> ux_dcd_dwc3_event;
    RTSS_InvalidateDCache_by_Addr(event_buffer -> buf, UX_DCD_DWC3_EVENT_BUFFER_SIZE);

    while(event_buffer->count > 0)
    {

        /* The time consuming algorithms-routine must be moved in application later */
        dwc3_register = ((ULONG)(*((ULONG *) (event_buffer -> buf + event_buffer -> lpos))));

        /* Check type of interrupt */

        if ((dwc3_register & 0x1) == 0x1)
        {
            /* To get the event_info */
            event_info = (dwc3_register >> 16) & 0xf;

            /* Device specific events. For now 5 events registered*/
            dwc3_register = _ux_dcd_dwc3_check_devt_event_type(dwc3_register);

            switch(dwc3_register)
            {
                ULONG    Index;
                case UX_DCD_EVENT_WAKEUP:
                     break;
                case UX_DCD_EVENT_DISCONNECT:
                     break;
                case UX_DCD_EVENT_EOPF:
                     break;
                case UX_DCD_EVENT_RESET:
                    if (device -> ux_slave_device_state ==  UX_DEVICE_CONFIGURED)
                    {
                       /* Device is reset, the behavior is the same as disconnection.  */
#ifdef DEBUG
                        printf("device disconnect\n");
#endif
                        if(endp_number == 4 || endp_number == 5)
                        {
                           _ux_dcd_dwc3_clear_trb(dcd_dwc3,endp_number);
                        }
                        _ux_utility_event_flags_set(&BULKIN_BULKOUT_FLAG, UX_DISCONNECT_EVENT, TX_OR);
                        status  = _ux_device_stack_disconnect();
                        if(status != UX_SUCCESS)
                        {
#ifdef DEBUG
                           printf("Device disconnection error\n");
#endif
                        }
                    }
                   /* Handling any pending setup transfers */
                     dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);
                     dwc3_register &= ~UX_DCD_DCTL_TSTCTRL_MASK;
                     _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, dwc3_register);
                     dcd_dwc3 -> test_mode = 0;
                      /* Clear STALL on all endpoints */
                      ux_dcd_ClearStallAllEp(dcd_dwc3);
                      for (Index = 0U; Index < (dcd_dwc3->NumInEps + dcd_dwc3->NumOutEps);
                               Index++)
                      {
                          dcd_dwc3->eps[Index].ux_dcd_dwc3_ed_status = 0U;
                          dcd_dwc3 -> ux_dcd_dwc3_ed[Index].ux_dcd_dwc3_ed_status = 0U;
                      }
                      dcd_dwc3->IsConfigDone = 0U;

                      /* Reset device address to zero */
                      _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCFG, UX_DCD_DWC3_DCFG_ADDR);

                      /* Mark the device as RESET now.  */
                      device -> ux_slave_device_state =  UX_DEVICE_RESET;
                      break;
                case UX_DCD_EVENT_CONNECT_DONE:
                     dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DCTL);
                     dwc3_register |= UX_DCD_DCTL_HIRD_THRES_MASK;
                     _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, dwc3_register);
                     dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GRXTHRCFG);
                     dwc3_register &= ~UX_DCD_DWC3_GRXTHRCFG_PKTCNTSEL;
                     _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GRXTHRCFG, dwc3_register);
                       /* Calculate numbr of RXFIFO packets */
                      _ux_dcd_dwc3_gadget_setup_nump(dcd_dwc3);
                      if (device -> ux_slave_device_state !=  UX_DEVICE_ATTACHED && device -> ux_slave_device_state !=  UX_DEVICE_CONFIGURED)
                      {
                         /* We are connected at high speed.  */
                         _ux_system_slave -> ux_system_slave_speed =  UX_HIGH_SPEED_DEVICE;
                         /* Complete the device initialization.  */
                         if(!dcd_dwc3 -> ConnectionDone)
                             _ux_dcd_dwc3_initialize_complete();

                         _ux_dcd_dwc3_enable_control_endpoint(dcd_dwc3, 64);
                         (VOID)_ux_dcd_RecvSetup(dcd_dwc3);

                         /* Mark the device as attached now.  */
                         device -> ux_slave_device_state =  UX_DEVICE_ATTACHED;
                      }
                      dcd_dwc3 -> ConnectionDone++;
                      break;
                case UX_DCD_EVENT_LINK_STATUS_CHANGE:
                     dcd_dwc3 -> link_state = event_info;
                     break;
                case UX_DCD_EVENT_HIBER_REQ:
                     break;
                default:
                     break;
             }
         }
         else
         {
            /* Endpoint specific events */
            UX_DCD_DWC3_ED *Ept;
            ULONG          EpStatus;
            UX_DCD_EP_TRB   *TrbPtr;
            ULONG Status;
            EpStatus =  dwc3_register >> UX_DCD_EVT_EPSTATUS_MASK;
            endp_number = _ux_dcd_dwc3_get_endpoint_num(dwc3_register);
            Ept = &dcd_dwc3->eps[endp_number];
            if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_ENABLED) == 0U)
            {
                break;
            }
              /* Calculate the endpoint index.  */
            dwc3_register = _ux_dcd_dwc3_check_depevt_event_type(dwc3_register);

            /* Get the physical endpoint associated with this endpoint.  */
             ed =  &dcd_dwc3 -> ux_dcd_dwc3_ed[endp_number];
              /* Get the logical endpoint from the physical endpoint.  */
             endpoint =  ed -> ux_dcd_dwc3_ed_endpoint;

              /* Reset the endpoint transfer status. */
             ed -> ux_dcd_dwc3_ed_transfer_status =  UX_DCD_DWC3_ED_TRANSFER_STATUS_IDLE;

            if (endp_number == 0 || endp_number == 1)
            {
               ULONG Ctrl;
               TrbPtr = &dcd_dwc3->endp0_trb;
               RTSS_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
               RTSS_InvalidateDCache_by_Addr(&dcd_dwc3->SetupData, UX_SETUP_SIZE);
               Ctrl = (ULONG)&dcd_dwc3->SetupData;

               /* Get the physical endpoint associated with this endpoint.  */
                 ed =  &dcd_dwc3 -> ux_dcd_dwc3_ed[endp_number];

              /* Get the pointer to the transfer request.  */
                 transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

               /* Flag the setup or completion in respective interrupts. */
               if (dwc3_register == UX_DCD_DWC3_DEPEVT_XFERCOMPLETE && dcd_dwc3 -> ep0state == EP0_SETUP_PHASE)
               {
                  ed -> ux_dcd_dwc3_ed_transfer_status =  UX_DCD_DWC3_ED_TRANSFER_STATUS_SETUP;
                   /* Read the ep0_trb and store the data into the setup buffer.  */
                 _ux_dcd_dwc3_fifo_read(dcd_dwc3, transfer_request -> ux_slave_transfer_request_setup, UX_SETUP_SIZE);
                 transfer_length = ((ULONG)(*((ULONG *) (Ctrl + 0x6))));
               }
               else if (dwc3_register == UX_DCD_DWC3_DEPEVT_XFERCOMPLETE && dcd_dwc3 -> ep0state == EP0_DATA_PHASE)
               {
                  ed -> ux_dcd_dwc3_ed_transfer_status =  UX_DCD_DWC3_ED_TRANSFER_STATUS_OUT_COMPLETION;
               }

               /* Save the length in the ED payload.  */
               ed -> ux_dcd_dwc3_ed_payload_length = transfer_length;

                /* Get the logical endpoint from the physical endpoint.  */
                endpoint =  ed -> ux_dcd_dwc3_ed_endpoint;

               /* Get the pointer to the transfer request.  */
                  transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

               /* Process the ep0 interrupts bases on event_type. */
               if(dwc3_register == UX_DCD_DWC3_DEPEVT_XFERNOTREADY)
               {
                  Ept = &dcd_dwc3->eps[endp_number];
                  ULONG     Status;
                  Status = UX_DCD_DWC3_TRB_SIZE_TRBSTS(TrbPtr->Size);

                  if (EpStatus == DEPEVT_STATUS_CONTROL_DATA)
                  {
                       /* We already have a DATA transfer in the controller's cache,
                     * if we receive a XferNotReady(DATA) we will ignore it, unless
                             * it's for the wrong direction.
                        *
                      * In that case, we must issue END_TRANSFER command to the Data
                                    * Phase we already have started and issue SetStall on the
                                       * control endpoint.
                           */
                       if (endp_number != dcd_dwc3 -> ep0_expect_in)
                       {
                          _ux_dcd_Ep0_EndControlData(dcd_dwc3, Ept);
                          _ux_dcd_Ep0StallRestart(dcd_dwc3);
                       }
                  }
                  if (EpStatus == DEPEVT_STATUS_CONTROL_STATUS)
                  {
                    _ux_dcd_Ep0StartStatus(dcd_dwc3, endp_number);
                  }
               }
              if(dwc3_register == UX_DCD_DWC3_DEPEVT_XFERCOMPLETE)
              {
                 UX_CTRL_REQUEST *Ctrl;
                 Ctrl = &dcd_dwc3->SetupData;
                 Ept = &dcd_dwc3->eps[endp_number];
                 TrbPtr = &dcd_dwc3->endp0_trb;
                 Ept->ux_dcd_dwc3_ed_status &= ~(UX_DCD_EP_BUSY);
                 Ept->ux_dcd_dwc3_ed_resource_index = 0U;
                 switch (dcd_dwc3->ep0state)
                 {
                     case EP0_SETUP_PHASE:
                         if (transfer_length == 0U)
                         {
                            dcd_dwc3 -> three_stage_setup = 0;
                            dcd_dwc3 -> ep0_expect_in = UX_DCD_EP_DIR_OUT;
                            dcd_dwc3 -> ep0state = (enum UX_DCD_EP0_STATE)EP0_NRDY_STATUS;
                          }
                          else
                          {
                             dcd_dwc3 -> three_stage_setup = 1U;
                             dcd_dwc3 -> ep0_expect_in = !!(Ctrl->bRequestType & UX_REQUEST_IN);
                          }
                           /* Process the ep0 interrupts via callback.  */
                           _ux_dcd_dwc3_transfer_callback(dcd_dwc3, transfer_request);
                           break;

                     case EP0_DATA_PHASE:
                         _ux_dcd_Ep0DataDone(dcd_dwc3, endp_number, transfer_request);
                          break;

                     case EP0_STATUS_PHASE:
                          _ux_dcd_Ep0StatusDone(dcd_dwc3);
                          break;
                     default:
                          break;
                 }
              }

            }
            else
            {
               /* bulk in anad bulk out events */
               ed =  &dcd_dwc3 -> ux_dcd_dwc3_ed[endp_number];

               /* Get the logical endpoint from the physical endpoint.  */
               endpoint =  ed -> ux_dcd_dwc3_ed_endpoint;

               /* Get the pointer to the transfer request.  */
              transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

              switch (dwc3_register)
              {
                 case UX_DCD_DWC3_DEPEVT_XFERINPROGRESS:
                     _ux_dcd_EpXferComplete(dcd_dwc3, dwc3_register, endp_number,
                      transfer_request -> ux_slave_transfer_request_current_data_pointer);
                      transfer_request -> ux_slave_transfer_request_actual_length += dcd_dwc3->NumBytes;
                      if(endp_number == 4)
                      {
                         dcd_dwc3 -> test_mode = 1;
                      }
                      break;
                 case UX_DCD_DWC3_DEPEVT_XFERCOMPLETE:
                      break;
                 case UX_DCD_DWC3_DEPEVT_XFERNOTREADY:
                     _ux_dcd_EpXferNotReady(dcd_dwc3, dwc3_register, endp_number);
                      break;
                 default:
                      break;
              }

            }
         }

         event_buffer -> lpos = (event_buffer -> lpos + 4) % UX_DCD_DWC3_EVENT_BUFFER_SIZE;
         event_buffer -> count -= 4;
        _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTCOUNT(0), 4);
    }

    event_buffer -> count = 0;
    dcd_dwc3 -> event_flag &= ~UX_DCD_DWC3_EVENT_PENDING;
    /* Unmask interrupt */
    dwc3_register = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_GEVNTSIZ(0));
    dwc3_register &= ~UX_DCD_GEVNTSIZ_INTMASK;
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_GEVNTSIZ(0), dwc3_register);
}

VOID    _ux_dcd_dwc3_enable_control_endpoint(UX_DCD_DWC3 *dcd_dwc3, ULONG value)
{
       ULONG RetVal;

        RetVal = _ux_dcd_EpEnable(dcd_dwc3, 0U, 0, value,
                                0, 0);

        RetVal = _ux_dcd_EpEnable(dcd_dwc3, 0U, 1, value,
                                0, 0);
}

UINT    _ux_dcd_EpEnable(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                        ULONG Maxsize, ULONG Type, ULONG Restore)
{
    UX_DCD_DWC3_ED *Ept;
    UX_DCD_EP_TRB *TrbStHw, *TrbLink;
    VOID *RetPtr;
    ULONG RegVal;
    ULONG PhyEpNum;


    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);

    Ept = &dcd_dwc3 -> eps[PhyEpNum];

    Ept->ux_dcd_dwc3_ed_index   = UsbEpNum;
    Ept->ux_dcd_dwc3_ed_direction  = Dir;
    Ept->ux_dcd_dwc3_ed_type       = Type;
    Ept->ux_dcd_dwc3_ed_maxpacket    = Maxsize;
    Ept->Phy_EPNum   = PhyEpNum;
    Ept->CurUf      = 0U;

    Ept->trb_enqueue = 0;
    Ept->trb_dequeue = 0;

    if (((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_ENABLED) == 0U))
    {
       if (_ux_dcd_StartEpConfig(dcd_dwc3, UsbEpNum, Dir) == 0)
	    {
#ifdef DEBUG
           printf("Start EPConfig failed\n");
#endif
		return 0;
            }
    }
    if (_ux_dcd_SetEpConfig(dcd_dwc3, UsbEpNum, Dir, Maxsize,
                                      Type, Restore) == 0)
    {
#ifdef DEBUG
	  printf("SetEP failed\n");
#endif
	      return 0;
    }

    if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_ENABLED) == 0U)
    {
        if (_ux_dcd_SetXferResource(dcd_dwc3, UsbEpNum,
			                    Dir)    == 0)
	    {
#ifdef DEBUG
		printf("UsbEpNum [%x] failed in XferResource\n",UsbEpNum);
#endif
		return 0;
	    }
    }
	Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_ENABLED;

    RegVal = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DALEPENA);
    RegVal |=  UX_DCD_DALEPENA_EP(Ept->Phy_EPNum);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DALEPENA, RegVal);

        /* Following code is only applicable for ISO XFER */
    TrbStHw = &Ept->EpTrb[0U];

        /* Link TRB. The HWO bit is never reset */
    TrbLink = &Ept->EpTrb[NO_OF_TRB_PER_EP];

    RetPtr = memset(TrbLink, 0x0, sizeof(struct UX_DCD_EP_TRB_STRUCT));
    if (RetPtr == NULL) {
          return 0;
     }

    TrbLink->BufferPtrLow = (ULONG)(LocalToGlobal(TrbStHw));
    TrbLink->BufferPtrHigh =  ((ULONG)TrbStHw >> 16U) >> 16U;
    TrbLink->Ctrl |= UX_DCD_DWC3_TRBCTL_LINK_TRB;
    TrbLink->Ctrl |= UX_DCD_DWC3_TRB_CTRL_HWO;

	return 1;

}

UINT    _ux_dcd_SetEpConfig(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                                                ULONG Size, ULONG Type, ULONG Restore)
{
    UX_DCD_DWC3_ED *Ept;
    UX_DCD_EP_PARAMS Params;
    ULONG PhyEpNum;

    Params.Param0 = 0x00U;
    Params.Param1 = 0x00U;
    Params.Param2 = 0x00U;

    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);
    Ept = &dcd_dwc3->eps[PhyEpNum];

    Params.Param0 = UX_DCD_DEPCFG_EP_TYPE(Type)
               | UX_DCD_DEPCFG_MAX_PACKET_SIZE(Size);


    Params.Param1 = UX_DCD_DEPCFG_XFER_COMPLETE_EN
                | UX_DCD_DEPCFG_XFER_NOT_READY_EN;

    if (Restore == 1) {
    }
     /*
      * We are doing 1:1 mapping for endpoints, meaning
      * Physical Endpoints 2 maps to Logical Endpoint 2 and
      * so on. We consider the direction bit as part of the physical
      * endpoint number. So USB endpoint 0x81 is 0x03.
      */
    Params.Param1 |= UX_DCD_DEPCFG_EP_NUMBER(PhyEpNum);

    if (Dir != UX_DCD_EP_DIR_OUT) {
                Params.Param0 |= UX_DCD_DEPCFG_FIFO_NUMBER(PhyEpNum >> 1U);
    }

    Params.Param1 |= UX_DCD_DEPCFG_XFER_IN_PROGRESS_EN;

    return _ux_dcd_SendEpCmd(dcd_dwc3, UsbEpNum, Dir,
                                       UX_DCD_DEPCMD_SETEPCONFIG,
                                       Params);
}



UINT _ux_dcd_SendEpCmd(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir,
                                  ULONG Cmd, UX_DCD_EP_PARAMS Params)
{
    ULONG     PhyEpNum;
    ULONG	  Retval;
    LONG      cmd_status = 0;
    LONG      timeout = 1000;


    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);

    /* Check controller config before issuing DEPCMD always */
    _ux_dcd_dwc3_controller_config_check(dcd_dwc3);

    if (UX_DCD_DEPCMD_CMD(Cmd) == UX_DCD_DEPCMD_STARTTRANSFER)
    {
        ULONG    needs_wakeup;

        needs_wakeup = (dcd_dwc3->link_state == UX_DCD_LINK_STATE_U1 ||
                            dcd_dwc3->link_state == UX_DCD_LINK_STATE_U2 ||
                            dcd_dwc3->link_state == UX_DCD_LINK_STATE_U3 ||
                            dcd_dwc3->link_state == UX_DCD_LINK_STATE_RX_DET);

       if (needs_wakeup)
       {
            Retval = _ux_dcd_transfer_wakeup(dcd_dwc3);
            if(Retval < 0)
            {
#ifdef DEBUG
	             printf("Wake up failed: Can't give commands\n");
#endif
            }
        }
    }

    if (UX_DCD_DEPCMD_CMD(Cmd) == UX_DCD_DEPCMD_UPDATETRANSFER && !dcd_dwc3->IsocTrans)
    {
        Cmd &= ~(UX_DCD_DEPCMD_CMDIOC | UX_DCD_DEPCMD_CMDACT);
    }
    else
    {
        Cmd |= UX_DCD_DEPCMD_CMDACT;
    }
    /* Issuing DEPCFG Command to ep_resource[1] (for EP 3 IN/ISOC/) */
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) + UX_DCD_DWC3_DEPCMDPAR1,(ULONG)LocalToGlobal( (void *)(Params.Param1)));
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) + UX_DCD_DWC3_DEPCMDPAR0, Params.Param0);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) + UX_DCD_DWC3_DEPCMDPAR2, Params.Param2);
    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) + UX_DCD_DWC3_DEPCMD, Cmd);

    do
    {
        /* Read the device endpoint Command register.  */
        Retval =  _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) + UX_DCD_DWC3_DEPCMD);
        if (!(Retval & UX_DCD_DEPCMD_CMDACT))
        {
           cmd_status = UX_DCD_DEPCMD_STATUS(Retval);
           switch(cmd_status)
           {
               case 0:
                   Retval = 1;
                   break;
               case DEPEVT_TRANSFER_NO_RESOURCE:
                   Retval = -1;
                   break;
               case DEPEVT_TRANSFER_BUS_EXPIRY:
                   Retval =  -1;
                   break;
               default:
                  printf("UNKNOW cmd status\n");
           }
           break;
        }
    }while(--timeout);

    if (timeout == 0)
    {
        Retval = -1;
        cmd_status = -1;
    }

    _ux_dcd_dwc3_controller_config_reset(dcd_dwc3);
    return Retval;
}

UINT _ux_dcd_SetXferResource(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir)
{
    UX_DCD_EP_PARAMS Params;

    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;

    Params.Param0 = UX_DCD_DEPXFERCFG_NUM_XFER_RES(1U);

    return _ux_dcd_SendEpCmd(dcd_dwc3, UsbEpNum, Dir,
                                     UX_DCD_DEPCMD_SETTRANSFRESOURCE,
                                     Params);
}

UINT _ux_dcd_RecvSetup(UX_DCD_DWC3 *dcd_dwc3)
{
    UX_DCD_EP_PARAMS Params;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Ret;
    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;

    /* Setup packet always on EP0 */
    Ept = &dcd_dwc3->eps[0U];
    if ((Ept->endp_flag & UX_DCD_EP_BUSY) != 0U) {
        return 0;
    }

    TrbPtr = &dcd_dwc3->endp0_trb;

    RTSS_CleanDCache_by_Addr(&dcd_dwc3->SetupData, UX_SETUP_SIZE);
    TrbPtr->BufferPtrLow =  lower_32_bits(LocalToGlobal(&dcd_dwc3->SetupData));
    TrbPtr->BufferPtrHigh = upper_32_bits(LocalToGlobal(&dcd_dwc3->SetupData));
    TrbPtr->Size = 8U;
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_SETUP;

    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                        | UX_DCD_DWC3_TRB_CTRL_LST
                        | UX_DCD_DWC3_TRB_CTRL_IOC
                        | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);
    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;
    dcd_dwc3->ep0state = EP0_SETUP_PHASE;

    Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_OUT,
                                UX_DCD_DEPCMD_STARTTRANSFER, Params);
    if (Ret != 1) {
                return 0;
    }

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
    Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                                Ept->ux_dcd_dwc3_ed_index,
                                                                Ept->ux_dcd_dwc3_ed_direction);
    return 1;

}

UINT _ux_dcd_EpGetTransferIndex(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum,
                                                                ULONG Dir)
{
    ULONG PhyEpNum;
    ULONG ResourceIndex;

    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);
    ResourceIndex = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DEP_BASE(PhyEpNum) +
                       UX_DCD_DWC3_DEPCMD);

    return UX_DCD_DEPCMD_GET_RSC_IDX(ResourceIndex);
}

UINT _ux_dcd_transfer_wakeup(UX_DCD_DWC3 * dcd_dwc3)
{
    LONG    retries = 20000;
    ULONG   ret;
    ULONG   reg;
    ULONG   link_state;
    ULONG   speed;

    /*
     * According to the Databook Remote wakeup request should
     * be issued only when the device is in early suspend state.
     *
     * We can check that via USB Link State bits in DSTS register.
    */
    reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DSTS);

    link_state = UX_DCD_DSTS_USBLNKST(reg);

    switch (link_state) {
        case UX_DCD_LINK_STATE_RX_DET:    /* in HS, means Early Suspend */
        case UX_DCD_LINK_STATE_U3:        /* in HS, means SUSPEND */
        case UX_DCD_LINK_STATE_U2:        /* in HS, means SUSPEND */
                break;
        default:
                return -1;
    }

    ret = ux_dcd_gadget_set_link_state(dcd_dwc3, UX_DCD_LINK_STATE_RECOV);
    if (ret < 0)
    {
#ifdef DEBUG
       printf("failed to put link in Recovery\n");
#endif
       return -1;
    }

    /* poll until Link State changes to ON */
    while (retries--)
    {
        reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DSTS);

        /* in HS, means ON */

        if (UX_DCD_DSTS_USBLNKST(reg) == UX_DCD_LINK_STATE_U0)
             break;
    }
    if (UX_DCD_DSTS_USBLNKST(reg) != UX_DCD_LINK_STATE_U0)
    {
#ifdef DEBUG
       printf("failed to send remote wakeup: Controller state not changed\n");
#endif
        return -1;
    }

    return 0;

}

UINT ux_dcd_gadget_set_link_state(UX_DCD_DWC3 *dcd_dwc3, enum ux_dcd_link_state state)
{
    ULONG reg;
    LONG retries = 10000;

    reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DSTS);
    reg &= ~UX_DCD_DCTL_ULSTCHNGREQ_MASK;

    /* set requested state */
    reg |= UX_DCD_DCTL_ULSTCHNGREQ(state);

    _ux_dcd_dwc3_register_write(dcd_dwc3, UX_DCD_DWC3_DCTL, reg);
    while (--retries)
    {
        reg = _ux_dcd_dwc3_register_read(dcd_dwc3, UX_DCD_DWC3_DSTS);

        if (UX_DCD_DSTS_USBLNKST(reg) == state)
        {
            return 0;
        }
    }

    return -1;
}


VOID ux_dcd_StopActiveTransfers(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG Epnum;

    for (Epnum = 2U; Epnum < UX_DCD_ENDPOINTS_NUM; Epnum++) {
        UX_DCD_DWC3_ED *Ept;

        Ept = &dcd_dwc3->eps[Epnum];

        if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_ENABLED) == 0U) {
            continue;
        }



        ux_dcd_StopTransfer(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index,
                            Ept->ux_dcd_dwc3_ed_direction, 1);
    }

}

VOID ux_dcd_ClearStallAllEp(UX_DCD_DWC3 *dcd_dwc3)
{
    ULONG Epnum;

    for (Epnum = 1U; Epnum < UX_DCD_ENDPOINTS_NUM; Epnum++) {
        UX_DCD_DWC3_ED *Ept;

        Ept = &dcd_dwc3->eps[Epnum];

        if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_ENABLED) == 0U) {
            continue;
        }

        if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_STALL) == 0U) {
            continue;
        }
#ifdef DEBUG
	printf("Clearing endpoint STALL in RESET\n");
#endif
	ux_dcd_EpClearStall(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index,
                                                Ept->ux_dcd_dwc3_ed_direction);
    }

}

VOID ux_dcd_StopTransfer(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum,
                        ULONG Dir, ULONG Force)
{
    UX_DCD_DWC3_ED *Ept;
    UX_DCD_EP_PARAMS Params;
    ULONG PhyEpNum;
    ULONG Cmd;

    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    Ept = &dcd_dwc3->eps[PhyEpNum];

    if (Ept->ux_dcd_dwc3_ed_resource_index == 0U) {
            return;
    }

    /*
     * - Issue EndTransfer WITH CMDIOC bit set
     * - Wait 100us
     */
    Cmd = UX_DCD_DEPCMD_ENDTRANSFER;
    Cmd |= (Force == 1) ? UX_DCD_DEPCMD_HIPRI_FORCERM : 0U;
    Cmd |= UX_DCD_DEPCMD_CMDIOC;
    Cmd |= UX_DCD_DEPCMD_PARAM(Ept->ux_dcd_dwc3_ed_resource_index);
    _ux_dcd_SendEpCmd(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index, Ept->ux_dcd_dwc3_ed_direction,
                                                        Cmd, Params);
    if (Force == 1) {
                Ept->ux_dcd_dwc3_ed_resource_index = 0U;
    }

    Ept->ux_dcd_dwc3_ed_status &= ~UX_DCD_EP_BUSY;
#ifdef DEBUG
    printf("Transferred stopped from somewhere\n");
#endif
    _ux_dcd_dwc3_delay(50);
}

VOID ux_dcd_EpClearStall(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum, ULONG Dir)
{
    ULONG      PhyEpNum;
    UX_DCD_DWC3_ED *Ept = NULL;
    UX_DCD_EP_PARAMS Params;

    PhyEpNum = UX_DCD_PhysicalEp(Epnum, Dir);
    Ept = &dcd_dwc3->eps[PhyEpNum];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    _ux_dcd_SendEpCmd(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index, Ept->ux_dcd_dwc3_ed_direction,
                                    UX_DCD_DEPCMD_CLEARSTALL, Params);

    Ept->ux_dcd_dwc3_ed_status &= ~UX_DCD_EP_STALL;
}

UINT   _ux_dcd_StartEpConfig(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEpNum, ULONG Dir)
{
    UX_DCD_EP_PARAMS Params;
    ULONG     Cmd;
    ULONG PhyEpNum;
    PhyEpNum = UX_DCD_PhysicalEp(UsbEpNum, Dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if (PhyEpNum != 1U) {
        Cmd = UX_DCD_DEPCMD_DEPSTARTCFG;
        /* XferRscIdx == 0 for EP0 and 2 for the remaining */
        if (PhyEpNum > 1U) {
            if (dcd_dwc3->IsConfigDone != 0U) {
                return 1;
            }
            dcd_dwc3->IsConfigDone = 1U;
            Cmd |= UX_DCD_DEPCMD_PARAM(2U);
        }

        return _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_OUT,
                                 Cmd, Params);
    }

    return 1;

}

VOID _ux_dcd_Ep0_EndControlData(UX_DCD_DWC3 *dcd_dwc3, UX_DCD_DWC3_ED  *Ept)
{
    UX_DCD_EP_PARAMS Params;
    ULONG     Cmd;

    if (Ept->ux_dcd_dwc3_ed_resource_index == 0U) {
                return;
    }

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    Cmd = UX_DCD_DEPCMD_ENDTRANSFER;
    Cmd |= UX_DCD_DEPCMD_PARAM(Ept->ux_dcd_dwc3_ed_resource_index);
    _ux_dcd_SendEpCmd(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index, Ept->ux_dcd_dwc3_ed_direction,
                                                Cmd, Params);
    Ept->ux_dcd_dwc3_ed_resource_index = 0U;
    _ux_dcd_dwc3_delay(50);
}

VOID _ux_dcd_Ep0StallRestart(UX_DCD_DWC3 *dcd_dwc3)
{
    UX_DCD_DWC3_ED    *Ept;

    /* reinitialize physical ep1 */
    Ept = &dcd_dwc3->eps[1U];
    Ept->ux_dcd_dwc3_ed_status = UX_DCD_EP_ENABLED;

    /* stall is always issued on EP0 */
    _ux_dcd_EpSetStall(dcd_dwc3, 0U, UX_DCD_EP_DIR_OUT);

    Ept = &dcd_dwc3->eps[0U];
    Ept->ux_dcd_dwc3_ed_status = UX_DCD_EP_ENABLED;
    dcd_dwc3->ep0state = EP0_SETUP_PHASE;
    _ux_dcd_RecvSetup(dcd_dwc3);

}

VOID _ux_dcd_EpSetStall(UX_DCD_DWC3 *dcd_dwc3, ULONG Epnum, ULONG Dir)
{
    ULONG  PhyEpNum;
    UX_DCD_DWC3_ED *Ept = NULL;
    UX_DCD_EP_PARAMS Params;

    PhyEpNum = UX_DCD_PhysicalEp(Epnum, Dir);
    Ept = &dcd_dwc3->eps[PhyEpNum];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    _ux_dcd_SendEpCmd(dcd_dwc3, Ept->ux_dcd_dwc3_ed_index, Ept->ux_dcd_dwc3_ed_direction,
                                    UX_DCD_DEPCMD_SETSTALL, Params);

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_STALL;
}

UINT   _ux_dcd_Ep0DataDone(UX_DCD_DWC3 *dcd_dwc3, ULONG endpoint_number, UX_SLAVE_TRANSFER *transfer_request)
{
    UX_DCD_DWC3_ED       *Ept;
    UX_DCD_EP_TRB      *TrbPtr;
    ULONG     Status;
    ULONG     Length;
    ULONG     Epnum;
    ULONG      Dir;
    UX_DCD_EP_PARAMS Params;

    Epnum = endpoint_number;
    Dir = !!Epnum;
    Ept = &dcd_dwc3->eps[Epnum];

    TrbPtr = &dcd_dwc3->endp0_trb;


    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    Status = UX_DCD_DWC3_TRB_SIZE_TRBSTS(TrbPtr->Size);

    if (Status == UX_DCD_DWC3_TRBSTS_SETUP_PENDING) {
#ifdef DEBUG
	printf("TRB transmission pending in DATA_PHASE with EP [%x]\n",endpoint_number);
#endif
        return -1;
    }

    Length = TrbPtr->Size & UX_DCD_DWC3_TRB_SIZE_MASK;

    if (Length == 0U) {
        Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes;
        transfer_request -> ux_slave_transfer_request_force_zlp =  UX_FALSE;
    }
    else {
#ifdef DEBUG
        printf("Dir is _IN and setting BytesTxed\n");
#endif
        if (Dir == UX_DCD_EP_DIR_IN) {
            Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes - Length;
        }
        else {
            if (Ept->UnalignedTx == 1U) {
                Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes;
                Ept->UnalignedTx = 0U;
            }
        }
   }
  return 0;
}

UINT   _ux_dcd_Ep0StartStatus(UX_DCD_DWC3 *dcd_dwc3, ULONG endpoint_number)
{
    UX_DCD_DWC3_ED       *Ept;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_EP_PARAMS Params;
    ULONG Type;
    ULONG Ret;
    ULONG Dir;

    Ept = &dcd_dwc3->eps[endpoint_number];
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) != 0U) {
#ifdef DEBUG
	printf("Ep0StartStatus not done\n");
#endif
                return 0;
    }

    Type = (dcd_dwc3->three_stage_setup != 0U) ?
                                     UX_DCD_DWC3_TRBCTL_CONTROL_STATUS3
                                   : UX_DCD_DWC3_TRBCTL_CONTROL_STATUS2;
    TrbPtr = &dcd_dwc3->endp0_trb;

    /* we use same TrbPtr for setup packet */
    TrbPtr->BufferPtrLow = (ULONG)LocalToGlobal(&dcd_dwc3->SetupData);
    TrbPtr->BufferPtrHigh = ((ULONG)&dcd_dwc3->SetupData >> 16U) >> 16U;
    TrbPtr->Size = 0U;
    TrbPtr->Ctrl = Type;
    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                     | UX_DCD_DWC3_TRB_CTRL_LST
                     | UX_DCD_DWC3_TRB_CTRL_IOC
                     | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);

    RTSS_CleanDCache_by_Addr(&dcd_dwc3->SetupData, UX_SETUP_SIZE);
    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;

    dcd_dwc3->ep0state = EP0_STATUS_PHASE;

    /*
     * Control OUT transfer - Status stage happens on EP0 IN - EP1
     * Control IN transfer - Status stage happens on EP0 OUT - EP0
     */
    Dir = !dcd_dwc3->ep0_expect_in;

    Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, Dir,
                            UX_DCD_DEPCMD_STARTTRANSFER,
                            Params);
    if (Ret != 1) {
        return 0;
    }

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
    Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                   Ept->ux_dcd_dwc3_ed_index,
                                               Ept->ux_dcd_dwc3_ed_direction);

    return 0;

}

UINT _ux_dcd_Ep0StatusDone(UX_DCD_DWC3 *dcd_dwc3)
{
    UX_DCD_EP_TRB      *TrbPtr;
    ULONG Status;

    TrbPtr = &dcd_dwc3->endp0_trb;
    Status = UX_DCD_DWC3_TRB_SIZE_TRBSTS(TrbPtr->Size);
    if (Status == UX_DCD_DWC3_TRBSTS_SETUP_PENDING) {
#ifdef DEBUG
	printf("Status was pending hence negative\n");
#endif
        return -1;
    }

    _ux_dcd_RecvSetup(dcd_dwc3);
    return 0;
}


UINT _ux_dcd_SendLineCoding(UX_DCD_DWC3 *dcd_dwc3)
{
    UX_DCD_EP_PARAMS Params;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Ret;

    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;

    /* Setup packet always on EP0 */
    Ept = &dcd_dwc3->eps[0U];
    if ((Ept->endp_flag & UX_DCD_EP_BUSY) != 0U) {
#ifdef DEBUG
       printf("Endpoint busy\n");
#endif
       return 0;
    }

    TrbPtr = &dcd_dwc3->endp0_trb_unaligned[Ept->trb_enqueue];
    Ept->trb_enqueue++;

    TrbPtr->BufferPtrLow = (ULONG)LocalToGlobal(&dcd_dwc3->LineCoding);
    TrbPtr->BufferPtrHigh = ((ULONG)&dcd_dwc3->LineCoding >> 16U) >> 16U;
    TrbPtr->Size = 7U;
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;
    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                 | UX_DCD_DWC3_TRB_CTRL_ISP_IMI | UX_DCD_DWC3_TRB_CTRL_CHN);
    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;

    TrbPtr = &dcd_dwc3->endp0_trb_unaligned[Ept->trb_enqueue]; //Added for extra packet trb
    TrbPtr->BufferPtrLow  = lower_32_bits(LocalToGlobal(dcd_dwc3->ep0_trb));
    TrbPtr->BufferPtrHigh  = upper_32_bits(LocalToGlobal(dcd_dwc3->ep0_trb));
    TrbPtr->Size = 64 - (7 % 64);
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;

    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                 | UX_DCD_DWC3_TRB_CTRL_ISP_IMI
                 | UX_DCD_DWC3_TRB_CTRL_IOC
                 | UX_DCD_DWC3_TRB_CTRL_LST);

    dcd_dwc3->ep0state = EP0_DATA_PHASE;

    Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_IN,
                            UX_DCD_DEPCMD_STARTTRANSFER, Params);
    if (Ret != 1) {
#ifdef DEBUG
        printf("start transfer failed in RecvLinecoding\n");
#endif
        return 0;
    }

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
    Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                   Ept->ux_dcd_dwc3_ed_index,
                                               Ept->ux_dcd_dwc3_ed_direction);
    Ept->trb_enqueue = 0;
    return 1;

}


UINT _ux_dcd_RecvLineCoding(UX_DCD_DWC3 *dcd_dwc3)
{

    UX_DCD_EP_PARAMS Params;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Ret;

    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;

    /* Setup packet always on EP0 */
    Ept = &dcd_dwc3->eps[0U];
    if ((Ept->endp_flag & UX_DCD_EP_BUSY) != 0U) {
#ifdef DEBUG
        printf("Endpoint busy\n");
#endif
        return 0;
    }

    TrbPtr = &dcd_dwc3->endp0_trb_unaligned[Ept->trb_enqueue];
    Ept->trb_enqueue++;

    TrbPtr->BufferPtrLow = (ULONG)LocalToGlobal(&dcd_dwc3->LineCoding);
    TrbPtr->BufferPtrHigh = ((ULONG)&dcd_dwc3->LineCoding >> 16U) >> 16U;
    TrbPtr->Size = 7U;
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;
    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                 | UX_DCD_DWC3_TRB_CTRL_ISP_IMI | UX_DCD_DWC3_TRB_CTRL_CHN);

    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;

    TrbPtr = &dcd_dwc3->endp0_trb_unaligned[Ept->trb_enqueue]; //Added for extra packet trb
    TrbPtr->BufferPtrLow  = lower_32_bits(LocalToGlobal(dcd_dwc3->ep0_trb));
    TrbPtr->BufferPtrHigh  = upper_32_bits(LocalToGlobal(dcd_dwc3->ep0_trb));
    TrbPtr->Size = 64 - (7 % 64);
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;

    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                 | UX_DCD_DWC3_TRB_CTRL_ISP_IMI
                 | UX_DCD_DWC3_TRB_CTRL_IOC
                 | UX_DCD_DWC3_TRB_CTRL_LST);

    dcd_dwc3->ep0state = EP0_DATA_PHASE;

    Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_OUT,
                            UX_DCD_DEPCMD_STARTTRANSFER, Params);
    if (Ret != 1) {
#ifdef DEBUG
	printf("start transfer failed in RecvLinecoding\n");
#endif
	return 0;
    }

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
    Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                   Ept->ux_dcd_dwc3_ed_index,
                                               Ept->ux_dcd_dwc3_ed_direction);
    Ept->trb_enqueue = 0;
    return 1;

}

VOID _ux_dcd_EpXferComplete(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG endp_number,
                  UCHAR *data_pointer)
{
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Length;
    ULONG     Epnum;
    ULONG      Dir;
    ULONG      TrbNum;
    Epnum = endp_number;
    Ept = &dcd_dwc3->eps[Epnum];
    TrbNum = Ept->trb_dequeue;
    dcd_dwc3->TrbNum = Ept->trb_dequeue;
    Dir = Ept->ux_dcd_dwc3_ed_direction;
    TrbPtr = &Ept->EpTrb[Ept->trb_dequeue];
    RTSS_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Ept->trb_dequeue++;
    if (Ept->trb_dequeue == NO_OF_TRB_PER_EP) {
        Ept->trb_dequeue = 0U;
    }
    if (dwc3_register == UX_DCD_DWC3_DEPEVT_XFERCOMPLETE) {
        Ept->ux_dcd_dwc3_ed_status &= ~(UX_DCD_EP_BUSY);
        Ept->ux_dcd_dwc3_ed_index = 0U;
        Ept->ux_dcd_dwc3_ed_resource_index = 0U;
    }
    Length = TrbPtr->Size & UX_DCD_DWC3_TRB_SIZE_MASK;
    if (Length == 0U) {
        Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes;
 }
    else
    {
        if (Dir == UX_DCD_EP_DIR_IN)
        {
             Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes - Length;
        }
        else
        {
            if (Ept->UnalignedTx == 1U)
            {
                Ept->BytesTxed = roundup(
                                Ept->ux_dcd_dwc3_ed_requested_bytes,
                                Ept->ux_dcd_dwc3_ed_maxpacket);
                                Ept->BytesTxed -= Length;
                                Ept->UnalignedTx = 0U;
            }
            else
            {
                /*
                 * Get the actual number of bytes transmitted
                 * by host
                 */
                Ept->BytesTxed = Ept->ux_dcd_dwc3_ed_requested_bytes - Length;

             }
        }

    }

    if ((Ept->Handler != NULL) && Ept->BytesTxed >= 0) {
         Ept->Handler(dcd_dwc3, Ept->ux_dcd_dwc3_ed_requested_bytes,
                               Ept->BytesTxed, data_pointer, TrbNum);
   }
}

VOID _ux_dcd_EpXferNotReady(UX_DCD_DWC3 *dcd_dwc3, ULONG dwc3_register, ULONG endp_number)
{
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Epnum;
    ULONG     CurUf;
    ULONG     Mask;

    Epnum = endp_number;
    Ept = &dcd_dwc3->eps[Epnum];

}

VOID BulkOutHandler(VOID *Controller, ULONG RequestedBytes, ULONG BytesTxed,
                UCHAR *data_pointer, ULONG TrbNum)
{
    TX_INTERRUPT_SAVE_AREA

    UX_DCD_DWC3 *dcd_dwc3 = (UX_DCD_DWC3 *)Controller;

    if(!dcd_dwc3) //Fault Condition shouldn't happen
      return;

    RTSS_InvalidateDCache_by_Addr(&dcd_dwc3->BulkData[TrbNum], BytesTxed);
    /* Lockout interrupts.  */
    TX_DISABLE

    _ux_utility_memory_copy(data_pointer, (UCHAR *)&dcd_dwc3->BulkData[TrbNum],
                         BytesTxed);
    dcd_dwc3 -> NumBytes = BytesTxed;
    /* Restore interrupts.  */
    TX_RESTORE
    _ux_utility_event_flags_set(&BULKIN_BULKOUT_FLAG, UX_BULKOUT_EVENT, TX_OR);
}

VOID BulkInHandler(VOID *Controller, ULONG RequestedBytes, ULONG BytesTxed,
                UCHAR *data_pointer, ULONG TrbNum)
{

    UX_DCD_DWC3 *dcd_dwc3 = (UX_DCD_DWC3 *)Controller;
    if(!dcd_dwc3) //Fault Condition shouldn't happen
       return;

    dcd_dwc3 -> NumBytes = BytesTxed;
    _ux_utility_event_flags_set(&BULKIN_BULKOUT_FLAG, UX_BULKIN_EVENT, TX_OR);

}
VOID _ux_dcd_dwc3_clear_trb(UX_DCD_DWC3 *dcd_dwc3,ULONG endpoint_number)
{
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG      TrbNum;
    Ept = &dcd_dwc3->eps[endpoint_number];
    TrbNum = Ept->trb_dequeue;
    TrbPtr = &Ept->EpTrb[Ept->trb_dequeue];
    if(TrbPtr->Ctrl & UX_DCD_DWC3_TRB_CTRL_HWO)
    {
        TrbPtr->Ctrl &= ~(UX_DCD_DWC3_TRB_CTRL_HWO);
    }
}
