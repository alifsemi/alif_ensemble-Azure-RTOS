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
 * @file     ux_dcd_dwc3_transfer_request.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    The function _ux_dcd_dwc3_transfer_request() will initiate a transfer
 *           to a specific endpoint,the endpoint can be Control, BULK IN, BULK OUT.
 * @bug      None.
 * @Note     None.
*******************************************************************************/

#define UX_SOURCE_CODE

/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc3_private.h"
#include "ux_utility.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "system_utils.h"

extern TX_EVENT_FLAGS_GROUP  BULKIN_BULKOUT_FLAG;

UINT  _ux_dcd_dwc3_transfer_request(UX_DCD_DWC3 *dcd_dwc3, UX_SLAVE_TRANSFER *transfer_request)
{

   UX_DCD_DWC3_ED          *ed;
   UX_DCD_DWC3_ED          *ep0;
   ULONG                   fifo_length;
   UX_SLAVE_ENDPOINT       *endpoint;
   ULONG                    endpoint_size;
   ULONG                    dwc3_endpoint_index;
   UINT                     status;
   ULONG                    retval;
   UX_SLAVE_DCD            *dcd;
   UX_DCD_DWC3            *dcd_dwc332;
   UX_SLAVE_DEVICE         *device;
   ULONG                    status_cmd;
   ULONG events;


    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the DEVkit DCD.  */
    dcd_dwc332 = (UX_DCD_DWC3 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Get the pointer to the logical endpoint from the transfer request.  */
    endpoint =  transfer_request -> ux_slave_transfer_request_endpoint;

    /* Get the physical endpoint from the logical endpoint.  */
    ed =  (UX_DCD_DWC3_ED *) endpoint -> ux_slave_endpoint_ed;

    /* Check for transfer direction.  Is this a IN endpoint ? */
    if (transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_OUT)
    {

        /* Get the size of the transfer, used for a IN transaction only.  */
        fifo_length =  transfer_request -> ux_slave_transfer_request_requested_length;

        /* Keep the FIFO length in the endpoint.  */
        ed -> ux_dcd_dwc3_ed_payload_length =  fifo_length;
        /* Point the FIFO buffer to the current transfer request buffer address.  */
        /* Adjust the data pointer.  */
        if ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT)
        {
             /* Get the size of the transfer, used for a IN transaction only.  */
             fifo_length =  transfer_request -> ux_slave_transfer_request_requested_length;

               /* Check if size is 0 as in ZLP.  */
             if (fifo_length == 0)
             {
                 /* Sending 0 length packet  */
                status_cmd = EpBufferSend(dcd_dwc3, UX_BULK_ENDPOINT,(ULONG *)&transfer_request -> ux_slave_transfer_request_data_pointer, fifo_length);
                if (status_cmd != UX_SUCCESS)
                {
                    /* Failure case needs to be handled */
                    return (status_cmd);
                }

                status_cmd = _ux_utility_event_flags_get(&BULKIN_BULKOUT_FLAG, UX_BULKIN_EVENT, UX_OR_CLEAR, &events,
                                    transfer_request -> ux_slave_transfer_request_timeout);

                if(status_cmd != TX_SUCCESS)
                {
                    return UX_NO_EVENTS;
                }
                else
                {
                    transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
                    return (UX_SUCCESS);
                }

             }
             status_cmd = EpBufferSend(dcd_dwc3, UX_BULK_ENDPOINT,(ULONG *)transfer_request -> ux_slave_transfer_request_data_pointer, fifo_length);
             if (status_cmd != UX_SUCCESS)
             {
                  /* Failure case needs to be handled */
                  return (status_cmd);
             }
             status_cmd = _ux_utility_event_flags_get(&BULKIN_BULKOUT_FLAG, UX_BULKIN_EVENT | UX_DISCONNECT_EVENT, TX_OR_CLEAR, &events,
                                 transfer_request -> ux_slave_transfer_request_timeout);
             if(status_cmd != UX_SUCCESS)
             {
                 return UX_NO_EVENTS;
             }
             else
             {
                 transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
                 return (UX_SUCCESS);
             }
        }
        else if((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_CONTROL_ENDPOINT)   /* For control endpoint process */
        {
            status_cmd = EpBufferSend(dcd_dwc3, UX_CONTROL_ENDPOINT,
                     (ULONG *)transfer_request -> ux_slave_transfer_request_data_pointer, fifo_length);
            if (!status_cmd)
            {
                /* Failure case needs to be handled */
                return (!UX_SUCCESS);
            }
            /* Set the completion code to no error.  */
            transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
        }
        else
        {
           /*  Do nothing */
        }

    }
    else if((transfer_request -> ux_slave_transfer_request_phase == UX_TRANSFER_PHASE_DATA_IN) &&
          ((endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT))
    {
        fifo_length = transfer_request -> ux_slave_transfer_request_requested_length;

        status_cmd = _ux_dcd_RecvBulkData(dcd_dwc3, UX_BULK_ENDPOINT,(ULONG *)&transfer_request -> ux_slave_transfer_request_data_pointer,fifo_length);

        if (status_cmd != UX_SUCCESS)
        {
           /* Failure case needs to be handled */
           return (status_cmd);
        }

        status_cmd = _ux_utility_event_flags_get(&BULKIN_BULKOUT_FLAG, UX_BULKOUT_EVENT | UX_DISCONNECT_EVENT, TX_OR_CLEAR, &events,
                                transfer_request -> ux_slave_transfer_request_timeout);

        if(status_cmd != TX_SUCCESS)
        {
             return UX_NO_EVENTS;
        }
        else
        {
             transfer_request -> ux_slave_transfer_request_completion_code =  UX_SUCCESS;
             return (UX_SUCCESS);
        }

    }
    else
    {
        /* Do nothing */
    }

    return(UX_SUCCESS);
}

UINT EpBufferSend(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                        ULONG *BufferPtr, ULONG BufferLen)
{
    if (UsbEp == 0 && BufferLen == 0)
    {
        return 1;
    }
    else
       return _ux_dcd_EpBufferSend(dcd_dwc3,
                              UsbEp, BufferPtr, BufferLen);

}

UINT _ux_dcd_EpBufferSend (UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
         ULONG *BufferPtr, ULONG BufferLen)
{
    ULONG      PhyEpNum;
    ULONG      cmd;
    ULONG     RetVal;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED *Ept;
    UX_DCD_EP_PARAMS Params;
    UINT status;
    ULONG events;
    ULONG Size;
    const ULONG *BufferIndex = NULL;

    PhyEpNum = UX_DCD_PhysicalEp(UsbEp, UX_DCD_EP_DIR_IN);
    if (PhyEpNum == 1U)
    {
        RetVal = _ux_dcd_Ep0Send(dcd_dwc3, BufferPtr, BufferLen);
        return RetVal;
    }

    Ept = &dcd_dwc3->eps[PhyEpNum];
    if (Ept->ux_dcd_dwc3_ed_direction != UX_DCD_EP_DIR_IN)
    {
#ifdef  DEBUG
        printf("Direction is wrong returning\n");
#endif
        return 1;
    }
    Ept->BytesTxed = 0U;
    Ept->BufferPtr = BufferPtr;
    Size = BufferLen;
    Ept->ux_dcd_dwc3_ed_requested_bytes = BufferLen;

        /*
         * 8.2.5 - An OUT transfer size (Total TRB buffer allocation)
         * must be a multiple of MaxPacketSize even if software is expecting a
         * fixed non-multiple of MaxPacketSize transfer from the Host.
         */
    if (IS_ALIGNED(BufferLen, Ept->ux_dcd_dwc3_ed_maxpacket))
    {
        Size = roundup(BufferLen, Ept->ux_dcd_dwc3_ed_maxpacket);
                Ept->UnalignedTx = 1U;
    }
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE

    TrbPtr = &Ept->EpTrb[Ept->trb_enqueue];
    Ept->trb_enqueue++;
    if (Ept->trb_enqueue == NO_OF_TRB_PER_EP)
    {
        Ept->trb_enqueue = 0U;
    }
    RTSS_CleanDCache_by_Addr(BufferPtr, BufferLen);
    TrbPtr->BufferPtrLow  = (ULONG)LocalToGlobal(BufferPtr);
    TrbPtr->BufferPtrHigh  = ((ULONG)BufferPtr >> 16U) >> 16U;
    TrbPtr->Size = Size;

    switch (Ept->ux_dcd_dwc3_ed_type)
    {
        case UX_DCD_ENDPOINT_XFER_ISOC:
           /*
           *  According to DWC3 datasheet, XUSBPSU_TRBCTL_ISOCHRONOUS and
           *  XUSBPSU_TRBCTL_CHN fields are only set when request has
           *  scattered list so these fields are not set over here.
           */
             TrbPtr->Ctrl = (UX_DCD_DWC3_TRBCTL_ISOCHRONOUS_FIRST
                           | UX_DCD_DWC3_TRB_CTRL_CSP);

                break;
        case UX_DCD_ENDPOINT_XFER_INT:
        case UX_DCD_ENDPOINT_XFER_BULK:
                TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_NORMAL;
                break;
        default:
                /* Do Nothing. */
                break;
    }
    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO |UX_DCD_DWC3_TRB_CTRL_IOC);

    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;
    TX_RESTORE
    if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) != 0U)
    {
        cmd = UX_DCD_DEPCMD_UPDATETRANSFER;
        cmd |= UX_DCD_DEPCMD_PARAM(Ept->ux_dcd_dwc3_ed_resource_index);
    }
    else
    {
        if (Ept->ux_dcd_dwc3_ed_type == UX_DCD_ENDPOINT_XFER_ISOC)
        {
            BufferPtr += BufferLen;
            UX_DCD_EP_TRB      *TrbTempNext;
            TrbTempNext = &Ept->EpTrb[Ept->trb_enqueue];
            Ept->trb_enqueue++;
            if(Ept->trb_enqueue == NO_OF_TRB_PER_EP)
            {
                 Ept->trb_enqueue = 0U;
            }
            TrbTempNext->BufferPtrLow  = (ULONG)LocalToGlobal(BufferPtr);
            TrbTempNext->BufferPtrHigh  = ((ULONG)BufferPtr >> 16U) >> 16U;
            TrbTempNext->Size = BufferLen & UX_DCD_DWC3_TRB_SIZE_MASK;
            TrbTempNext->Ctrl = (UX_DCD_DWC3_TRBCTL_ISOCHRONOUS_FIRST
                                | UX_DCD_DWC3_TRB_CTRL_CSP
                                | UX_DCD_DWC3_TRB_CTRL_HWO
                                | UX_DCD_DWC3_TRB_CTRL_IOC
                                | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);
        }

        cmd = UX_DCD_DEPCMD_STARTTRANSFER;
    }

    RetVal = _ux_dcd_SendEpCmd(dcd_dwc3, UsbEp, Ept->ux_dcd_dwc3_ed_direction,
                                                                cmd, Params);
    if (RetVal != 1)
    {

        return RetVal;
    }

     if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) == 0U)
     {
         Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                Ept->ux_dcd_dwc3_ed_index,
                                Ept->ux_dcd_dwc3_ed_direction);

         Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
     }

     return UX_SUCCESS;
}

UINT _ux_dcd_Ep0Send(UX_DCD_DWC3 *dcd_dwc3, ULONG *BufferPtr, ULONG BufferLen)
{
     /* Control IN - EP1 */
    UX_DCD_EP_PARAMS Params;
    UX_DCD_DWC3_ED       *Ept;
    UX_DCD_EP_TRB      *TrbPtr;
    ULONG Ret;
    UCHAR *SetupData;
    dcd_dwc3->BufferPtr = BufferPtr;

    Ept = &dcd_dwc3->eps[1U];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    SetupData = (UCHAR *)BufferPtr;

    if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) != 0U)
    {
#ifdef DEBUG
       printf("Endpoint 1 already busy returning\n");
#endif
       return 0;
    }

    Ept->ux_dcd_dwc3_ed_payload_length = BufferLen;
    Ept->ux_dcd_dwc3_ed_requested_bytes = BufferLen;
    Ept->BytesTxed = 0U;
    Ept->BufferPtr = BufferPtr;
    TrbPtr = &dcd_dwc3->endp0_trb;
    RTSS_CleanDCache_by_Addr(BufferPtr, BufferLen);
    TrbPtr->BufferPtrLow  =  lower_32_bits(LocalToGlobal(BufferPtr));
    TrbPtr->BufferPtrHigh  = upper_32_bits(LocalToGlobal(BufferPtr));
    TrbPtr->Size = BufferLen;
    TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;
    TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                    | UX_DCD_DWC3_TRB_CTRL_ISP_IMI
                    | UX_DCD_DWC3_TRB_CTRL_LST
                    | UX_DCD_DWC3_TRB_CTRL_IOC);

    RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (ULONG)TrbPtr;
    dcd_dwc3->ep0state = EP0_DATA_PHASE;
    Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_IN,
                                        UX_DCD_DEPCMD_STARTTRANSFER, Params);
    if (Ret != 1)
    {
#ifdef DEBUG
       printf("Failed in sending data over EP1\n");
#endif
       return 0;
    }

    Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;
    Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                 Ept->ux_dcd_dwc3_ed_index,
                                                 Ept->ux_dcd_dwc3_ed_direction);

    return 1;

}

UINT  _ux_dcd_dwc3_state_change(UX_DCD_DWC3 *dcd_dwc3, ULONG state)
{

   /* This function always succeeds.  */
    return(UX_SUCCESS);
}

UINT _ux_dcd_RecvBulkData(UX_DCD_DWC3 *dcd_dwc3, ULONG UsbEp,
                                ULONG *BufferPtr, ULONG BufferLen)
{
    UX_DCD_EP_PARAMS Params;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG     Ret;
    ULONG      PhyEpNum;
    ULONG Size;
    ULONG cmd;
    ULONG TrbNum;
    ULONG *BufferIndex = NULL;
    UINT status;
    ULONG events;

    PhyEpNum = UX_DCD_PhysicalEp(UsbEp, UX_DCD_EP_DIR_OUT);

    if (PhyEpNum == 0U)
    {
        Ret = _ux_dcd_Ep0Recv(dcd_dwc3, BufferPtr, BufferLen);
        return Ret;
    }

    Params.Param0 = 0U;
    Params.Param1 = 0U;
    Params.Param2 = 0U;

     /* Bulk Setup packet always on BULK OUT packet */
    Ept = &dcd_dwc3->eps[PhyEpNum];
    if (Ept->ux_dcd_dwc3_ed_direction != UX_DCD_EP_DIR_OUT)
    {
#ifdef DEBUG
        printf("Wrong BULK endpoint direction\n");
#endif
        return 1;
    }
    if(BufferLen < Ept->ux_dcd_dwc3_ed_maxpacket)
    {
       Size = Ept->ux_dcd_dwc3_ed_maxpacket;
       Ept->ux_dcd_dwc3_ed_requested_bytes = Ept->ux_dcd_dwc3_ed_maxpacket;
    }
    else
    {
       Size = BufferLen;
       Ept->ux_dcd_dwc3_ed_requested_bytes = BufferLen;
    }
    Ept->BytesTxed = 0U;
    Ept->BufferPtr = BufferPtr;
    TrbNum = Ept->trb_enqueue;

        /*
         * 8.2.5 - An OUT transfer size (Total TRB buffer allocation)
         * must be a multiple of MaxPacketSize even if software is expecting a
         * fixed non-multiple of MaxPacketSize transfer from the Host.
         */
     if (IS_ALIGNED(BufferLen, Ept->ux_dcd_dwc3_ed_maxpacket))
     {
         Size = roundup(BufferLen, Ept->ux_dcd_dwc3_ed_maxpacket);
         Ept->UnalignedTx = 1U;
     }
     BufferIndex = (ULONG *)&dcd_dwc3->BulkData[Ept->trb_enqueue];
     _ux_utility_memory_set((UCHAR *)BufferIndex, 0x00, 512);

        /* Macro for saving and restoring the interrupts to support concurrent access*/
     TX_INTERRUPT_SAVE_AREA
     TX_DISABLE
     TrbPtr = &Ept->EpTrb[Ept->trb_enqueue];
     Ept->trb_enqueue += 1U;
     if (Ept->trb_enqueue == NO_OF_TRB_PER_EP)
     {
        Ept->trb_enqueue = 0U;
     }
     TrbPtr->BufferPtrLow  = (ULONG)LocalToGlobal(BufferIndex);
     TrbPtr->BufferPtrHigh = ((ULONG)BufferIndex >> 16U) >> 16U;
     TrbPtr->Size = Size;
     TX_RESTORE
     switch (Ept->ux_dcd_dwc3_ed_type)
     {

        case UX_DCD_ENDPOINT_XFER_ISOC:
                /*
                 *  According to Linux driver, UX_DCD_DWC3_TRBCTL_ISOCHRONOUS and
                 *  UX_DCD_DWC3_TRBCTL_CHN fields are only set when request has
                 *  scattered list so these fields are not set over here.
                 */
              TrbPtr->Ctrl = (UX_DCD_DWC3_TRBCTL_ISOCHRONOUS_FIRST
                                | UX_DCD_DWC3_TRB_CTRL_CSP);
                break;
        case UX_DCD_ENDPOINT_XFER_INT:
        case UX_DCD_ENDPOINT_XFER_BULK:
             TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_NORMAL;
                break;
        default:
                /* Do Nothing. Added for making MISRA-C complaint */
                break;
     }

     TrbPtr->Ctrl |= ( UX_DCD_DWC3_TRB_CTRL_CSP | UX_DCD_DWC3_TRB_CTRL_IOC
                        | UX_DCD_DWC3_TRB_CTRL_ISP_IMI
                        | UX_DCD_DWC3_TRB_CTRL_HWO);

     RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
     Params.Param0 = 0;
     Params.Param1 = (ULONG)TrbPtr;
     if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) != 0U)
     {
         cmd = UX_DCD_DEPCMD_UPDATETRANSFER;
         cmd |= UX_DCD_DEPCMD_PARAM(Ept->ux_dcd_dwc3_ed_resource_index);
     }
     else
     {
        if (Ept->ux_dcd_dwc3_ed_type == UX_DCD_ENDPOINT_XFER_ISOC)
        {
            BufferPtr += BufferLen;
            UX_DCD_EP_TRB      *TrbTempNext;
            TrbTempNext = &Ept->EpTrb[Ept->trb_enqueue];
            Ept->trb_enqueue++;
            if (Ept->trb_enqueue == NO_OF_TRB_PER_EP)
            {
               Ept->trb_enqueue = 0U;
            }

            TrbTempNext->BufferPtrLow  = (ULONG)LocalToGlobal(BufferPtr);
            TrbTempNext->BufferPtrHigh  = ((ULONG)BufferPtr >> 16U) >> 16U;
            TrbTempNext->Size = BufferLen & UX_DCD_DWC3_TRB_SIZE_MASK;

            TrbTempNext->Ctrl = (UX_DCD_DWC3_TRBCTL_ISOCHRONOUS_FIRST
                                        | UX_DCD_DWC3_TRB_CTRL_CSP
                                        | UX_DCD_DWC3_TRB_CTRL_HWO
                                        | UX_DCD_DWC3_TRB_CTRL_IOC
                                        | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);
        }
        cmd = UX_DCD_DEPCMD_STARTTRANSFER;

     }

     Ret = _ux_dcd_SendEpCmd(dcd_dwc3, UsbEp, Ept->ux_dcd_dwc3_ed_direction,
                                                                cmd, Params);
     if (Ret != 1)
     {
#ifdef DEBUG
        printf("SendEpCmd failed\n");
#endif
        return 1;
     }

     if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) == 0U)
     {
        Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                               Ept->ux_dcd_dwc3_ed_index,
                                                                Ept->ux_dcd_dwc3_ed_direction);

        Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;

     }
     return UX_SUCCESS;
}

UINT _ux_dcd_Ep0Recv(UX_DCD_DWC3 *dcd_dwc3, ULONG *BufferPtr, ULONG BufferLen)
{
    UX_DCD_EP_PARAMS Params;
    UX_DCD_EP_TRB      *TrbPtr;
    UX_DCD_DWC3_ED       *Ept;
    ULONG Size;
    ULONG Ret;
    Ept = &dcd_dwc3->eps[0U];
    if ((Ept->ux_dcd_dwc3_ed_status & UX_DCD_EP_BUSY) != 0U)
    {
        return 0;
    }

    Params.Param0 = 0U;
    Params.Param1 = 0U;
    Params.Param2 = 0U;
    Ept->ux_dcd_dwc3_ed_requested_bytes = BufferLen;
    Size = BufferLen;
    Ept->BytesTxed = 0U;
    Ept->BufferPtr = BufferPtr;
        /*
         * 8.2.5 - An OUT transfer size (Total TRB buffer allocation)
         * must be a multiple of MaxPacketSize even if software is expecting a
         * fixed non-multiple of MaxPacketSize transfer from the Host.
         */
     if (!IS_ALIGNED(BufferLen, Ept->ux_dcd_dwc3_ed_maxpacket))
     {
         ULONG TmpSize = Ept->ux_dcd_dwc3_ed_maxpacket;
         Size = (ULONG)roundup(BufferLen, (ULONG)TmpSize);
         Ept->UnalignedTx = 1U;
     }

     TrbPtr = &dcd_dwc3->endp0_trb;
     TrbPtr->BufferPtrLow = (ULONG)LocalToGlobal(BufferPtr);
     TrbPtr->BufferPtrHigh = ((ULONG)BufferPtr >> 16U) >> 16U;
     TrbPtr->Size = Size;
     TrbPtr->Ctrl = UX_DCD_DWC3_TRBCTL_CONTROL_DATA;

     TrbPtr->Ctrl |= (UX_DCD_DWC3_TRB_CTRL_HWO
                        | UX_DCD_DWC3_TRB_CTRL_LST
                        | UX_DCD_DWC3_TRB_CTRL_IOC
                        | UX_DCD_DWC3_TRB_CTRL_ISP_IMI);

     RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
     Params.Param0 = 0U;
     Params.Param1 = (ULONG)TrbPtr;

     dcd_dwc3->ep0state = EP0_DATA_PHASE;
     Ret = _ux_dcd_SendEpCmd(dcd_dwc3, 0U, UX_DCD_EP_DIR_OUT,
                                UX_DCD_DEPCMD_STARTTRANSFER, Params);
     if (Ret != 1)
     {
        return 0;
     }

     Ept->ux_dcd_dwc3_ed_resource_index = _ux_dcd_EpGetTransferIndex(dcd_dwc3,
                                                        Ept->ux_dcd_dwc3_ed_index,
                                                        Ept->ux_dcd_dwc3_ed_direction);
     Ept->ux_dcd_dwc3_ed_status |= UX_DCD_EP_BUSY;

     return 1;
}
