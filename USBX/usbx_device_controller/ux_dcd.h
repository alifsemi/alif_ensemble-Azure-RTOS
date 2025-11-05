/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     ux_dcd.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     04-Sep-2025
 * @brief    System Control Device information for USB
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef UX_DCD_H
#define UX_DCD_H

#include "usbd_spec.h"
#include "ux_api.h"
#include "ux_device_stack.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UX_DCD_SLAVE_CONTROLLER   0x90
#define UX_MASK_ENDPOINT_NUM      0xF
#define UX_DCD_EP_STATE_IDLE      0
#define UX_DCD_EP_STATE_DATA_TX   1
#define UX_DCD_EP_STATE_DATA_RX   2
#define UX_DCD_EP_STATE_STATUS_TX 3
#define UX_DCD_EP_STATE_STATUS_RX 4

/* USB Bulk IN  and BULK OUT endpoint, Disconnect event  */
typedef enum _UX_DATA_EVENTS {
    UX_DATA_IN_EVENT       = (1 << 0),
    UX_DATA_OUT_EVENT      = (1 << 1),
    UX_DISCONNECT_EVENT    = (1 << 2),
    UX_XFERNOTREADY_EVENT  = (1 << 3)
} UX_DATA_EVENTS;

void ux_dcd_reset_cb(USB_DRIVER *drv);
void ux_dcd_setupstage_cb(USB_DRIVER *drv, uint8_t phy_ep);
void ux_dcd_ep0_data_stage_cb(USB_DRIVER *drv, uint8_t phy_ep);
void ux_dcd_data_stage_cb(USB_DRIVER *drv, uint8_t phy_ep);
void ux_dcd_disconnect_cb(void);
void ux_dcd_connect_cb(void);

uint32_t ux_dcd_transfer_request_abort(USB_DRIVER *drv, UX_SLAVE_TRANSFER *transfer_request);
int32_t  ux_dcd_transfer_request(USB_DRIVER *drv, UX_SLAVE_TRANSFER *transfer_request);
uint32_t ux_dcd_initialize_complete(void);
uint32_t ux_dcd_endpoint_destroy(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint);
uint32_t ux_dcd_endpoint_reset(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint);
uint32_t ux_dcd_endpoint_stall(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint);
uint32_t ux_dcd_endpoint_create(USB_DRIVER *drv, UX_SLAVE_ENDPOINT *endpoint);
uint32_t ux_dcd_function(UX_SLAVE_DCD *dcd, uint32_t function, VOID *parameter);
uint32_t ux_dcd_ep0_events(USB_DRIVER *drv, uint8_t phy_ep, UX_SLAVE_TRANSFER *transfer_request);
uint32_t ux_dcd_initialize(void);

static inline int32_t usb_endpoint_dir(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return epd->bEndpointAddress & UX_ENDPOINT_DIRECTION;
}

static inline int32_t ux_endpoint_num(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return epd->bEndpointAddress & UX_MASK_ENDPOINT_NUM;
}

static inline int32_t ux_endpoint_type(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return epd->bmAttributes & UX_MASK_ENDPOINT_TYPE;
}
static inline int32_t ux_endpoint_xfer_control(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return ((epd->bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_CONTROL_ENDPOINT);
}

static inline int32_t ux_endpoint_xfer_bulk(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return ((epd->bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_BULK_ENDPOINT);
}

static inline int32_t ux_endpoint_xfer_int(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return ((epd->bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT);
}

static inline int32_t ux_endpoint_xfer_isoc(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return ((epd->bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_ISOCHRONOUS_ENDPOINT);
}

static inline int32_t ux_endpoint_maxp(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return (epd->wMaxPacketSize) & UX_MAX_PACKET_SIZE_MASK;
}

#ifdef __cplusplus
}
#endif
#endif /* UX_DCD_H */
