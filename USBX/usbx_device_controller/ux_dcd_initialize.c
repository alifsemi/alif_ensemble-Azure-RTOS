/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     ux_dcd_initialize.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This file  initializes the USB controller in device/host mode.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "sys_clocks.h"
#include "sys_ctrl_usb.h"

#include "ux_dcd.h"

TX_EVENT_FLAGS_GROUP DATA_IN_OUT_EVENT_FLAG;

/**
 *\fn           USB_IRQHandler
 *\brief        This function is to service the interrupt.
 *\param[in]    none
 *\return       none
 **/

#ifndef USB_HOST
/* In case of USB host mode xhci driver has USB_IRQHandler  */
void USB_IRQHandler(void)
{
    UX_SLAVE_DCD *dcd;
    USB_DRIVER   *drv;

    /* Get the pointer to the DCD.  */
    dcd = &_ux_system_slave->ux_system_slave_dcd;
    /* Get the pointer to the DCD.  */
    drv = (USB_DRIVER *) dcd->ux_slave_dcd_controller_hardware;
    if (drv == UX_NULL) {
        return;
    }
    usbd_interrupt_handler(drv);
}
#endif

/**
 *\fn           ux_dcd_event_buf_allocation
 *\brief        This function is to allocate the memory for event buffer.
 *\param[in]    pointer to the controller context structure
 *\return       returns event buff on success, else NULL
 **/

USBD_EVENT_BUFFER *ux_dcd_event_buf_allocation(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf =
        _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(USBD_EVENT_BUFFER));
    /* Check if memory was properly allocated.  */
    if (event_buf == UX_NULL) {
        return event_buf;
    }
    event_buf->length = USB_EVENT_BUFFER_SIZE;
    event_buf->buf =
        _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, USB_EVENT_BUFFER_SIZE);
    if (event_buf->buf == UX_NULL) {
        _ux_utility_memory_free(event_buf);
        event_buf = NULL;
        return event_buf;
    }
    memset(event_buf->buf, 0, USB_EVENT_BUFFER_SIZE);
    return event_buf;
}

/**
 *\fn           usbd_init
 *\brief        This function initialize the USB controller in device mode..
 *\param[in]    drv pointer to the controller context structure
 *\return       On success returns 0, else  error
 **/

static uint32_t usbd_init(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;
    UX_SLAVE_DCD      *dcd;
    uint32_t           ret;

    ret = _ux_utility_event_flags_create(&DATA_IN_OUT_EVENT_FLAG, "USB_DATA_IN_OUT_EVENT_FLAG");
    if (ret != UX_SUCCESS) {
        return ret;
    }
    drv->ux_dcd_reset_cb                  = ux_dcd_reset_cb;
    drv->ux_dcd_connect_cb                = ux_dcd_connect_cb;
    drv->ux_dcd_setup_stage_cb            = ux_dcd_setupstage_cb;
    drv->ux_dcd_disconnect_cb             = ux_dcd_disconnect_cb;
    drv->ux_dcd_ep0_data_stage_cb         = ux_dcd_ep0_data_stage_cb;
    drv->ux_dcd_data_stage_cb             = ux_dcd_data_stage_cb;
    /* Get the pointer to the DCD.  */
    dcd                                   = &_ux_system_slave->ux_system_slave_dcd;
    /* Set the pointer to the DCD.  */
    dcd->ux_slave_dcd_controller_hardware = (void *) drv;
    /* The controller initialized here is of controller type.  */
    dcd->ux_slave_dcd_controller_type     = UX_DCD_SLAVE_CONTROLLER;
    /* Initialize the function collector for this DCD.  */
    dcd->ux_slave_dcd_function            = ux_dcd_function;
    /* Allocate memory for event buffer.  */
    event_buf                             = ux_dcd_event_buf_allocation(drv);
    if (event_buf == UX_NULL) {
#ifdef DEBUG
        printf("MEMORY allocation failed\n");
#endif
        _ux_utility_event_flags_delete(&DATA_IN_OUT_EVENT_FLAG);
        return UX_MEMORY_INSUFFICIENT;
    }
    drv->event_buf = event_buf;
    /* Configures the event buffer registers in the controller */
    usbd_configure_event_buffer_registers(drv);
    /* get the No of IN and OUT eps */
    usbd_get_eps(drv);
    /* initialize the eps */
    usbd_initialize_eps(drv);
    /* Set speed on Controller */
    usbd_set_speed(drv);
    /* Configure usbd initial ep commands */
    usbd_configure_initial_ep_commands(drv);
    /* enable usb device specific event */
    usbd_enable_device_events(drv);
    /* Starting usb controller . */
    ret = usbd_start(drv);
    if (ret != 0) {
        _ux_utility_event_flags_delete(&DATA_IN_OUT_EVENT_FLAG);
        _ux_utility_memory_free(event_buf);
        drv->event_buf = NULL;
        return ret;
    }
    /* Set the state of the controller to OPERATIONAL now.  */
    dcd->ux_slave_dcd_status = UX_DCD_STATUS_OPERATIONAL;
    return ret;
}

/**
 *\fn           ux_dcd_initialize_complete
 *\brief        This function completes the initialization of USB device
 *\param[in]    void
 *\return       success
 **/

uint32_t ux_dcd_initialize_complete(void)
{
    UX_SLAVE_DCD      *dcd;
    USB_DRIVER        *drv;
    uint8_t           *device_framework;
    UX_SLAVE_DEVICE   *device;
    UX_SLAVE_TRANSFER *transfer_request;

    /* Get the pointer to the DCD.  */
    dcd    = &_ux_system_slave->ux_system_slave_dcd;
    /* Get the pointer to the DCD.  */
    drv    = (USB_DRIVER *) dcd->ux_slave_dcd_controller_hardware;
    if (drv == UX_NULL) {
        return UX_CONTROLLER_UNKNOWN;
    }
    /* Get the pointer to the device.  */
    device = &_ux_system_slave->ux_system_slave_device;
    /* Check the speed and set the correct descriptor.  */
    if (_ux_system_slave->ux_system_slave_speed == UX_FULL_SPEED_DEVICE) {
        /* The device is operating at full speed.  */
        _ux_system_slave->ux_system_slave_device_framework =
            _ux_system_slave->ux_system_slave_device_framework_full_speed;
        _ux_system_slave->ux_system_slave_device_framework_length =
            _ux_system_slave->ux_system_slave_device_framework_length_full_speed;
    } else {
        /* The device is operating at high speed.  */
        _ux_system_slave->ux_system_slave_device_framework =
            _ux_system_slave->ux_system_slave_device_framework_high_speed;
        _ux_system_slave->ux_system_slave_device_framework_length =
            _ux_system_slave->ux_system_slave_device_framework_length_high_speed;
    }
    /* Get the device framework pointer.  */
    device_framework = _ux_system_slave->ux_system_slave_device_framework;
    /* And create the decompressed device descriptor structure.  */
    _ux_utility_descriptor_parse(device_framework,
                                 _ux_system_device_descriptor_structure,
                                 UX_DEVICE_DESCRIPTOR_ENTRIES,
                                 (UCHAR *) &device->ux_slave_device_descriptor);

    /* Now we create a transfer request to accept the first SETUP packet
     * and get the ball running. First get the address of the endpoint
     * transfer request container.
     */
    transfer_request = &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    /* Set the timeout to be for Control Endpoint.  */
    transfer_request->ux_slave_transfer_request_timeout =
        UX_MS_TO_TICK(UX_CONTROL_TRANSFER_TIMEOUT);

    /* Adjust the current data pointer as well.  */
    transfer_request->ux_slave_transfer_request_current_data_pointer =
        transfer_request->ux_slave_transfer_request_data_pointer;

    /* Update the transfer request endpoint pointer with the default endpoint. */
    transfer_request->ux_slave_transfer_request_endpoint =
        &device->ux_slave_device_control_endpoint;

    /* The control endpoint max packet size needs to be filled manually in its descriptor.  */
    transfer_request->ux_slave_transfer_request_endpoint->ux_slave_endpoint_descriptor
        .wMaxPacketSize = device->ux_slave_device_descriptor.bMaxPacketSize0;

    /* On the control endpoint, always expect the maximum.  */
    transfer_request->ux_slave_transfer_request_requested_length =
        device->ux_slave_device_descriptor.bMaxPacketSize0;

    /* Create the default control endpoint attached to the device.
     * Once this endpoint is enabled, the host can then send a setup packet
     * The device controller will receive it and will call the setup function module.
     */
    dcd->ux_slave_dcd_function(dcd,
                               UX_DCD_CREATE_ENDPOINT,
                               (VOID *) &device->ux_slave_device_control_endpoint);

    /* Mark the phase as SETUP.  */
    transfer_request->ux_slave_transfer_request_type               = UX_TRANSFER_PHASE_SETUP;

    /* Mark for three stage setup for usb controller */
    drv->three_stage_setup                                         = false;
    /* Mark this transfer request as pending.  */
    transfer_request->ux_slave_transfer_request_status             = UX_TRANSFER_STATUS_PENDING;

    /* Ask for 8 bytes of the SETUP packet.  */
    transfer_request->ux_slave_transfer_request_requested_length   = UX_SETUP_SIZE;
    transfer_request->ux_slave_transfer_request_in_transfer_length = UX_SETUP_SIZE;

    /* Reset the number of bytes sent/received.  */
    transfer_request->ux_slave_transfer_request_actual_length      = 0;
    /* prepare the TRB for Setup packet   */
    usbd_prepare_setup(drv);

    /* We are now ready for the USB device to accept the first packet when connected.  */
    return UX_SUCCESS;
}

/**
 *\fn           usbd_set_controller_mode
 *\brief        This function sets the USB controller mode (device/host/OTG).
 *\param[in]    drv  pointer to the controller context structure
 *\return       On success 0, else error
 */

static int32_t usbd_set_controller_mode(USB_DRIVER *drv)
{
    int32_t ret = USB_SUCCESS;

    switch (drv->dr_mode) {
    case USB_DR_MODE_HOST:
        usbd_set_port_capability(drv, USB_GCTL_PRTCAP_HOST);
        break;
    case USB_DR_MODE_PERIPHERAL:
        usbd_set_port_capability(drv, USB_GCTL_PRTCAP_DEVICE);
        ret = usbd_init(drv);
        if (ret) {
#ifdef DEBUG
            printf("USB device init failed: %d\n", ret);
#endif
        }
        break;
    case USB_DR_MODE_OTG:
        ret = USB_MODE_UNSUPPORTED;
        break;
    default:
        ret = USB_INIT_ERROR;
    }

    return ret;
}

/**
 *\fn           ux_dcd_initialize
 *\brief        This function initialize the USB controller.
 *\param[in]    void
 *\return       On success returns 0, else  error
 **/

uint32_t ux_dcd_initialize(void)
{
    USB_DRIVER *drv;
    uint32_t    ret = UX_SUCCESS;

    /* Enable peripheral clk for USB  */
    enable_usb_periph_clk();
    /* USB phy reset */
    usb_phy_por_clear();
    /* Allocate memory for this DCD instance.  */
    drv = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(USB_DRIVER));
    /* Check if memory was properly allocated.  */
    if (drv == UX_NULL) {
        ret = UX_MEMORY_INSUFFICIENT;
        return ret;
    }
    drv->regs = (USB_Type *) USB_BASE;
    /* Validate the controller core */
    if (!usbd_verify_ip_core(drv)) {
#ifdef DEBUG
        printf("Invalid USB controller ip core detected\n");
#endif
        ret = USB_CORE_INVALID;
        goto drv_err;
    }
    usbd_set_default_config(drv);
    /* Read the controller hardware param registers */
    usbd_read_hw_params(drv);
    /* Validate the controller mode */
    ret = usbd_validate_mode(drv);
    if (ret) {
        goto drv_err;
    }
    /* Perform the device soft reset */
    ret = usbd_device_soft_reset(drv);
    if (ret != UX_SUCCESS) {
#ifdef DEBUG
        printf("device soft reset failed\n");
#endif
        goto drv_err;
    }
    /* Reset the PHY and core */
    sys_busy_loop_us(5000);
    /* Reset the PHY and core */
    usbd_reset_phy_and_core(drv);
    /* Configure the USB2 PHY */
    usbd_configure_usb2_phy(drv);
    /* Configure the global control register */
    usbd_configure_global_control_reg(drv);
    /* Configure the frame length adjustment register */
    usbd_configure_fladj_register(drv);
    /* Configure burst transfer settings for the controller */
    usbd_configure_burst_transfer(drv);
    /* Set the controller mode */
    ret = usbd_set_controller_mode(drv);
    if (ret != 0) {
#ifdef DEBUG
        printf("failed to set the controller mode\n");
#endif
        goto clean_evt_buf;
    }
    /* Return successful completion.  */
    return ret;

clean_evt_buf:
    usbd_cleanup_event_buffer(drv);

drv_err:
    _ux_utility_memory_free(drv);
    return ret;
}
