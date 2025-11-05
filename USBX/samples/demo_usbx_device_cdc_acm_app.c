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
 * @file     demo_usbx_device_cdc_acm_app.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     31-May-2022
 * @brief    This is CDC ACM USB read and write.
 * @Note     None.
 *******************************************************************************/

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "se_services_port.h"
#include "app_utils.h"
#include "ux_dcd.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_stdout.h"
#include "retarget_init.h"
#endif /* RTE_Compiler_IO_STDOUT */

/* Define macros  */
#define ONE_KB                            1024
#define UX_DEMO_NS_SIZE                   (32 * ONE_KB)
#define UX_DEMO_STACK_SIZE                (2 * ONE_KB)
#define RECV_BUFF_SIZE                    512

/* CDC-ACM reception data buffer. */
static UCHAR recv_buffer[RECV_BUFF_SIZE];
/* A pointer to store CDC-ACM device instance. */
static UX_SLAVE_CLASS_CDC_ACM *volatile g_cdc = UX_NULL;
/* Define local function prototypes.  */
VOID    demo_thread_entry(ULONG arg);
VOID    ux_cdc_device0_instance_activate(VOID *activated);
VOID    ux_cdc_device0_instance_acm_parameter(VOID *activated);
VOID    ux_cdc_device0_instance_deactivate(VOID *deactivated);
static VOID  error_handler(uint32_t status, uint32_t lineNumber);

/* Define global data structures.   */
TX_THREAD  demo_thread;
uint8_t dma_buf[UX_DEMO_NS_SIZE] ATTR_SECTION("usb_dma_buf");

UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm0_parameter;

static UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_ACM_LineCoding = {
    9600,   /* baud rate      */
    0x00,   /* stop bits-1    */
    0x00,   /* parity - none  */
    0x08    /* no of bits 8   */
};

static UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER CDC_ACM_LineState = {
    1, /* rts bit */
    1  /* dtr bit */
};
/*  FULL speed descriptors */
static UCHAR device_framework_full_speed[] = {

   /*
    * Device descriptor 18 bytes
    * 0x02 bDeviceClass: CDC class code
    * 0x00 bDeviceSubclass: CDC class sub code 0x00 bDeviceProtocol: CDC Device protocol
    * idVendor & idProduct - https://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02,
    0x02, 0x00, 0x01, 0x40,
    0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,

    /* Configuration  descriptor 9 bytes  */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor  */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x03, 0x01,

    /* FULL Speed Notify endpoint descriptor 7 bytes */
    0x07, 0x05, 0x83, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor  */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* ACM FS BULK IN Endpoint descriptor 7 bytes*/
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00,

    /* ACM FS BULK OUT Endpoint descriptor 7 bytes */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00

};
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED     sizeof(device_framework_full_speed)
/* High speed decriptors */
static UCHAR device_framework_high_speed[] = {

   /*
    * Device descriptor 18 bytes
    * 0x02 bDeviceClass: CDC class code
    * 0x00 bDeviceSubclass: CDC class sub code 0x00 bDeviceProtocol: CDC Device protocol
    * idVendor & idProduct - https://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02,
    0x02, 0x00, 0x01, 0x40,
    0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,

    /* device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x01, 0x00,

    /* Configuration  descriptor 9 bytes  */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x03, 0x01,

    /* High Speed Notify  endpoint descriptor 7 bytes  */
    0x07, 0x05, 0x81, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor Requirement  */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* ACM HS IN Endpoint descriptor 7 bytes*/
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00,

    /* ACM HS OUT Endpoint descriptor 7 bytes */
    0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00
};
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED      sizeof(device_framework_high_speed)

  /* String Device Framework :
   * Byte 0 and 1 : containing the language ID : 0x0904 for US
   * Byte 2       : containing the index of the descriptor
   * Byte 3       : containing the length of the descriptor string
   */

static uint8_t string_framework[] = {

    0x09, 0x04,   /* language id  */
    0x01,         /* Index        */
    0x11,         /* bLength      */
    'A', 'l', 'i', 'f', 'S', 'e', 'm', 'i', 'c', 'o', 'n', 'd', 'u', 'c', 't', 'o', 'r',

    0x09, 0x04,   /* language id  */
    0x02,         /* Index        */
    0x06,         /* bLength      */
    'D', 'e', 'v', 'k', 'i', 't',

    0x09, 0x04,   /* language id  */
    0x03,         /* Index        */
    0x04,         /* bLength      */
    '1', '2', '0', '0'
};

#define STRING_FRAMEWORK_LENGTH                sizeof(string_framework)

    /* Multiple languages are supported on the device, to add
     *  a language besides english, the unicode language code must
     *  be appended to the language_id_framework array and the length
     * adjusted accordingly.
     */
static uint8_t language_id_framework[] = {

    /* English. */
    0x09, 0x04
};
#define LANGUAGE_ID_FRAMEWORK_LENGTH    sizeof(language_id_framework)

int  main(void)
{
    UINT            error_code = 0;
    UINT            service_error_code = 0;
    run_profile_t   runp = {0};

    /* Initialize the SE services */
    se_services_port_init();
    /* Enable the USB clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                       /*clock_enable_t*/ CLKEN_CLK_20M,
                                               /*bool enable   */ true,
                                                          &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    runp.phy_pwr_gating |=  USB_PHY_MASK;
    runp.memory_blocks   = SRAM0_MASK | MRAM_MASK;

    /* Set the current run configuration to SE */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t    ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif
    /* Enter the ThreadX kernel.  */
    printf("Started USBx CDC-ACM app\n");
    tx_kernel_enter();
    return 0;
}


void tx_application_define(void *first_unused_memory)
{
    UINT    status;

    /* Initialize USBX Memory */
    status = ux_system_initialize(dma_buf, UX_DEMO_NS_SIZE, UX_NULL, 0x00);
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* initialize the usbx device stack */
    status =  ux_device_stack_initialize(device_framework_high_speed,
            DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED, device_framework_full_speed,
            DEVICE_FRAMEWORK_LENGTH_FULL_SPEED, string_framework, STRING_FRAMEWORK_LENGTH,
            language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH, UX_NULL);
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Setting cdc acm activation and deavtivation functionality */
    cdc_acm0_parameter.ux_slave_class_cdc_acm_instance_activate   =
            ux_cdc_device0_instance_activate;

    cdc_acm0_parameter.ux_slave_class_cdc_acm_instance_deactivate =
            ux_cdc_device0_instance_deactivate;

    cdc_acm0_parameter.ux_slave_class_cdc_acm_parameter_change    =
            ux_cdc_device0_instance_acm_parameter;

    /* Register the CDC-ACM class */
    status =  ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
            ux_device_class_cdc_acm_entry, 1, 0, (VOID *)&cdc_acm0_parameter);
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Create the main demo thread.  */
    status = tx_thread_create(&demo_thread, "USBx demo", demo_thread_entry, 0,
            first_unused_memory, UX_DEMO_STACK_SIZE, 20, 20, 100, TX_AUTO_START);
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* initialize the usb controller driver. */
    status =  ux_dcd_initialize();
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
}

VOID  demo_thread_entry(ULONG arg)
{
    ULONG actual_length = 0;
    UINT status;

    UX_PARAMETER_NOT_USED(arg);
    while (1) {
        if (g_cdc != UX_NULL) {
            /* Read the received data from the host*/
            status = ux_device_class_cdc_acm_read(g_cdc, recv_buffer, sizeof(recv_buffer),
                    (ULONG *) &actual_length);
            if ((status != UX_SUCCESS) && (status != UX_TRANSFER_NO_ANSWER)) {
                error_handler(status, __LINE__);
            }
            /* Send data over the ux_device_class_cdc_acm_write to host */
            status = ux_device_class_cdc_acm_write(g_cdc, recv_buffer, actual_length,
                    (ULONG *) &actual_length);
            if ((status != UX_SUCCESS) && (status != UX_TRANSFER_NO_ANSWER)) {
                error_handler(status, __LINE__);
            }
        }
    }
}

/* CDC-ACM activation callback */
VOID  ux_cdc_device0_instance_activate(VOID *activated)
{
     /* Save the CDC instance.  */
    g_cdc = (UX_SLAVE_CLASS_CDC_ACM *)activated;

}
/* CDC-ACM deactivate callback */
VOID  ux_cdc_device0_instance_deactivate(VOID *deactivated)
{
    g_cdc = UX_NULL;
}
/* CDC-ACM parameter change callback */
VOID  ux_cdc_device0_instance_acm_parameter(VOID *cdc_acm_instance)
{
    UX_PARAMETER_NOT_USED(cdc_acm_instance);
    ULONG request;
    UX_SLAVE_TRANSFER *transfer_request;
    UX_SLAVE_DEVICE *device;

    /* Get the pointer to the device */
    device = &_ux_system_slave->ux_system_slave_device;

    /* Get the pointer to the transfer request associated with the control endpoint */
    transfer_request =
            &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    request = *(transfer_request->ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

    switch (request) {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING:
        /* Get the Line Coding parameters */
        if (ux_device_class_cdc_acm_ioctl(g_cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                           &CDC_ACM_LineCoding) != UX_SUCCESS) {
            printf("Set line coding error\n");
        }
        break;
    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING:
        /* Set the Line Coding parameters */
        if (ux_device_class_cdc_acm_ioctl(g_cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                           &CDC_ACM_LineCoding) != UX_SUCCESS) {
            printf("Get line coding failed\n");
        }
        break;
    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:
        /* Set the control line state bit */
        if (ux_device_class_cdc_acm_ioctl(g_cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_STATE,
                                                 &CDC_ACM_LineState) != UX_SUCCESS) {
            printf("Set control line state failed\n");
        }
        break;
    default:
        break;
    }
    return;
}


static VOID  error_handler(uint32_t status, uint32_t lineNumber)
{
    uint32_t    error_code = 0;
    uint32_t    service_error_code = 0;
    run_profile_t   runp = {0};

    printf("In ERROR handler error 0x%X occurred at %d\n", status, lineNumber);
    /* Disable the USB clock  */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
            /*clock_enable_t*/ CLKEN_CLK_20M,
            /*bool disable   */ false,
            &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp,
            &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    runp.phy_pwr_gating &= ~USB_PHY_MASK;

    /* Set the current run configuration to SE */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp,
            &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    WAIT_FOREVER_LOOP
}
