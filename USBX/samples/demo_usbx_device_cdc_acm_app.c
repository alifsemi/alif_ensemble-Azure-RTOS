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
 * @file     demo_usbx_device_cdc_acm_app.c
 * @author   Anil
 * @email    anil.appana@alifsemi.com
 * @version  V1.0.0
 * @date     31-May-2022
 * @brief    This is CDC ACM USB read and write.
 * @Note     None.
 *******************************************************************************/

#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_dcd_dwc3.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "system_utils.h"
#include "se_services_port.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "Driver_Common.h"
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* Define constants.  */
#define ONE_KB                            1024
#define UX_DEMO_NS_SIZE                   (32*ONE_KB)
#define UX_DEMO_STACK_SIZE_32             (32*ONE_KB)

static ULONG  error_counter;
/* CDC-ACM reception data buffer. */
static UCHAR recv_buffer[512] = {0x00};

/* A pointer to store CDC-ACM device instance. */
static UX_SLAVE_CLASS_CDC_ACM* g_cdc = UX_NULL;

/* Define local function prototypes.  */

VOID    demo_thread_entry(ULONG arg);
VOID    ux_cdc_device0_instance_activate(VOID *activated);
VOID    ux_cdc_device0_instance_acm_parameter(VOID *activated);
VOID    ux_cdc_device0_instance_deactivate(VOID *deactivated);
VOID    error_handler(void);

/* Define global data structures.   */

TX_THREAD  demo_thread;
uint8_t dma_buf[UX_DEMO_NS_SIZE]__attribute__((section("usb_dma_buf")));
UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm0_parameter;

/* Change the endpoint packet size later */
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED 93
UCHAR device_framework_full_speed[] = {

    /*
    Device descriptor 18 bytes
    0x02 bDeviceClass: CDC class code
    0x00 bDeviceSubclass: CDC class sub code 0x00 bDeviceProtocol: CDC Device protocol
    idVendor & idProduct - https://www.linux-usb.org/usb.ids
    */

    0x12, 0x01, 0x00, 0x02,
    0x02, 0x00, 0x01, 0x40,
    0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,

    /* Configuration  descriptor 9 bytes  */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor  */
    0x08, 0x0b, 0x00,
    0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x0f,
    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x03, 0x01, /* Data interface */

    /* High Speed Notify  descriptor 7 bytes */
    0x07, 0x05, 0x83, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor  */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* ACM HS IN Endpoint descriptor 7 bytes*/
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00,

    /* ACM HS OUT Endpoint descriptor 7 bytes */
        0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00

};

#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED 93

UCHAR device_framework_high_speed[] = {

    /*
    Device descriptor 18 bytes
    0x02 bDeviceClass: CDC class code
    0x00 bDeviceSubclass: CDC class sub code 0x00 bDeviceProtocol: CDC Device protocol
    idVendor & idProduct - https://www.linux-usb.org/usb.ids
    */

    0x12, 0x01, 0x00, 0x02,
    0x02, 0x00, 0x01, 0x40,
    0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,

    /* Configuration  descriptor 9 bytes  */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor */
    0x08, 0x0b, 0x00,
    0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01, 0x03, 0x01, /* Data interface */

    /* High Speed Notify  descriptor 7 bytes  */
    0x07, 0x05, 0x83, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor Requirement  */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* ACM HS IN Endpoint descriptor 7 bytes*/
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00,

    /* ACM HS OUT Endpoint descriptor 7 bytes */
        0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00

};

    /* String Device Framework :
     Byte 0 and 1 : Word containing the language ID : 0x0904 for US
     Byte 2       : Byte containing the index of the descriptor
     Byte 3       : Byte containing the length of the descriptor string
    */

#define STRING_FRAMEWORK_LENGTH 42
UCHAR string_framework[] = {

  (UCHAR) (0x0409), /* 0 Supported Language Code */
  (UCHAR) (0x0409 >> 8), /* 1 Supported Language Code */
  0x01, /* 2 Index */
  17, /* 3 bLength */
  'A', 'l', 'i', 'f', 'S', 'e', 'm', 'i', 'c', 'o', 'n', 'd', 'u', 'c', 't', 'o', 'r',

  (UCHAR) (0x0409), /* 0 Supported Language Code */
  (UCHAR) (0x0409 >> 8), /* 1 Supported Language Code */
  0x02, /* 2 Index */
  6, /* 3 bLength */
  'D', 'e', 'v', 'k', 'i', 't',

  (UCHAR) (0x0409), /* 0 Supported Language Code */
  (UCHAR) (0x0409 >> 8), /* 1 Supported Language Code */
  0x03, /* 2 Index */
  4, /* 3 bLength */
  '1', '2', '0', '0'

};

    /* Multiple languages are supported on the device, to add
       a language besides english, the unicode language code must
       be appended to the language_id_framework array and the length
     adjusted accordingly. */
#define LANGUAGE_ID_FRAMEWORK_LENGTH 2
UCHAR language_id_framework[] = {

    /* English. */
        0x09, 0x04
    };

int  main(void)
{
    UINT    error_code = 0;
    UINT    service_error_code = 0;
    run_profile_t   runp = {0};
    /* Initialize the SE services */
    se_services_port_init();
    /* Example code to enable the CLKEN_USB clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                       /*clock_enable_t*/ CLKEN_USB,
                                               /*bool enable   */ true,
                                                          &service_error_code);
    if(error_code)
    {
        printf("SE: clk enable = %d\n", error_code);
        return 0;
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if(error_code)
    {
         printf("SE: get_run_cfg error = %d\n", error_code);
         return 0;
    }
    runp.phy_pwr_gating |=  USB_PHY_MASK;
    runp.memory_blocks = SRAM2_MASK | SRAM3_MASK | MRAM_MASK;

    /* Set the current run configuration to SE */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if(error_code)
    {
         printf("SE: set_run_cfg error = %d\n", error_code);
         return 0;
    }
    #if defined(RTE_Compiler_IO_STDOUT_User)
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK)
    {
        while(1)
        {
        }
    }
    #endif

    /* Enter the ThreadX kernel.  */
    printf("Started USBx driver app\n");
    tx_kernel_enter();
    return(0);
}


void  tx_application_define(void *first_unused_memory)
{

   UINT    status;

    /* Initialize USBX Memory */
   status = ux_system_initialize(dma_buf, UX_DEMO_NS_SIZE, UX_NULL, 0x00);

    if(status != UX_SUCCESS)
    {
       error_handler();
    }

    /* The code below is required for installing the device portion of USBX.
       In this demo, DFU is possible and we have a call back for state change. */
    status =  ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                       device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                       string_framework, STRING_FRAMEWORK_LENGTH,
                                       language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH,UX_NULL);

    if(status != UX_SUCCESS)
    {
       error_handler();
    }
    /* Setting cdc acm activation and deavtivation functionality */
    cdc_acm0_parameter.ux_slave_class_cdc_acm_instance_activate = ux_cdc_device0_instance_activate;

    cdc_acm0_parameter.ux_slave_class_cdc_acm_instance_deactivate = ux_cdc_device0_instance_deactivate;

    cdc_acm0_parameter.ux_slave_class_cdc_acm_parameter_change = ux_cdc_device0_instance_acm_parameter;

    /* Register the class driver as CDC-ACM  */
    status =  ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry,
                                                1, 0, (VOID *)&cdc_acm0_parameter);
    if(status != UX_SUCCESS)
    {
        error_handler();
    }

    status =  _ux_dcd_dwc3_initialize();

    if(status != UX_SUCCESS)
    {
        error_handler();
    }

    /* Create the main demo thread.  */
    status = tx_thread_create(&demo_thread, "USBx demo", demo_thread_entry, 0,  first_unused_memory,UX_DEMO_STACK_SIZE_32,
            20, 20, 100, TX_AUTO_START);

    if(status != UX_SUCCESS)
    {
       error_handler();
    }
}

VOID    demo_thread_entry(ULONG arg)
{
    ULONG actual_length = 0;
    UINT status;
    while (1)
    {
       if(g_cdc != UX_NULL)
       {

          status = ux_device_class_cdc_acm_read(g_cdc, recv_buffer,sizeof(recv_buffer), (ULONG *) &actual_length);
          if((status != UX_SUCCESS)&&(status != UX_TRANSFER_NO_ANSWER))
          {
              error_handler();
          }
          status = ux_device_class_cdc_acm_write(g_cdc, recv_buffer,actual_length, (ULONG *) &actual_length);
          if((status != UX_SUCCESS) && (status != UX_TRANSFER_NO_ANSWER))
          {
              error_handler();
          }

       }
    }
}


VOID  ux_cdc_device0_instance_activate(VOID *activated)
{
     /* Save the CDC instance.  */
    g_cdc = (UX_SLAVE_CLASS_CDC_ACM *)activated;

}

VOID  ux_cdc_device0_instance_deactivate(VOID *deactivated)
{
    g_cdc = UX_NULL;
}

VOID  ux_cdc_device0_instance_acm_parameter(VOID *activated)
{
    /* For now only notifications.  */
}


VOID  error_handler(void)
{

    UINT    error_code = 0;
    UINT    service_error_code = 0;
    run_profile_t   runp = {0};
    /* Increment error counter.  */
    error_counter++;
    printf("ERR : Initialization of USB failed with error counter :%ld\n",error_counter);
    /* Example code to enable the CLKEN_USB clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                       /*clock_enable_t*/ CLKEN_USB,
                                               /*bool disable   */ false,
                                                         &service_error_code);
    if(error_code)
    {
        printf("SE: clk disable = %d\n", error_code);
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if(error_code)
    {
         printf("SE: get_run_cfg error = %d\n", error_code);
    }
    runp.phy_pwr_gating &= ~USB_PHY_MASK;

    /* Set the current run configuration to SE */
    error_code = SERVICES_set_run_cfg(se_services_s_handle, &runp,
                                              &service_error_code);
    if(error_code)
    {
         printf("SE: set_run_cfg error = %d\n", error_code);
    }
    while(1)
    {
        /* Error - just spin here!  Look at call tree in debugger
           to see where the error occurred.  */
    }
}
