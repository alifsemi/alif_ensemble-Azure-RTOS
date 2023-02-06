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
 * @file     demo_usbx_device_cdc_acm_app_with_callbacks.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     23-Nov-2022
 * @brief    This is CDC ACM USB read and write with callbacks.
 * @Note     None.
 *******************************************************************************/


#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_dcd_dwc3.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "system_utils.h"

/* Define constants.  */
#define ONE_KB                            1024
#define UX_DEMO_NS_SIZE                   (64*ONE_KB)
#define UX_DEMO_STACK_SIZE_32             (32*ONE_KB)

#define UX_CDC_ACM_ACTIVATED_EVENT           0x01
#define USB_CDC_ACM_WRITE_COMPLETED_EVENT    0x02

static ULONG  error_counter;
/* CDC-ACM reception data buffer. */
static UCHAR recv_buffer[512] = {0x00};
static UCHAR tx_buffer[] = "Alif";
static ULONG actual_length = 0;

/* A pointer to store CDC-ACM device instance. */
static UX_SLAVE_CLASS_CDC_ACM* g_cdc = UX_NULL;
TX_EVENT_FLAGS_GROUP usb_cdc_acm_event_flags;

/* Define local function prototypes.  */

VOID    demo_thread_entry(ULONG arg);
VOID    ux_cdc_device0_instance_activate(VOID *activated);
VOID    ux_cdc_device0_instance_acm_parameter(VOID *activated);
VOID    ux_cdc_device0_instance_deactivate(VOID *deactivated);
VOID    error_handler(void);
UINT    cdc_acm_read_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length);
UINT    cdc_acm_write_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm,UINT status,ULONG length);

/* Define global data structures.   */

TX_THREAD  demo_thread;
uint8_t dma_buf[UX_DEMO_NS_SIZE]__attribute__((section("usb_dma_buf")));
UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm0_parameter;
UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER   call_back;

/* For Release build disable printf and semihosting */
#define DISABLE_SEMIHOSTING
#ifdef DISABLE_SEMIHOSTING
/* Also Disable Semihosting */
#if __ARMCC_VERSION >= 6000000
        __asm(".global __use_no_semihosting");
#elif __ARMCC_VERSION >= 5000000
        #pragma import(__use_no_semihosting)
#else
        #error Unsupported compiler
#endif

void _sys_exit(int return_code)
{
    while (1);
}
int _sys_open(void *p)
{
    return 0;
}
int _sys_close(void *p)
{
  return 0;

}
int _sys_read(void *p)
{
    return 0;
}
int _sys_write(void *p)
{
   return 0;
}
int _sys_istty(void *p)
{
   return 0;
}
int _sys_seek(void *p)
{
   return 0;
}
int _sys_flen(void *p)
{
    return 0;
}
void _ttywrch(int ch)
{
}
#endif /* DISABLE_SEMIHOSTING */

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

    /* Configuration 1 descriptor 9 bytes..  check ibNumonfiguration..  
     * 7th byte from 0x40 to 0xC0 .. 5th byte from 0x2 to 0x1 */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor. 6th bytes from 0x00 to 0x01 .. 0x1 to 0x00, 
     * 7th byte from 0x00 to 0x07 */
    0x08, 0x0b, 0x00,
    0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement. 8th bytes. from 0x05 to 0x00 */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor 3rd bytes from 0x0f to 0x02.. 0x0f to 0x02 .. 0x02 to 0x06.. 0x0f to 0x0*/
    0x04, 0x24, 0x02, 0x0f,
    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor 3rd byte from 0x00 to 0x03  4th bytes,  * from 0x00 to 0x02 then 0x1 */
    0x05, 0x24, 0x01, 0x03, 0x01, /* Data interface */

    /* High Speed Notify  descriptor 7 bytes.. from 0x10 to 0x08 then 0x0a */
    0x07, 0x05, 0x83, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor Requirement.. 8th byte 0x06 to 0x00 */
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

    /* Configuration 1 descriptor 9 bytes..  check ibNumonfiguration..  
     * 7th byte from 0x40 to 0xC0 .. 5th byte from 0x2 to 0x1 */
    0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor. 6th bytes from 0x00 to 0x01 .. 0x1 to 0x00, 
     * 7th byte from 0x00 to 0x07 */
    0x08, 0x0b, 0x00,
    0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement. 8th bytes. from 0x05 to 0x00 */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* ACM Functional Descriptor 3rd bytes from 0x0f to 0x02.. 0x0f to 0x02 .. 0x02 to 0x06.. 0x0f to 0x0*/
    0x04, 0x24, 0x02, 0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Management Functional Descriptor 3rd byte from 0x00 to 0x03  4th bytes,  * from 0x00 to 0x02 then 0x1 */
    0x05, 0x24, 0x01, 0x03, 0x01, /* Data interface */

    /* High Speed Notify  descriptor 7 bytes.. from 0x10 to 0x08 then 0x0a */
    0x07, 0x05, 0x83, 0x03, 0x0a, 0x00, 0x09,

    /* Data Interface Descriptor Requirement.. 8th byte 0x06 to 0x00 */
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
 
    /* Initialize the device storage class. The class is connected with interface 0 on configuration 1. */
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
    status = _ux_utility_event_flags_create(&usb_cdc_acm_event_flags,"USB_CDC_ACM_FLAG");

    if (status != UX_SUCCESS)
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
    static UINT flag = UX_FALSE;
    ULONG events;
    UINT status;
    call_back.ux_device_class_cdc_acm_parameter_write_callback = cdc_acm_write_callback;
    call_back.ux_device_class_cdc_acm_parameter_read_callback = cdc_acm_read_callback;

    /* wait till enumeration of USB device as CDC-ACM  \ref ux_cdc_device0_instance_activate callback */
    status = _ux_utility_event_flags_get(&usb_cdc_acm_event_flags, UX_CDC_ACM_ACTIVATED_EVENT, TX_OR_CLEAR, &events,UX_WAIT_FOREVER);

    if(status != UX_SUCCESS)
    {
        printf("No EVENTS\n");
    }
    /* starting transmission for reading and writing   */
    if(g_cdc != UX_NULL)
    {
       status  = ux_device_class_cdc_acm_ioctl(g_cdc, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START, &call_back);
       if(status !=  UX_SUCCESS)
       {
           printf("Error In Ioctl\n");
       }
       status = ux_device_class_cdc_acm_write_with_callback(g_cdc, tx_buffer, sizeof(tx_buffer));
       if(status !=  UX_SUCCESS)
       {
          printf("Error in write callback: %d\n",status);
       }
    }
    while(1)
    {
         if(g_cdc != UX_NULL)
         {
              status = _ux_utility_event_flags_get(&usb_cdc_acm_event_flags, USB_CDC_ACM_WRITE_COMPLETED_EVENT, TX_OR_CLEAR, &events,UX_WAIT_FOREVER);
              if(status == UX_SUCCESS)
              {
                  status = ux_device_class_cdc_acm_write_with_callback(g_cdc, recv_buffer, actual_length);
                  if(status !=  UX_SUCCESS)
                  {
                      printf("Error in write call back: %d\n",status);
                  }
              }
              else
              {
                   printf("No Events\n");
              }
         }
     }
}
UINT  cdc_acm_write_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm,UINT status,ULONG length)
{
    if(status == UX_SUCCESS)
    {
         actual_length = length;
        _ux_utility_event_flags_set(&usb_cdc_acm_event_flags, USB_CDC_ACM_WRITE_COMPLETED_EVENT, TX_OR);
        return (UX_SUCCESS);
    }
}

UINT cdc_acm_read_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length)
{

     if (data_pointer == NULL)
     {
        printf("Invalid Pointer\n");
        return UX_ERROR;
     }
     else
     {
         _ux_utility_memory_copy(recv_buffer,data_pointer,length);
         actual_length = length;
         printf("%s\n",recv_buffer);
         return UX_SUCCESS;
     }
}

VOID  ux_cdc_device0_instance_activate(VOID *activated)
{
     /* Save the CDC instance.  */
    g_cdc = (UX_SLAVE_CLASS_CDC_ACM *)activated;
    _ux_utility_event_flags_set(&usb_cdc_acm_event_flags, UX_CDC_ACM_ACTIVATED_EVENT, TX_OR);

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

    /* Increment error counter.  */
    error_counter++;
    printf("ERR : Initialization of USB failed with error counter :%ld\n",error_counter);

    while(1)
    {

        /* Error - just spin here!  Look at call tree in debugger
           to see where the error occurred.  */
    }
}

