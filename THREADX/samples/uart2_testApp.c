/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     uart2_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     26-May-2023
 * @brief    TestApp to verify UART interface using Threadx as an operating system.
 *           UART interactive console application (using UART2 instance):
 *             if UART Receives 'Enter' key on serial terminal:
 *                   UART Sends "Hello World!" back to serial terminal.
 *             else UART Receives any other character on serial terminal:
 *                   UART Sends received character back to serial terminal.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */

/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* include for UART Driver */
#include "Driver_USART.h"

/* PINMUX Driver */
#include "pinconf.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#include "RTE_Device.h"
/*
 * UART2 CLK SOURCE: Defines UART2 clock source.
 *    <0=> CLK_38.4MHz (CLKEN_HFOSC)
 *    <1=> CLK_100MHz
 */
#if (RTE_UART2_CLK_SOURCE == 0) /* CLK_38.4MHz */
#include "se_services_port.h"
#endif

/* UART Driver instance (UART0-UART7) */
#define UART      2

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

void myUART_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                        1024
#define DEMO_BYTE_POOL_SIZE                    9120

#define UART_CB_TX_EVENT           (1U << 0)
#define UART_CB_RX_EVENT           (1U << 1)
#define UART_CB_RX_TIMEOUT         (1U << 2)
#define UART_CB_RX_BREAK           (1U << 3)
#define UART_CB_RX_FRAMING_ERROR   (1U << 4)
#define UART_CB_RX_PARITY_ERROR    (1U << 5)
#define UART_CB_RX_OVERFLOW        (1U << 6)

TX_THREAD               UART_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_uart;


/**
 * @function    int hardware_init(void)
 * @brief       UART hardware pin initialization using PIN-MUX driver
 * @note        none
 * @param       void
 * @retval      execution status
 */
int hardware_init(void)
{
    /* UART2_RX_A */
    pinconf_set( PORT_1, PIN_0, PINMUX_ALTERNATE_FUNCTION_1, PADCTRL_READ_ENABLE);

    /* UART2_TX_A */
    pinconf_set( PORT_1, PIN_1, PINMUX_ALTERNATE_FUNCTION_1, 0);

    return 0;
}

/**
 * @function    void myUART_callback(UINT event)
 * @brief       UART isr callabck
 * @note        none
 * @param       event: USART Event
 * @retval      none
 */
void myUART_callback(UINT event)
{
    if (event & ARM_USART_EVENT_SEND_COMPLETE)
    {
        /* Send Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_TX_EVENT, TX_OR);
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        /* Receive Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_EVENT, TX_OR);
    }

    if (event & ARM_USART_EVENT_RX_TIMEOUT)
    {
        /* Receive Success with rx timeout: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_TIMEOUT, TX_OR);
    }

    if (event & ARM_USART_EVENT_RX_BREAK)
    {
        /* Receive RX Break: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_BREAK, TX_OR);
    }

    if (event & ARM_USART_EVENT_RX_FRAMING_ERROR)
    {
        /* Receive RX Framing error: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_FRAMING_ERROR, TX_OR);
    }

    if (event & ARM_USART_EVENT_RX_PARITY_ERROR)
    {
        /* Receive RX Parity error: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_PARITY_ERROR, TX_OR);
    }

    if (event & ARM_USART_EVENT_RX_OVERFLOW)
    {
        /* Receive RX Overflow: Wake-up Thread. */
        tx_event_flags_set(&event_flags_uart, UART_CB_RX_OVERFLOW, TX_OR);
    }
}

/**
 * @function        void myUART_Thread_entry(ULONG thread_input)
 * @brief       TestApp to verify UART interface
 *          UART interactive console application:
 *              if UART Receives 'Enter' key on serial terminal:
 *                  UART Sends "Hello World!" back to serial terminal.
 *              else UART Receives any other character on serial terminal:
 *                  UART Sends received character back to serial terminal.
 * @note        none
 * @param       argument
 * @retval      none
 */
void myUART_Thread_entry(ULONG thread_input)
{
    CHAR  cmd    = 0;
    INT   ret    = 0;
    ULONG events = 0;
    ULONG temp = 0;
    ARM_DRIVER_VERSION version;

#if (RTE_UART2_CLK_SOURCE == 0) /* CLK_38.4MHz */
    uint32_t service_error_code;
    uint32_t error_code = SERVICES_REQ_SUCCESS;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                           /*clock_enable_t*/ CLKEN_HFOSC,
                           /*bool enable   */ true,
                                              &service_error_code);
    if(error_code)
        printf("SE: clk enable = %d\n", error_code);
#endif /* CLK_38.4MHz */

    printf("\r\n >>> UART testApp starting up!!!...<<< \r\n");

    version = USARTdrv->GetVersion();
    printf("\r\n UART version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize UART hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART hardware_init.\r\n");
        return;
    }

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(myUART_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Initialize.\r\n");
        return;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Power Up.\r\n");
        goto error_uninitialize;
    }

    /* Configure UART to 115200 Bits/sec */
    ret =  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                ARM_USART_DATA_BITS_8 |
                                ARM_USART_PARITY_NONE |
                                ARM_USART_STOP_BITS_1 |
                                ARM_USART_FLOW_CONTROL_NONE, 115200);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }

    /* Enable Receiver and Transmitter lines */
    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Control TX.\r\n");
        goto error_poweroff;
    }

    ret =  USARTdrv->Control(ARM_USART_CONTROL_RX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Control RX.\r\n");
        goto error_poweroff;
    }

    /* Press 'Enter' or any character on serial terminal to receive a message */
    printf("\r\n Press Enter or any character on serial terminal to receive a message:\r\n");

    ret = USARTdrv->Send("\nPress Enter or any character to receive a message\n", 51);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Send.\r\n");
        goto error_poweroff;
    }

    /* wait till Send complete event comes in isr callback */
    ret = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf(" \r\n Error in tx_event_flags_get.\r\n");
        goto error_poweroff;
    }

    while(1)
    {
        /* clear event flag (if set) before UART call */
        tx_event_flags_get(&event_flags_uart, (UART_CB_RX_EVENT | UART_CB_RX_TIMEOUT), \
                           TX_OR_CLEAR, &temp, TX_NO_WAIT);

        /* Get byte from UART */
        ret = USARTdrv->Receive(&cmd, 1);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error in UART Receive.\r\n");
            goto error_poweroff;
        }

        /* wait till Receive complete or Receive timeout event comes in isr callback */
        ret = tx_event_flags_get(&event_flags_uart, (UART_CB_RX_EVENT | UART_CB_RX_TIMEOUT | UART_CB_RX_BREAK ), \
                                 TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf(" \r\n Error in tx_event_flags_get.\r\n");
            goto error_poweroff;
        }

        /* check for any RX error. */
        if (events & (UART_CB_RX_BREAK | UART_CB_RX_FRAMING_ERROR | \
                      UART_CB_RX_PARITY_ERROR | UART_CB_RX_OVERFLOW))
        {
            /* clear error flags. */
            if (events & (UART_CB_RX_BREAK))
            {
                printf("\r\n RX Break sequence detected.\r\n");
                tx_event_flags_get(&event_flags_uart, UART_CB_RX_BREAK, TX_OR_CLEAR, &temp, TX_NO_WAIT);
            }

            if (events & (UART_CB_RX_FRAMING_ERROR))
            {
                printf("\r\n RX Framing Error.\r\n");
                tx_event_flags_get(&event_flags_uart, UART_CB_RX_FRAMING_ERROR, TX_OR_CLEAR, &temp, TX_NO_WAIT);
            }

            if (events & (UART_CB_RX_PARITY_ERROR))
            {
                printf("\r\n RX Parity Error.\r\n");
                tx_event_flags_get(&event_flags_uart, UART_CB_RX_PARITY_ERROR, TX_OR_CLEAR, &temp, TX_NO_WAIT);
            }

            if (events & (UART_CB_RX_OVERFLOW))
            {
                printf("\r\n RX Overflow Error.\r\n");
                tx_event_flags_get(&event_flags_uart, UART_CB_RX_OVERFLOW, TX_OR_CLEAR, &temp, TX_NO_WAIT);
            }
        }

        if (events & (UART_CB_RX_EVENT))
        {
            if (cmd == 13) /* received 'Enter', send back "Hello World!". */
            {
                ret = USARTdrv->Send("\nHello World!\n", 14);
            }
            else /* else send back received character. */
            {
                ret = USARTdrv->Send(&cmd, 1);
            }

            /* wait till Send complete event comes in isr callback */
            ret = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
            if (ret != TX_SUCCESS)
            {
                printf(" \r\n Error in tx_event_flags_get.\r\n");
                goto error_poweroff;
            }
        }
    }

error_poweroff:

    printf("\r\n Received Error: Powering off UART peripheral. \r\n");

    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Power OFF.\r\n");
    }

error_uninitialize:

    printf("\r\n Received Error: Uninitializing UART peripheral. \r\n");

    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Uninitialize.\r\n");
    }

#if (RTE_UART2_CLK_SOURCE == 0) /* CLK_38.4MHz */
    /* disable the HFOSC clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                       /*clock_enable_t*/ CLKEN_HFOSC,
                       /*bool enable   */ false,
                                          &service_error_code);
    if(error_code)
        printf("SE: clk enable = %d\n", error_code);
#endif

    printf("\r\n XXX UART demo thread exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
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
    tx_kernel_enter();
}


/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    INT      status  = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted create information.  */

    /* Create the event flags group used by UART thread */
    status = tx_event_flags_create(&event_flags_uart, "event flags UART");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&UART_thread, "UART_thread", myUART_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread1 \n");
        return;
    }

}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
