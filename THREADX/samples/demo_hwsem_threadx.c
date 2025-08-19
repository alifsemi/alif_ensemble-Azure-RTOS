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
 * @file     : demo_hwsem_threadx.c
 * @author   : Khushboo Singh
 * @email    : khushboo.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 27-June-2022
 * @brief    TestApp to verify HWSEM interface with AzureRTOS.
 *             It locks the HWSEM, sends the message through UART and then unlocks.
 *             This application would need to run on two cores
 *             to demonstrate the working of the Hardware Semaphore.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "tx_api.h"

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_HWSEM.h"
#include "Driver_USART.h"
#include "board_config.h"
#include "app_utils.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#if defined(RTSS_HP)
const char *msg     = "\nPrinting from RTSS_HP";
const char *acq_msg = "\nRTSS_HP acquiring the semaphore, printing 10 messages\r\n";
const char *rel_msg = "\nRTSS_HP releasing the semaphore\r\n\n";
#else
const char *msg     = "\nPrinting from RTSS_HE";
const char *acq_msg = "\nRTSS_HE acquiring the semaphore, printing 10 messages\r\n";
const char *rel_msg = "\nRTSS_HE releasing the semaphore\r\n\n";
#endif

#define DEMO_STACK_SIZE 1024

TX_THREAD               hwsem_thread;
TX_EVENT_FLAGS_GROUP    event_flags_hwsem;

#define HWSEM_CB_EVENT  0x01

/* HWSEM Driver instance */
#define HWSEM           0
/* UART driver instance */
#define UART            4

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/* HWSEM Driver */
extern ARM_DRIVER_HWSEM ARM_Driver_HWSEM_(HWSEM);
static ARM_DRIVER_HWSEM *HWSEMdrv = &ARM_Driver_HWSEM_(HWSEM);

#if !RTE_UART4_BLOCKING_MODE_ENABLE

#define UART_CB_TX_EVENT  0x01
TX_EVENT_FLAGS_GROUP      event_flags_uart;

/**
 * @function    static VOID myUART_callback(UINT event)
 * @brief       UART isr callabck
 * @note        none
 * @param       event: USART Event
 * @retval      none
 */
static VOID myUART_callback(UINT event)
{
    if (event & ARM_USART_EVENT_SEND_COMPLETE) {
        tx_event_flags_set(&event_flags_uart, UART_CB_TX_EVENT, TX_OR);
    }
}
#endif

/**
 * @function   static void HWSEM_callback(INT event, UCHAR sem_id)
 * @brief      HWSEM isr callabck
 * @note       none
 * @param      event: HWSEM Event, sem_id : HWSEM id
 * @retval     none
 */
static VOID myHWSEM_callback(INT event, UCHAR sem_id)
{
    (VOID) sem_id;

    if (event & HWSEM_AVAILABLE_CB_EVENT) {
        tx_event_flags_set(&event_flags_hwsem, HWSEM_CB_EVENT, TX_OR);
    }
}

/**
 * @function    static void hwsem_demo_entry(ULONG thread_input)
 * @brief       TestApp to verify HWSEM interface
 *              Get the lock, send message through UART
 *              and then unlock
 * @note        none
 * @param       thread_input
 * @retval      none
 */
static void hwsem_demo_entry(ULONG thread_input)
{
    INT len, init = 1, ret;
    UINT status;
    CHAR uart_msg[30];
    ULONG events_uart = 0, events_hwsem = 0, timer_ticks = 100;
    ARM_DRIVER_VERSION version;

    (VOID) thread_input;

    version = HWSEMdrv->GetVersion();

    printf("\r\n HWSEM version api:%X driver:%X...\r\n", version.api, version.drv);

    /* Initialize HWSEM driver */
    ret = HWSEMdrv->Initialize(myHWSEM_callback);

    if (ret != ARM_DRIVER_OK) {
        printf("\r\n HWSEM initialization failed\r\n");
        goto error_exit;
    }

    while (1) {
        /* Acquire the lock */
        ret = HWSEMdrv->TryLock();

        if (ret == ARM_DRIVER_ERROR) {
            printf("\r\n HWSEM lock failed\r\n");
            goto error_uninitialize;
        }

        /* If HWSEM already locked, then wait for the interrupt */
        while (ret == ARM_DRIVER_ERROR_BUSY) {
            status = tx_event_flags_get(&event_flags_hwsem, HWSEM_CB_EVENT,
                                        TX_OR_CLEAR, &events_hwsem, TX_WAIT_FOREVER);

            if (status == TX_SUCCESS) {
                /* Acquire the lock */
                ret = HWSEMdrv->TryLock();
            }
        }

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n HWSEM lock failed\n");
            goto error_uninitialize;
        }

        /* Initialize the pinmux in the first iteration */
        if (init) {
            /* pin mux and configuration for all device IOs requested from pins.h*/
            ret = board_pins_config();
            if (ret != 0) {
                printf("Error in pin-mux configuration: %d\n", ret);
                goto error_unlock;
            }
            init = 0;
        }

#if !RTE_UART4_BLOCKING_MODE_ENABLE
        /* Initialize UART driver */
        ret = USARTdrv->Initialize(myUART_callback);
#else
        /* Initialize UART driver */
        ret = USARTdrv->Initialize(NULL);
#endif

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Initialize.\r\n");
            goto error_unlock;
        }

        /* Power up UART peripheral */
        ret = USARTdrv->PowerControl(ARM_POWER_FULL);

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Power Up.\r\n");
            goto error_unlock;
        }

        /* Configure UART */
        ret =  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                 ARM_USART_DATA_BITS_8       |
                                 ARM_USART_PARITY_NONE       |
                                 ARM_USART_STOP_BITS_1       |
                                 ARM_USART_FLOW_CONTROL_NONE,
                                 115200);

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Control.\r\n");
            goto error_unlock;
        }

        /* Enable UART tx */
        ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Control.\r\n");
            goto error_unlock;
        }

        ret = USARTdrv->Send(acq_msg, strlen(acq_msg));

        if (ret != ARM_DRIVER_OK) {
            printf("\r\nError in UART Send.\r\n");
            goto error_unlock;
        }

#if !RTE_UART4_BLOCKING_MODE_ENABLE
        /* wait for event flag after UART call */
        status = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT,
                                    TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);

        if (status != TX_SUCCESS) {
            printf(" \r\n Error in tx_event_flags_get.\r\n");
            goto error_unlock;
        }
#endif
        /* Print 10 messages */
        for (int iter = 1; iter <= 10; iter++) {
            len = sprintf(uart_msg, "%s %d\r\n", msg, iter);

            ret = USARTdrv->Send(uart_msg, len);

            if (ret != ARM_DRIVER_OK) {
                printf("\r\nError in UART Send.\r\n");
                goto error_unlock;
            }

#if !RTE_UART4_BLOCKING_MODE_ENABLE
            /* wait for event flag after UART call */
            status = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT,
                                        TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);

            if (status != TX_SUCCESS) {
                printf(" \r\n Error in tx_event_flags_get.\r\n");
                goto error_unlock;
            }
#endif
            tx_thread_sleep(timer_ticks);
        }
        ret = USARTdrv->Send(rel_msg, strlen(rel_msg));

        if (ret != ARM_DRIVER_OK) {
            printf("\r\nError in UART Send.\r\n");
            goto error_unlock;
        }

#if !RTE_UART4_BLOCKING_MODE_ENABLE
        /* wait for event flag after UART call */
        status = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT,
                                    TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);

        if (status != TX_SUCCESS) {
            printf(" \r\n Error in tx_event_flags_get.\r\n");
            goto error_unlock;
        }
#endif
        ret = USARTdrv->PowerControl(ARM_POWER_OFF);

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Power OFF.\r\n");
            goto error_unlock;
        }

        ret = USARTdrv->Uninitialize();

        if (ret != ARM_DRIVER_OK) {
            printf("\r\n Error in UART Uninitialize.\r\n");
            goto error_unlock;
        }

        /* Unlock the HW Semaphore */
        ret = HWSEMdrv->Unlock();

        if (ret == ARM_DRIVER_ERROR) {
            printf("\r\n HWSEM unlock failed\r\n");
            goto error_uninitialize;
        }

        tx_thread_sleep(timer_ticks);
    }

    tx_block_release(uart_msg);

    HWSEMdrv->Uninitialize();

    return;

error_unlock:
    /* Unlock the HW Semaphore */
    HWSEMdrv->Unlock();

error_uninitialize:
    /* Uninitialize the HWSEM Driver */
    HWSEMdrv->Uninitialize();

    tx_block_release(uart_msg);

error_exit:
    return;
}

int main()
{
    int32_t ret;

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Enter the ThreadX kernel. */
    tx_kernel_enter();
}

/* Define what the initial system looks like. */
void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    UINT     status;

#if !RTE_UART4_BLOCKING_MODE_ENABLE
    /* Create the event flags group used by UART thread */
    status = tx_event_flags_create(&event_flags_uart, "event flags UART");

    if (status != TX_SUCCESS) {
        printf("Could not create event flags\n");
        return;
    }
#endif
    /* Create the event flags group used by HWSEM thread */
    status = tx_event_flags_create(&event_flags_hwsem, "event flags HWSEM");

    if (status != TX_SUCCESS) {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread. */
    status = tx_thread_create(&hwsem_thread, "HWSEM_thread", hwsem_demo_entry, 0,
                              first_unused_memory, DEMO_STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) {
        printf("Could not create HWSEM demo thread \n");
        return;
    }
}
