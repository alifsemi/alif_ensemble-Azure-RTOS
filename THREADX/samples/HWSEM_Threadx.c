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
 * @file     HWSEM_ThreadX.c
 * @author   Khushboo Singh
 * @email    khushboo.singh@alifsemi.com
 * @version  V1.0.0
 * @date     27-June-2022
 * @brief    TestApp to verify HWSEM interface with AzureRTOS.
 *             It locks the HWSEM, sends the message through UART and then unlocks.
 *               This application would need to run on two cores
 *               to demonstrate the working of the Hardware Semaphore.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes --------------------------------------------------------------------------- */

/* System Includes */
#include <stdio.h>
#include <string.h>
#include "tx_api.h"

/* Project Includes */

/* include for HwSem Driver */
#include "RTE_Components.h"
#include CMSIS_device_header

/* include for UART Driver */
#include "Driver_USART.h"
/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"
/* HWSEM Driver */
#include "Driver_HWSEM.h"

#ifdef M55_HP
const char * startup_msg = "\n<<<<M55_HP : HWSEM testApp starting up>>>>\r\n";
const char * msg = "\nPrinting from M55_HP";
const char * acq_msg = "\nM55_HP acquiring the semaphore, printing 10 messages\r\n";
const char * rel_msg = "\nM55_HP releasing the semaphore\r\n\n";
#else
const char * startup_msg = "\n<<<<M55_HE : HWSEM testApp starting up>>>>\r\n";
const char * msg = "\nPrinting from M55_HE";
const char * acq_msg = "\nM55_HE acquiring the semaphore, printing 10 messages\r\n";
const char * rel_msg = "\nM55_HE releasing the semaphore\r\n\n";
#endif


/*Define for Threadx*/
#define DEMO_STACK_SIZE                        1024
#define DEMO_BYTE_POOL_SIZE                    9120

TX_THREAD               HWSEM_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_uart;
TX_EVENT_FLAGS_GROUP    event_flags_hwsem;


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

void _sys_exit(int return_code) {
        while (1);
}


int _sys_open(void *p){

   return 0;
}


int _sys_close(void *p){

   return 0;
}


int _sys_read(void *p){

   return 0;
}

int _sys_write(void *p){

   return 0;
}

int _sys_istty(void *p){

   return 0;
}

int _sys_seek(void *p){

   return 0;
}

int _sys_flen(void *p){

    return 0;
}

void _ttywrch(int ch){

}

#endif /* DISABLE_SEMIHOSTING */

#define UART_CB_TX_EVENT          0x01

#define HWSEM_CB_EVENT            0x01

/* HWSEM Driver instance */
#define HWSEM                     0
/* Mention the Uart instance */
#define UART                      6

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/* HWSEM Driver */
extern ARM_DRIVER_HWSEM ARM_Driver_HWSEM_(HWSEM);

/* UART Driver instance */
static ARM_DRIVER_HWSEM *HWSEMdrv = &ARM_Driver_HWSEM_(HWSEM);


void Hwsem_Thread_0_entry(ULONG thread_input);


/**
 * @function    void myUART_callback(uint32_t event)
 * @brief       UART isr callabck
 * @note        none
 * @param       event: USART Event
 * @retval      none
 */
void myUART_callback(uint32_t event)
{
    if (event & ARM_USART_EVENT_SEND_COMPLETE)
    {
        /* Send Success */
        tx_event_flags_set(&event_flags_uart, UART_CB_TX_EVENT, TX_OR);
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {

    }

    if (event & ARM_USART_EVENT_RX_TIMEOUT)
    {

    }
}

/**
 * @function   void HWSEM_callback(uint32_t event)
 * @brief      HWSEM isr callabck
 * @note       none
 * @param      event: HWSEM Event
 * @retval     none
 */
void myHWSEM_callback(int32_t event, uint8_t sem_id)
{
    if (event & HWSEM_AVAILABLE_CB_EVENT)
    {
        /* Success: Wakeup Thread */
        tx_event_flags_set(&event_flags_hwsem, HWSEM_CB_EVENT, TX_OR);
    }
}

/**
 * @function    int32_t hardware_init(void)
 * @brief       Uart initialization
 * @note        none
 * @param       void
 * @retval      execution status
 */
int32_t hardware_init(void)
{
    int32_t ret = ARM_DRIVER_OK;
    ARM_DRIVER_VERSION version;

    /* PINMUX UART6_A */

    /* Configure GPIO Pin : P1_14 as UART6_RX_A */
    ret = PINMUX_Config(PORT_NUMBER_1, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_1);

    if (ret != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    /* PINMUX UART6_A */

    /* Configure GPIO Pin : P1_15 as UART6_TX_A */
    ret = PINMUX_Config(PORT_NUMBER_1, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_1);

    if (ret != ARM_DRIVER_OK)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(myUART_callback);

    if (ret)
    {
        printf("\r\n Error in UART Initialize.\r\n");
        return ARM_DRIVER_ERROR;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);

    if (ret)
    {
        printf("\r\n Error in UART Power Up.\r\n");
        goto error_uninitialize;
    }

    /* Configure UART */
    ret =  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                      ARM_USART_DATA_BITS_8 |
                                      ARM_USART_PARITY_NONE |
                                      ARM_USART_STOP_BITS_1 |
                                      ARM_USART_FLOW_CONTROL_NONE,
                                      115200);

    if (ret)
    {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }

    /* Enable Transmitter lines */
    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if (ret)
    {
        printf("\r\n Error in UART Control.\r\n");
        goto error_poweroff;
    }
    return ret;

error_poweroff:
    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Power OFF.\r\n");
    }

error_uninitialize:
    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in UART Uninitialize.\r\n");
    }
    return ret;
}

/**
 * @function    void Hwsem_Thread_0_entry(ULONG thread_input)
 * @brief       TestApp to verify HWSEM interface
 *              Get the lock, send message through UART
 *              and then unlock
 * @note        none
 * @param       none
 * @retval      none
 */
void Hwsem_Thread_0_entry(ULONG thread_input)
{
    INT ret_hwsem, ret_uart, len, init = 1, ret;
    CHAR * uart_msg;
    ULONG events_uart = 0, events_hwsem, timer_ticks = 100;
    ARM_DRIVER_VERSION version;

    ret = tx_byte_allocate(&byte_pool_0, (void*)&uart_msg, 30, 500);

    if (ret == TX_NO_MEMORY)
    {
        printf("\r\nNo memory left in Block POOL\n");
        return;
    }

    version = HWSEMdrv->GetVersion();
    printf("\r\n HWSEM version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize HWSEM driver */
    ret_hwsem = HWSEMdrv->Initialize(myHWSEM_callback);
    if (ret_hwsem != ARM_DRIVER_OK)
    {
        printf("\r\n HWSEM initialization failed\r\n");
        goto error_initialize;
    }

    while(1)
    {
        /* Acquire the lock */
        ret_hwsem = HWSEMdrv->Lock();

        if (ret_hwsem == ARM_DRIVER_ERROR)
        {
            printf("\r\n HWSEM lock failed\r\n");
            goto error_lock;
        }

        /* If HWSEM already locked, then wait for the interrupt */
        while (ret_hwsem == ARM_DRIVER_ERROR_BUSY)
        {
            ret_hwsem = tx_event_flags_get(&event_flags_hwsem, HWSEM_CB_EVENT, TX_OR_CLEAR, &events_hwsem, TX_WAIT_FOREVER);

            if (ret_hwsem == TX_SUCCESS)
            {
                /* Acquire the lock */
                ret_hwsem = HWSEMdrv->Lock();
            }
            else
            {
                ret_hwsem = ARM_DRIVER_ERROR_BUSY;
            }
        }

        /* Initialize the UART Driver */
        if (init)
        {
            hardware_init();
            if (ret_uart != 0)
            {
                printf("\r\n Error in UART hardware_init.\r\n");
                goto error_hw_init;
            }
            init = 0;
            ret_uart = USARTdrv->Send(startup_msg, strlen(startup_msg));
            if (ret_uart != ARM_DRIVER_OK)
            {
                printf("\r\nError in UART Send.\r\n");
            }

            /* wait for event flag after UART call */
            ret_uart = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);
            if (ret_uart != TX_SUCCESS)
            {
                printf(" \r\n Error in tx_event_flags_get.\r\n");
            }
        }


        ret_uart = USARTdrv->Send(acq_msg, strlen(acq_msg));
        if (ret_uart != ARM_DRIVER_OK)
        {
            printf("\r\nError in UART Send.\r\n");
        }

        /* wait for event flag after UART call */
        ret_uart = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);
        if (ret_uart != TX_SUCCESS)
        {
            printf(" \r\n Error in tx_event_flags_get.\r\n");
        }

        /* Print 10 messages from each core */
        for (int iter = 1; iter <= 10; iter++)
        {
            len = sprintf(uart_msg, "%s %d\r\n", msg, iter);

            ret_uart = USARTdrv->Send(uart_msg, len);
            if (ret_uart != ARM_DRIVER_OK)
            {
                printf("\r\nError in UART Send.\r\n");
            }

            /* wait for event flag after UART call */
            ret_uart = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);
            if (ret_uart != TX_SUCCESS)
            {
                printf(" \r\n Error in tx_event_flags_get.\r\n");
            }
            tx_thread_sleep(timer_ticks);
        }

        ret_uart = USARTdrv->Send(rel_msg, strlen(rel_msg));
        if (ret_uart != ARM_DRIVER_OK)
        {
            printf("\r\nError in UART Send.\r\n");
        }

        /* wait for event flag after UART call */
        ret_uart = tx_event_flags_get(&event_flags_uart, UART_CB_TX_EVENT, TX_OR_CLEAR, &events_uart, TX_WAIT_FOREVER);
        tx_thread_sleep(timer_ticks);

        if (ret_uart != TX_SUCCESS)
        {
            printf(" \r\n Error in tx_event_flags_get.\r\n");
        }

        /* Unlock the HW Semaphore */
        HWSEMdrv->UnLock();

        if (ret_hwsem == ARM_DRIVER_ERROR)
        {
            printf("\r\n HWSEM unlock failed\r\n");
            goto error_lock;
        }

        tx_thread_sleep(timer_ticks);
    }
    ret = tx_block_release(uart_msg);

    /* Uninitialize the HWSEM Driver */
    HWSEMdrv->Uninitialize();

error_hw_init:
    /* Unlock the HW Semaphore */
    HWSEMdrv->UnLock();
error_lock:
    /* Uninitialize the HWSEM Driver */
    HWSEMdrv->Uninitialize();
    tx_thread_sleep(timer_ticks);
    tx_block_release(uart_msg);
error_initialize:
    return;
}

//* Define main entry point.  */
int main()
{
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
    /* Create the event flags group used by UART thread */
    status = tx_event_flags_create(&event_flags_uart, "event flags UART");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the event flags group used by HWSEM thread */
    status = tx_event_flags_create(&event_flags_hwsem, "event flags HWSEM");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not allocate memory for thread stack.\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&HWSEM_thread, "HWSEM_thread", Hwsem_Thread_0_entry, 0, pointer, DEMO_STACK_SIZE,
                                                                      1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create HWSEM demo thread \n");
        return;
    }

}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
