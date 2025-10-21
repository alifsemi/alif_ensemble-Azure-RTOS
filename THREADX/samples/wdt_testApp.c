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
 * @file     wdt_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     30-April-2021
 * @brief    TestApp to verify Wachdog Driver using Threadx as an operating system.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */
/* System Includes */
#include "tx_api.h"
#include "stdio.h"

/* Project Includes */
/* include for watchdog Driver */
#include "Driver_WDT.h"
#include "app_utils.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif  /* RTE_CMSIS_Compiler_STDOUT */


/* watchdog Driver instance 0 */
extern ARM_DRIVER_WDT Driver_WDT0;
static ARM_DRIVER_WDT *WDTdrv = &Driver_WDT0;

void watchdog_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE         1024
#define DEMO_BYTE_POOL_SIZE     9120

TX_THREAD               watchdog_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];


void NMI_Handler(void)
{
    printf("\r\n NMI_Handler: Received Interrupt from Watchdog! \r\n");
    WAIT_FOREVER_LOOP
}

/**
  \fn          void watchdog_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify watchdog peripheral,
               This demo thread does:
                 - initialize watchdog with timeout value
                 - start the watchdog;
                 - feed  the watchdog;
                 - stop feeding after n iterations;
                 - wait for system to RESET on timeout
  \param[in]   thread_input : thread input
  \return      none
*/
void watchdog_demo_thread_entry(ULONG thread_input)
{
    UINT wdog_timeout_msec = 0; /* watchdog timeout value in msec        */
    UINT time_to_reset = 0;     /* watchdog remaining time before reset. */
    UINT iter = 3;
    INT  ret = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> watchdog demo threadx starting up!!! <<< \r\n");

    version = WDTdrv->GetVersion();
    printf("\r\n watchdog version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Watchdog timeout is set to 5000 msec (5 sec). */
    wdog_timeout_msec = 5000;

    /* Initialize watchdog driver */
    ret = WDTdrv->Initialize(wdog_timeout_msec);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog init failed\n");
        return;
    }

    /* Power up watchdog peripheral */
    ret = WDTdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog Power up failed\n");
        goto error_uninitialize;
    }

    /* Watchdog initialize will lock the timer, unlock it to change the register value. */
    ret = WDTdrv->Control(ARM_WATCHDOG_UNLOCK, 0);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog unlock failed\n");
        goto error_stop;
    }

    /* Start the watchDog Timer. */
    ret = WDTdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog start failed\n");
        goto error_stop;
    }

    while(iter--)
    {
        /* Sleep for 3 sec. */
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 3);

        /* Get watchdog remaining time before reset. */
        ret = WDTdrv->GetRemainingTime(&time_to_reset);
        if(ret != ARM_DRIVER_OK){
            printf("\r\n Error: watchdog get remaining time failed\n");
            goto error_stop;
        }

        printf("\r\n Feed the WatchDog: %d...\r\n",iter);
        ret = WDTdrv->Feed();
        if(ret != ARM_DRIVER_OK){
            printf("\r\n Error: watchdog feed failed\n");
            goto error_stop;
        }
    }

    printf("\r\n now stop feeding to the watchdog, system will RESET on timeout. \r\n");
    WAIT_FOREVER_LOOP


error_stop:
    /* First Unlock and then Stop watchdog peripheral. */
    ret = WDTdrv->Control(ARM_WATCHDOG_UNLOCK, 0);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog unlock failed\n");
    }

    /* Stop watchdog peripheral */
    ret = WDTdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog Stop failed.\r\n");
    }

error_poweroff:
    /* Power off watchdog peripheral */
    ret = WDTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog Power OFF failed.\r\n");
    }

error_uninitialize:
    /* Un-initialize watchdog driver */
    ret = WDTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: watchdog Uninitialize failed.\r\n");
    }

    printf("\r\n XXX watchdog demo thread exiting XXX...\r\n");
}


/* Define main entry point.  */
int main()
{
    #if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t ret;
    ret = stdout_init();
    if(ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
    #endif

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


/* Define what the initial system looks like.  */

void tx_application_define(void *first_unused_memory)
{
    CHAR *pointer = TX_NULL;
    INT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not allocate memory for thread stack.\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&watchdog_thread, "watchdog_thread", watchdog_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create watchdog demo thread\n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
