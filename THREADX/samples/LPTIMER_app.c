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
 * @file     LPTIMER_app.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     23-April-2021
 * @brief    demo application for lptimer.
 *           - Configuring the lptimer channel 0 for 5 seconds.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "tx_api.h"
#include "Driver_LPTIMER.h"
#include <stdio.h>

#define DEMO_BYTE_POOL_SIZE        (1024)
#define LPTIMER_THREAD_STACK_SIZE  (512)
#define LPTIMER_CALLBACK_EVENT     0x1
#define LPTIMER_CHANNEL_0          0      /* lptimer have 0-3 channels, using channel zero for demo app */
#define LPTIMER_EVENT_WAIT_TIME    (6 * TX_TIMER_TICKS_PER_SECOND)  /* interrupt wait time: 6 seconds */

UCHAR                              memory_area[DEMO_BYTE_POOL_SIZE];
TX_BYTE_POOL                       memory_pool;
TX_THREAD                          lptimer_thread;
TX_EVENT_FLAGS_GROUP               lptimer_event_flag;
ULONG                              events;

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

static void lptimer_cb_fun (uint8_t event)
{
    if (event & ARM_LPTIMER_EVENT_UNDERFLOW) {
        tx_event_flags_set( &lptimer_event_flag, LPTIMER_CALLBACK_EVENT, TX_OR);
    }
}

static void lptimer_app (ULONG thread_input)
{
    extern ARM_DRIVER_LPTIMER DRIVER_LPTIMER0;
    ARM_DRIVER_LPTIMER *ptrDrv = &DRIVER_LPTIMER0;

    /* Configuring the lptimer channel 0 for 5 seconds
     *Clock Source is depends on RTE_LPTIMER_CHANNEL_CLK_SRC in RTE_Device.h
     *RTE_LPTIMER_CHANNEL_CLK_SRC = 0 : 32.768KHz freq (Default)
     *RTE_LPTIMER_CHANNEL_CLK_SRC = 1 : 128KHz freq.
     *
     * Selected clock frequency (F)= 32.768KHz
     *
     * time for 1 count T = 1/F = 1/(32.768*10^3) = 30.51 * 10^-6
     *
     * To increment timer by 1 count, takes 30.51 micro sec
     *
     * So count for 5sec = 5/(30.51 *(10^-6)) = 163880
     *
     * DEC = 163880
     * HEX = 0x28028
    */

    /* Timer channel configured 5 sec */

    int32_t ret;
    uint32_t status, count = 0x28028;
    uint8_t channel = LPTIMER_CHANNEL_0;

    ret = ptrDrv->Initialize (channel, lptimer_cb_fun);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: channel '%d'failed to initialize\r\n", channel);
        return;
    }

    ret = ptrDrv->PowerControl (channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: channel '%d'failed to power up\r\n", channel);
        goto error_uninstall;
    }

    /**< Loading the counter value >*/
    ret = ptrDrv->Control (channel, ARM_LPTIMER_SET_COUNT1, &count);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: channel '%d'failed to load count\r\n", channel);
        goto error_poweroff;
    }

    printf("demo application: lptimer channel '%d'configured for 5 sec \r\n\n", channel);

    ret = ptrDrv->Start (channel);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: failed to start channel '%d' timer\n", channel);
        goto error_poweroff;
    } else {
        printf("timer started\r\n");
    }

    status = tx_event_flags_get (&lptimer_event_flag, LPTIMER_CALLBACK_EVENT, TX_OR_CLEAR, &events, LPTIMER_EVENT_WAIT_TIME);
    if (status != TX_SUCCESS) {
        printf("ERROR : event not received, timeout happened \n");
        goto error_poweroff;
    }

    printf("5 sec timer expired \r\n");

    ret = ptrDrv->Stop(channel);
    if(ret != ARM_DRIVER_OK) {
        printf("ERROR: failed to stop channel %d\n", channel);
    } else {
        printf("timer stopped\r\n\n");
    }

error_poweroff:

    ret = ptrDrv->PowerControl(channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: failed to power off channel '%d'\n", channel);
    }

error_uninstall:

    ret = ptrDrv->Uninitialize(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: failed to un-initialize channel %d\n", channel);
    }

    printf("demo application: completed \r\n");
}

/* Define main entry point.  */
int main ()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define (void *first_unused_memory)
{
    CHAR *pointer = TX_NULL;
    UINT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create (&memory_pool, "memory pool", memory_area, DEMO_BYTE_POOL_SIZE);

    if (status != TX_SUCCESS) {
        printf("failed to create to byte Pool\r\n");
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */
    /* Create a event flag for lptimer group.  */
    status = tx_event_flags_create (&lptimer_event_flag, "LPTIMER_EVENT_FLAG");

    if (status != TX_SUCCESS) {
        printf("failed to create lptimer event flag\r\n");
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate (&memory_pool, (VOID **) &pointer, LPTIMER_THREAD_STACK_SIZE, TX_NO_WAIT);

    if (status != TX_SUCCESS) {
        printf("failed to allocate stack for lptimer thread\r\n");
    }

    /* Create the main thread.  */
    status = tx_thread_create (&lptimer_thread, "LPTIMER DEMO", lptimer_app, 0, pointer, LPTIMER_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS) {
        printf("failed to create lptimer demo thread\r\n");
    }
}
