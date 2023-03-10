/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     dac_testapp.c
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     22-Feb-2022
 * @brief    TestApp to verify DAC Driver using Threadx as an operating system.
             -As DAC is 12 bit resolution, If the input value is maximum than the
                maximum DAC input value(0xFFF)then the input value will be set to
                DAC maximum input value.
             -And If the input value is equal to maximum dac input value then
                input value will be set to 0.
             -If the input value is not greater than the maximum dac input value
               then input value will be incremented by 1000.

             Hardware Setup :
              -when the application uses DAC0 channel,then connect DAC0 to P0_18
               GPIO pin,according to DAC input the output will be observed in P0_18
               GPIO pin through the logic analyzer.

              -And when the application uses DAC1 channel,then connect DAC1 to
               P0_19 GPIO pin,according to DAC input the output will be observed
               in P0_19 GPIO pin through the logic analyzer.
 ******************************************************************************/
/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* include for DAC Driver */
#include "Driver_DAC.h"

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

/* DAC Driver instance */
extern ARM_DRIVER_DAC Driver_DAC0;
static ARM_DRIVER_DAC *DACdrv = &Driver_DAC0;

/* DAC maximum resolution is 12-bit */
#define DAC_MAX_INPUT_VALUE   (0xFFF)

void dac_demo_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks... */
#define DEMO_STACK_SIZE                 1024
#define DEMO_BYTE_POOL_SIZE             9120

TX_THREAD               dac_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];

/**
 @fn           void dac_demo_Thread_entry(ULONG thread_input)
 @brief        DAC demo Thread:
               This thread initializes the DAC. And then in a loop,
               according to the input value, the output will change.
               If the input value is maximum than the maximum DAC input
               value then the input value will be set to DAC maximum input value.
               And If the input value is equal to maximum dac input value then
               input value will be set to 0.If the input value is not greater
               than the maximum dac input value then input value will be incremented by 1000.
 @param[in]    thread_input : thread input
 @return       none
*/
void dac_demo_Thread_entry(ULONG thread_input)
{
    UINT input_value = 0;
    INT  ret         = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n >>> DAC demo threadX starting up!!! <<< \r\n");

    version = DACdrv->GetVersion();
    printf("\r\n DAC version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize DAC driver */
    ret = DACdrv->Initialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: DAC init failed\n");
        return;
    }

    /* Enable the power for DAC */
    ret = DACdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: DAC Power up failed\n");
        goto error_uninitialize;
    }

    /* start dac */
    ret = DACdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: DAC Start failed\n");
        goto error_uninitialize;
    }

    input_value = 0;

    while(1)
    {
        /* set dac input */
        ret = DACdrv->SetInput(input_value);
        if(ret != ARM_DRIVER_OK){
            printf("\r\n Error: DAC Set Input failed\n");
            goto error_stop;
        }

        /* Sleep for n second */
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 1);

        /* If the input value is equal to maximum dac input value then input
           value will be set to 0 */
        if(input_value == DAC_MAX_INPUT_VALUE)
        {
            input_value = 0;
        }

        /* If the input value is not greater than the maximum dac input value then input
           value will be incremented by 1000 */
        else
        {
            input_value += 1000;
        }

        /* If the input value is maximum than the maximum DAC input value then the input
           value will be set to DAC maximum input value */
        if(input_value > DAC_MAX_INPUT_VALUE)
        {
            input_value = DAC_MAX_INPUT_VALUE;
        }

    }

error_stop :

    /* Stop the DAC driver */
    ret = DACdrv->Stop();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: DAC Stop failed.\r\n");
    }

error_poweroff:

    /* Power off DAC peripheral */
    ret = DACdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: DAC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize DAC driver */
    ret = DACdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: DAC Uninitialize failed.\r\n");
    }

    printf("\r\n XXX DAC demo thread exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like. */

void tx_application_define(void *first_unused_memory)
{
    CHAR    *pointer = TX_NULL;
    INT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
        create information.  */

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&dac_thread, "dac_thread", dac_demo_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
