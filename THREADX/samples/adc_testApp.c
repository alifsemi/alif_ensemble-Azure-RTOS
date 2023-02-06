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
 * @file     : adc_testApp.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 22-Feb-2022
 * @brief    : TestApp to verify ADC Driver using Threadx as an operating system.
 *              - Input in analog signal corresponding output is digital value.
 *              - Converted digital value are stored in user provided memory
 *                address.
 *
 *              Hardware setup:
 *              - GPIO port P0_0  to P0_5  pins are reserved for ADC instance 0 input.
 *              - GPIO port P0_6  to P0_11 pins are reserved for ADC instance 1 input.
 *              - GPIO port P0_12 to P0_17 pins are reserved for ADC instance 2 input.
 *              - Jumper setting: Short J411 (VREFN) for the negative reference voltage.
 *
 *              Single channel Scan (selective channel scan)
 *              - User can select the particular channel using
 *                 ARM_ADC_CHANNEL_#, where # denotes the channel number.
 *
 *              Continuous channel Scan (Multiple channel scan)
 *              - it rotate through all the ADC channels and
 *                 continuously stores the value in given memory buffer.
 *              - User can skip channels using ARM_ADC_MASK_CHANNEL_# macro.
 *
 *              Comparator
 *              - comparing channels for both the scan for below threshold
 *                • Above / below threshold A
 *                • Above / below threshold B
 *                • Between/outside threshold A and B
 *
 *              ADC configurations for Demo testApp:
 *                Single channel scan(Default scan)
 *                 - GPIO pin P0_12 & P0_13 are connected to 2 channel Regulated DC Power supply.
 *                    DC Power supply:
 *                     - first  Channel, +ve connected to P0_12 (ADC2 channel 0) at 1.0V
 *                     - second Channel, +ve connected to P0_13 (ADC2 channel 1) at 0.4V
 *
 *                Continuous Channel scan
 *                - Used ADC instance 2, pin from P0_12 to P0_17 all are connected to dc supply
 *                - channel 0 and 1 are masked using MASK_CHANNEL macro.
 *                address.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "tx_api.h"
#include "system_utils.h"

/* include for ADC Driver */
#include "Driver_ADC.h"

/* single     channel scan use ARM_ADC_CH_SINGLE_SCAN
 * continuous channel scan use ARM_ADC_CH_CONTINOUS_SCAN
 */
#define ADC_SCAN    ARM_ADC_CH_SINGLE_SCAN
//#define ADC_SCAN    ARM_ADC_CH_CONTINOUS_SCAN

/* For Release build disable printf and semihosting */
//#define DISABLE_PRINTF

#ifdef DISABLE_PRINTF
    #define printf(fmt, ...) (0)
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
#endif

/* ADC Driver instance */
extern ARM_DRIVER_ADC Driver_ADC2;
static ARM_DRIVER_ADC *ADCdrv = &Driver_ADC2;

#define COMP_A_THLD_VALUE                   (0X00)                                                /* Comparator A threshold value */
#define COMP_B_THLD_VALUE                   (0x00)                                                /* Comparator B threshold value */
#define MASK_CHANNEL                        (ARM_ADC_MASK_CHANNEL_0 | ARM_ADC_MASK_CHANNEL_1)     /* Masking particular channels  */

void adc_demo_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define DEMO_BYTE_POOL_SIZE             9120
#define TX_ADC_INT_AVG_SAMPLE_RDY       0x01

TX_THREAD               adc_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_adc;

/* store comparator result */
uint32_t comp_value[6] = {0};

#define TOTAL_SAMPLES             (16 * 1024) /* 16K samples */

/* Demo purpose memory buffer */
uint32_t memory_buff[TOTAL_SAMPLES];

/*
 *    @func        : void adc_conversion_callback(uint32_t event)
 *    @brief       : adc conversion isr callback
 *.   @return      : NONE
*/
static void adc_conversion_callback(uint32_t event)
{
    if (event & ARM_ADC_EVENT_CONVERSION_COMPLETE)
    {
        /* Sample ready Wake-up Thread. */
        tx_event_flags_set(&event_flags_adc, TX_ADC_INT_AVG_SAMPLE_RDY, TX_OR);
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A)
    {
        comp_value[0] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B)
    {
        comp_value[1] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A)
    {
        comp_value[2] += 1;
    }
    if (event & ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B)
    {
        comp_value[3] += 1;
    }
    if(event & ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B)
    {
        comp_value[4] += 1;
    }
    if(event & ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B)
    {
        comp_value[5] += 1;
    }
}

/**
 *    @func         : void adc_demo_Thread_entry(ULONG thread_input)
 *    @brief        : ADC demo thread
 *                  - test to verify the adc.
 *                  - input analog signal corresponding convert into digital value
 *                  - converted value is the allocated user memory address.
 *    @parameter[1] : thread_input : thread input
 *    @return       : NONE
*/
void adc_demo_Thread_entry(ULONG thread_input)
{

    ULONG events   = 0;
    INT   ret      = 0;
    ARM_DRIVER_VERSION version;
    ARM_ADC_CAPABILITIES capabilities;

    printf("\r\n >>> ADC demo threadX starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize ADC driver */
    ret = ADCdrv->Initialize(adc_conversion_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC init failed\n");
        return;
    }

    /* Power control ADC */
    ret = ADCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

#if (ADC_SCAN == ARM_ADC_CH_CONTINOUS_SCAN)
    /* set sequencer controller */
    ret = ADCdrv->Control(ARM_ADC_SEQUENCER_CTRL, ARM_ADC_CH_CONTINOUS_SCAN);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC sequencer controller failed\n");
        goto error_uninitialize;
    }

    /* Masking the channel */
    ret = ADCdrv->Control(ARM_ADC_SEQUENCER_MSK_CH_CTRL, MASK_CHANNEL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC sequencer masking channel failed\n");
        goto error_uninitialize;
    }
#else /*single scan mode */
    /* set sequencer controller */
    ret = ADCdrv->Control(ARM_ADC_SEQUENCER_CTRL, ARM_ADC_CH_SINGLE_SCAN);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC sequencer controller failed\n");
        goto error_uninitialize;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_0);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC channel failed\n");
        goto error_uninitialize;
    }
#endif

    /* set comparator a value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_A, COMP_A_THLD_VALUE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Comparator A failed\n");
        goto error_uninitialize;
    }

    /* set comparator b value */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_B, COMP_B_THLD_VALUE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Comparator B failed\n");
        goto error_uninitialize;
    }

    /* select the threshold comparison */
    ret = ADCdrv->Control(ARM_ADC_THRESHOLD_COMPARISON, ARM_ADC_ABOVE_A_AND_ABOVE_B);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Threshold comparison failed\n");
        goto error_uninitialize;
    }

    /* ENABLE or DISABLE the comparator */
    ret = ADCdrv->Control(ARM_ADC_COMPARATOR_CONTROLLER , ENABLE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Comparator failed\n");
        goto error_uninitialize;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n",(uint32_t)memory_buff);
    /* Start ADC */
    ret = ADCdrv->Start((uint32_t *)memory_buff, TOTAL_SAMPLES);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Power up failed\n");
        goto error_poweroff;
    }

    /* wait till conversion comes ( isr callback ) */
    ret = tx_event_flags_get(&event_flags_adc, TX_ADC_INT_AVG_SAMPLE_RDY, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS) {
        printf("Error: ADC tx_event_flags_get\n");
        goto error_poweroff;
    }

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Power up failed\n");
        goto error_poweroff;
    }

    printf("\n >>> ADC conversion completed \n");
    printf(" Converted value are stored in user allocated memory address.\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);

error_poweroff:

        /* Power off ADC peripheral */
        ret = ADCdrv->PowerControl(ARM_POWER_OFF);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: ADC Power OFF failed.\r\n");
        }

error_uninitialize:

        /* Un-initialize ADC driver */
        ret = ADCdrv->Uninitialize();
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: ADC Uninitialize failed.\r\n");
        }

        printf("\r\n XXX ADC demo thread exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
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

    /* Create the event flags group used by ADC thread */
    status = tx_event_flags_create(&event_flags_adc, "event flags ADC");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&adc_thread, "adc_thread", adc_demo_Thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
