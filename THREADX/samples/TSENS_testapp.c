/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */
/**************************************************************************//**
 * @file     : TSENS_testapp.c.c
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 21-AUG-2023
 * @brief    : ThreadX demo application code for ADC driver temperature sensor
 *              - Internal input of temperature  in analog signal corresponding
 *                output is digital value.
 *              - Converted digital value are stored in user provided memory
 *                address.
 *
 *            Hardware Connection:
 *            Common temperature sensor is internally connected to ADC12 6th channel
 *            of each instance.
 *            no hardware setup required.
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "tx_api.h"
#include "system_utils.h"

/* include for ADC Driver */
#include "Driver_ADC.h"
#include "temperature.h"

#include "se_services_port.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

#define ADC_CONVERSION    ARM_ADC_SINGLE_SHOT_CH_CONV

void tsens_demo_Thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define TX_ADC_INT_AVG_SAMPLE_RDY       0x01

TX_THREAD               tsens_thread;
TX_EVENT_FLAGS_GROUP    event_flags_adc;

/* Instance for ADC12 */
extern ARM_DRIVER_ADC Driver_ADC122;
static ARM_DRIVER_ADC *ADCdrv = &Driver_ADC122;

#define TEMPERATURE_SENSOR       ARM_ADC_CHANNEL_6
#define NUM_CHANNELS             (8)

/* Demo purpose Channel_value*/
UINT adc_sample[NUM_CHANNELS];

volatile UINT num_samples = 0;

/*
 * @func   : void adc_conversion_callback(uint32_t event, uint8_t channel, uint32_t sample_output)
 * @brief  : adc conversion isr callback
 * @return : NONE
*/
static void adc_conversion_callback(uint32_t event, uint8_t channel, uint32_t sample_output)
{
    if (event & ARM_ADC_EVENT_CONVERSION_COMPLETE)
    {
        num_samples += 1;

        /* Store the value for the respective channels */
        adc_sample[channel] = sample_output;

        /* Sample ready Wake up Thread. */
        tx_event_flags_set(&event_flags_adc, TX_ADC_INT_AVG_SAMPLE_RDY, TX_OR);
    }
}

/**
 *    @func         : void tsens_demo_thread_entry(ULONG thread_input)
 *    @brief        : tsens demo (temperature sensor)
 *                  - test to verify the temperature sensor of adc.
 *                  - Internal input of temperature  in analog signal corresponding
 *                    output is digital value.
 *                  - converted value is the allocated user memory address.
 *    @parameter[1] : thread_input : thread input
 *    @return       : NONE
*/
void tsens_demo_thread_entry(ULONG thread_input)
{
    ULONG events            = 0;
    UINT ret                = 0;
    UINT error_code         = SERVICES_REQ_SUCCESS;
    UINT service_error_code;
    float    temp;
    ARM_DRIVER_VERSION version;

    (void) thread_input;

    /* Initialize the SE services */
    se_services_port_init();

    /* enable the 160 MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                           /*clock_enable_t*/ CLKEN_CLK_160M,
                           /*bool enable   */ true,
                                              &service_error_code);
    if (error_code)
    {
        printf("SE Error: 160 MHz clk enable = %d\n", error_code);
        return;
    }

    printf("\r\n >>> ADC demo threadX starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%X driver:%X...\r\n",version.api, version.drv);

    /* Initialize ADC driver */
    ret = ADCdrv->Initialize(adc_conversion_callback);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC init failed\n");
        goto error_uninitialize;
    }

    /* Power control ADC */
    ret = ADCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

    /* set conversion mode */
    ret = ADCdrv->Control(ARM_ADC_CONVERSION_MODE_CTRL, ADC_CONVERSION);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC select conversion mode failed\n");
        goto error_poweroff;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, TEMPERATURE_SENSOR);
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC channel init failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%X <<<\n",(uint32_t)(adc_sample + TEMPERATURE_SENSOR));
    /* Start ADC */
    ret = ADCdrv->Start();
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    /* wait for num_samples */
    while (num_samples == 0);

    /* wait till conversion comes ( isr callback ) */
    ret = tx_event_flags_get(&event_flags_adc, TX_ADC_INT_AVG_SAMPLE_RDY,
                             TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS) {
        printf("Error: ADC tx_event_flags_get\n");
        goto error_poweroff;
    }

    /* Reading temperature */
    temp = get_temperature(adc_sample[TEMPERATURE_SENSOR]);
    if (temp == ARM_DRIVER_ERROR) {
        printf("\r\n Error: Temperature is outside range\n");
        goto error_poweroff;
    }
    else
    {
        printf("\n Current temp %.1f°C\n",temp);
    }

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if (ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Stop failed\n");
        goto error_poweroff;
    }

    printf("\n >>> ADC conversion completed \n");
    printf(" Converted value are stored in user allocated memory address.\n");
    printf("\n ---END--- \r\n wait forever >>> \n");
    while(1);

error_poweroff:

    /* Power off ADC peripheral */
    ret = ADCdrv->PowerControl(ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: ADC Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize ADC driver */
    ret = ADCdrv->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: ADC Uninitialize failed.\r\n");
    }
    /* disable the 160MHz clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                           /*clock_enable_t*/ CLKEN_CLK_160M,
                           /*bool enable   */ false,
                                              &service_error_code);
    if (error_code)
    {
        printf("SE Error: 160 MHz clk disable = %d\n", error_code);
        return;
    }

    printf("\r\n ADC demo exiting...\r\n");
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
    INT status;

    /* Create the event flags group used by ADC thread */
    status = tx_event_flags_create(&event_flags_adc, "event flags ADC");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&tsens_thread, "tsens_thread", tsens_demo_thread_entry, 0,
            first_unused_memory, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}
