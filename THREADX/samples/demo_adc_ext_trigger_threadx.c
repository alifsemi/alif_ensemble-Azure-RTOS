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
 * @file     : demo_adc_ext_trigger_threadx.c 
 * @author   : Prabhakar kumar
 * @email    : prabhakar.kumar@alifsemi.com
 * @version  : V1.0.0
 * @date     : 04-Sept-2023
 * @brief    : Testapp demo application code for testing the external trigger
 *             feature for the ADC12 & ADC24
 *              - Generating the pulse from the Utimer channel 0 Driver A for starting
 *                the ADC conversion.
 *              - Input in analog signal corresponding output is digital value.
 *              - Converted digital value are stored in user provided memory
 *                address.
 *              ADC configurations for Demo testApp:
 *              Single channel scan(Default scan ADC12) (For E1C P0_0)
 *              - GPIO pin P1_4 are connected to Regulated DC Power supply.
 *                DC Power supply:
 *                - +ve connected to P1_4 (ADC122 channel 0) at 1.0V
 *                - -ve connect to GND.
 *              Differential input
 *              -ADC12 (For E1C P0_0 and P0_4)
 *                GPIO pin P1_4 and P1_5 are connected to Regulated DC Power supply.
 *                2 channel DC Power supply:
 *                - +ve connected to P1_4 (ADC122 channel 0) at 1.0V and
 *                  +ve connected to P1_5 (ADC122 channel 1) at 0.4V
 *                - -ve connect to GND.
 *              -ADC24
                  GPIO pins P1_4 and P1_5 are connected to Regulated DC Power supply.
 *                2 channel DC Power supply:
 *                - +ve connected to P0_0 (ADC122 channel 0) at 1.0V and
 *                  +ve connected to P0_4 (ADC122 channel 1) at 0.5V
 *                - -ve connect to GND.
 *
 * @Note     : From RTE_Device.h file set following in UTimer
 *             - RTE_UTIMER_CHANNEL0_DRIVER_A set marco to 1.
 *             - RTE_UTIMER_CHANNEL0_DRV_A_OP_AT_MATCH_COUNTset to 3.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "tx_api.h"
#include <inttypes.h>

/* Project include */
#include "Driver_UTIMER.h"
#include "Driver_ADC.h"
#include "pinconf.h"
#include "board_config.h"

#include "se_services_port.h"
#include "RTE_Components.h"
#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER Driver_UTIMER0;
ARM_DRIVER_UTIMER       *ptrUTIMER = &Driver_UTIMER0;

/* Macro for ADC12 and ADC24 */
#define ADC_12    1
#define ADC_24    0

/* For ADC12 use ADC_INSTANCE ADC12  */
/* For ADC24 use ADC_INSTANCE ADC24  */
#define ADC_INSTANCE         ADC_12
//#define ADC_INSTANCE         ADC_24

#if (ADC_INSTANCE == ADC_12)
/* Instance for ADC12 */
extern ARM_DRIVER_ADC  ARM_Driver_ADC12(BOARD_ADC12_INSTANCE);
static ARM_DRIVER_ADC *ADCdrv = &ARM_Driver_ADC12(BOARD_ADC12_INSTANCE);
#else
/* Instance for ADC24 */
extern ARM_DRIVER_ADC  Driver_ADC24;
static ARM_DRIVER_ADC *ADCdrv = &Driver_ADC24;
#endif

#define NUM_CHANNELS              8
#define NUM_TEST_SAMPLES          3
#define NUM_PULSE_GENERATE        3

void adc_ext_trigger_demo(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024

#define TX_ADC_INT_AVG_SAMPLE_RDY       0x01
#define TX_UTIMER_START                 0X02
#define TX_UTIMER_EVENT_COMPARE_A       0X04

TX_THREAD               adc_thread;
TX_THREAD               utimer_thread;
TX_EVENT_FLAGS_GROUP    event_flags;

/* Demo purpose adc_sample*/
UINT adc_sample[NUM_CHANNELS];

volatile UINT num_samples = 0;

volatile UINT num_pulses = 0;

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_adc_pins_config(void)
 * @brief   Configure ADC12 and ADC24 pinmux settings not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_adc_pins_config(void)
{
    int32_t ret = 0U;

    if (ADC_INSTANCE == ADC_12) {
        /* channel 0 */
        ret = pinconf_set(PORT_(BOARD_ADC12_CH0_GPIO_PORT),
                          BOARD_ADC12_CH0_GPIO_PIN,
                          PINMUX_ALTERNATE_FUNCTION_7,
                          PADCTRL_READ_ENABLE);
        if (ret) {
            printf("ERROR: Failed to configure PINMUX \r\n");
            return ret;
        }
    }

    if (ADC_INSTANCE == ADC_24) {
        /* channel 0 */
        ret = pinconf_set(PORT_(BOARD_ADC24_CH0_POS_GPIO_PORT),
                          BOARD_ADC24_CH0_POS_GPIO_PIN,
                          PINMUX_ALTERNATE_FUNCTION_7,
                          PADCTRL_READ_ENABLE);
        if (ret) {
            printf("ERROR: Failed to configure PINMUX \r\n");
            return ret;
        }
        /* channel 0 */
        ret = pinconf_set(PORT_(BOARD_ADC24_CH0_NEG_GPIO_PORT),
                          BOARD_ADC24_CH0_NEG_GPIO_PIN,
                          PINMUX_ALTERNATE_FUNCTION_7,
                          PADCTRL_READ_ENABLE);
        if (ret) {
            printf("ERROR: Failed to configure PINMUX \r\n");
            return ret;
        }
    }
    return ret;
}
#endif

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

        /* Store the value for the respected channels */
        adc_sample[channel] = sample_output;

        if (num_samples == NUM_TEST_SAMPLES)
        {
            /* Sample ready Wake-up Thread. */
            tx_event_flags_set(&event_flags, TX_ADC_INT_AVG_SAMPLE_RDY, TX_OR);
        }
    }
}

/**
 * @function    void utimer_compare_mode_cb_func(event)
 * @brief       utimer compare mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_compare_mode_cb_func(uint8_t event)
{
    if (event == ARM_UTIMER_EVENT_COMPARE_A) {

        if (++num_pulses == NUM_PULSE_GENERATE)
        {
            /* Sample ready Wake-up Thread. */
            tx_event_flags_set(&event_flags, TX_UTIMER_EVENT_COMPARE_A, TX_OR);
        }
    }
}

/**
 * @function    void utimer_compare_mode_app(ULONG thread_input)
 * @brief       utimer compare mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_compare_mode_app(ULONG thread_input)
{
    ULONG events;
    INT   ret     = 0;
    UCHAR channel = 0;
    UINT  count_array[3];

    /*
     *utimer channel 0 is configured for utimer compare mode (driver A).
     */
    /*
     * System CLOCK frequency (F)= 400Mhz
     *
     * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
     *
     * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
     *
     * So count for 1 sec = 1/(0.0025*(10^-6)) = 400000000
     * DEC = 400000000
     * HEX = 0x17D78400
     *
     * So count for 500ms = (500*(10^-3)/(0.0025*(10^-6)) = 200000000
     * DEC = 200000000
     * HEX = 0xBEBC200
     */
    count_array[0] =  0x000000000;       /*< initial counter value >*/
    count_array[1] =  0x17D78400;        /*< over flow count value >*/
    count_array[2] =  0xBEBC200;         /*< compare a/b value>*/

    (void) thread_input;

    tx_event_flags_get(&event_flags, TX_UTIMER_START, TX_OR_CLEAR,
                       &events, TX_WAIT_FOREVER);

    ret = ptrUTIMER->Initialize (channel, utimer_compare_mode_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed power up \n", channel);
        goto error_compare_mode_uninstall;
    }

    ret = ptrUTIMER->ConfigCounter (channel, ARM_UTIMER_MODE_COMPARING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d mode configuration failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR, count_array[0]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR_PTR, count_array[1]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_COMPARE_A, count_array[2]);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d set count failed \n", channel);
        goto error_compare_mode_poweroff;
    }

    ret = ptrUTIMER->Start(channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed to start \n", channel);
        goto error_compare_mode_poweroff;
    }

    /* wait till conversion comes ( isr callback ) */
    ret = tx_event_flags_get(&event_flags, TX_UTIMER_EVENT_COMPARE_A, TX_OR_CLEAR,
                             &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS) {
        printf("Error: Utimer tx_event_flags_get\n");
        goto error_compare_mode_poweroff;
    }


    ret = ptrUTIMER->Stop (channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed to stop \n", channel);
    }

error_compare_mode_poweroff:

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed power off \n", channel);
    }

error_compare_mode_uninstall:

    ret = ptrUTIMER->Uninitialize (channel);
    if(ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed to un-initialize \n", channel);
    }

    printf("\nutimer exiting...\n");
}

void adc_ext_trigger_demo(ULONG thread_input)
{
    ULONG events;
    int32_t ret                = 0;
    uint32_t error_code        = SERVICES_REQ_SUCCESS;
    uint32_t service_error_code;
    ARM_DRIVER_VERSION version;

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#else
    /*
     * NOTE: The ADC122 and ADC24 pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_adc_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }
#endif

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

    printf("\r\n >>> ADC demo starting up!!! <<< \r\n");

    version = ADCdrv->GetVersion();
    printf("\r\n ADC version api:%" PRIx16 " driver:%" PRIx16 "...\r\n", version.api, version.drv);

    /* Initialize ADC driver */
    ret = ADCdrv->Initialize(adc_conversion_callback);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC init failed\n");
        return;
    }

    /* Power control ADC */
    ret = ADCdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC Power up failed\n");
        goto error_uninitialize;
    }

    /* set initial channel */
    ret = ADCdrv->Control(ARM_ADC_CHANNEL_INIT_VAL, ARM_ADC_CHANNEL_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC channel failed\n");
        goto error_poweroff;
    }

    printf(">>> Allocated memory buffer Address is 0x%" PRIx32 " <<<\n", (uint32_t) adc_sample);

    /* Start ADC from External trigger pulse */
    ret = ADCdrv->Control(ARM_ADC_EXTERNAL_TRIGGER_ENABLE, ARM_ADC_EXTERNAL_TRIGGER_SRC_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC External trigger enable failed\n");
        goto error_poweroff;
    }

    /* Start ADC */
    ret = ADCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Start failed\n");
        goto error_poweroff;
    }

    tx_event_flags_set(&event_flags, TX_UTIMER_START, TX_OR);

    /* wait till conversion comes ( isr callback ) */
    ret = tx_event_flags_get(&event_flags, TX_ADC_INT_AVG_SAMPLE_RDY, TX_OR_CLEAR,
                             &events, TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS) {
        printf("Error: ADC tx_event_flags_get\n");
        goto error_poweroff;
    }

    /* Stop ADC external trigger conversion */
    ret = ADCdrv->Control(ARM_ADC_EXTERNAL_TRIGGER_DISABLE, ARM_ADC_EXTERNAL_TRIGGER_SRC_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: ADC External trigger disable failed\n");
        goto error_poweroff;
    }

    /* Stop ADC */
    ret = ADCdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: ADC Start failed\n");
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
        printf("SE: clk enable = %" PRId32 "\n", error_code);
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
    status = tx_event_flags_create(&event_flags, "event flags ADC");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread */
    status = tx_thread_create(&adc_thread, "adc_thread", adc_ext_trigger_demo, 0,
             first_unused_memory, DEMO_STACK_SIZE,
             2, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }

    /* Create the main thread */
    status = tx_thread_create(&utimer_thread, "utimer_thread", utimer_compare_mode_app, 0,
                              (first_unused_memory + DEMO_STACK_SIZE), DEMO_STACK_SIZE,
                               1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}
