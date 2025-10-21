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
 * @file     CMP_testapp.c
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     18-June-2022
 * @brief    Test application code for analog Comparator.
 *              - CMP0 instance is used - in RTE_Device.h we set the input muxes
 *              - Input A is set by RTE_CMP0_SEL_POSITIVE
 *              - Input A is set to RTE_CMP0_POSITIVE_PIN_PO_00 (analog pin P0_0)
 *              - Input B is set by RTE_CMP0_SEL_NEGATIVE
 *              - Input B is set to RTE_CMP_NEGATIVE_DAC6(which provide 0.9v)
 *              E7: Hardware setup (1 wires needed):
 *              - Connect P0_0(+ve pin) to P12_3(GPIO output) and DAC6 is set as negative
 *                pin,check CMP0 output in the pin P14_7 using saleae logic analyzer.
 *              E1C: Hardware setup (1 wires needed):
 *              - Connect P0_6(+ve pin) to P4_7(GPIO output) and DAC6 is set as negative
 *                pin,check CMP0 output in the pin P8_0 using saleae logic analyzer.
 *              - If +ve input is greater than -ve input, interrupt will be generated,
 *                and the output will be high.
 *              - If -ve input is greater than +ve input, interrupt will be generated,
 *                and the output will be low.
 *              E7: For window control feature:
 *              - As glb_events/utimer events are active for few clocks, set prescalar
 *                value to 0. These interrupt will occur continuously because Utimer
 *                is running continuously.
 ******************************************************************************/

/* System Includes */
#include "Driver_IO.h"
#include <stdio.h>
#include "tx_api.h"
#include "pinconf.h"
#include "Driver_UTIMER.h"
#include "app_utils.h"
#include "board_config.h"

/* include for Comparator Driver */
#include "Driver_CMP.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

// Set to 0: Use application-defined CMP pin configuration.
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

/* LED configurations */
#define LED0_R                         BOARD_LEDRGB0_R_GPIO_PIN /* LED0_R gpio pin             */

/* For E7: To read the CMP0 output status set CMP_OUTPIN as BOARD_CMP0_OUT_GPIO_PIN, for CMP1
 * set CMP_OUTPIN as BOARD_CMP1_OUT_GPIO_PIN, for CMP2 set CMP_OUTPIN as BOARD_CMP2_OUT_GPIO_PIN,
 * and for CMP3 set CMP_OUTPIN as BOARD_CMP3_OUT_GPIO_PIN.
 * For E1C: To read the CMP0 output status set CMP_OUTPIN as BOARD_CMP0_OUT_GPIO_PIN and for
 * CMP1 set CMP_OUTPIN as BOARD_CMP1_OUT_GPIO_PIN
 * */
#define CMP_OUTPIN                     BOARD_CMP0_OUT_GPIO_PIN

#define NUM_TAPS                       3 /* Filter taps: choose between 3 and 8 */

#define LP_CMP                         0
#define CMP                            1

/* To configure for CMP, use CMP_INSTANCE CMP */
/* To configure for LPCMP, use CMP_INSTANCE LP_CMP */
#define CMP_INSTANCE                   CMP

/* To enable comparator window control, change the macro value from 0 to 1
 * The glb_events/utimer events define the window where to look at the cmp_input.
 * As GLB_events/Utimer_events are active for few clocks, there is no reason to set
 * prescaler value, so set Prescaler value to 0 when using window control.
 * As Utimer is running continuously, the CMP interrupts will occur continuously.
 */
#define CMP_WINDOW_CONTROL             0

#if CMP_WINDOW_CONTROL
#define SAMPLING_RATE        0  /* Set the prescaler values as 0 for windowing function */
#else
#define SAMPLING_RATE        8  /* Set the prescaler values from 0 to 31 */
#endif

#define ERROR    -1
#define SUCCESS   0

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_LEDRGB0_R_GPIO_PORT);
ARM_DRIVER_GPIO       *ledDrv = &ARM_Driver_GPIO_(BOARD_LEDRGB0_R_GPIO_PORT);

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_CMP0_OUT_GPIO_PORT);
ARM_DRIVER_GPIO       *CMPout = &ARM_Driver_GPIO_(BOARD_CMP0_OUT_GPIO_PORT);

#if (CMP_INSTANCE == LP_CMP)
#if !defined(RTSS_HE)
#error "This Demo application works only on RTSS_HE"
#endif
extern ARM_DRIVER_CMP  Driver_LPCMP;
static ARM_DRIVER_CMP *CMPdrv = &Driver_LPCMP;
#else
/* Instance for CMP */
extern ARM_DRIVER_CMP  ARM_Driver_CMP(BOARD_POTENTIOMETER_CMP_INSTANCE);
static ARM_DRIVER_CMP *CMPdrv = &ARM_Driver_CMP(BOARD_POTENTIOMETER_CMP_INSTANCE);
#endif

volatile uint32_t call_back_counter = 0;
uint32_t value =0;

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define TX_CMP_FILTER_EVENT             0x01

TX_THREAD               CMP_thread;
TX_EVENT_FLAGS_GROUP    event_flags;

/* Use window control(External trigger using UTIMER or QEC) to trigger the comparator comparison */
#if(CMP_INSTANCE == CMP)
#if CMP_WINDOW_CONTROL

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER DRIVER_UTIMER0;
ARM_DRIVER_UTIMER *ptrUTIMER = &DRIVER_UTIMER0;

#define UTIMER_COMPARE_A_CB_EVENT                (1U << 2U)
#define UTIMER_COMPARE_MODE_WAIT_TIME            (5U * TX_TIMER_TICKS_PER_SECOND)              /* 5 seconds wait time */
#define TX_UTIMER_START                           0X02

ULONG                                            events;
TX_THREAD                                        compare_mode_thread;

/**
 * @function    void utimer_compare_mode_cb_func(event)
 * @brief       utimer compare mode callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_compare_mode_cb_func(UCHAR  event)
{
    if (event == ARM_UTIMER_EVENT_COMPARE_A) {
        tx_event_flags_set(&event_flags, UTIMER_COMPARE_A_CB_EVENT, TX_OR);
    }
}

/**
 * @function    void utimer_compare_mode_app(void)
 * @brief       utimer compare mode application
 * @note        none
 * @param       none
 * @retval      none
 */
static void utimer_compare_mode_app(ULONG thread_input)
{
    INT ret;
    UCHAR channel = 0;
    UINT count_array[3];

    /*
     * utimer channel 0 is configured for utimer compare mode (driver A).
     * observe driver A output signal from P1_2.
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
    count_array[1] =  0x17D78400;        /*< over flow count value > */
    count_array[2] =  0xBEBC200;         /*< compare a/b value> */

    tx_event_flags_get(&event_flags, TX_UTIMER_START, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);

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

    ret = tx_event_flags_get(&event_flags, UTIMER_COMPARE_A_CB_EVENT, TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
    if(ret != TX_SUCCESS) {
        printf("ERROR : event not received \n");
        goto error_compare_mode_poweroff;
    }

    return;

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
}

#endif
#endif

/**
 * @fn      static int32_t board_cmp_pins_config(void)
 * @brief   Configure CMP pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_cmp_pins_config(void)
{
    int32_t status;

    /* Configure CMP0 output */
    status = pinconf_set(PORT_(BOARD_CMP0_OUT_GPIO_PORT),
                         BOARD_CMP0_OUT_GPIO_PIN,
                         BOARD_CMP0_OUT_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* LPCMP_IN0 input to the positive terminal of LPCMP */
    status = pinconf_set(PORT_(BOARD_LPCMP_POS_INPUT_GPIO_PORT),
                         BOARD_LPCMP_POS_INPUT_GPIO_PIN,
                         BOARD_LPCMP_POS_INPUT_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* LPCMP_IN0 input to the negative terminal of LPCMP */
    status = pinconf_set(PORT_(BOARD_LPCMP_NEG_INPUT_GPIO_PORT),
                         BOARD_LPCMP_NEG_INPUT_GPIO_PIN,
                         BOARD_LPCMP_NEG_INPUT_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* CMP0_IN0 input to the positive terminal of CMP0 */
    status = pinconf_set(PORT_(BOARD_CMP0_POS_INPUT_GPIO_PORT),
                         BOARD_CMP0_POS_INPUT_GPIO_PIN,
                         BOARD_CMP0_POS_INPUT_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    /* VREF_IN0 input to the negative terminal of CMP0 and CMP1 */
    status = pinconf_set(PORT_(BOARD_CMP_NEG_INPUT_GPIO_PORT),
                         BOARD_CMP_NEG_INPUT_GPIO_PIN,
                         BOARD_CMP_NEG_INPUT_ALTERNATE_FUNCTION,
                         PADCTRL_READ_ENABLE);
    if (status) {
        return status;
    }

    return APP_SUCCESS;
}

/**
 * @fn        led_init(void)
 * @brief     - Initialize the LED0_R
 *            - Enable the power for LED0_R
 *            - Set direction for LED0_R
 *            - Set value for LED0_R
 * @param[in]  None
 * return      status
 */
static int32_t led_init(void)
{
    int32_t ret1 = 0;

    /* Initialize the LED0_R */
    ret1 = ledDrv->Initialize(LED0_R, NULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return ERROR;
    }

    ret1 = CMPout->Initialize(CMP_OUTPIN, NULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize\n");
        return ERROR;
    }

    /* Enable the power for LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_FULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize_LED;
    }

    /* Enable the power for LED0_R */
    ret1 = CMPout->PowerControl(CMP_OUTPIN, ARM_POWER_FULL);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize_LED;
    }

   ret1 = ledDrv->SetDirection(LED0_R, GPIO_PIN_DIRECTION_OUTPUT);
   if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

   ret1 = CMPout->SetDirection(CMP_OUTPIN, GPIO_PIN_DIRECTION_OUTPUT);
   if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

    ret1 = ledDrv->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_HIGH);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off_LED;
    }

    return SUCCESS;

error_power_off_LED:
    /* Power-off the LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_OFF);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

error_uninitialize_LED:
    /* Uninitialize the LED0_R */
    ret1 = ledDrv->Uninitialize(LED0_R);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}

/**
 * @fn        cmp_get_status(void)
 * @brief     - Get the Status of CMP output pin.
 * @param[in]  None
 * return      status
 */
static int32_t cmp_get_status(void)
{
    int32_t ret = 0;
    ret = CMPout->GetValue(CMP_OUTPIN, &value);
    if(ret != ARM_DRIVER_OK) {
    printf("ERROR: Failed to toggle LEDs\n");
    goto error_power_off_LED;
    }
    return value;

    error_power_off_LED:
    /* Power-off the CMP_OUTPIN */
    ret = CMPout->PowerControl(CMP_OUTPIN, ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

    /* Uninitialize the CMP_OUTPIN */
    ret = CMPout->Uninitialize(CMP_OUTPIN);
    if(ret != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}

/**
 * @fn         led_toggle(void)
 * @brief      - set LED0_R for toggle
 * @param[in]  None
 * return      status
 */
int32_t led_toggle(void)
{
    int32_t ret1 = 0;

    ret1 = ledDrv->SetValue(LED0_R, GPIO_PIN_OUTPUT_STATE_TOGGLE);
    if(ret1 != ARM_DRIVER_OK) {
        printf("ERROR: Failed to toggle LEDs\n");
        goto error_power_off_LED;
    }
    return SUCCESS;

error_power_off_LED:
    /* Power-off the LED0_R */
    ret1 = ledDrv->PowerControl(LED0_R, ARM_POWER_OFF);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("ERROR: Failed to power off \n");
    }

    /* Uninitialize the LED0_R */
    ret1 = ledDrv->Uninitialize(LED0_R);
    if(ret1 != ARM_DRIVER_OK)  {
        printf("Failed to Un-initialize \n");
    }
    return ERROR;
}

/**
 * @fn       void CMP_filter_callback(uint32_t event)
 * @brief    - This code expects the LED Blinky application to be running
 *             and the pin P12_3 should toggle every 1 second.
 *           - The comparator compares the voltage of P12_3 which is connected
 *             to positive comparator input which is compared to the 0.9v DAC6.
 *           - When the comparator input changes from HIGH to LOW or from LOW to HIGH,
 *             interrupt will be generated and it will set the call back event
 *           - According to the interrupt generation the call_back_counter will be incremented.
 * @return   None
 */
static void CMP_filter_callback(uint32_t event)
{
    if(event & ARM_CMP_FILTER_EVENT_OCCURRED)
    {
        /* Received Comparator filter event */
        tx_event_flags_set(&event_flags, TX_CMP_FILTER_EVENT, TX_OR);
    }
    call_back_counter++;
}

void CMP_demo_Thread_entry(ULONG thread_input)
{
    ULONG event = 0;
    INT  ret = 0;
    UINT loop_count = 10;
    ARM_DRIVER_VERSION version;
    CHAR status = 0;

    printf("\r\n >>> Comparator demo threadX starting up!!! <<< \r\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#else
    /*
     * NOTE: The CMP pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_cmp_pins_config();
    if (ret != 0) {
        printf("Error in pin-mux configuration: %d\n", ret);
        return;
    }
#endif

    version = CMPdrv->GetVersion();
    printf("\r\n Comparator version api:%X driver:%X...\r\n", version.api, version.drv);

    /* Initialize the configurations for LED0_R */
    if(led_init())
    {
        printf("Error: LED initialization failed\n");
        return;
    }

    /* Initialize the Comparator driver */
    ret = CMPdrv->Initialize(CMP_filter_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator init failed\n");
        return;
    }

    /* Enable the power for Comparator */
    ret = CMPdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Power up failed\n");
        goto error_uninitialize;
    }

#if(CMP_INSTANCE == CMP)

#if CMP_WINDOW_CONTROL
    /* Start CMP using window control */
    ret = CMPdrv->Control(ARM_CMP_WINDOW_CONTROL_ENABLE, ARM_CMP_WINDOW_CONTROL_SRC_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CMP External trigger enable failed\n");
        goto error_poweroff;
    }
#endif

    /* Filter function for analog comparator*/
    ret = CMPdrv->Control(ARM_CMP_FILTER_CONTROL, NUM_TAPS);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Filter control failed\n");
        goto error_poweroff;
    }

    /* Prescaler function for the comparator */
    ret = CMPdrv->Control(ARM_CMP_PRESCALER_CONTROL, SAMPLING_RATE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Prescaler control failed\n");
        goto error_poweroff;
    }
#endif

    /* To Start the Comparator module */
    ret = CMPdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Start failed\n");
        goto error_poweroff;
    }

#if(CMP_INSTANCE == CMP)
#if CMP_WINDOW_CONTROL
    tx_event_flags_set(&event_flags, TX_UTIMER_START, TX_OR);
#endif
#endif

    while(loop_count --)
    {
        /* Toggle the LED0_R */
        if(led_toggle())
        {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_poweroff;
        }

        /* wait till the input changes ( isr callback ) */
        ret = tx_event_flags_get(&event_flags, TX_CMP_FILTER_EVENT, TX_OR_CLEAR, &event, TX_WAIT_FOREVER);
        if(ret != TX_SUCCESS) {
            printf("Error: Comparator tx_event_flags_get\n");
            goto error_poweroff;
        }

        /* Introducing a delay to stabilize input voltage for comparator measurement*/
        tx_thread_sleep(1 * TX_TIMER_TICKS_PER_SECOND);

        /* Check the status of the CMP output pin */
        status = cmp_get_status();

        /* If user give +ve input voltage more than -ve input voltage, status will be set to 1*/
        if(status == 1)
        {
            printf("\n CMP positive input voltage is greater than negative input voltage\n");
        }
        /* If user give -ve input voltage more than +ve input voltage, status will be set to 0*/
        else if(status == 0)
        {
            printf("\n CMP negative input voltage is greater than the positive input voltage\n");
        }
        else
        {
            printf("ERROR: Status detection is failed\n");
            goto error_poweroff;
        }

    }

#if(CMP_INSTANCE == CMP)
#if CMP_WINDOW_CONTROL
    /* Disable CMP window control */
    ret = CMPdrv->Control(ARM_CMP_WINDOW_CONTROL_DISABLE, ARM_CMP_WINDOW_CONTROL_SRC_0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: CMP External trigger enable failed\n");
        goto error_poweroff;
    }
#endif
#endif

    /* To Stop the comparator module */
    ret = CMPdrv->Stop();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Stop failed\n");
        goto error_poweroff;
    }

    printf("\n Comparator Filter event completed and the call_back_counter value is %d\n",call_back_counter );

error_poweroff:
    /* Power off Comparator peripheral */
    ret = CMPdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Power OFF failed.\r\n");
    }

error_uninitialize:
    /* UnInitialize Comparator driver */
    ret = CMPdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Comparator Uninitialize failed.\r\n");
    }
}

/* Define main entry point */
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

    /* Enter the ThreadX kernel */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    INT status;

    /* Create the event flags group used by Comparator thread */
    status = tx_event_flags_create(&event_flags, "event flags CMP");
    if(status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&CMP_thread, "CMP_thread", CMP_demo_Thread_entry,
                              0,first_unused_memory, DEMO_STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if(status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }

#if(CMP_INSTANCE == CMP)
#if CMP_WINDOW_CONTROL
    /* Create the utimer compare mode thread.  */
    status = tx_thread_create(&compare_mode_thread, "UTIMER COMPARE MODE THREAD", utimer_compare_mode_app,
                              0,first_unused_memory + DEMO_STACK_SIZE, DEMO_STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) {
        printf("failed to create utimer compare mode thread\r\n");
    }

#endif
#endif
}

    /************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
