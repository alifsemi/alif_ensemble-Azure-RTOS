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
 *              Hardware setup (1 wires needed):
 *              - Use a wire to connect P0_0 to P12_3(J14 it will be labeled)
 *              - This test expects the LED Blinky application is running and
 *                the P12_3 should be toggling every 1 second
 *              - When the comparator input changes from HIGH to LOW or from LOW
 *                to HIGH, interrupt will be generated
 *              - Check CMP0 output in the pin P14_7 using saleae logic analyzer
 ******************************************************************************/

/* System Includes */
#include "Driver_GPIO.h"
#include <stdio.h>
#include "tx_api.h"
#include "system_utils.h"
#include "pinconf.h"

/* include for Comparator Driver */
#include "Driver_CMP.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* LED configurations */
#define GPIO12_PORT                     12       /* Use LED0_R,LED0_B GPIO port */
#define LED0_R                          PIN_3    /* LED0_R gpio pin             */

#define NUM_TAPS                        5 /* Number of filter taps     */
#define SAMPLING_RATE                   8 /* clock rate for filtering */

#define LPCMP                0
#define HSCMP                1

/* To configure for HSCMP, use CMP_INSTANCE HSCMP */
/* To configure for LPCMP, use CMP_INSTANCE LPCMP */
#define CMP_INSTANCE         HSCMP

#define ERROR    -1
#define SUCCESS   0

extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO12_PORT);
ARM_DRIVER_GPIO *ledDrv = &ARM_Driver_GPIO_(GPIO12_PORT);

#if(CMP_INSTANCE == LPCMP)
#if !defined(M55_HE)
#error "This Demo application works only on RTSS_HE"
#endif
extern ARM_DRIVER_CMP Driver_LPCMP;
static ARM_DRIVER_CMP *CMPdrv = &Driver_LPCMP;
#else
extern ARM_DRIVER_CMP Driver_CMP0;
static ARM_DRIVER_CMP *CMPdrv = &Driver_CMP0;
#endif

volatile uint32_t call_back_counter = 0;

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                 1024
#define DEMO_BYTE_POOL_SIZE             9120
#define TX_CMP_FILTER_EVENT             0x01

TX_THREAD               CMP_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_CMP;

/**
 * @fn          void cmp_pinmux_config(void)
 * @brief       Initialize the pinmux for CMP output
 * @return      status
*/
int32_t cmp_pinmux_config(void)
{
    int32_t status;

    /* Configure HSCMP0 output */
    status = pinconf_set(PORT_14, PIN_7, PINMUX_ALTERNATE_FUNCTION_1, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* Configure HSCMP1 output */
    status = pinconf_set(PORT_14, PIN_6, PINMUX_ALTERNATE_FUNCTION_1, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* Configure HSCMP2 output */
    status = pinconf_set(PORT_14, PIN_5, PINMUX_ALTERNATE_FUNCTION_1, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    /* Configure HSCMP3 output */
    status = pinconf_set(PORT_14, PIN_4, PINMUX_ALTERNATE_FUNCTION_1, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    return SUCCESS;
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
int32_t led_init(void)
{
    int32_t ret1 = 0;

    /* gpio12 pin3 can be used as Red LED of LED0 */
    pinconf_set(GPIO12_PORT, LED0_R, PINMUX_ALTERNATE_FUNCTION_0, 0);

    /* Initialize the LED0_R */
    ret1 = ledDrv->Initialize(LED0_R, NULL);
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

   ret1 = ledDrv->SetDirection(LED0_R, GPIO_PIN_DIRECTION_OUTPUT);
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
        tx_event_flags_set(&event_flags_CMP, TX_CMP_FILTER_EVENT, TX_OR);
    }
    call_back_counter++;
}

void CMP_demo_Thread_entry(ULONG thread_input)
{
    ULONG event = 0;
    INT  ret = 0;
    UINT loop_count = 10;
    ARM_DRIVER_VERSION version;
    ARM_COMPARATOR_CAPABILITIES capabilities;

    printf("\r\n >>> Comparator demo threadX starting up!!! <<< \r\n");

    /* Configure the CMP output pins */
    if(cmp_pinmux_config())
    {
        printf("CMP pinmux failed\n");
    }

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

#if(CMP_INSTANCE == HSCMP)
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

    while(loop_count--)
    {
        /* Toggle the LED0_R */
        if(led_toggle())
        {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_poweroff;
        }

        /* wait till the input changes ( isr callback ) */
        ret = tx_event_flags_get(&event_flags_CMP, TX_CMP_FILTER_EVENT, TX_OR_CLEAR, &event, TX_WAIT_FOREVER);
        if(ret != TX_SUCCESS) {
            printf("Error: Comparator tx_event_flags_get\n");
            goto error_poweroff;
        }

        /* Introducing a delay to stabilize input voltage for comparator measurement*/
        tx_thread_sleep(1 * TX_TIMER_TICKS_PER_SECOND);

        printf("\n >>> Filter event occurred <<< \n");
    }

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
    CHAR    *pointer = TX_NULL;
    INT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if(status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
    create information.  */

    /* Create the event flags group used by Comparator thread */
    status = tx_event_flags_create(&event_flags_CMP, "event flags CMP");
    if(status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if(status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&CMP_thread, "CMP_thread", CMP_demo_Thread_entry, 0,
             pointer, DEMO_STACK_SIZE,
             1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if(status != TX_SUCCESS)
    {
        printf("Could not create thread\n");
        return;
    }
}

    /************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
