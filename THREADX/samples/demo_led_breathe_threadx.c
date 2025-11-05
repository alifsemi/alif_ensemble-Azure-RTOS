/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     demo_led_breathe_threadx.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     18-Sept-2023
 * @brief    ThreadX demo application for LED brightness control using PWM.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "tx_api.h"
#include <stdio.h>
#include <inttypes.h>

#include "Driver_UTIMER.h"
#include "pinconf.h"
#include "board_config.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_stdout.h"
#endif /* RTE_CMSIS_Compiler_STDOUT */

#include "app_utils.h"

// Set to 0: Use application-defined UTIMER pin configuration (from board_utimer_pins_config).
// Set to 1: Use Conductor-generated pin configuration (from pins.h).
#define USE_CONDUCTOR_TOOL_PINS_CONFIG 0

#define UT_LED_DEMO_THREAD_STACK_SIZE   (512U)

TX_THREAD                               led_ctrl_thread;

#define RED_LED    1U
#define GREEN_LED  2U
#define BLUE_LED   3U

/* Enable any one LED */
#define LED_USED   BLUE_LED

/*
 * UTIMER Counter value calculation:
 * System CLOCK frequency (F)= 400Mhz
 *
 * Time for 1 count T = 1/F = 1/(400*10^6) = 0.0025 * 10^-6
 *
 * To Increment or Decrement Timer by 1 count, takes 0.0025 micro sec
 *
 * So count for 600us = (600*(10^-6)/(0.0025*(10^-6)) = 240000
 * DEC = 240000
 *
 * So count for 200us (33 % duty cycle) = (200*(10^-6)/(0.0025*(10^-6)) = 80000
 * DEC = 80000
 */
#define UT_INIT_COUNTER_VALUE          0U
#define UT_MAX_COUNTER_VALUE           BOARD_LED_PWM_UT_MAX_COUNTER_VALUE
#define UT_33_PERC_DT_COUNTER_VALUE    (UT_MAX_COUNTER_VALUE / 3)
#define UT_66_PERC_DT_COUNTER_VALUE    (UT_33_PERC_DT_COUNTER_VALUE * 2)
#define UT_100_PERC_DT_COUNTER_VALUE   UT_MAX_COUNTER_VALUE
#define UT_CHANNEL_RED_LED             BOARD_RED_LED_UTIMER_INSTANCE
#define UT_CHANNEL_GREEN_LED           BOARD_GREEN_LED_UTIMER_INSTANCE
#define UT_CHANNEL_BLUE_LED            BOARD_BLUE_LED_UTIMER_INSTANCE

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER Driver_UTIMER0;
ARM_DRIVER_UTIMER       *ptrUTIMER = &Driver_UTIMER0;

#if (!USE_CONDUCTOR_TOOL_PINS_CONFIG)
/**
 * @fn      static int32_t board_utimer_pins_config (void)
 * @brief   Configure UTIMER pinmux which not
 *          handled by the board support library.
 * @retval  execution status.
 */
static int32_t board_utimer_pins_config(void)
{
    int32_t ret;
#if (LED_USED == RED_LED)
    ret = pinconf_set(PORT_(BOARD_LEDRGB0_R_GPIO_PORT),
                      BOARD_LEDRGB0_R_GPIO_PIN,
                      BOARD_LEDRGB0_R_UT_ALTERNATE_FUNCTION,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Red LED PINMUX.\r\n");
        return ret;
    }
#elif (LED_USED == GREEN_LED)
    ret = pinconf_set(PORT_(BOARD_LEDRGB0_G_GPIO_PORT),
                      BOARD_LEDRGB0_G_GPIO_PIN,
                      BOARD_LEDRGB0_G_UT_ALTERNATE_FUNCTION,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Green LED PINMUX.\r\n");
        return ret;
    }
#elif (LED_USED == BLUE_LED)
    ret = pinconf_set(PORT_(BOARD_LEDRGB0_B_GPIO_PORT),
                      BOARD_LEDRGB0_B_GPIO_PIN,
                      BOARD_LEDRGB0_B_UT_ALTERNATE_FUNCTION,
                      PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Blue LED PINMUX.\r\n");
        return ret;
    }
#else
#error "ERROR: Selected LED is not correct"
#endif

    return 0;
}
#endif

/**
 * @function    void utimer_led_cb_func (event)
 * @brief       utimer callback function
 * @note        none
 * @param       event
 * @retval      none
 */
static void utimer_led_cb_func (UCHAR event)
{
    if (event == ARM_UTIMER_EVENT_COMPARE_A) {
        //empty
    }
    if (event == ARM_UTIMER_EVENT_COMPARE_B) {
        //empty
    }
    if (event == ARM_UTIMER_EVENT_OVER_FLOW) {
        //empty
    }
}

/**
 * @function    INT led_init (UCHAR channel)
 * @brief       UTIMER channel init for mentioned LED
 * @note        none
 * @param       channel
 * @retval      execution status
 */
static INT led_init (UCHAR channel)
{
    INT ret = 0;

    ret = ptrUTIMER->Initialize (channel, utimer_led_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed initialize \n", channel);
        return -1;
    }

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed power up \n", channel);
        return -1;
    }

    ret = ptrUTIMER->ConfigCounter (channel, ARM_UTIMER_MODE_COMPARING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " mode configuration failed \n", channel);
        return -1;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR, UT_INIT_COUNTER_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        return -1;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR_PTR, UT_MAX_COUNTER_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        return -1;
    }

    return 0;
}

/**
 * @function    INT led_start (UCHAR channel)
 * @brief       UTIMER channel counter start for mentioned LED
 * @note        none
 * @param       channel
 * @retval      execution status
 */
static INT led_start (UCHAR channel)
{
    INT ret = 0;

    ret = ptrUTIMER->Start (channel);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to start \n", channel);
        return -1;
    }
    return 0;
}

/**
 * @function    INT led_set_brightness (UCHAR channel,  ARM_UTIMER_COUNTER counter, UINT duty_cycle)
 * @brief       UTIMER channel set Compare value for mentioned LED
 * @note        none
 * @param       channel
 * @param       counter
 * @param       duty_cycle
 * @retval      execution status
 */
static INT led_set_brightness (UCHAR channel, ARM_UTIMER_COUNTER counter, UINT duty_cycle)
{
    INT ret = 0;

    ret = ptrUTIMER->SetCount (channel, counter, duty_cycle);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " set count failed \n", channel);
        return -1;
    }

    return 0;
}

/**
 * @function    INT led_stop (UCHAR channel)
 * @brief       UTIMER channel counter stop for mentioned LED
 * @note        none
 * @param       channel
 * @retval      execution status
 */
static INT led_stop (UCHAR channel)
{
    INT ret = 0;

    ret = ptrUTIMER->Stop (channel, ARM_UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %" PRIu8 " failed to stop \n", channel);
        return -1;
    }

    return 0;
}

/**
 * @function    void led_brightness_ctrl_thread (ULONG thread_input)
 * @brief       LED brightness control using pwm
 * @note        none
 * @param       thread_input
 * @retval      none
 */
static void led_breathe_thread (ULONG thread_input)
{
    INT ret;
    UCHAR channel;
    ARM_UTIMER_COUNTER counter_type;

    printf("*** utimer ThreadX demo application for LED brightness control ***\n");

#if USE_CONDUCTOR_TOOL_PINS_CONFIG
    /* pin mux and configuration for all device IOs requested from pins.h*/
    ret = board_pins_config();
#else
    /*
     * NOTE: The UTIMER pins used in this test application are not configured
     * in the board support library.Therefore, it is being configured manually here.
     */
    ret = board_utimer_pins_config();
#endif
    if (ret != 0) {
        printf("Error in pin-mux configuration: %" PRId32 "\n", ret);
        return;
    }

#if (LED_USED == RED_LED)
    printf("Red LED brightness control has been started\n");
    channel = UT_CHANNEL_RED_LED;
#if BOARD_RED_LED_UTIMER_COUNTER_TYPE
    counter_type = ARM_UTIMER_COMPARE_B;
#else
    counter_type = ARM_UTIMER_COMPARE_A;
#endif
#elif (LED_USED == GREEN_LED)
    printf("Green LED brightness control has been started\n");
    channel = UT_CHANNEL_GREEN_LED;
#if BOARD_GREEN_LED_UTIMER_COUNTER_TYPE
    counter_type = ARM_UTIMER_COMPARE_B;
#else
    counter_type = ARM_UTIMER_COMPARE_A;
#endif
#elif (LED_USED == BLUE_LED)
    printf("Blue LED brightness control has been started\n");
    channel = UT_CHANNEL_BLUE_LED;
#if BOARD_BLUE_LED_UTIMER_COUNTER_TYPE
    counter_type = ARM_UTIMER_COMPARE_B;
#else
    counter_type = ARM_UTIMER_COMPARE_A;
#endif
#else
#error "ERROR: Selected LED is not correct"
#endif

    ret = led_init (channel);
    if (ret) {
        printf("\r\n Error in UT init.\r\n");
        WAIT_FOREVER_LOOP
    }

    ret = led_start (channel);
    if (ret) {
        printf("\r\n Error in UT LED start.\r\n");
        WAIT_FOREVER_LOOP
    }

    while (1) {
        ret = led_set_brightness (channel, counter_type, UT_33_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            WAIT_FOREVER_LOOP
        }

        /* delay for 1s */
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

        ret = led_set_brightness (channel, counter_type, UT_66_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            WAIT_FOREVER_LOOP
        }

        /* delay for 1s */
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

        ret = led_set_brightness (channel, counter_type, UT_100_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            WAIT_FOREVER_LOOP
        }

        /* delay for 1s */
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);
    }

    ret = led_stop (channel);
    if (ret) {
        printf("\r\n Error in UT LED stop.\r\n");
        while(1);
    }

    printf("*** LED brightness control Threadx demo application completed *** \r\n\n");
}

/* Define main entry point.  */
int main ()
{
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    extern int stdout_init(void);
    int32_t    ret;
    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define (void *first_unused_memory)
{
    UINT ret;

    /* Create the main thread.  */
    ret = tx_thread_create (&led_ctrl_thread, "LED BREATHE DEMO", led_breathe_thread,
            0, first_unused_memory, UT_LED_DEMO_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (ret != TX_SUCCESS) {
        printf("failed to create led brightness control demo thread\r\n");
    }
}
/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
