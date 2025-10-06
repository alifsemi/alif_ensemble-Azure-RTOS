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
 * @file     LED_Breathe_testapp.c
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

#include "Driver_UTIMER.h"
#include "pinconf.h"

#include "RTE_Components.h"
#include CMSIS_device_header

#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

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
#define UT_INIT_COUNTER_VALUE         0U
#define UT_MAX_COUNTER_VALUE          240000U
#define UT_33_PERC_DT_COUNTER_VALUE   80000U
#define UT_66_PERC_DT_COUNTER_VALUE   UT_33_PERC_DT_COUNTER_VALUE * 2U
#define UT_100_PERC_DT_COUNTER_VALUE  UT_33_PERC_DT_COUNTER_VALUE * 3U
#define UT_CHANNEL_RED_LED            9U
#define UT_CHANNEL_GREEN_LED          10U
#define UT_CHANNEL_BLUE_LED           8U

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER Driver_UTIMER0;
ARM_DRIVER_UTIMER *ptrUTIMER = &Driver_UTIMER0;

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
        printf("utimer channel %d failed initialize \n", channel);
        return -1;
    }

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed power up \n", channel);
        return -1;
    }

    ret = ptrUTIMER->ConfigCounter (channel, ARM_UTIMER_MODE_COMPARING, ARM_UTIMER_COUNTER_UP);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d mode configuration failed \n", channel);
        return -1;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR, UT_INIT_COUNTER_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d set count failed \n", channel);
        return -1;
    }

    ret = ptrUTIMER->SetCount (channel, ARM_UTIMER_CNTR_PTR, UT_MAX_COUNTER_VALUE);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d set count failed \n", channel);
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
        printf("utimer channel %d failed to start \n", channel);
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
        printf("utimer channel %d set count failed \n", channel);
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
        printf("utimer channel %d failed to stop \n", channel);
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

#if (LED_USED == RED_LED)
    printf("Red LED brightness control has been started\n");
    channel = UT_CHANNEL_RED_LED;
    counter_type = ARM_UTIMER_COMPARE_B;

    ret = pinconf_set (PORT_12, PIN_3, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Red LED PINMUX.\r\n");
    }
#elif (LED_USED == GREEN_LED)
    printf("Green LED brightness control has been started\n");
    channel = UT_CHANNEL_GREEN_LED;
    counter_type = ARM_UTIMER_COMPARE_A;

    ret = pinconf_set (PORT_7, PIN_4, PINMUX_ALTERNATE_FUNCTION_6, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Green LED PINMUX.\r\n");
    }
#elif (LED_USED == BLUE_LED)
    printf("Blue LED brightness control has been started\n");
    channel = UT_CHANNEL_BLUE_LED;
    counter_type = ARM_UTIMER_COMPARE_A;

    ret = pinconf_set (PORT_12, PIN_0, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error in Blue LED PINMUX.\r\n");
    }
#else
#error "ERROR: Selected LED is not correct"
#endif

    ret = led_init (channel);
    if (ret) {
        printf("\r\n Error in UT init.\r\n");
        while(1);
    }

    ret = led_start (channel);
    if (ret) {
        printf("\r\n Error in UT LED start.\r\n");
        while(1);
    }

    while (1)
    {
        ret = led_set_brightness (channel, counter_type, UT_33_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            while(1);
        }

        /* delay for 1s */
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

        ret = led_set_brightness (channel, counter_type, UT_66_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            while(1);
        }

        /* delay for 1s */
        tx_thread_sleep (TX_TIMER_TICKS_PER_SECOND);

        ret = led_set_brightness (channel, counter_type, UT_100_PERC_DT_COUNTER_VALUE);
        if (ret) {
            printf("\r\n Error in UT LED brightness setup.\r\n");
            while(1);
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
