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
 * @file     LED_blink_app.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     23-April-2021
 * @brief    DEMO application for LED blink.
 *           - toggled LED ds1 and ds2 for every 1sec
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "tx_api.h"
#include "Driver_GPIO.h"
#include <stdio.h>

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

#define DEMO_BYTE_POOL_SIZE             (1024)
#define LED_BLINK_THREAD_STACK_SIZE     (512)

#define GPIO1_PIN14                     (14) /*< ds1 led connected to this gpio pin >*/
#define GPIO1_PIN15                     (15) /*< ds2 led connected to this gpio pin >*/
#define GPIO1                           1 /* Use LEDS in the GPIO1 port */

UCHAR                                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_BYTE_POOL                            memory_pool;
TX_THREAD                               led_thread;


void led_blink_app (ULONG thread_input)
{
  /*
   * gpio1 pin14 is connected active high led ds1.
   * gpio1 pin15 is connected active high led ds2.
   *
   * This demo application is about.
   *   - Keeping ds1 led state 'on' and ds2 led state 'off' for 1sec.
   *   - After 1 sec ticks swap status of both led's for next 1 sec ticks.
   *   - ie. ds1 led to 'off' and ds2 led to 'on'.
   *   - This cycle will keep repeating infinite times.
   */

    extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO1);
    ARM_DRIVER_GPIO *ptrDrv = &ARM_Driver_GPIO_(GPIO1);
    uint8_t led_ds1 = GPIO1_PIN14;
    uint8_t led_ds2 = GPIO1_PIN15;
    uint32_t ret1 = 0;
    uint32_t ret2 = 0;

    printf("led blink demo application started\n\n");

    ret1 = ptrDrv->Initialize(led_ds1, NULL);
    ret2 = ptrDrv->Initialize(led_ds2, NULL);

    if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to initialize\n");
        return;
    }

    ret1 = ptrDrv->PowerControl(led_ds1, ARM_POWER_FULL);
    ret2 = ptrDrv->PowerControl(led_ds2, ARM_POWER_FULL);

    if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize;
    }

    ret1 = ptrDrv->SetDirection(led_ds1, GPIO_PIN_DIRECTION_OUTPUT);
    ret2 = ptrDrv->SetDirection(led_ds2, GPIO_PIN_DIRECTION_OUTPUT);

    if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off;
    }

    ret1 = ptrDrv->SetValue(led_ds1, GPIO_PIN_OUTPUT_STATE_HIGH);
    ret2 = ptrDrv->SetValue(led_ds2, GPIO_PIN_OUTPUT_STATE_LOW);

    if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to configure\n");
        goto error_power_off;
    }

    while (1)
    {
        ret1 = ptrDrv->SetValue(led_ds1, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        ret2 = ptrDrv->SetValue(led_ds2, GPIO_PIN_OUTPUT_STATE_TOGGLE);

        if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
            printf("ERROR: Failed to toggle LEDs\n");
            goto error_power_off;
        }

        tx_thread_sleep(1 * TX_TIMER_TICKS_PER_SECOND); /*< thread will sleep for one sec >*/
    };

error_power_off:

    ret1 = ptrDrv->PowerControl(led_ds1, ARM_POWER_OFF);
    ret2 = ptrDrv->PowerControl(led_ds2, ARM_POWER_OFF);

	if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("ERROR: Failed to power off \n");
    } else {
        printf("LEDs power off \n");
    }

error_uninitialize:

    ret1 = ptrDrv->Uninitialize (led_ds1);
    ret2 = ptrDrv->Uninitialize (led_ds2);

    if ((ret1 != ARM_DRIVER_OK) || (ret2 != ARM_DRIVER_OK)) {
        printf("Failed to Un-initialize \n");
    } else {
        printf("Un-initialized \n");
    }
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
    INT ret;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    ret = tx_byte_pool_create (&memory_pool, "memory pool", memory_area, DEMO_BYTE_POOL_SIZE);

    if (ret != TX_SUCCESS) {
        printf("failed to create to byte Pool\r\n");
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */

    /* Allocate the stack for thread 0.  */
    ret = tx_byte_allocate (&memory_pool, (VOID **) &pointer, LED_BLINK_THREAD_STACK_SIZE, TX_NO_WAIT);

    if (ret != TX_SUCCESS) {
        printf("failed to allocate stack for led blink demo thread\r\n");
    }

    /* Create the main thread.  */
    ret = tx_thread_create (&led_thread, "LED BLINK DEMO", led_blink_app, 0, pointer, LED_BLINK_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (ret != TX_SUCCESS) {
        printf("failed to create led blink demo thread\r\n");
    }
}
