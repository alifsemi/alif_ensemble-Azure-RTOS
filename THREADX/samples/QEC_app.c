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
 * @file     QEC_app.c
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     2-November-2022
 * @brief    Threadx demo application for QEC0 channel.
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#include <stdio.h>
#include "tx_api.h"
#include "Driver_UTIMER.h"
#include "Driver_GPIO.h"
#include "Driver_PINMUX_AND_PINPAD.h"

#define DEMO_BYTE_POOL_SIZE             (1024)
#define QEC0_THREAD_STACK_SIZE          (512)

#define QEC_COMPARE_A_CB_EVENT          (1 << 0)
#define QEC_COMPARE_B_CB_EVENT          (1 << 1)

UCHAR                                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_BYTE_POOL                            memory_pool;
TX_THREAD                               qec0_thread;
TX_EVENT_FLAGS_GROUP                    qec_event_flag;
ULONG                                   events;

/* GPIO related definitions */
#define GPIO1                           1
#define GPIO1_PIN0                      0
#define GPIO1_PIN1                      1
#define GPIO1_PIN2                      2

/* UTIMER0 Driver instance */
extern ARM_DRIVER_UTIMER DRIVER_UTIMER0;
ARM_DRIVER_UTIMER *ptrUTIMER = &DRIVER_UTIMER0;

/* GPIO1 Driver instance */
extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO1);
ARM_DRIVER_GPIO *ptrDrv = &ARM_Driver_GPIO_(GPIO1);

volatile UINT cb_qec_status = 0;

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

/**
 * @function    INT pinmux_config()
 * @brief       QEC h/w pin init using pinmux driver
 * @note        none
 * @param       none
 * @retval      execution status
 */
static INT pinmux_config()
{
    INT ret;

    ret = PINMUX_Config (PORT_NUMBER_2, PIN_NUMBER_28, PINMUX_ALTERNATE_FUNCTION_5);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    ret = PINMUX_Config (PORT_NUMBER_2, PIN_NUMBER_29, PINMUX_ALTERNATE_FUNCTION_5);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    ret = PINMUX_Config (PORT_NUMBER_2, PIN_NUMBER_30, PINMUX_ALTERNATE_FUNCTION_4);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    return ARM_DRIVER_OK;
}

/**
 * @function    INT gpio_init()
 * @brief       GPIO initialization using gpio driver
 * @note        none
 * @param       none
 * @retval      execution status
 */
static INT gpio_init()
{
    INT ret = ARM_DRIVER_OK;

    /* init P1_0 as GPIO */
    ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_0, PINMUX_ALTERNATE_FUNCTION_0);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }
    ret = PINPAD_Config (PORT_NUMBER_1, PIN_NUMBER_0, (PAD_FUNCTION_READ_ENABLE|PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    ret = ptrDrv->Initialize(GPIO1_PIN0, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN0 as GPIO\n");
        return -1;
    }

    ret = ptrDrv->PowerControl(GPIO1_PIN0, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Powered the GPIO1_PIN0\n");
        return -1;
    }

    ret = ptrDrv->SetDirection(GPIO1_PIN0, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN0\n");
        return -1;
    }

    ret = ptrDrv->SetValue(GPIO1_PIN0, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN0\n");
        return -1;
    }

    /* init P1_1 as GPIO */
    ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_0);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }
    ret = PINPAD_Config (PORT_NUMBER_1, PIN_NUMBER_1, (PAD_FUNCTION_READ_ENABLE|PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    ret = ptrDrv->Initialize(GPIO1_PIN1, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN1 as GPIO\n");
        return -1;
    }

    ret = ptrDrv->PowerControl(GPIO1_PIN1, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Powered the GPIO1_PIN1\n");
        return -1;
    }

    ret = ptrDrv->SetDirection(GPIO1_PIN1, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN1\n");
        return -1;
    }

    ret = ptrDrv->SetValue(GPIO1_PIN1, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN1\n");
        return -1;
    }

    /* init P1_2 as GPIO */
    ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_0);
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }
    ret = PINPAD_Config (PORT_NUMBER_1, PIN_NUMBER_2, (PAD_FUNCTION_READ_ENABLE|PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));
    if(ret != ARM_DRIVER_OK) {
        printf("\r\n Error in PINMUX.\r\n");
        return -1;
    }

    ret = ptrDrv->Initialize(GPIO1_PIN2, NULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to initialize GPIO1_PIN2 as GPIO\n");
        return -1;
    }

    ret = ptrDrv->PowerControl(GPIO1_PIN2, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to Powered the GPIO1_PIN2\n");
        return -1;
    }

    ret = ptrDrv->SetDirection(GPIO1_PIN2, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set direction for GPIO1_PIN2\n");
        return -1;
    }

    ret = ptrDrv->SetValue(GPIO1_PIN2, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set value for GPIO1_PIN2\n");
        return -1;
    }

    return ARM_DRIVER_OK;
}

static void qec_cb_func(uint32_t event)
{
    if (event == ARM_QEC_COMPARE_A_EVENT) {
        tx_event_flags_set ( &qec_event_flag, QEC_COMPARE_A_CB_EVENT, TX_OR);
    }
}

/**
 * @function    void qec0_app(thread_input)
 * @brief       QEC0 demo application
 * @note        none
 * @param       thread_input
 * @retval      none
 */
static void qec0_app(ULONG thread_input)
{
    INT ret;
    UINT buf, buf1, status;
    UCHAR channel = 12;
    UINT count_array[ARM_QEC_COUNT_NUMBER];

    UTIMER_MODE_CONFIG config_info = {
        .mode = ARM_UTIMER_MODE_TRIGGERING|ARM_UTIMER_MODE_CAPTURING,
        .direction = ARM_UTIMER_COUNT_DIRECTION_TRIANGLE,
        .count_array = count_array,
        .count_number = ARM_QEC_COUNT_NUMBER
    };

    UTIMER_GET_OPERATION_CONFIG get_cntr = {
        .operation_type = ARM_UTIMER_GET_COUNT_OF_CURRENT_RUNNING_TIMER,
        .count = &buf
    };

    UTIMER_GET_OPERATION_CONFIG get_capt = {
        .operation_type = ARM_UTIMER_GET_COUNT_OF_DRIVE_A_CAPTURE_VALUE,
        .count = &buf1
    };

    /*
     * utimer channel 12 is configured as QEC0.
     * QEC pins are connected to motors/actuators to measure speed and direction.
     * For testing purpose, QEC inputs are connected to gpio's. And GPIO's are toggled accordingly.
     *
     * H/W connection : short P1_0 and P2_28, short p1_1 and P2_29, short p1_2 and P2_30.
     **/

    printf("*** QEC demo application started ***\n");

    count_array[0] = 0;        /*< initial counter value >*/
    count_array[1] = 100;      /*< over flow count value >*/

    ret = pinmux_config();
    if (ret != ARM_DRIVER_OK) {
        printf("pinmux failed\n");
        return;
    }

    ret = gpio_init();
    if (ret != ARM_DRIVER_OK) {
        printf("gpio init failed\n");
        return;
    }

    ret = ptrUTIMER->Initialize (channel, qec_cb_func);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed initialize \n", channel);
        return;
    }

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed power up \n", channel);
        goto error_qec_uninstall;
    }

    ret = ptrUTIMER->Control (channel, ARM_UTIMER_MODE_CONFIG, &config_info);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d mode configuration failed \n", channel);
        goto error_qec_poweroff;
    }

    /* Config Trigger for up count */
    ret = ptrUTIMER->Configure_Trigger (channel, ARM_UTIMER_TRIGGER_FOR_UPCOUNT, \
        (ARM_UTIMER_DRIVE_A_RISING_B_0|ARM_UTIMER_DRIVE_A_FALLING_B_1|ARM_UTIMER_DRIVE_B_FALLING_A_0|ARM_UTIMER_DRIVE_B_RISING_A_1));
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %d triggered for up count \n", channel);
    }

    /* Config Trigger for down count */
    ret = ptrUTIMER->Configure_Trigger (channel, ARM_UTIMER_TRIGGER_FOR_DOWNCOUNT, \
        (ARM_UTIMER_DRIVE_A_FALLING_B_0|ARM_UTIMER_DRIVE_A_RISING_B_1|ARM_UTIMER_DRIVE_B_RISING_A_0|ARM_UTIMER_DRIVE_B_FALLING_A_1));
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %d triggered for down count \n", channel);
    }

    /* Config Trigger for capture counter value */
    ret = ptrUTIMER->Configure_Trigger (channel, ARM_UTIMER_TRIGGER_FOR_CAPTURE_A, \
            (ARM_UTIMER_TRIG0_RISING|ARM_UTIMER_TRIG0_FALLING));
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d trigger configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("utimer channel %d triggered for capture counter value \n", channel);
    }

	/* Toggling gpio's connected to x/y for 10 times to increment cnt value, so cnt value will be 20 */
    for(int i=0;i<10;i++)
    {
        ret = ptrDrv->SetValue(GPIO1_PIN0, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        ret = ptrDrv->SetValue(GPIO1_PIN1, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set data \n");
        }
    }

    ret = ptrUTIMER->Control (channel, ARM_UTIMER_GET_COUNT, &get_cntr);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d reading counter value failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("current counter value : %d\n", buf);
    }

    /* Toggling gpio's connected to x/y for 5 times to decrement cnt value, so cnt value will be reduced by 10 */
    for(int i=0;i<5;i++)
    {
        ret = ptrDrv->SetValue(GPIO1_PIN1, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        ret = ptrDrv->SetValue(GPIO1_PIN0, GPIO_PIN_OUTPUT_STATE_TOGGLE);
        if (ret != ARM_DRIVER_OK) {
            printf("ERROR: Failed to set data \n");
        }
    }

    /* Making z event as high to capture count value */
    ret = ptrDrv->SetValue(GPIO1_PIN2, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        printf("ERROR: Failed to set data \n");
    }

    /* Check for an interrupt */
    status = tx_event_flags_get (&qec_event_flag, QEC_COMPARE_A_CB_EVENT, TX_OR_CLEAR, &events, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("ERROR : event not received, timeout happened \n");
        goto error_qec_poweroff;
    }
    printf("Capture A interrupt occurred\n");

    ret = ptrUTIMER->Control (channel, ARM_UTIMER_GET_COUNT, &get_capt);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d mode configuration failed \n", channel);
        goto error_qec_poweroff;
    } else {
        printf("capture A value : %d\n", buf1);
    }

    ret = ptrUTIMER->Stop (channel, UTIMER_COUNTER_CLEAR);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed to stop \n", channel);
    } else {
        printf("utimer channel %d :timer stopped\n", channel);
    }

error_qec_poweroff:

    ret = ptrUTIMER->PowerControl (channel, ARM_POWER_OFF);
    if (ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed power off \n", channel);
    }

error_qec_uninstall:

    ret = ptrUTIMER->Uninitialize (channel);
    if(ret != ARM_DRIVER_OK) {
        printf("utimer channel %d failed to un-initialize \n", channel);
    }

    printf("*** QEC demo application completed *** \r\n\n");
}

/* Define main entry point */
int main ()
{
    /* Enter the ThreadX kernel */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void tx_application_define (void *first_unused_memory)
{
    CHAR *pointer = TX_NULL;
    UINT status;

    /* Create a event flag for qec group.  */
    status = tx_event_flags_create (&qec_event_flag, "UTIMER_COMPARE_MODE_EVENT_FLAG");
    if (status != TX_SUCCESS) {
        printf("failed to create utimer compare mode event flag\r\n");
    }

    /* Create a byte memory pool from which to allocate the thread stacks */
    status = tx_byte_pool_create (&memory_pool, "memory pool", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS) {
        printf("failed to create to byte Pool\r\n");
    }

    /* Allocate the stack for thread */
    status = tx_byte_allocate (&memory_pool, (VOID **) &pointer, QEC0_THREAD_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS) {
        printf("failed to allocate stack for led blink demo thread\r\n");
    }

    /* Create the main thread */
    status = tx_thread_create (&qec0_thread, "LED BLINK DEMO", qec0_app, 0, pointer, QEC0_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS) {
        printf("failed to create led blink demo thread\r\n");
    }
}
