/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/**************************************************************************//**
 * @file     i3c_slaveside_loopback_testapp.c
 * @author   Prabhakar kumar
 * @email    prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     27-May-2023
 * @brief    TestApp to verify slave side loop back test
 *           - Data received from master is stored in buffer and
 *             same data is transmitted back to the master.
 *
 *           I3C slave configuration.
 *           - In control API parameter:
 *             control : I3C_SET_SLAVE_ADDR(set slave address)
 *             arg     : I3C_SLAVE_ADDRESS macro is defined to set address
 *                       of slave
 *
 *           Hardware Setup:
 *            Required two boards one for Master and one for Slave
 *             (as there is only one i3c instance is available on ASIC).
 *
 *           Connect SDA to SDA and SCL to SCL and GND to GND.
 *            - SDA P7_6 -> SDA P7_6
 *            - SCL P7_7 -> SCL P7_7
 *            - GND      -> GND
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
#include "Driver_I3C.h"
#include "Driver_GPIO.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "pinconf.h"

#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* set slave address */
#define I3C_SLAVE_ADDRESS             (0X48)

/* receive data from i3c */
uint8_t rx_data[4] = {0x00};

uint32_t tx_cnt = 0;
uint32_t rx_cnt = 0;

void i3c_slave_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024

TX_THREAD               I3C_thread;
TX_EVENT_FLAGS_GROUP    event_flags_i3c;

/* i3c callback events */
typedef enum {
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1)
}I3C_CB_EVENTS;

/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
int32_t hardware_init(void)
{
    /* for I3C_D(PORT_7 PIN_6(SDA)/PIN_7(SCL)) instance,
      *  for I3C in I3C mode (not required for I3C in I2C mode)
      *  GPIO voltage level(flex) has to be change to 1.8-V power supply.
      *
      *  GPIO_CTRL Register field VOLT:
      *   Select voltage level for the 1.8-V/3.3-V (flex) I/O pins
      *    0x0: I/O pin will be used with a 3.3-V power supply
      *    0x1: I/O pin will be used with a 1.8-V power supply
      */

     /* Configure GPIO flex I/O pins to 1.8-V:
      *  P7_6 and P7_7 pins are part of GPIO flex I/O pins,
      *   so we can use any one of the pin to configure flex I/O.
      */
 #define GPIO7_PORT          7

     extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO7_PORT);
     ARM_DRIVER_GPIO *gpioDrv = &ARM_Driver_GPIO_(GPIO7_PORT);

     int32_t  ret = 0;
     uint32_t arg = 0;

     ret = gpioDrv->Initialize(PIN_6, NULL);
     if (ret != 0)
     {
         printf("ERROR: Failed to initialize GPIO \n");
         return -1;
     }

     ret = gpioDrv->PowerControl(PIN_6, ARM_POWER_FULL);
     if (ret != 0)
     {
         printf("ERROR: Failed to powered full GPIO \n");
         return -1;
     }

     /* select control argument as flex 1.8-V */
     arg = ARM_GPIO_FLEXIO_VOLT_1V8;
     ret = gpioDrv->Control(PIN_6, ARM_GPIO_CONFIG_FLEXIO, &arg);
     if (ret != 0)
     {
         printf("ERROR: Failed to control GPIO Flex \n");
         return -1;
     }

     /* I3C_SDA_D */
     ret = pinconf_set(PORT_7, PIN_6, PINMUX_ALTERNATE_FUNCTION_6,
                 PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

     /* I3C_SCL_D */
     ret = pinconf_set(PORT_7, PIN_7, PINMUX_ALTERNATE_FUNCTION_6,
                 PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP |
                 PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

     return ret;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
void I3C_callback(UINT event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE)
    {
        /* Transfer Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_SUCCESS, TX_OR);
    }

    if (event & ARM_I3C_EVENT_TRANSFER_ERROR)
    {
        /* Transfer Error: Wake-up Thread. */
        tx_event_flags_set(&event_flags_i3c, I3C_CB_EVENT_ERROR, TX_OR);
    }
}

/**
  \fn          void i3c_slave_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify i3c slave mode
               This demo thread does:
                 - initialize i3c driver;
                 - set slave address and initialize slave
                 - Receive and transmit byte from master
  \param[in]   thread_input : thread input
  \return      none
*/
void i3c_slave_demo_thread_entry(ULONG thread_input)
{
    INT   ret    = 0;
    INT   len    = 0;
    ULONG actual_events = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> Slave loop back demo with Azure RTOS ThreadX starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",  \
                           version.api, version.drv);

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* Initialize I3C driver */
    ret = I3Cdrv->Initialize(I3C_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        goto error_uninitialize;
    }

    /* Control I3C interface */
    ret = I3Cdrv->Control(I3C_SLAVE_SET_ADDR, I3C_SLAVE_ADDRESS);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control failed.\r\n");
        goto error_poweroff;
    }

    while(1)
    {
        len = 4;

        /* Slave Receive */
        ret = I3Cdrv->SlaveReceive(rx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C slave Receive failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c,
                           I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR,
                           TX_OR_CLEAR,
                           &actual_events,
                           TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C rx_event_flags_get failed.\n");
            goto error_poweroff;
        }

        if(actual_events == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Slave receive failed\n");
            while(1);
        }

        /* clear callback */
        actual_events = 0;

        rx_cnt += 1;

        /* For loop back test,
         * Slave will send received data back to Master.
         */

        /* Slave Transmit*/
        ret = I3Cdrv->SlaveTransmit(rx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C slave Transmit failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c,
                           I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR,
                           TX_OR_CLEAR,
                           &actual_events,
                           TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C tx_event_flags_get failed.\n");
            goto error_poweroff;
        };

        if(actual_events == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Slave transmit failed\n");
            while(1);
        }

        /* clear callback */
        actual_events = 0;

        tx_cnt += 1;
    }

error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I3C Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
    }

    printf("\r\n XXX I3C demo thread exiting XXX...\r\n");
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
    INT      status  = 0;

    /* Create the event flags group used by i3c thread  */
    status = tx_event_flags_create(&event_flags_i3c, "event flags I3C");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&I3C_thread, "I3C_thread",
                              i3c_slave_demo_thread_entry, 0,
                              first_unused_memory, DEMO_STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE***/

