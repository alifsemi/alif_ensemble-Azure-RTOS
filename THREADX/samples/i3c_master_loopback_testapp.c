/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/**************************************************************************//**
 * @file     i3c_master_loopback_testapp.c
 * @author   Prabhakar kumar
 * @email    prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     27-May-2023
 * @brief    TestApp to verify master loop back test
 *           - Data transmitted from master and slave receive's it.And,
 *             same data is transmitted from slave and master receive it.
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
#include <string.h>
#include "tx_api.h"

/* Project Includes */
#include "Driver_I3C.h"
#include "Driver_IO.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "pinconf.h"

#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* i3c Driver instance */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* I3C slave target address */
#define I3C_SLV_TAR           (0x48)

void i3c_master_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024

TX_THREAD               I3C_thread;
TX_EVENT_FLAGS_GROUP    event_flags_i3c;

/* transmit buffer from i3c */
uint8_t tx_data[4] = {0x00, 0x01, 0x02, 0x03};

/* receive buffer from i3c */
uint8_t rx_data[4] = {0x00};

uint32_t tx_cnt = 0;
uint32_t rx_cnt = 0;

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
static int32_t hardware_init(void)
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

     extern  ARM_DRIVER_IO ARM_Driver_IO_(GPIO7_PORT);
     ARM_DRIVER_IO *gpioDrv = &ARM_Driver_IO_(GPIO7_PORT);

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
     arg = ARM_IO_FLEXIO_VOLT_1V8;
     ret = gpioDrv->Control(PIN_6, ARM_IO_CONFIG_FLEXIO, &arg);
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
  \fn          void i3c_master_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify i3c slave mode
               This demo thread does:
                 - initialize i3c driver;
                 - set i3c speed mode to I3C_BUS_MODE_PURE;
                 - Data transmitted from master and slave receive's it.And,
                   same data is transmitted from slave and master receive it.
  \param[in]   thread_input : thread input
  \return      none
*/
void i3c_master_demo_thread_entry(ULONG thread_input)
{
    INT     ret        = 0;
    INT     len        = 0;
    UINT    cmp        = 0;
    UCHAR   slave_addr = 0x00;

    ARM_I3C_CMD i3c_cmd;

    ULONG actual_events = 0;
    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> Master loop back demo with Azure RTOS ThreadX starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",
                           version.api, version.drv);

    if((version.api < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U))        ||
       (version.drv < ARM_DRIVER_VERSION_MAJOR_MINOR(7U, 0U)))
    {
        printf("\r\n Error: >>>Old driver<<< Please use new one \r\n");
        return;
    }

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

    /* Initialize I3C master */
    ret = I3Cdrv->Control(I3C_MASTER_INIT, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Master Init control failed.\r\n");
        goto error_poweroff;
    }

    /* i3c Speed Mode Configuration: Bus mode slow  */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                          I3C_BUS_SLOW_MODE);

    /* Reject Hot-Join request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_HOT_JOIN_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Hot Join control failed.\r\n");
        goto error_poweroff;
    }

    /* Reject Master request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_MR_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Master Request control failed.\r\n");
        goto error_poweroff;
    }

    /* Reject Slave Interrupt request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_SIR_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Slave Interrupt Request control failed.\r\n");
        goto error_poweroff;
    }

    sys_busy_loop_us(1000);

    /* Reset all slaves' address */
    i3c_cmd.rw            = 0U;
    i3c_cmd.cmd_id        = I3C_CCC_RSTDAA(true);
    i3c_cmd.len           = 0U;
    i3c_cmd.addr          = 0;

    ret = I3Cdrv->MasterSendCommand(&i3c_cmd);
    if(ret != ARM_DRIVER_OK)
    {
        goto error_poweroff;
    }

    /* wait for callback event. */
    ret = tx_event_flags_get(&event_flags_i3c, \
                       I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                       TX_OR_CLEAR,                               \
                       &actual_events,                            \
					   TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I3C tx_event_flags_get failed.\n");
        goto error_poweroff;
    }

    if(actual_events & I3C_CB_EVENT_ERROR)
    {
        printf("\nError: I3C Slaves' Address Reset failed\n");
    }


    /* Assign Dynamic Address to i3c slave */
    printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_SLV_TAR);

    i3c_cmd.rw            = 0U;
    i3c_cmd.cmd_id        = I3C_CCC_SETDASA;
    i3c_cmd.len           = 1U;
    /* Assign Slave's Static address */
    i3c_cmd.addr          = I3C_SLV_TAR;
    i3c_cmd.data          = NULL;
    i3c_cmd.def_byte      = 0U;

    ret = I3Cdrv->MasterAssignDA(&i3c_cmd);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
        goto error_poweroff;
    }

    /* wait for callback event. */
    ret = tx_event_flags_get(&event_flags_i3c,
                             I3C_CB_EVENT_SUCCESS |I3C_CB_EVENT_ERROR,
                             TX_OR_CLEAR,
                             &actual_events,
                             TX_WAIT_FOREVER);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I3C tx_event_flags_get failed.\n");
        goto error_poweroff;
    }

    if(actual_events == I3C_CB_EVENT_ERROR)
    {
        printf("\r\n Error: First attempt failed. retrying \r\n");

        /* Delay */
        sys_busy_loop_us(1000);

        actual_events = 0;

        /* Observation:
         *  Master needs to send "MasterAssignDA" two times,
         *  First time slave is not giving ACK.
         */

        /* Assign Dynamic Address to i3c slave */
        ret = I3Cdrv->MasterAssignDA(&i3c_cmd);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
            goto error_poweroff;
        }

        /* wait for callback event. */
        ret = tx_event_flags_get(&event_flags_i3c,
                                 I3C_CB_EVENT_SUCCESS |I3C_CB_EVENT_ERROR,
                                 TX_OR_CLEAR,
                                 &actual_events,
                                 TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C tx_event_flags_get failed.\n");
            goto error_poweroff;
        }

        if(actual_events == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C MasterAssignDA failed\n");
            while(1);
        }
    }

    /* Get assigned dynamic address for the static address */
    ret = I3Cdrv->GetSlaveDynAddr(I3C_SLV_TAR, &slave_addr);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Failed to get Dynamic Address.\r\n");
        goto error_poweroff;
    }
    else
    {
        printf("\r\n >> i3c: Rcvd dyn_addr:0x%X for static addr:0x%X\r\n",
                slave_addr,I3C_SLV_TAR);
    }

    /* i3c Speed Mode Configuration: Normal I3C mode */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                          I3C_BUS_NORMAL_MODE);

    while(1)
    {
        len = 4;

        /* Delay */
        sys_busy_loop_us(1000);

        /* fill any random TX data. */
        tx_data[0] += 1;
        tx_data[1] += 1;
        tx_data[2] += 1;
        tx_data[3] += 1;

        /* clear callback */
        actual_events = 0;

        /* Master transmit */
        ret = I3Cdrv->MasterTransmit(slave_addr, tx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Transmit failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c,
                                 I3C_CB_EVENT_SUCCESS |I3C_CB_EVENT_ERROR,
                                 TX_OR_CLEAR,
                                 &actual_events,
                                 TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C tx_event_flags_get failed.\n");
            goto error_poweroff;
        }

        if(actual_events == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Master transmit failed\n");
            while(1);
        }

        /* clear callback */
        actual_events = 0;

        tx_cnt += 1;

        /* Delay */
        sys_busy_loop_us(1000);

        /* clear rx_data buffer */
        rx_data[0] = 0x00;
        rx_data[1] = 0x00;
        rx_data[2] = 0x00;
        rx_data[3] = 0x00;

        /* Master receive */
        ret = I3Cdrv->MasterReceive(slave_addr, rx_data, len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Master Receive failed. \r\n");
            goto error_poweroff;
        }

        ret = tx_event_flags_get(&event_flags_i3c,
                                 I3C_CB_EVENT_SUCCESS|I3C_CB_EVENT_ERROR,
                                 TX_OR_CLEAR,
                                 &actual_events,
                                 TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
        {
            printf("Error: I3C rx_event_flags_get failed.\n");
            goto error_poweroff;
        };

        if(actual_events == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Master receive failed\n");
            while(1);
        }

        /* clear callback */
        actual_events = 0;

        rx_cnt += 1;

        /* compare tx and rx data, stop if data does not match */
        cmp = memcmp(tx_data, rx_data, len);
        if(cmp != 0)
        {
           printf("\nError: TX and RX data mismatch.\n");
           while(1);
        }
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

    printf("\r\n I3C demo thread exiting...\r\n");
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
    status = tx_thread_create(&I3C_thread, "I3C_thread", i3c_master_demo_thread_entry, 0,
            first_unused_memory, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
