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
 * @file     i2c_using_i3c_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     31-May-2023
 * @brief    TestApp to verify i2c communication with
 *            multiple i2c slave devices using i3c IP
 *            with Azure RTOS (ThreadX)
 *            as an Operating System.
 *
 *           hardware setup
 *           BMI Slave is internally connected with the I3C_D
 *           No hardware setup is required.
 *           Pins used:
 *           P7_6 (SDA)
 *           P7_7 (SCL)
 *
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/* PINMUX Driver */
#include "pinconf.h"

/* i3c Driver */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

void i2c_using_i3c_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD               I3C_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_i3c;


/* i3c callback events */
typedef enum _I3C_CB_EVENT{
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1)
}I3C_CB_EVENT;

/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      ARM_DRIVER_OK : success; ARM_DRIVER_ERROR : failure
*/
INT hardware_init(void)
{
  /* I3C_SDA_D */
  pinconf_set(PORT_7, PIN_6, PINMUX_ALTERNATE_FUNCTION_6,
          PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

  /* I3C_SCL_D */
  pinconf_set( PORT_7, PIN_7, PINMUX_ALTERNATE_FUNCTION_6,
          PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
          PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

    return ARM_DRIVER_OK;
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
  \fn          void i2c_using_i3c_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify i2c communication with
                multiple i2c slave devices using i3c IP
                with Azure RTOS (ThreadX).

               This demo thread does:
                 - initialize i3c driver;
                 - set i2c speed mode to Standard mode 100 KBPS;
                 - attach all i2c slave devices to i3c;
                 - continuously read from register address 0x0000(chip-id)
                    for all the attached slaves;
                 - display result depending on whether
                   slave has given ACK or NACK.
  \param[in]   thread_input : thread input
  \return      none
*/
void i2c_using_i3c_demo_thread_entry(ULONG thread_input)
{

/* Maximum 8 Slave Devices are supported */
#define MAX_SLAVE_SUPPORTED   3

/* BMI slave address (chip-id 0x43) */
#define BMI_ADDR              0x69

/* Dummy Slave */
#define DUMMY_SLAVE1          0x50
#define DUMMY_SLAVE2          0x55

    INT   i      = 0;
    INT   ret    = 0;
    INT   len    = 0;

    /* @NOTE:
     *  I3C expects data to be aligned in 4-bytes (multiple of 4) for DMA.
     */
    UCHAR tx_data[4] = {0};   /* transmit data to   i3c */
    UCHAR rx_data[4] = {0};   /* receive  data from i3c */

    ULONG actual_events = 0;
    ULONG wait_timer_tickes = 0;
    ARM_DRIVER_VERSION version;

    /* array of i2c slave address(static) */
    uint8_t slave_addr[MAX_SLAVE_SUPPORTED] =
    {
        BMI_ADDR,
        DUMMY_SLAVE1,
        DUMMY_SLAVE2
    };

    printf("\r\n \t\t >>> i2c using i3c demo with Azure RTOS ThreadX starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",  \
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
        goto error_uninitialize;
    }

    /* i2c Speed Mode Configuration:
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 MBPS
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 KBPS
     *  I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 KBPS
     */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                          I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control failed.\r\n");
        goto error_poweroff;
    }

    /* Reject Hot-Join request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_HOT_JOIN_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Hot Join control failed.\r\n");
        goto error_uninitialize;
    }

    /* Reject Master request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_MR_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Master Request control failed.\r\n");
        goto error_uninitialize;
    }

    /* Reject Slave Interrupt request */
    ret = I3Cdrv->Control(I3C_MASTER_SETUP_SIR_ACCEPTANCE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Slave Interrupt Request control failed.\r\n");
        goto error_uninitialize;
    }

    /* Attach all the slave address */
    printf("\r\n Start attaching all i2c slave addr to i3c.\r\n");
    for(i=0; i<MAX_SLAVE_SUPPORTED; i++)
    {
        printf("\r\n  >> i=%d attaching i2c slave addr:0x%X to i3c...\r\n",  \
                           i, slave_addr[i]);

        ret = I3Cdrv->AttachSlvDev(ARM_I3C_DEVICE_TYPE_I2C, slave_addr[i]);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Attach I2C device failed.\r\n");
            goto error_poweroff;
        }
    }

    /*
     * @Note:
     *  How much data(register address + actual data) user has to Transmit/Receive ?
     *   it depends on Slave's register address location bytes.
     *
     *  Generally, Camera Slave supports      16-bit(2 Byte) reg-addr and (8/16/32 bit) data
     *   Others Accelerometer/EEPROM supports  8-bit(1 Byte) reg-addr and (8/16/32 bit) data
     *
     *  First LSB[7-0] will be added to TX FIFO and first transmitted on the i3c bus;
     *   remaining bytes will be added in LSB -> MSB order.
     *
     *  For Slave who supports 16-bit(2 Byte) register address and data:
     *   Register Address[15:8] : Needs to be Transmit First  to the i3c
     *   Register Address[07:0] : Needs to be Transmit Second to the i3c
     *
     *  That means,
     *
     *  While transmitting to TX FIFO,
     *   MSB of TX data needs to be added first to the TX FIFO.
     *
     *  While receiving from RX FIFO,
     *   First MSB will be received from RX FIFO.
     *
     *  START          I3C FIFO           END
     *  MSB                               LSB
     *  24-31 bit | 16-23 bit | 8-15 bit | 0-7 bit
     *
     *  So, USER has to modify
     *  Transmit/Receive data (Little Endian <-> Big Endian and vice versa)
     *  before sending/after receiving to/from i3c TX/RX FIFO.
     */


    /* Let's Continuously read from register address 0x0000(chip-id) for
     * all the attached slaves and display received data depending on
     * whether slave has given ACK or NACK.
    */
    while(1)
    {
        for(i=0; i<MAX_SLAVE_SUPPORTED; i++)
        {
            /* To Read from register address 0x0000(chip-id):
             *  First write 0x0000 using MasterTransmit and
             *   then Read  data   using MasterReceive
             */
            tx_data[0] = 0x00; /* Register Address[15:8] goes first  */
            tx_data[1] = 0x00; /* Register Address[07:0] goes second */

            /* TX/RX length is 2 Byte
             * (assume slave requires 16-bit data for TX/RX).
             */
            len = 2;

            printf("\r\n ------------------------------------------------------------ \r\n");
            printf("\r\n >> i=%d TX slave addr:0x%X reg_addr:[0]0x%X [1]0x%X \r\n",  \
                               i, slave_addr[i], tx_data[0],tx_data[1]);

            /* For TX, User has to pass
             * Slave Address + TX data + length of the TX data.
             */
            ret = I3Cdrv->MasterTransmit(slave_addr[i], tx_data, len);
            if(ret != ARM_DRIVER_OK)
            {
                printf("\r\n Error: i3c Master Transmit failed. \r\n");
                goto error_detach;
            }

            /* wait till any event success/error comes in isr callback,
             *  and if event is set then clear that event.
             *   if the event flags are not set,
             *    this service suspends for a maximum of 100 timer-ticks.
             */
            wait_timer_tickes = 100;
            ret = tx_event_flags_get(&event_flags_i3c, \
                               I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                               TX_OR_CLEAR,                               \
                               &actual_events,                            \
                               wait_timer_tickes);
            if (ret != TX_SUCCESS)
            {
                printf("Error: I3C tx_event_flags_get failed.\n");
                goto error_detach;
            }

            if(actual_events & I3C_CB_EVENT_SUCCESS)
            {
                /* TX Success: Got ACK from slave */
                printf("\r\n \t\t >> i=%d TX Success: Got ACK from slave addr:0x%X.\r\n",  \
                               i, slave_addr[i]);
            }

            if(actual_events & I3C_CB_EVENT_ERROR)
            {
                /* TX Error: Got NACK from slave */
                printf("\r\n \t\t >> i=%d TX Error: Got NACK from slave addr:0x%X \r\n",  \
                               i, slave_addr[i]);
            }

            printf("\r\n\r\n >> i=%d RX slave addr:0x%X \r\n",i, slave_addr[i]);

            /* clear rx data buffer. */
            rx_data[0] = 0;
            rx_data[1] = 0;
            rx_data[2] = 0;

            len = 3;
            /* For RX, User has to pass
             * Slave Address + Pointer to RX data + length of the RX data.
             */
            ret = I3Cdrv->MasterReceive(slave_addr[i], rx_data, len);
            if(ret != ARM_DRIVER_OK)
            {
                printf("\r\n Error: i3c Master Receive failed. \r\n");
                goto error_detach;;
            }

            /* wait till any event success/error comes in isr callback,
             *  and if event is set then clear that event.
             *   if the event flags are not set,
             *    this service suspends for a maximum of 100 timer-ticks.
             */
            wait_timer_tickes = 100;
            ret = tx_event_flags_get(&event_flags_i3c, \
                               I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                               TX_OR_CLEAR,                               \
                               &actual_events,                            \
                               wait_timer_tickes);
            if (ret != TX_SUCCESS)
            {
                printf("Error: I3C tx_event_flags_get failed. \n");
                goto error_detach;
            }

            /* Display received data depending on whether slave has given ACK or NACK.*/
            if(actual_events & I3C_CB_EVENT_SUCCESS)
            {
                /* RX Success: Got ACK from slave */
                printf("\r\n \t\t >> i=%d RX Success: Got ACK from slave addr:0x%X.\r\n",  \
                               i, slave_addr[i]);
                printf("\r\n \t\t >> i=%d RX Received Data from slave:[0]0x%X [1]0x%X [2]0x%X\r\n",  \
                               i,rx_data[0],rx_data[1],rx_data[2]);
            }

            if(actual_events & I3C_CB_EVENT_ERROR)
            {
                /* RX Error: Got NACK from slave */
                printf("\r\n \t\t >> i=%d RX Error: Got NACK from slave addr:0x%X \r\n",  \
                               i, slave_addr[i]);
            }

            printf("\r\n ---------------------------XXX------------------------------ \r\n");

            /* Sleep for 1 second. */
            tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * 1);
        }
    }


error_detach:

    /* Detach all attached slave address */
    for(i=0; i<MAX_SLAVE_SUPPORTED; i++)
    {
        printf("\r\n i=%d detaching i2c slave addr:0x%X from i3c.\r\n",i, slave_addr[i]);
        ret = I3Cdrv->Detachdev(slave_addr[i]);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Detach I2C device failed.\r\n");
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
    CHAR    *pointer = TX_NULL;
    INT      status  = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Create the event flags group used by i3c thread */
    status = tx_event_flags_create(&event_flags_i3c, "event flags I3C");
    if (status != TX_SUCCESS)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&I3C_thread, "I3C_thread", i2c_using_i3c_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
