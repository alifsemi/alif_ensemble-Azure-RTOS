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
 * @file     mix_bus_i2c_i3c_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     31-May-2023
 * @brief    TestApp to verify Mix Bus i2c and i3c communication with
 *            i2c + i3c slave devices using i3c IP
 *            with Azure RTOS (ThreadX)
 *            as an Operating System.
 *
 *           Select appropriate i3c Speed mode as per i2c or i3c slave device.
 *             I3C_BUS_MODE_PURE                             : Only Pure I3C devices
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
 *             I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
 *
 *           Hardware setup
 *            TestApp will communicate with Accelerometer and BMI Slave,
 *             which are on-board connected with the I3C_D.
 *             (so no any external hardware connection are required).
 *              Pins used:
 *               P7_6 (SDA)
 *               P7_7 (SCL)
 *               GND
 *
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>
#include "tx_api.h"

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "pinconf.h"
#include "Driver_GPIO.h"

#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

void mix_bus_i2c_i3c_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE             1024
#define DEMO_BYTE_POOL_SIZE         9120

TX_THREAD               I3C_thread;
TX_BYTE_POOL            byte_pool_0;
UCHAR                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP    event_flags_i3c;

/* i3c callback events */
typedef enum {
    I3C_CB_EVENT_SUCCESS   = (1 << 0),
    I3C_CB_EVENT_ERROR     = (1 << 1)
}I3C_CB_EVENT;


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
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize GPIO \n");
        return ARM_DRIVER_ERROR;
    }

    ret = gpioDrv->PowerControl(PIN_6, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to powered full GPIO \n");
        return ARM_DRIVER_ERROR;
    }

    /* select control argument as flex 1.8-V */
    arg = ARM_GPIO_FLEXIO_VOLT_1V8;
    ret = gpioDrv->Control(PIN_6, ARM_GPIO_CONFIG_FLEXIO, &arg);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to control GPIO Flex \n");
        return ARM_DRIVER_ERROR;
    }

    /* I3C_SDA_D */
    pinconf_set(PORT_7, PIN_6, PINMUX_ALTERNATE_FUNCTION_6,
                PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
                PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

    /* I3C_SCL_D */
    pinconf_set(PORT_7, PIN_7, PINMUX_ALTERNATE_FUNCTION_6,
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
  \fn          void mix_bus_i2c_i3c_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify mix bus i2c and i3c communication with
                i2c + i3c slave devices using i3c IP
                with Azure RTOS (ThreadX).

               This demo thread does:
                 - initialize i3c driver;
                 - set i3c speed mode to Mixed bus i2c/i3c Fast Mode 400 Kbps;
                 - assign dynamic address and attach all i3c slave devices to i3c;
                 - send/receive i3c CCC (Common Command Codes) only for i3c slaves
                 - attach all i2c slave devices to i3c;
                 - continuously read from specific register address(chip-id)
                    for all the attached slaves;
                 - display result depending on whether
                   slave has given ACK or NACK.
  \param[in]   thread_input : thread input
  \return      none
*/
void mix_bus_i2c_i3c_demo_thread_entry(ULONG thread_input)
{

/* Maximum 8 Slave Devices are supported */
#define MAX_SLAVE_SUPPORTED   8

/* Added 2 slaves for demo purpose
 *   i3c : Accelerometer
 *   i2c : BMI
 */
#define TOTAL_SLAVE           2

/* ICM-42670-P Accelerometer Slave address(On-chip attached to Board) */
#define I3C_ACCERO_ADDR       0x68

/* BMI323 Slave address(On-chip attached to Board) */
#define I2C_BMI_ADDR          0x69

/* ICM-42670-P Accelerometer Slave chip-id register(WHO AM I) address and value
 *  as per datasheet
 */
#define I3C_ACCERO_REG_WHO_AM_I_ADDR        0x75
#define I3C_ACCERO_REG_WHO_AM_I_VAL         0x67

/* BMI323 Slave Chip-id register address and value as per datasheet. */
#define I2C_BMI_REG_CHIP_ID_ADDR            0x00
#define I2C_BMI_REG_CHIP_ID_VAL             0x43

    INT   i      = 0;
    INT   ret    = 0;
    INT   len    = 0;

    /* Array of slave address :
     *       Dynamic Address for i3c and
     *       Static  Address for i2c
     */
    uint8_t slave_addr[TOTAL_SLAVE] =
    {
        0, /* I3C Accero  Dynamic Address: To be updated later using MasterAssignDA */
        I2C_BMI_ADDR /* I2C BMI Slave Address. */
    };

    /* Slave Register Address */
    uint8_t slave_reg_addr[TOTAL_SLAVE] =
    {
        I3C_ACCERO_REG_WHO_AM_I_ADDR,
        I2C_BMI_REG_CHIP_ID_ADDR
    };

    /* @NOTE:
     *  I3C expects data to be aligned in 4-bytes (multiple of 4) for DMA.
     */

    /* transmit data to i3c */
    uint8_t tx_data[4] = {0};

    /* receive data from i3c */
    uint8_t rx_data[4] = {0};

    /* receive data used for comparison. */
    uint8_t cmp_rx_data = 0;

    /* actual receive data as per slave datasheet */
    uint8_t actual_rx_data[TOTAL_SLAVE] =
    {
        I3C_ACCERO_REG_WHO_AM_I_VAL,
        I2C_BMI_REG_CHIP_ID_VAL
    };

    ULONG actual_events = 0;
    ULONG wait_timer_ticks = 0;
    ARM_DRIVER_VERSION version;

    /* I3C CCC (Common Command Codes) */
    I3C_CMD i3c_cmd;
    uint8_t i3c_cmd_tx_data[4] = {0x0F};
    uint8_t i3c_cmd_rx_data[4] = {0};


    printf("\r\n \t\t >>> mix bus i2c and i3c communication demo with Azure RTOS ThreadX starting up!!! <<< \r\n");

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

    /* i3c Speed Mode Configuration:
     *  I3C_BUS_MODE_PURE                             : Only Pure I3C devices
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
     *  I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
     *  I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
     */
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,  \
                      I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control failed.\r\n");
        goto error_poweroff;
    }

    /* Delay for n micro second.
     *  @Note: Minor delay is required if prints are disable.
     */
    sys_busy_loop_us(1000);

    /* Assign Dynamic Address to i3c Accelerometer */
    printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_ACCERO_ADDR);

    ret = I3Cdrv->MasterAssignDA(&slave_addr[0], I3C_ACCERO_ADDR);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
        goto error_poweroff;
    }

    /* wait till any event success/error comes in isr callback,
     *  and if event is set then clear that event.
     *   if the event flags are not set,
     *    this service suspends for a maximum of 100 timer-ticks.
     */
    wait_timer_ticks = 100;
    ret = tx_event_flags_get(&event_flags_i3c, \
                       I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                       TX_OR_CLEAR,                               \
                       &actual_events,                            \
                       wait_timer_ticks);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I3C tx_event_flags_get failed.\n");
        goto error_detach;
    }

    if(actual_events & I3C_CB_EVENT_ERROR)
    {
        printf("\nError: I3C MasterAssignDA failed\n");
    }

    printf("\r\n >> i3c: Received dyn_addr:0x%X for static addr:0x%X. \r\n",   \
                                 slave_addr[0],I3C_ACCERO_ADDR);

    /* Delay for n micro second. */
    sys_busy_loop_us(1000);

    /* demo for I3C CCC (Common Command Codes) APIs */

    /* write I3C_CCC_SETMWL (Set Max Write Length) command to Accelerometer slave */
    i3c_cmd.rw     = 0;
    i3c_cmd.cmd_id = I3C_CCC_SETMWL(false);
    i3c_cmd.len    = 1;
    i3c_cmd.addr   = slave_addr[0];
    i3c_cmd.data   = i3c_cmd_tx_data;

    ret = I3Cdrv->MasterSendCommand(&i3c_cmd);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterSendCommand failed.\r\n");
        goto error_detach;
    }

    /* wait till any event success/error comes in isr callback,
     *  and if event is set then clear that event.
     *   if the event flags are not set,
     *    this service suspends for a maximum of 100 timer-ticks.
     */
    wait_timer_ticks = 100;
    ret = tx_event_flags_get(&event_flags_i3c, \
                       I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                       TX_OR_CLEAR,                               \
                       &actual_events,                            \
                       wait_timer_ticks);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I3C tx_event_flags_get failed.\n");
        goto error_detach;
    }

    if(actual_events & I3C_CB_EVENT_ERROR)
    {
        printf("\nError: I3C MasterSendCommand failed\n");
    }

    /* Delay for n micro second. */
    sys_busy_loop_us(1000);

    /* read I3C_CCC_GETMWL (Get Max Write Length) command from Accelerometer slave */
    i3c_cmd.rw     = 1;
    i3c_cmd.cmd_id = I3C_CCC_GETMWL;
    i3c_cmd.len    = 1;
    i3c_cmd.addr   = slave_addr[0];
    i3c_cmd.data   = i3c_cmd_rx_data;

    ret = I3Cdrv->MasterSendCommand(&i3c_cmd);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C MasterSendCommand failed.\r\n");
        goto error_detach;
    }

    /* wait till any event success/error comes in isr callback,
     *  and if event is set then clear that event.
     *   if the event flags are not set,
     *    this service suspends for a maximum of 100 timer-ticks.
     */
    wait_timer_ticks = 100;
    ret = tx_event_flags_get(&event_flags_i3c, \
                       I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                       TX_OR_CLEAR,                               \
                       &actual_events,                            \
                       wait_timer_ticks);
    if (ret != TX_SUCCESS)
    {
        printf("Error: I3C tx_event_flags_get failed.\n");
        goto error_detach;
    }

    if(actual_events & I3C_CB_EVENT_ERROR)
    {
        printf("\nError: I3C MasterSendCommand failed\n");
    }

    /* compare tx and rx command data for Accelerometer slave */
    if( memcmp(i3c_cmd_rx_data, i3c_cmd_tx_data, 1) == 0 )
    {
        printf("\r\n \t\t >> i3c Accelerometer SendCommand Success.\r\n");
    }
    else
    {
        printf("\r\n \t\t >> i3c Accelerometer SendCommand failed.\r\n");
    }

    /* Delay for n micro second. */
    sys_busy_loop_us(1000);

    /* Attach i2c BMI slave using static address */
    printf("\r\n >> i2c: Attaching i2c BMI slave addr:0x%X to i3c...\r\n",slave_addr[1]);

    ret = I3Cdrv->AttachI2Cdev(slave_addr[1]);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Attach I2C device failed.\r\n");
        goto error_poweroff;
    }


    /*
     * @Note:
     *  How much data(register address + actual data) user has to Transmit/Receive ?
     *   it depends on Slave's register address location bytes.
     *
     *  Generally, Camera Slave supports       16-bit(2 Byte) reg-addr and (8/16/32 bit) data
     *   Others Accero/BMI/EEPROM supports      8-bit(1 Byte) reg-addr and (8/16/32 bit) data
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


    /* Let's Continuously read from chip-id register address for
     *  all the attached slaves and display received data depending on
     *  whether slave has given ACK or NACK.
    */
    while(1)
    {
        for(i=0; i<TOTAL_SLAVE; i++)
        {
            /* To Read from any register address:
             *  First write register address using MasterTransmit and
             *   then Read data using MasterReceive
             */

            /* TX/RX length is 1 Byte
             * (assume slave requires 8-bit data for TX/RX).
             */
            len = 1;

            printf("\r\n ------------------------------------------------------------ \r\n");
            printf("\r\n >> i=%d TX slave addr:0x%X reg_addr:[0]0x%X \r\n",  \
                                 i, slave_addr[i], slave_reg_addr[i]);

            /* Delay for n micro second. */
            sys_busy_loop_us(1000);

            /* For TX, User has to pass
             * Slave Address + TX data + length of the TX data.
             */
            tx_data[0] = slave_reg_addr[i];

            ret = I3Cdrv->MasterTransmit(slave_addr[i], tx_data, len);
            if(ret != ARM_DRIVER_OK)
            {
                printf("\r\n Error: I3C Master Transmit failed. \r\n");
                goto error_detach;
            }

            /* wait till any event success/error comes in isr callback,
             *  and if event is set then clear that event.
             *   if the event flags are not set,
             *    this service suspends for a maximum of 100 timer-ticks.
             */
            wait_timer_ticks = 100;
            ret = tx_event_flags_get(&event_flags_i3c, \
                               I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                               TX_OR_CLEAR,                               \
                               &actual_events,                            \
                               wait_timer_ticks);
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


            /* RX */
            printf("\r\n\r\n >> i=%d RX slave addr:0x%X \r\n",i, slave_addr[i]);

            /* clear rx data buffer. */
            rx_data[0] = 0;
            rx_data[1] = 0;
            rx_data[2] = 0;

            /* TX/RX length is 1 Byte
             * (assume slave requires 8-bit data for TX/RX).
             */
            len = 1;

            if(slave_addr[i] == I2C_BMI_ADDR)
            {
                /* BMI slave supports(as per datasheet ch-4):
                 *  - 8-bit addressing and 16-bit data
                 *  - Each register read operation required
                 *     2 bytes of dummy data (for i2c/i3c) before the payload.
                 *  - if only LSB of an register needed,
                 *     only the first byte has to be read and
                 *     second one will be discarded by slave automatically.
                 *     (so RX length = 3 (2 dummy bytes + LSB byte))
                 */
                len = 3;
            }

            /* Delay for n micro second. */
            sys_busy_loop_us(1000);

            /* For RX, User has to pass
             * Slave Address + Pointer to RX data + length of the RX data.
             */
            ret = I3Cdrv->MasterReceive(slave_addr[i], rx_data, len);
            if(ret != ARM_DRIVER_OK)
            {
                printf("\r\n Error: I3C Master Receive failed. \r\n");
                goto error_detach;;
            }

            /* wait till any event success/error comes in isr callback,
             *  and if event is set then clear that event.
             *   if the event flags are not set,
             *    this service suspends for a maximum of 100 timer-ticks.
             */
            wait_timer_ticks = 100;
            ret = tx_event_flags_get(&event_flags_i3c, \
                               I3C_CB_EVENT_SUCCESS | I3C_CB_EVENT_ERROR, \
                               TX_OR_CLEAR,                               \
                               &actual_events,                            \
                               wait_timer_ticks);
            if (ret != TX_SUCCESS)
            {
                printf("Error: I3C tx_event_flags_get failed. \n");
                goto error_detach;
            }

            /* Display received data depending on whether slave has given ACK or NACK.*/
            if(actual_events & I3C_CB_EVENT_SUCCESS)
            {
                cmp_rx_data = rx_data[0];

                if(slave_addr[i] == I2C_BMI_ADDR)
                {
                    /* BMI read: gives 2 bytes of dummy data[0][1] + LSB[2] */
                    cmp_rx_data = rx_data[2];
                }

                /* RX Success: Got ACK from slave */
                printf("\r\n \t\t >> i=%d RX Success: Got ACK from slave addr:0x%X.\r\n",  \
                               i, slave_addr[i]);

                printf("\r\n \t\t >> i=%d RX Received Data from slave:[0]0x%X. actual data:0x%X\r\n",  \
                               i,cmp_rx_data,actual_rx_data[i]);

                if(cmp_rx_data == actual_rx_data[i])
                {
                    printf("\r\n \t\t >> i=%d RX Received Data from slave is VALID.\r\n",i);
                }
                else
                {
                    printf("\r\n \t\t >> i=%d RX Received Data from slave is INVALID.\r\n",i);
                }
            }

            if(actual_events & I3C_CB_EVENT_ERROR)
            {
                /* RX Error: Got NACK from slave */
                printf("\r\n \t\t >> i=%d RX Error: Got NACK from slave addr:0x%X \r\n",  \
                               i, slave_addr[i]);
            }


            printf("\r\n ---------------------------XXX------------------------------ \r\n");
        }
    }


error_detach:

    /* Detach all attached i2c/i3c slave device. */
    for(i=0; i<TOTAL_SLAVE; i++)
    {
        printf("\r\n i=%d detaching i2c or i3c slave addr:0x%X from i3c.\r\n",i, slave_addr[i]);
        ret = I3Cdrv->Detachdev(slave_addr[i]);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Detach device failed.\r\n");
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

    /* Create the event flags group used by i3c thread  */
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
    status = tx_thread_create(&I3C_thread, "I3C_thread", mix_bus_i2c_i3c_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("Could not create thread \n");
        return;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
