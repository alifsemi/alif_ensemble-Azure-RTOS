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
 * @file     mix_bus_i2c_i3c_testApp.c
 * @author   Tanay Rami
 * @email    tanay@alifsemi.com
 * @version  V1.0.0
 * @date     16-May-2022
 * @brief    TestApp to verify Mix Bus i2c and i3c communication with
 *            multiple i2c + i3c slave devices using i3c IP
 *            with Azure RTOS (ThreadX)
 *            as an Operating System.
 *
 *           Select appropriate i3c Speed mode as per i2c or i3c slave device.
 *             I3C_BUS_MODE_PURE                             : Only Pure I3C devices
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS  : Fast Mode Plus   1 Mbps
 *             I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS : Fast Mode      400 Kbps
 *             I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS : Standard Mode  100 Kbps
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "system_utils.h"

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

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

/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C0;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C0;

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
}I3C_CB_EVENTS;


/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
INT hardware_init(void)
{
	INT ret = 0;

	/* i3c Pin-Mux */

	/* Configure GPIO Pin : P3_8 as I3C_SDA_B */
	ret = PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_3);
	if(ret != 0)
	{
		printf("\r\n Error: PINMUX failed.\r\n");
		return -1;
	}

	/* Configure GPIO Pin : P3_9 as I3C_SCL_B */
	ret = PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_4);
	if(ret != 0)
	{
		printf("\r\n Error: PINMUX failed.\r\n");
		return -1;
	}

	/* i3c Pin-Pad */

	/* Pin-Pad P3_8 as I3C_SDA_B
	 * Pad function: weak pull up(0x8) + read enable(0x01)
	 *               + Output drive strength 4mA(0x20)
	 */
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_8,  \
                          ( PAD_FUNCTION_READ_ENABLE                          |
                            PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP    |
                            PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS ) );
	if(ret != 0)
	{
		printf("\r\n Error: PINPAD failed.\r\n");
		return -1;
	}

	/* Pin-Pad P3_9 as I3C_SCL_B
	 * Pad function: weak pull up(0x8) + read enable(0x01)
	 *               + Output drive strength 4mA(0x20)
	 */
	ret = PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_9,  \
                          ( PAD_FUNCTION_READ_ENABLE                          |
                            PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_UP    |
                            PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS ) );
	if(ret != 0)
	{
		printf("\r\n Error: PINPAD failed.\r\n");
		return -1;
	}

	return 0;
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
                multiple i2c + i3c slave devices using i3c IP
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

/* Added 3 slaves for demo purpose
 *   i3c : Accelerometer and Magnometer,
 *   i2c : EEPROM
 */
#define TOTAL_SLAVE           3

/* ICM-42670-P Accelerometer Slave address(On-chip attached to A1 Base Board) */
#define I3C_ACCERO_ADDR       0x68

/* MMC5633NJL Magnometer Slave address(On-chip attached to A1 Base Board) */
#define I3C_MAGNETO_ADDR      0x30

/* EEPROM Slave address(On-chip attached to MEIB Board) */
#define I2C_EEPROM_ADDR       0x50

/* ICM-42670-P Accelerometer Slave chip-id register(WHO AM I) address and value
 *  as per datasheet
 */
#define I3C_ACCERO_REG_WHO_AM_I_ADDR        0x75
#define I3C_ACCERO_REG_WHO_AM_I_VAL         0x67

/* MMC5633NJL Magnometer Slave chip-id register(Product ID 1) address and value
 *  as per datasheet
 */
#define I3C_MAGNETO_REG_PRODUCT_ID_1_ADDR   0x39
#define I3C_MAGNETO_REG_PRODUCT_ID_1_VAL    0x10

/* Any EEPROM Location and its value. */
#define I2C_EEPROM_LOCATION                 0x32
#define I2C_EEPROM_LOCATION_VALUE           0xCD

	INT   i      = 0;
	INT   ret    = 0;
	INT   len    = 0;

    /* Array of slave address :
     *       Dynamic Address for i3c and
     *       Static  Address for i2c
     */
	UCHAR slave_addr[TOTAL_SLAVE] =
	{
		0, /* I3C Accero  Dynamic Address: To be updated later using MasterAssignDA */
		0, /* I3C Magneto Dynamic Address: To be updated later using MasterAssignDA */
		I2C_EEPROM_ADDR /* I2C EEPROM Slave Address. */
	};

	/* transmit data to i3c */
	UCHAR tx_data[TOTAL_SLAVE] =
	{
		I3C_ACCERO_REG_WHO_AM_I_ADDR,
		I3C_MAGNETO_REG_PRODUCT_ID_1_ADDR,
		I2C_EEPROM_LOCATION
	};

	/* receive data from i3c */
	UCHAR rx_data[1] = {0};

	/* actual receive data as per slave datasheet */
	UCHAR actual_rx_data[TOTAL_SLAVE] =
	{
		I3C_ACCERO_REG_WHO_AM_I_VAL,
		I3C_MAGNETO_REG_PRODUCT_ID_1_VAL,
		I2C_EEPROM_LOCATION_VALUE
	};

	ULONG actual_events = 0;
	ULONG wait_timer_ticks = 0;
	ARM_DRIVER_VERSION version;

	/* I3C CCC (Common Command Codes) */
	I3C_CMD i3c_cmd;
	uint8_t i3c_cmd_tx_data[1] = {0x0F};
	uint8_t i3c_cmd_rx_data[6] = {0};

	/* i3c Magneto Slave 48-bit Provisional ID. */
	uint8_t i3c_magneto_PID[6] = {0x04, 0xA2, 0x00, 0x00, 0xF0, 0x00};


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
	PMU_delay_loop_us(1000);

	/* Attach all i3c slave using dynamic address */

	/* Assign Dynamic Address to i3c Accelerometer */
	printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_ACCERO_ADDR);

	ret = I3Cdrv->MasterAssignDA(&slave_addr[0], I3C_ACCERO_ADDR);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
		goto error_poweroff;
	}
	printf("\r\n >> i3c: Received dyn_addr:0x%X for static addr:0x%X. \r\n",   \
                                 slave_addr[0],I3C_ACCERO_ADDR);

	/* Assign Dynamic Address to i3c Magnometer */
	printf("\r\n >> i3c: Get dynamic addr for static addr:0x%X.\r\n",I3C_MAGNETO_ADDR);

	/* Delay for n micro second.
	 *  @Note: Minor delay is required if prints are disable.
	 */
	PMU_delay_loop_us(1000);

	ret = I3Cdrv->MasterAssignDA(&slave_addr[1], I3C_MAGNETO_ADDR);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: I3C MasterAssignDA failed.\r\n");
		goto error_poweroff;
	}

	printf("\r\n >> i3c: Received dyn_addr:0x%X for static addr:0x%X. \r\n",   \
                                 slave_addr[1],I3C_MAGNETO_ADDR);

	/* Delay for n micro second. */
	PMU_delay_loop_us(1000);

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

	/* Delay for n micro second. */
	PMU_delay_loop_us(1000);

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

	/* Delay for n micro second. */
	PMU_delay_loop_us(1000);

	/* read I3C_CCC_GETPID (Get Provisional ID 48-bit) command from Magneto slave */
	i3c_cmd.rw     = 1;
	i3c_cmd.cmd_id = I3C_CCC_GETPID;
	i3c_cmd.len    = 6;
	i3c_cmd.addr   = slave_addr[1];
	i3c_cmd.data   = i3c_cmd_rx_data;

	ret = I3Cdrv->MasterSendCommand(&i3c_cmd);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: I3C MasterSendCommand failed.\r\n");
		goto error_detach;
	}

	/* Delay for n micro second. */
	PMU_delay_loop_us(1000);

	/* compare received 48-bit Provisional ID with actual for Magneto slave */
	if( memcmp(i3c_cmd_rx_data, i3c_magneto_PID, 6) == 0 )
	{
		printf("\r\n \t\t >> i3c magneto PID is VALID.\r\n");
	}
	else
	{
		printf("\r\n \t\t >> i3c magneto PID is INVALID.\r\n");
	}

	/* Delay for n micro second. */
	PMU_delay_loop_us(1000);

	/* Attach all i2c slave using static address */
	printf("\r\n >> i2c: Attaching i2c slave addr:0x%X to i3c...\r\n",slave_addr[2]);

	ret = I3Cdrv->AttachI2Cdev(slave_addr[2]);
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
	 *   Others Accero/Magneto/EEPROM supports  8-bit(1 Byte) reg-addr and (8/16/32 bit) data
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
                                 i, slave_addr[i], tx_data[i]);

			/* Delay for n micro second. */
			PMU_delay_loop_us(1000);

			/* For TX, User has to pass
			 * Slave Address + TX data + length of the TX data.
			 */
			ret = I3Cdrv->MasterTransmit(slave_addr[i], &tx_data[i], len);
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

			/* TX/RX length is 1 Byte
			 * (assume slave requires 8-bit data for TX/RX).
			 */
			len = 1;

			/* Delay for n micro second. */
			PMU_delay_loop_us(1000);

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
				/* RX Success: Got ACK from slave */
				printf("\r\n \t\t >> i=%d RX Success: Got ACK from slave addr:0x%X.\r\n",  \
                               i, slave_addr[i]);
				printf("\r\n \t\t >> i=%d RX Received Data from slave:[0]0x%X. actual data:0x%X\r\n",  \
                               i,rx_data[0],actual_rx_data[i]);

				if(rx_data[0] == actual_rx_data[i])
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
