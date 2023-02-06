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
 * @file     THERMOMETER_app.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     16-June-2021
 * @brief    demo application for SPI.
 *           - Configuring the DS1722 Thermometer in continuous conversion mode with 12 bit resolution.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "tx_api.h"
#include <stdio.h>
#include "Driver_GPIO.h"
#include "Driver_SPI.h"
#include "Driver_PINMUX_AND_PINPAD.h"

#define DS1722_CMD_READ(address)                        (address<<8)
#define DS1722_CMD_WRITE(address,data)                  ((address<<8) | data)

#define DEMO_BYTE_POOL_SIZE                             (1024)
#define DS1722_THREAD_STACK_SIZE                        (512)
#define THERMOMETER_CALLBACK_EVENT                      0x1
#define THERMOMETER_EVENT_WAIT_TIME                     1000

#define DS1722_THERMOMETER_OK                           0
#define DS1722_THERMOMETER_ERROR                        (-1)

#define DS1722_THERMOMETER_SHUTDOWN_BIT                 0
#define DS1722_THERMOMETER_SHUTDOWN_BIT_mask            (0x1UL << DS1722_THERMOMETER_SHUTDOWN_BIT)

#define DS1722_THERMOMETER_RESOLUTION_BIT               1
#define DS1722_THERMOMETER_RESOLUTION_BIT_mask          (0x07UL << DS1722_THERMOMETER_RESOLUTION_BIT)
#define DS1722_THERMOMETER_RESOLUTION_R0_BIT            0x2
#define DS1722_THERMOMETER_RESOLUTION_R1_BIT            0x4
#define DS1722_THERMOMETER_RESOLUTION_R2_BIT            0x8

#define DS1722_THERMOMETER_ONE_SHOT_BIT                 4
#define DS1722_THERMOMETER_ONE_SHOT_BIT_mask            (0x01UL << DS1722_THERMOMETER_ONE_SHOT_BIT)

#define DS1722_THERMOMETER_CONVERSIONS_INACTIVE         0
#define DS1722_THERMOMETER_CONVERSIONS_ACTIVE           DS1722_THERMOMETER_SHUTDOWN_BIT_mask

#define DS1722_THERMOMETER_RESOLUTION_08                0
#define DS1722_THERMOMETER_RESOLUTION_09                DS1722_THERMOMETER_RESOLUTION_R0_BIT
#define DS1722_THERMOMETER_RESOLUTION_10                DS1722_THERMOMETER_RESOLUTION_R1_BIT
#define DS1722_THERMOMETER_RESOLUTION_11                (DS1722_THERMOMETER_RESOLUTION_R0_BIT | DS1722_THERMOMETER_RESOLUTION_R1_BIT)
#define DS1722_THERMOMETER_RESOLUTION_12                DS1722_THERMOMETER_RESOLUTION_R2_BIT

#define DS1722_THERMOMETER_CONTINUOUS_CONVERSION_MODE   0
#define DS1722_THERMOMETER_DISCONTINUOUS_CONVERSION_MODE DS1722_THERMOMETER_ONE_SHOT_BIT_mask

#define DS1722_THERMOMETER_RESERVED_BIT                 0xE0

#define DS1722_SET_CONFIGURATION_ADDR                   0x80U
#define DS1722_GET_CONFIGURATION_ADDR                   0x00U
#define DS1722_TEMPERATURE_MSB_ADDR                     0x02U
#define DS1722_TEMPERATURE_LSB_ADDR                     0x01U

#define GPIO                                            2        /* GPIO 2 */
#define SPI                                             3        /* DS1722 connected to SPI instance 3 */

#define SPI_PORT                                        PORT_NUMBER_2

#define SPI_MISO_PIN                                    PIN_NUMBER_21
#define SPI_MOSI_PIN                                    PIN_NUMBER_22
#define SPI_CLOCK_PIN                                   PIN_NUMBER_23
#define SPI_SS0_PIN                                     PIN_NUMBER_24

#define SPI_MISO_PIN_AF                                 PINMUX_ALTERNATE_FUNCTION_4
#define SPI_MOSI_PIN_AF                                 PINMUX_ALTERNATE_FUNCTION_3
#define SPI_CLOCK_PIN_AF                                PINMUX_ALTERNATE_FUNCTION_3
#define SPI_SS0_PIN_AF                                  PINMUX_ALTERNATE_FUNCTION_0

UCHAR                                                   memory_area[DEMO_BYTE_POOL_SIZE];
TX_BYTE_POOL                                            memory_pool;
TX_THREAD                                               ds1722_thread;
ULONG                                                   events;
TX_EVENT_FLAGS_GROUP                                    thermometer_event_flag;

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO);
ARM_DRIVER_GPIO *ptrGPIO = &ARM_Driver_GPIO_(GPIO);

extern ARM_DRIVER_SPI ARM_Driver_SPI_(SPI);
ARM_DRIVER_SPI *ptrSPI_temp = &ARM_Driver_SPI_(SPI);

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
 * @fn      void ARM_SPI_Cb_Func (uint32_t event)
 * @brief   Ds1722 application call back function.
 * @note    none.
 * @param   event: SPI event.
 * @retval  None
 */
static void ARM_SPI_Cb_Func (uint32_t event)
{
    if (event == ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_LOW);

        tx_event_flags_set( &thermometer_event_flag, THERMOMETER_CALLBACK_EVENT, TX_OR);
    }
}

/**
 * @fn      void ds1722_app (ULONG thread_input)
 * @brief   Used to read the temperature from DS1722 thermometer.
 * @note    none.
 * @param   thread_input : thread input.
 * @retval  none.
 */
static void ds1722_app (ULONG thread_input)
{
    uint16_t temp_lsb, temp_msb, config_cmd_and_value;
    int32_t ctrl_buf, ret = 0;
    uint32_t status, control_code, arg;
    uint16_t higher_byte[2], lower_byte[2];
    ARM_SPI_STATUS var;

    /* Doing pinmux configuration for SPI pin's */
    ret = PINMUX_Config(SPI_PORT, SPI_MISO_PIN, SPI_MISO_PIN_AF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure PINMUX for SPI_MISO_PIN\n");
        return;
    }

    ret = PINMUX_Config(SPI_PORT, SPI_MOSI_PIN, SPI_MOSI_PIN_AF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure PINMUX for SPI_MOSI_PIN\n");
        return;
    }

    ret = PINMUX_Config(SPI_PORT, SPI_CLOCK_PIN, SPI_CLOCK_PIN_AF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure PINMUX for SPI_CLOCK_PIN\n");
        return;
    }

    ret = PINMUX_Config(SPI_PORT, SPI_SS0_PIN, SPI_SS0_PIN_AF);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure PINMUX for SPI_SS0_PIN\n");
        return;
    }

    /*
     * DS1722 Thermometer sensor "CE (Chip Enable)" pin behavior is opposite to standard SPI chip selection behavior.
     * To operate CE (Chip Enable) pin configuring SPI_SS0_PIN as GPIO, following Bit-Banging method.
    */

    ret = ptrGPIO->Initialize(SPI_SS0_PIN, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize SPI_SS0_PIN as GPIO\n");
        return;
    }

    ret = ptrGPIO->PowerControl(SPI_SS0_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Powered the SPI_SS0_PIN\n");
        goto error_gpio_uninitialize;
    }

    ret = ptrGPIO->SetDirection(SPI_SS0_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to set direction for SPI_SS0_PIN\n");
        goto error_gpio_power_off;
    }

    ret = ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to set the state for SPI_SS0_PIN\n");
        goto error_gpio_power_off;
    }

    /*
     * SPI-3 connected to DS1722 Thermometer sensor. configuring to read the temperature.
     */

    control_code = (ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_SW | ARM_SPI_CPOL0_CPHA1|
                    ARM_SPI_DATA_BITS(16));

    arg = 1000000;  // SPI Baud rate 1 MHz

    temp_lsb  =      DS1722_CMD_READ (DS1722_TEMPERATURE_LSB_ADDR);
    temp_msb  =      DS1722_CMD_READ (DS1722_TEMPERATURE_MSB_ADDR);
    config_cmd_and_value =  DS1722_CMD_WRITE (DS1722_SET_CONFIGURATION_ADDR, \
                            (DS1722_THERMOMETER_RESOLUTION_08 | DS1722_THERMOMETER_CONTINUOUS_CONVERSION_MODE));

    printf("*** Thermometer demo app using SPI is starting ***\n");

    ret = ptrSPI_temp->Initialize(ARM_SPI_Cb_Func);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to initialize the SPI3\n");
        goto error_gpio_power_off;
    }

    ret = ptrSPI_temp->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to power SPI3\n");
        goto error_spi_uninitialize;
    }

    ret = ptrSPI_temp->Control(control_code, arg);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to configure SPI3\n");
        goto error_spi_power_off;
    }

    ret = ptrSPI_temp->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to enable the slave select of SPI3\n");
        goto error_spi_power_off;
    }

    ret = ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Set SPI_SS0_PIN state to high\n");
        goto error_spi_power_off;
    }

    /* Configuring the DS1722 thermometer to 8 bit resolution, Continues operation mode */
    ret = ptrSPI_temp->Send(&config_cmd_and_value, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Configure the Ds1722-THERMOMETER\n");
        goto error_spi_power_off;
    }

    status = tx_event_flags_get (&thermometer_event_flag, THERMOMETER_CALLBACK_EVENT, TX_OR_CLEAR, &events, THERMOMETER_EVENT_WAIT_TIME);
    if (status != TX_SUCCESS)
    {
        printf("ERROR : event not received, timeout happened \n");
        goto error_spi_power_off;
    }

    ret = ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Set SPI_SS0_PIN state to high\n");
        goto error_spi_power_off;
    }

    /* Receive DS1722 thermometer configuration register value which is at 0x0 of thermometer */
    ret = ptrSPI_temp->Receive(&ctrl_buf, 1);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to Configure the Ds1722-THERMOMETER\n");
        goto error_spi_power_off;
    }

    status = tx_event_flags_get (&thermometer_event_flag, THERMOMETER_CALLBACK_EVENT, TX_OR_CLEAR, &events, THERMOMETER_EVENT_WAIT_TIME);
    if (status != TX_SUCCESS)
    {
        printf("ERROR : event not received, timeout happened \n");
        goto error_spi_power_off;
    } else {
        printf("Thermometer configuration reg value :0x%x\n", ctrl_buf);
    }

    while(1)
    {
        ret = ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (ret != ARM_DRIVER_OK)
        {
            printf("ERROR: Failed to configure O/p state SPI_SS0_PIN\n");
            goto error_spi_power_off;
        }

        ret = ptrSPI_temp->Transfer(&temp_msb, higher_byte, 1);
        if (ret != ARM_DRIVER_OK)
        {
            printf("ERROR: Failed to SPI Transfer %x\n", ret);
            goto error_spi_power_off;
        }

        status = tx_event_flags_get (&thermometer_event_flag, THERMOMETER_CALLBACK_EVENT, TX_OR_CLEAR, &events, THERMOMETER_EVENT_WAIT_TIME);
        if (status != TX_SUCCESS)
        {
            printf("ERROR : event not received, timeout happened \n");
            goto error_spi_power_off;
        }

        var = ptrSPI_temp->GetStatus();

        while(var.busy == 1)
        {
            var = ptrSPI_temp->GetStatus();
        }

        /* Between the command providing delay of 5 ticks */
        tx_thread_sleep(5);

        ptrGPIO->SetValue(SPI_SS0_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
        if (ret != ARM_DRIVER_OK)
        {
            printf("ERROR: Failed to configure O/p state SPI_SS0_PIN\n");
            goto error_spi_power_off;
        }

        ret = ptrSPI_temp->Transfer(&temp_lsb, lower_byte, 1);
        if (ret != ARM_DRIVER_OK)
        {
            printf("ERROR: Failed to SPI Transfer %x\n", ret);
            goto error_spi_power_off;
        }

        status = tx_event_flags_get (&thermometer_event_flag, THERMOMETER_CALLBACK_EVENT, TX_OR_CLEAR, &events, THERMOMETER_EVENT_WAIT_TIME);
        if (status != TX_SUCCESS)
        {
            printf("ERROR : event not received, timeout happened \n");
            goto error_spi_power_off;
        }

        printf("Temperature = %d.%d Degree Celsius \n", higher_byte[0], (lower_byte[0] >>4)*0625);
    }

error_spi_power_off :
    ret = ptrSPI_temp->PowerControl(ARM_POWER_OFF);
    if (ret == ARM_DRIVER_OK)
    {
        printf("Power off SPI3\n");
    }

error_spi_uninitialize :
    ret = ptrSPI_temp->Uninitialize();
    if (ret == ARM_DRIVER_OK)
    {
        printf("Un-Initialized SPI3\n");
    }

error_gpio_power_off :
    ret = ptrGPIO->PowerControl(SPI_SS0_PIN, ARM_POWER_OFF);
    if (ret == ARM_DRIVER_OK)
    {
        printf("Power off SPI_SS0_PIN\n");
    }

error_gpio_uninitialize :
    ret = ptrGPIO->Uninitialize(SPI_SS0_PIN);
    if (ret == ARM_DRIVER_OK)
    {
        printf("Un-Initialized SPI_SS0_PIN\n");
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
    UINT status;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create (&memory_pool, "memory pool", memory_area, DEMO_BYTE_POOL_SIZE);

    if (status != TX_SUCCESS) {
        printf("failed to create to byte Pool\r\n");
    }

    /* Create a event flag for thermometer group.  */
    status = tx_event_flags_create (&thermometer_event_flag, "THERMOMETER_EVENT_FLAG");

    if (status != TX_SUCCESS) {
        printf("failed to create thermometer event flag\r\n");
        goto tx_error_delete_pool;
    }

    /* Allocate the stack for thread 0.  */
    status = tx_byte_allocate (&memory_pool, (VOID **) &pointer, DS1722_THREAD_STACK_SIZE, TX_NO_WAIT);

    if (status != TX_SUCCESS) {
        printf("failed to allocate stack for ds1722 thread\r\n");
        goto tx_error_delete_event_flag;
    }

    /* Create the main thread.  */
    status = tx_thread_create (&ds1722_thread, "DS1722 DEMO", ds1722_app, 0, pointer, DS1722_THREAD_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS) {
        printf("failed to create ds1722 demo thread\r\n");

    }

    return;

tx_error_delete_event_flag:
    tx_event_flags_delete (&thermometer_event_flag);

tx_error_delete_pool:
    tx_byte_pool_delete (&memory_pool);
}
