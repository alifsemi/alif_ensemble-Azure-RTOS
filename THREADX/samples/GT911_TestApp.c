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
 * @file     GT911_TestApp.c
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     17-January-2023
 * @brief    TestApp to verify GT911 touch screen with
 *            Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "tx_api.h"

/* PINMUX Driver */
#include "pinconf.h"

/*touch screen driver */
#include "Driver_Touch_Screen.h"

/* Enable/Disable Redirect printf to UART.
* Providing option to user to select where to display GT911 touch screen coordinates.
*/

#define PRINTF_REDIRECT	0

#if PRINTF_REDIRECT

/* USART Driver */
#include "Driver_USART.h"
#include "uart.h"
#include <RTE_Components.h>
#include CMSIS_device_header

/* UART Driver instance (UART0-UART7) */
#define UART      4

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/* Disable Semihosting */
#if __ARMCC_VERSION >= 6000000
__asm(".global __use_no_semihosting");
#elif __ARMCC_VERSION >= 5000000
#pragma import(__use_no_semihosting)
#else
#error Unsupported compiler
#endif

void _sys_exit(int return_code)
{
    while (1);
}

/* Used by fputc() */
void uartWrite(char c);

/* Used by fgetc() */
char uartRead(void);

FILE __stdout;
FILE __stdin;

#endif

/* Touch screen driver instance */
extern ARM_DRIVER_TOUCH_SCREEN GT911;
static ARM_DRIVER_TOUCH_SCREEN *Drv_Touchscreen = &GT911;

void touchscreen_demo_thread_entry(ULONG thread_input);

#define GT911_TOUCH_INT_GPIO_PORT        PORT_9
#define GT911_TOUCH_INT_PIN_NO           PIN_4
#define GT911_TOUCH_I2C_SDA_PORT         PORT_7
#define GT911_TOUCH_I2C_SDA_PIN_NO       PIN_2
#define GT911_TOUCH_I2C_SCL_PORT         PORT_7
#define GT911_TOUCH_I2C_SCL_PIN_NO       PIN_3
#define PRINTF_REDIRECT_UART_RX_PORT     PORT_12
#define PRINTF_REDIRECT_UART_RX_PIN_NO   PIN_1
#define PRINTF_REDIRECT_UART_TX_PORT     PORT_12
#define PRINTF_REDIRECT_UART_TX_PIN_NO   PIN_2
#define ACTIVE_TOUCH_POINTS              5

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                  2048
#define DEMO_BYTE_POOL_SIZE              9120

TX_THREAD                                touchscreen_thread;
TX_BYTE_POOL                             byte_pool_0;
UCHAR                                    memory_area[DEMO_BYTE_POOL_SIZE];

#if PRINTF_REDIRECT

/* Redefine standard fputc function to redirect printf to UART instead of standard output */
int fputc(int c, FILE * stream)
{
    uartWrite(c);
    /* return the character written to denote a successful write */
    return c;
}

/* Redefine standard fgetc function to get input from UART instead of standard input */
int fgetc(FILE * stream)
{
    char c = uartRead();
    /* To echo Received characters back to serial Terminal */
    uartWrite(c);
    return c;
}

/* Used by fputc() */
void uartWrite(char c)
{
    UART_Type *uart_reg_ptr = (UART_Type *)UART4_BASE;

    /* wait until uart is to ready to send */
    while ( (uart_reg_ptr->UART_USR & UART_USR_TRANSMIT_FIFO_NOT_FULL) == 0 );

    /* write a char to thr transmit holding register */
    uart_reg_ptr->UART_THR = c;
}

/* Used by fgetc() */
char uartRead(void)
{
    /* Device specific code to Receive a byte from RX pin.
     * return received chararacter(byte)
     */
    UART_Type *uart_reg_ptr = (UART_Type *)UART4_BASE;

    /* wait until uart is ready to receive */
    while ( (uart_reg_ptr->UART_USR & UART_USR_RECEIVE_FIFO_NOT_EMPTY) == 0 );

    /* read a char from receive buffer register */
    return (int32_t)uart_reg_ptr->UART_RBR;
}

/**
  \fn          void UART_callback(UINT event)
  \brief       UART callback
  \param[in]   event: UART Event
  \return      none
  */
void UART_callback(UINT event)
{
    if (event & ARM_USART_EVENT_SEND_COMPLETE)
    {
        /* We are using polling method so interrupt events not used*/
    }

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        /* We are using polling method so interrupt events not used*/
    }

    if (event & ARM_USART_EVENT_RX_TIMEOUT)
    {
        /* We are using polling method so interrupt events not used*/
    }
}

#endif

/**
  \fn          int hardware_cfg(void)
  \brief       i2c hardware pin initialization:
                   -  PIN-MUX configuration
                   -  PIN-PAD configuration
                 -  GPIO9 initialization:
                   -  PIN-MUX configuration
                   -  PIN-PAD configuration
                 -  UART hardware pin initialization (if printf redirection to UART is chosen):
                   -  PIN-MUX configuration for UART receiver
                   -  PIN-MUX configuration for UART transmitter
  \param[in]   none
  \return      ARM_DRIVER_OK: success; 0: failure
  */
int hardware_cfg(void)
{
    INT ret = 0;

    /* gpio9 config for interrupt
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP |
     *               PADCTRL_SCHMITT_TRIGGER_ENABLE
     */
    ret = pinconf_set(GT911_TOUCH_INT_GPIO_PORT, GT911_TOUCH_INT_PIN_NO, PINMUX_ALTERNATE_FUNCTION_0, PADCTRL_READ_ENABLE |\
                     PADCTRL_SCHMITT_TRIGGER_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: GPIO PINMUX failed.\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P7_2 as i2c1_sda_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(GT911_TOUCH_I2C_SDA_PORT, GT911_TOUCH_I2C_SDA_PIN_NO, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE | \
                     PADCTRL_DRIVER_DISABLED_PULL_UP);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C SDA PINMUX failed.\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P7_3 as i2c1_scl_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(GT911_TOUCH_I2C_SCL_PORT, GT911_TOUCH_I2C_SCL_PIN_NO, PINMUX_ALTERNATE_FUNCTION_5,PADCTRL_READ_ENABLE | \
                     PADCTRL_DRIVER_DISABLED_PULL_UP);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I2C SCL PINMUX failed.\r\n");
        return ret;
    }

#if PRINTF_REDIRECT
    /* Configure GPIO Pin : P12_1 as UART4 RX_B
     * Pad function: PADCTRL_READ_ENABLE
     */
    ret = pinconf_set (PRINTF_REDIRECT_UART_RX_PORT, PRINTF_REDIRECT_UART_RX_PIN_NO, PINMUX_ALTERNATE_FUNCTION_2,
                      PADCTRL_READ_ENABLE);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: UART PINMUX as receiver failed.\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P12_2 as UART4 TX_B */
    ret = pinconf_set (PRINTF_REDIRECT_UART_TX_PORT, PRINTF_REDIRECT_UART_TX_PIN_NO, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: UART PINMUX as transmitter failed.\r\n");
        return ret;
    }

#endif

    return ARM_DRIVER_OK;
}

/**
 * @function    void touchscreen_demo_thread_entry(ULONG thread_input)
\brief          TestApp to verify GT911 touch screen with
                Azure RTOS ThreadX as an Operating System.
                This demo thread does:
                    - initialize i2c AND gpio9 port hardware pins
                    - initialize UART hardware pins if print redirection to UART is chosen
                    - initialize USART driver if printf redirection to UART is chosen.
                    - initialize GT911 Touch screen driver with call back function.
                    - check if touch screen is pressed or not
                    - if pressed then printupto 5 coordinate positions where display touch screen was touched.
  \param[in]   thread_input : thread input
  \return      none
  */
void touchscreen_demo_thread_entry(ULONG thread_input)
{
    INT ret = 0;
    INT count = 0;
    ARM_DRIVER_VERSION version;
    ARM_TOUCH_STATE state;

    /* Initialize i2c and GPIO9 hardware pins using PinMux Driver. */
    /* Initialize UART4 hardware pins using PinMux driver if printf redirection to UART is selected */
    ret = hardware_cfg();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in hardware configuration */
        printf("\r\n Error: Hardware configuration failed.\r\n");
    }

#if PRINTF_REDIRECT

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(UART_callback);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Initialize. */
        printf("\r\n ERROR: Initialize UART failed.\r\n");
        return;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Power Up. */
        printf("ERROR: UART power ON failed.\r\n");
        goto error_uninitialize_UART;
    }

    /* Configure UART to 115200 Bits/sec */
    ret = USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                              ARM_USART_DATA_BITS_8       |
                              ARM_USART_PARITY_NONE       |
                              ARM_USART_STOP_BITS_1       |
                              ARM_USART_FLOW_CONTROL_NONE, 115200);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control. */
        printf("ERROR: UART configuration failed.\r\n");
        goto error_UART_poweroff;
    }

    /* Enable Receiver and Transmitter lines */
    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control TX. */
        printf("ERROR: UART transmitter configuration failed.\r\n");
        goto error_UART_poweroff;
    }

    ret =  USARTdrv->Control(ARM_USART_CONTROL_RX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control RX. */
        printf("ERROR: UART receiver configuration failed.\r\n");
        goto error_UART_poweroff;
    }

#endif

    /* Touch screen version */
    version = Drv_Touchscreen->GetVersion();
    printf("\r\n Touchscreen driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    /* Initialize GT911 touch screen */
    ret = Drv_Touchscreen->Initialize();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in GT911 touch screen initialize */
        printf("\r\n Error: GT911 touch screen initialization failed.\r\n");
        goto error_GT911_uninitialize;
    }

    /* Power ON GT911 touch screen */
    ret = Drv_Touchscreen->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in GT911 touch screen power up */
        printf("\r\n Error: GT911 touch screen Power Up failed.\r\n");
        goto error_GT911_uninitialize;
    }

    while(1)
    {
        /* Reading GT911 touch screen press status */
        ret =Drv_Touchscreen->GetState(&state);
        if(ret != ARM_DRIVER_OK)
        {
            /* Error in GT911 touch screen read status */
            printf("\r\n Error: GT911 touch screen read  status failed.\r\n");
            goto error_GT911_poweroff;
        }

        if(state.numtouches){
            for(count = 1; count <= ACTIVE_TOUCH_POINTS; count++)
            {
                /* Print coordinates positions of pressing on GT911 touch screen up to max touch points set */
                printf("x%d: %d y%d: %d \r\n",count, state.coordinates[count - 1].x, count, state.coordinates[count - 1].y);
            }
            memset(state.coordinates,0,sizeof(state.coordinates));
        }
    }

error_GT911_poweroff:
    /* Received error Power off GT911 touch screen peripheral */
    ret = Drv_Touchscreen->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in GT911 Touch screen Power OFF. */
        printf("ERROR: Could not power OFF touch screen\n");
        return;
    }

error_GT911_uninitialize:
    /* Received error Un-initialize Touch screen driver */
    ret = Drv_Touchscreen->Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        /* Error in GT911 Touch screen uninitialize. */
        printf("ERROR: Could not unintialize touch screen\n");
        return;
    }

#if PRINTF_REDIRECT

error_UART_poweroff:
    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Power OFF. */
        printf("ERROR: Could not power OFF UART\n");
        return;
    }

error_uninitialize_UART:
    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Uninitialize. */
        printf("ERROR: Could not uninitialize UART\n");
        return;
    }

#endif
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
    UINT     status  = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte pool\n");
        return;
    }

    /* Allocate the stack for GT911 touch screen thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte allocate\n");
        goto error_delete_byte_pool;
    }

    /* Create the main thread for GT911 touch screen.  */
    status = tx_thread_create(&touchscreen_thread, "TouchScreen_thread", touchscreen_demo_thread_entry, 0,
                             pointer, DEMO_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create thread \n");
        goto error_release_allocated_byte;
    }

    /* SUCCESS */
    return;

error_release_allocated_byte:
    /* Received release allocated bytes */
    status = tx_byte_release((VOID *) pointer);
    if (status != TX_SUCCESS)
    {
        /* Error in releasing allocated bytes */
        printf("ERROR: Could not release byte allocate\n");
    }

error_delete_byte_pool:
    /* Received delete byte pool */
    status = tx_byte_pool_delete(&byte_pool_0);
    if (status != TX_SUCCESS){
        /* Error in deleting byte pool */
        printf("ERROR: Could not delete byte pool\n");
    }

    while(1);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
