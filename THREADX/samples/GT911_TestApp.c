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
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/*touch screen driver */
#include "Driver_Touch_Screen.h"

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
#define ACTIVE_TOUCH_POINTS              5

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                  2048
#define DEMO_BYTE_POOL_SIZE              9120

TX_THREAD                                touchscreen_thread;
TX_BYTE_POOL                             byte_pool_0;
UCHAR                                    memory_area[DEMO_BYTE_POOL_SIZE];


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
