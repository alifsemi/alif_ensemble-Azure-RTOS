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
 * @file     i2s_testapp.c
 * @author   Sudarshan Iyengar
 * @email    sudarshan.iyengar@alifsemi.com
 * @version  V2.0.0
 * @date     28-Sep-2020
 * @brief    Test Application for I2S for Carrier board.
 *           I2S0 is configured as master transmitter with MCLK 24.576Mhz and 24bit
 *           I2S2 is configured as master receiver SPH0645LM4H-1 device 24bit
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

/*System Includes */
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include <Driver_SAI.h>
#include <Driver_PINMUX_AND_PINPAD.h>

/*Threadx Includes */
#include "tx_api.h"

/*Audio samples */
#include "i2s_samples.h"

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                        1024
#define DEMO_BYTE_POOL_SIZE                    4096


/* Enable this to feed the predefined hello sample in the TX path and RX path is disabled */
//#define DAC_PREDEFINED_SAMPLES
/* 1 to send the data stream continuously , 0 to send data only once */
#define REPEAT_TX 1

#define ERROR  -1
#define SUCCESS 0

#define I2S_DAC 0		     /* DAC I2S Controller 0 */
#define I2S_ADC 2                    /* ADC I2S Controller 2 */

#define DAC_SEND_COMPLETE_EVENT    (0x1)
#define ADC_RECEIVE_COMPLETE_EVENT (0x2)
#define ADC_RECEIVE_OVERFLOW_EVENT (0x3)

#define ADC_RECEIVE_TIMEOUT        (10000)
#define DAC_SEND_TIMEOUT           (10000)

#define NUM_SAMPLES_IN_SINGLE_POOL 10000
#define SAMPLES_POOL_CNT           3
#define BLOCK_POOL_METADATA_SIZE   16
#define BLOCK_POOL_SIZE            ((SAMPLES_POOL_CNT * NUM_SAMPLES_IN_SINGLE_POOL * 4)\
                                   + (SAMPLES_POOL_CNT * BLOCK_POOL_METADATA_SIZE))

UCHAR                   memory[DEMO_BYTE_POOL_SIZE];
UCHAR                   samples_area[BLOCK_POOL_SIZE];
TX_BYTE_POOL            byte_pool_0;
TX_BLOCK_POOL           sample_block_pool;

TX_THREAD               DAC_thread;/* Thread id of thread: DAC WM8524 Transmitter */
TX_EVENT_FLAGS_GROUP    event_flags_dac;

TX_THREAD               ADC_thread;/* Thread id of thread: ADC Receiver */
TX_EVENT_FLAGS_GROUP    event_flags_adc;

ULONG                   events;

TX_QUEUE                samples_msgq;

/* message information for queueing */
typedef struct _samples_msgq
{
    void     *buf;
    uint32_t num_items;
}samples_msgq_t;

samples_msgq_t       samples_msg_queue[SAMPLES_POOL_CNT];

uint32_t wlen = 24;
uint32_t sampling_rate = 48000;        /* 48Khz audio sampling rate */



/**
  \fn          void dac_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
void dac_callback(uint32_t event)
{
    if(event & ARM_SAI_EVENT_SEND_COMPLETE)
    {
	/* Send Success: Wake-up Thread. */
	tx_event_flags_set(&event_flags_dac, DAC_SEND_COMPLETE_EVENT, TX_OR);
    }
}

/**
  \fn          void dac_pinmux_config(void)
  \brief       Initialize the pinmux for DAC
  \return      status
*/
int32_t dac_pinmux_config(void)
{
    int32_t status;

    /* Configure I2S0 SDO */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_29, PINMUX_ALTERNATE_FUNCTION_3);
    if(status)
        return ERROR;

    /* Configure I2S0 WS */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_31, PINMUX_ALTERNATE_FUNCTION_3);
    if(status)
        return ERROR;

    /* Configure I2S0 SCLK */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_30, PINMUX_ALTERNATE_FUNCTION_2);
    if(status)
        return ERROR;

    return SUCCESS;
}

/**
  \fn          void DAC_Thread(ULONG thread_input)
  \brief       DAC thread for master transmission
  \param[in]   argument Thread input
*/
void DAC_Thread(ULONG thread_input)
{
    ARM_DRIVER_VERSION   version;
    ARM_DRIVER_SAI       *i2s_drv;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;
    samples_msgq_t       samples_msg;

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_DAC);

    (void)thread_input;

    /* Configure the dac pins */
    if(dac_pinmux_config())
    {
        printf("DAC pinmux failed\n");
        return;
    }

    /* Use the I2S0 as Trasmitter */
    i2s_drv = &ARM_Driver_SAI_(I2S_DAC);

    /* Verify the I2S API version for compatibility*/
    version = i2s_drv->GetVersion();
    printf("I2S API version = %d\n", version.api);

    /* Verify if I2S protocol is supported */
    cap = i2s_drv->GetCapabilities();
    if(!cap.protocol_i2s)
    {
        printf("I2S is not supported\n");
        return;
    }

    /* Initializes I2S0 interface */
    status = i2s_drv->Initialize(dac_callback);
    if(status)
    {
        printf("DAC Init failed status = %d\n", status);
        goto error_initialize;
    }

    /* Enable the power for I2S0 */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if(status)
    {
        printf("DAC Power Failed status = %d\n", status);
        goto error_power;
    }

    /* configure I2S0 Transmitter to Asynchronous Master */
    status = i2s_drv->Control(ARM_SAI_CONFIGURE_TX |
                                ARM_SAI_MODE_MASTER  |
                                ARM_SAI_ASYNCHRONOUS |
                                ARM_SAI_PROTOCOL_I2S |
                                ARM_SAI_DATA_SIZE(wlen), wlen*2, sampling_rate);
    if(status)
    {
        printf("DAC Control status = %d\n", status);
        goto error_control;
    }

    /* enable Transmitter */
    status = i2s_drv->Control(ARM_SAI_CONTROL_TX, 1, 0);
    if(status)
    {
        printf("DAC TX status = %d\n", status);
        goto error_control;
    }

    do
    {
#ifdef DAC_PREDEFINED_SAMPLES
        /* If we are using predefined samples, update the pointer and total len */
        samples_msg.buf = hello_samples_24bit_48khz;
        samples_msg.num_items = sizeof(hello_samples_24bit_48khz)/sizeof(hello_samples_24bit_48khz[0]);

        /* Transmit the samples */
        status = i2s_drv->Send(samples_msg.buf, samples_msg.num_items);
        if(status)
        {
            printf("DAC Send status = %d\n", status);
            goto error_send;
        }

        /* Wait for the completion event */
        status = tx_event_flags_get(&event_flags_dac,DAC_SEND_COMPLETE_EVENT, TX_OR_CLEAR, &events,DAC_SEND_TIMEOUT);
        if(status)
        {
            printf("Timeout occurred while sending the data to DAC\n");
            goto error_send;
        }

#else
        /* Read the samples from the queue */
        status = tx_queue_receive(&samples_msgq, &samples_msg, TX_WAIT_FOREVER);
        if(status == TX_SUCCESS)
        {
            /* Transmit the samples */
            status = i2s_drv->Send(samples_msg.buf, samples_msg.num_items);
            if(status)
            {
                printf("DAC Send status = %d\n", status);
                tx_block_release(samples_msg.buf);
                goto error_send;
            }

            /* Wait for the completion event */
            status = tx_event_flags_get(&event_flags_dac,DAC_SEND_COMPLETE_EVENT, TX_OR_CLEAR, &events ,DAC_SEND_TIMEOUT);
            if(status)
            {
                printf("Timeout occurred while sending the data to DAC\n");
                tx_block_release(samples_msg.buf);
                goto error_send;
            }

            status = tx_block_release(samples_msg.buf);
            if(status)
            {
                printf("Error in release of memory\n");
                goto error_send;
            }

        }
#endif
    }while(REPEAT_TX);

    /* Stop the TX */
    status = i2s_drv->Control(ARM_SAI_CONTROL_TX, 0, 0);
    if(status)
    {
        printf("DAC TX status = %d\n", status);
        goto error_control;
    }

error_send:
error_control:
    i2s_drv->PowerControl(ARM_POWER_OFF);
error_power:
    i2s_drv->Uninitialize();
error_initialize:
    tx_thread_sleep(TX_WAIT_FOREVER);
}

/**
  \fn          void adc_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
void adc_callback(uint32_t event)
{
    if(event & ARM_SAI_EVENT_RECEIVE_COMPLETE)
    {
	/* Send Success: Wake-up Thread. */
	tx_event_flags_set(&event_flags_adc, ADC_RECEIVE_COMPLETE_EVENT, TX_OR);
    }

    if(event & ARM_SAI_EVENT_RX_OVERFLOW)
    {
	/* Send Success: Wake-up Thread. */
	tx_event_flags_set(&event_flags_dac, ADC_RECEIVE_OVERFLOW_EVENT, TX_OR);
    }
}

/**
  \fn          void adc_pinmux_config(void)
  \brief       Initialize the pinmux for ADC
  \return      status
*/
int32_t adc_pinmux_config(void)
{
    int32_t status;

    /* Configure I2S2 WS */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_2);
    if(status)
        return ERROR;

    /* Configure I2S2 SCLK */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_3);
    if(status)
        return ERROR;

    /* Configure I2S2 SDI */
    status = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_3);
    if(status)
        return ERROR;

    return SUCCESS;
}

/**
  \fn          void ADC_Thread(ULONG thread_input)
  \brief       ADC Thread to handle receive
  \param[in]   argument Pointer to the argument
*/
void ADC_Thread(ULONG thread_input)
{
    ARM_DRIVER_VERSION   version;
    ARM_DRIVER_SAI       *i2s_drv;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;
    samples_msgq_t       samples_msg;

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_ADC);
    (void)thread_input;

    /* Configure the adc pins */
    if(adc_pinmux_config())
    {
        printf("ADC pinmux failed\n");
        return;
    }

    /* Use the I2S2 as Receiver */
    i2s_drv = &ARM_Driver_SAI_(I2S_ADC);

    /* Verify the I2S API version for compatibility*/
    version = i2s_drv->GetVersion();
    printf("I2S API version = %d\n", version.api);

    /* Verify if I2S protocol is supported */
    cap = i2s_drv->GetCapabilities();
    if(!cap.protocol_i2s)
    {
        printf("I2S is not supported\n");
        return;
    }

    /* Initializes I2S2 interface */
    status = i2s_drv->Initialize(adc_callback);
    if(status)
    {
        printf("ADC Init failed status = %d\n", status);
        goto error_adc_initialize;
    }

    /* Enable the power for I2S2 */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if(status)
    {
        printf("ADC Power failed status = %d\n", status);
        goto error_adc_power;
    }

    /* configure I2S2 Receiver to Asynchronous Master */
    status = i2s_drv->Control(ARM_SAI_CONFIGURE_RX |
                                ARM_SAI_MODE_MASTER  |
                                ARM_SAI_ASYNCHRONOUS |
                                ARM_SAI_PROTOCOL_I2S |
                                ARM_SAI_DATA_SIZE(wlen), wlen*2, sampling_rate);
    if(status)
    {
      printf("ADC Control status = %d\n", status);
      goto error_adc_control;
    }

    /* enable Receiver */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if(status)
    {
        printf("ADC RX status = %d\n", status);
        goto error_adc_control;
    }

    for(;;)
    {
        /* Get one block from the samples pool. If not, return error */
        status = tx_block_allocate(&sample_block_pool, (VOID **)&samples_msg.buf,TX_WAIT_FOREVER);
        if (status)
	{
            printf("Sample Block allocation failed\n");
            goto error_adc_alloc;
        }

        samples_msg.num_items = NUM_SAMPLES_IN_SINGLE_POOL;
        memset((void*)samples_msg.buf, 0x0, samples_msg.num_items * 4);

        /* Receive data */
        status = i2s_drv->Receive(samples_msg.buf, samples_msg.num_items);
        if(status)
        {
            printf("ADC Receive status = %d\n", status);
            tx_block_release(samples_msg.buf);
            goto error_adc_receive;
        }

        /* Wait for the completion event */
        status = tx_event_flags_get(&event_flags_adc,ADC_RECEIVE_COMPLETE_EVENT, TX_OR_CLEAR, &events , ADC_RECEIVE_TIMEOUT);
        if(status)
        {
            printf("\r\nTimeout occurred while receiving the data from ADC\r\n");
            tx_block_release(samples_msg.buf);
            goto error_adc_receive;
        }

        /* Push the sample info to the queue for the dac to process it */
        status = tx_queue_send(&samples_msgq, &samples_msg, TX_WAIT_FOREVER);
        if(status)
        {
            /* Error occurred, then free the memory and don't process it */
            tx_block_release(samples_msg.buf);
        }
    }

    /* Stop the RX */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if(status)
    {
      printf("DAC RX status = %d\n", status);
      goto error_adc_control;
    }

error_adc_alloc:
error_adc_receive:
error_adc_control:
    i2s_drv->PowerControl(ARM_POWER_OFF);
error_adc_power:
    i2s_drv->Uninitialize();
error_adc_initialize:
    tx_thread_sleep(TX_WAIT_FOREVER);
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

    (void)first_unused_memory;

    /* Create message queue of SAMPLES_POOL_CNT count each having size of samples_msgq_t */
    status = tx_queue_create(&samples_msgq,
                             "I2S_SAMPLES_MSGQ",
                             TX_2_ULONG,
                             &samples_msg_queue,
                             sizeof(samples_msg_queue));
    if( status )
    {
        printf("Could not create tx_queue \n");
        return;
    }

    /* Create a block memory pool from which to allocate the thread stacks.  */
    status = tx_block_pool_create(&sample_block_pool,
                                  "sample_block_pool",
                                  (NUM_SAMPLES_IN_SINGLE_POOL * 4),
                                  samples_area,
                                  BLOCK_POOL_SIZE);
    if( status )
    {
        printf("Could not create block pool\n");
        return;
    }

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory, DEMO_BYTE_POOL_SIZE);
    if (status )
    {
        printf("Could not create byte pool\n");
        return;
    }

    /* Put system definition stuff in here, e.g. thread creates and other assorted create information.  */
#ifndef DAC_PREDEFINED_SAMPLES
    /* Create the event flags group used by ADC thread */
    status = tx_event_flags_create(&event_flags_adc, "event flags ADC");
    if (status )
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status )
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the ADC thread.  */
    status = tx_thread_create(&ADC_thread, "ADC_thread", ADC_Thread, 0,pointer, DEMO_STACK_SIZE,1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status )
    {
        printf("Could not create thread \n");
        return;
    }
#endif
    /* Put system definition stuff in here, e.g. thread creates and other assorted create information.  */

    /* Create the event flags group used by DAC thread */
    status = tx_event_flags_create(&event_flags_dac, "event flags DAC");
    if (status )
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status )
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the DAC thread.  */
    status = tx_thread_create(&DAC_thread, "DAC_thread", DAC_Thread, 0,pointer, DEMO_STACK_SIZE,6, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status )
    {
        printf("Could not create thread \n");
        return;
    }

}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
