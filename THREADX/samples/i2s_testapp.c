/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/****************************************************************************
 * @file     i2s_testapp.c
 * @author   Sudhir Sreedharan | Sudarshan Iyengar
 * @email    sudhir@alifsemi.com | sudarshan.iyengar@alifsemi.com
 * @version  V3.0.0
 * @date     28-Apr-2023
 * @brief    Test Application for I2S for Devkit
 *           For HP, I2S1 is configured as master transmitter (DAC).
 *           For HE, LPI2S will be used as DAC.
 *           I2S3(ADC) is configured as master receiver SPH0645LM4H-1 device 24bit
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

/*System Includes */
#include <stdio.h>
#include <string.h>

/* Project Includes */
#include <Driver_SAI.h>
#include <pinconf.h>
#include "RTE_Components.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */


/*Threadx Includes */
#include "tx_api.h"

/*Audio samples */
#include "i2s_samples.h"

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                        1024
#define DEMO_BYTE_POOL_SIZE                    4096


/* 1 to send the data stream continuously , 0 to send data only once */
#define REPEAT_TX 1

#define ERROR  -1
#define SUCCESS 0

#if defined (M55_HE)
#define I2S_DAC LP            /* DAC LPI2S Controller */
#else
/* Enable this to feed the predefined hello sample in the
 * Send function. Receive will be disabled.
 */
//#define DAC_PREDEFINED_SAMPLES
#define I2S_DAC 1             /* DAC I2S Controller 1 */
#endif
#define I2S_ADC 3             /* ADC I2S Controller 3 */

#define DAC_SEND_COMPLETE_EVENT    (1U << 0)
#define ADC_RECEIVE_COMPLETE_EVENT (1U << 1)
#define ADC_RECEIVE_OVERFLOW_EVENT (1U << 2)

#define ADC_RECEIVE_TIMEOUT        (10000)
#define DAC_SEND_TIMEOUT           (10000)

#define NUM_SAMPLES_IN_SINGLE_POOL 8000
#define SAMPLES_POOL_CNT           4

#define BLOCK_POOL_METADATA_SIZE   16
#define BLOCK_POOL_SIZE            ((SAMPLES_POOL_CNT * NUM_SAMPLES_IN_SINGLE_POOL * 4)\
                                   + (SAMPLES_POOL_CNT * BLOCK_POOL_METADATA_SIZE))

static UCHAR                   memory[DEMO_BYTE_POOL_SIZE];
static UCHAR                   samples_area[BLOCK_POOL_SIZE];
static TX_BYTE_POOL            byte_pool_0;
static TX_BLOCK_POOL           sample_block_pool;

static TX_THREAD               DAC_thread;/* Thread id of DAC Transmitter */
static TX_EVENT_FLAGS_GROUP    event_flags_dac;

#ifndef DAC_PREDEFINED_SAMPLES
static TX_THREAD               ADC_thread;/* Thread id of ADC Receiver */
static TX_EVENT_FLAGS_GROUP    event_flags_adc;
#endif

static ULONG                   events;

static TX_QUEUE                samples_msgq;

/* message information for queueing */
typedef struct _samples_msgq
{
    void     *buf;
    uint32_t num_items;
}samples_msgq_t;

static samples_msgq_t       samples_msg_queue[SAMPLES_POOL_CNT];

static uint32_t wlen = 24;
static uint32_t sampling_rate = 48000;        /* 48Khz audio sampling rate */



/**
  \fn          void dac_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
static void dac_callback(uint32_t event)
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
static int32_t dac_pinmux_config(void)
{
    int32_t status;
#if (I2S_DAC == LP)
    /* Configure LPI2S_C SDO */
    status = pinconf_set(PORT_13, PIN_5, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(status)
        return ERROR;

    /* Configure LPI2S_C WS */
    status = pinconf_set(PORT_13, PIN_7, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(status)
        return ERROR;

    /* Configure LPI2S_C SCLK */
    status = pinconf_set(PORT_13, PIN_6, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(status)
        return ERROR;
#else
    /* Configure I2S1_A SDO */
    status = pinconf_set(PORT_3, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if(status)
        return ERROR;

    /* Configure I2S1_A WS */
    status = pinconf_set(PORT_4, PIN_0, PINMUX_ALTERNATE_FUNCTION_3, 0);
    if(status)
        return ERROR;

    /* Configure I2S1_A SCLK */
    status = pinconf_set(PORT_3, PIN_4, PINMUX_ALTERNATE_FUNCTION_4, 0);
    if(status)
        return ERROR;

#endif

    return SUCCESS;
}

/**
  \fn          void DAC_Thread(ULONG thread_input)
  \brief       DAC thread for master transmission
  \param[in]   argument Thread input
*/
static void DAC_Thread(ULONG thread_input)
{
    ARM_DRIVER_VERSION   version;
    ARM_DRIVER_SAI       *i2s_drv;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;
    UINT                 tx_status;
    samples_msgq_t       samples_msg;

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_DAC);

    (void)thread_input;

    /* Configure the dac pins */
    if(dac_pinmux_config())
    {
        printf("DAC pinmux failed\n");
        return;
    }

    /* Use the I2S as Trasmitter */
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

    /* Initializes I2S interface */
    status = i2s_drv->Initialize(dac_callback);
    if(status)
    {
        printf("DAC Init failed status = %d\n", status);
        goto error_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if(status)
    {
        printf("DAC Power Failed status = %d\n", status);
        goto error_power;
    }

    /* configure I2S Transmitter to Asynchronous Master */
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
        samples_msg.buf = (void *)hello_samples_24bit_48khz;
        samples_msg.num_items = sizeof(hello_samples_24bit_48khz)
                                       / sizeof(hello_samples_24bit_48khz[0]);

        /* Transmit the samples */
        status = i2s_drv->Send(samples_msg.buf, samples_msg.num_items);
        if(status)
        {
            printf("DAC Send status = %d\n", status);
            goto error_send;
        }

        /* Wait for the completion event */
        tx_status = tx_event_flags_get(&event_flags_dac,
                                       DAC_SEND_COMPLETE_EVENT,
                                       TX_OR_CLEAR, &events,
                                       DAC_SEND_TIMEOUT);
        if(tx_status)
        {
            printf("Timeout occurred while sending the data to DAC\n");
            goto error_send;
        }

#else
        /* Read the samples from the queue */
        tx_status = tx_queue_receive(&samples_msgq, &samples_msg, TX_WAIT_FOREVER);
        if(tx_status == TX_SUCCESS)
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
            tx_status = tx_event_flags_get(&event_flags_dac,
                                           DAC_SEND_COMPLETE_EVENT,
                                           TX_OR_CLEAR,
                                           &events,
                                           DAC_SEND_TIMEOUT);
            if(tx_status)
            {
                printf("Timeout occurred while sending the data to DAC\n");
                tx_block_release(samples_msg.buf);
                goto error_send;
            }

            tx_status = tx_block_release(samples_msg.buf);
            if(tx_status)
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

#ifndef DAC_PREDEFINED_SAMPLES
/**
  \fn          void adc_callback(uint32_t event)
  \brief       Callback routine from the i2s driver
  \param[in]   event Event for which the callback has been called
*/
static void adc_callback(uint32_t event)
{
    if(event & ARM_SAI_EVENT_RECEIVE_COMPLETE)
    {
        /* Send Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_adc, ADC_RECEIVE_COMPLETE_EVENT, TX_OR);
    }

    if(event & ARM_SAI_EVENT_RX_OVERFLOW)
    {
        /* Send Success: Wake-up Thread. */
        tx_event_flags_set(&event_flags_adc, ADC_RECEIVE_OVERFLOW_EVENT, TX_OR);
    }
}

/**
  \fn          void adc_pinmux_config(void)
  \brief       Initialize the pinmux for ADC
  \return      status
*/
static int32_t adc_pinmux_config(void)
{
    int32_t status;

    /* Configure I2S3_B WS */
    status = pinconf_set(PORT_8, PIN_7, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(status)
        return ERROR;

    /* Configure I2S3_B SCLK */
    status = pinconf_set(PORT_8, PIN_6, PINMUX_ALTERNATE_FUNCTION_2, 0);
    if(status)
        return ERROR;

    /* Configure I2S3_B SDI */
    status = pinconf_set(PORT_9, PIN_0, PINMUX_ALTERNATE_FUNCTION_2, PADCTRL_READ_ENABLE);
    if(status)
        return ERROR;

    return SUCCESS;
}

/**
  \fn          void ADC_Thread(ULONG thread_input)
  \brief       ADC Thread to handle receive
  \param[in]   argument Pointer to the argument
*/
static void ADC_Thread(ULONG thread_input)
{
    ARM_DRIVER_VERSION   version;
    ARM_DRIVER_SAI       *i2s_drv;
    ARM_SAI_CAPABILITIES cap;
    int32_t              status;
    UINT                 tx_status;
    samples_msgq_t       samples_msg;

    extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_ADC);
    (void)thread_input;

    /* Configure the adc pins */
    if(adc_pinmux_config())
    {
        printf("ADC pinmux failed\n");
        return;
    }

    /* Use the I2S as Receiver */
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

    /* Initializes I2S interface */
    status = i2s_drv->Initialize(adc_callback);
    if(status)
    {
        printf("ADC Init failed status = %d\n", status);
        goto error_adc_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if(status)
    {
        printf("ADC Power failed status = %d\n", status);
        goto error_adc_power;
    }

    /* configure I2S Receiver to Asynchronous Master */
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
        tx_status = tx_block_allocate(&sample_block_pool,
                                      (VOID **)&samples_msg.buf,
                                      TX_WAIT_FOREVER);
        if(tx_status)
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
        tx_status = tx_event_flags_get(&event_flags_adc,
                                       ADC_RECEIVE_COMPLETE_EVENT,
                                       TX_OR_CLEAR,
                                       &events,
                                       ADC_RECEIVE_TIMEOUT);
        if(tx_status)
        {
            printf("\r\nTimeout occurred while receiving the data from ADC\r\n");
            tx_block_release(samples_msg.buf);
            goto error_adc_receive;
        }
        else if(ADC_RECEIVE_OVERFLOW_EVENT & events)
        {
            /* Wait for the completion event */
            tx_status = tx_event_flags_get(&event_flags_adc,
                                           ADC_RECEIVE_OVERFLOW_EVENT,
                                           TX_OR_CLEAR,
                                           &events,
                                           TX_NO_WAIT);
            if(tx_status)
            {
                printf("\r\nError in Clearing the ADC overflow state \r\n");
            }
            printf("\r\nADC: Overflow\r\n");
        }

        /* Push the sample info to the queue for the dac to process it */
        tx_status = tx_queue_send(&samples_msgq, &samples_msg, TX_WAIT_FOREVER);
        if(tx_status)
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
#endif

/* Define main entry point.  */
int main(void)
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
    UINT     tx_status  = 0;

    (void)first_unused_memory;

    /*
     * Create message queue of SAMPLES_POOL_CNT count each
     * having size of samples_msgq_t
     */
    tx_status = tx_queue_create(&samples_msgq,
                                "I2S_SAMPLES_MSGQ",
                                TX_2_ULONG,
                                &samples_msg_queue,
                                sizeof(samples_msg_queue));
    if(tx_status)
    {
        printf("Could not create tx_queue \n");
        return;
    }

    /* Create a block memory pool from which to allocate the thread stacks.  */
    tx_status = tx_block_pool_create(&sample_block_pool,
                                     "sample_block_pool",
                                     (NUM_SAMPLES_IN_SINGLE_POOL * 4),
                                     samples_area,
                                     BLOCK_POOL_SIZE);
    if(tx_status)
    {
        printf("Could not create block pool\n");
        return;
    }

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_status = tx_byte_pool_create(&byte_pool_0, "byte pool 0",
                                    memory, DEMO_BYTE_POOL_SIZE);
    if(tx_status)
    {
        printf("Could not create byte pool\n");
        return;
    }

    /*
     * Put system definition stuff in here,
     * e.g. thread creates and other assorted create information
     */
#ifndef DAC_PREDEFINED_SAMPLES
    /* Create the event flags group used by ADC thread */
    tx_status = tx_event_flags_create(&event_flags_adc, "event flags ADC");
    if(tx_status)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    tx_status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer,
                                 DEMO_STACK_SIZE, TX_NO_WAIT);
    if(tx_status)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the ADC thread.  */
    tx_status = tx_thread_create(&ADC_thread,
                                 "ADC_thread",
                                 ADC_Thread,
                                 0,
                                 pointer,
                                 DEMO_STACK_SIZE,
                                 1,
                                 1,
                                 TX_NO_TIME_SLICE,
                                 TX_AUTO_START);
    if(tx_status)
    {
        printf("Could not create thread \n");
        return;
    }
#endif
    /*
     * Put system definition stuff in here,
     * e.g. thread creates and other assorted create information
     */

    /* Create the event flags group used by DAC thread */
    tx_status = tx_event_flags_create(&event_flags_dac, "event flags DAC");
    if(tx_status)
    {
        printf("Could not create event flags\n");
        return;
    }

    /* Allocate the stack for thread.  */
    tx_status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer,
                                 DEMO_STACK_SIZE, TX_NO_WAIT);
    if(tx_status)
    {
        printf("Could not create byte allocate\n");
        return;
    }

    /* Create the DAC thread */
    tx_status = tx_thread_create(&DAC_thread,
                                 "DAC_thread",
                                 DAC_Thread,
                                 0,
                                 pointer,
                                 DEMO_STACK_SIZE,
                                 6,
                                 1,
                                 TX_NO_TIME_SLICE,
                                 TX_AUTO_START);
    if(tx_status)
    {
        printf("Could not create thread \n");
        return;
    }

}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
