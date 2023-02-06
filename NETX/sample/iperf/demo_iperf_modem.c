/*
 * demo_iperf_modem.c
 *
 * This program retrieves the IP address from the modem stack and then
 * creates and initializes an IP instance (with Alif's modem network driver).
 * After this, the program starts the netxduo iperf tests (which creates the
 * iperf server/clients and the webserver for configuring the test).
 *
 * Following changes were made to the original netxduo iperf program to
 * adapt it to the modem environment.
 *
 * 1. Calculate and display the throughput in units of Kbps instead of Mbps.
 * 2. Rate limit the UDP Tx test to 1Mbps.
 *
 * ATCMD middleware related wrapper routines are reused from the ATCMD demo
 * applications.
 *
 * Author   : Silesh C V <silesh@alifsemi.com>
 *
 * Copyright (C) 2021 ALIF SEMICONDUCTOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
  \file demo_iperf_modem.c
  \brief iperf demo program for the modem network driver
*/

#include   "tx_api.h"
#include   "nx_api.h"
#include   <ATCMD.h>

/* Driver include */
#include "nx_modem_driver.h"

#define PRINTF_REDIRECT_UART              2
#ifdef PRINTF_REDIRECT_UART
#include "Driver_USART.h"
#include "UART_dev.h"
/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"
FILE __stdout;
FILE __stdin;

/* Also Disable Semihosting */
#if __ARMCC_VERSION >= 6000000
    __asm(".global __use_no_semihosting");
#elif __ARMCC_VERSION >= 5000000
    #pragma import(__use_no_semihosting)
#else
    #error Unsupported compiler
#endif

#define UART_BAUDRATE                     921600
/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(PRINTF_REDIRECT_UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTprintfdrv = &ARM_Driver_USART_(PRINTF_REDIRECT_UART);
#endif /* PRINTF_REDIRECT_UART */


#define     DEMO_STACK_SIZE     2048
#define     PACKET_SIZE         1536

/** \brief The pool size. Allocate a pool to support 128 packets */
#define     POOL_SIZE           ((sizeof(NX_PACKET) + PACKET_SIZE) * 64)

/** \brief The port number that the TCP echo server listens on */
#define TCP_SERVER_PORT         7
/** \brief The port number that the UDP echo server listens on */
#define UDP_SERVER_PORT         7

/* Stack areas for http and iperf */
#define                 HTTP_STACK_SIZE         2048
#define                 IPERF_STACK_SIZE        2048

/* Define the ThreadX and NetX object control blocks...  */
TX_THREAD               main_thread;
TX_THREAD               thread_0;

NX_PACKET_POOL          pool_0;
NX_IP                   ip_0;

/* \brief The pool buffer area. */
UCHAR                   pool_buffer[POOL_SIZE];
UCHAR                   ip_thread_stack[2048];
UCHAR                   http_stack[HTTP_STACK_SIZE];
UCHAR                   iperf_stack[IPERF_STACK_SIZE];

/* The IP address used by the IP instance, will be modified after the IP address
 * is retrieved from the modem.
 */
static ULONG server_ip_addr = 0;

ULONG error_counter;

/* Define thread prototypes.  */
void main_thread_entry(ULONG thread_input);
void thread_0_entry(ULONG thread_input);
extern  VOID nx_iperf_entry(NX_PACKET_POOL *pool_ptr, NX_IP *ip_ptr, UCHAR* http_stack, ULONG http_stack_size, UCHAR *iperf_stack, ULONG iperf_stack_size);

#define MAX_ATCMD_LEN                      4096
#define MAX_RESPONSE_BUF_SIZE              3500
#define ERROR                              -1
#define SUCCESS                            0
#define ATCMD_READ_PIN_AUTH_STATUS         "AT+CPIN?\r"
#define ATCMD_PIN_AUTH_READY_STATUS        "\r\n+CPIN:READY\r\n"
#define ATCMD_ENABLE_PHONE_FUNC            "AT+CFUN=1\r"
#define ATCMD_READ_CGATT                   "AT+CGATT?\r"
#define ATCMD_ATTACH                       "AT+CGATT=1"
#define ATCMD_GET_IPADDRESS                "AT+CGPADDR=5\r"
#define ATCMD_ATTACH_RESP_STR              "+CGATT:"
#define ATCMD_RESP_IPADDRESS_STR           "+CGPADDR:"


/* Tx and Rx buffers to pass through Xfer function */
uint8_t buf[MAX_ATCMD_LEN];
uint8_t resp_buf[MAX_RESPONSE_BUF_SIZE];

/* Store the IP address received from the modem */
static char src_ipaddr[16];

/* Store the attach status received from the modem */
static char attach_status_buf[5];

#ifdef PRINTF_REDIRECT_UART
void _sys_exit(int return_code)
{
    while (1);
}

void uartWrite(char c)//Used by fputc
{
#define PREP_UART_BASE(x) (UART ##x##_BASE)
#define UART_BASE(x)      PREP_UART_BASE(x)
    uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)  UART_BASE(PRINTF_REDIRECT_UART);

    /* Wait until uart is ready to send */
    while((uart_reg_ptr -> usr & UART_USR_TRANSMIT_FIFO_NOT_FULL) == 0);

    /* Write a char to thr transmit holding register */
    uart_reg_ptr -> rbr_thr_dll = c;
}

char uartRead(void)//Used by fgetc()
{
    return 0;
}

int fputc(int c, FILE *stream)
{
    uartWrite(c);
    return c; //return the character written to denote a successful write
}

int fgetc(FILE *stream)
{
    char c = uartRead();

    uartWrite(c);//To echo characters back to the serial terminal

    return c;
}

/**
  \fn          void uart_printf_redirect_callback(UINT event)
  \brief       Uart Callback function from the driver
  \param[in]   event  Uart events
*/
void uart_printf_redirect_callback(UINT event)
{
    if(event & ARM_USART_EVENT_SEND_COMPLETE)
    {

    }

    if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {

    }

    if(event & ARM_USART_EVENT_RX_TIMEOUT)
    {

    }
}

/**
  \function    int32_t hardware_init(void)
  \brief       Uart initialization
  \note        none
  \param       void
  \retval      execution status
 */
int32_t hardware_init(void)
{
    int32_t ret = ARM_DRIVER_OK;


#if (PRINTF_REDIRECT_UART == 2)
    /* PINMUX UART2_A */

    /* Configure GPIO Pin : P1_10 as UART2_RX_A */
    ret = PINMUX_Config(PORT_NUMBER_1, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in PINMUX: P1_10\r\n");
        goto error_pinmux;
    }

    /* Configure GPIO Pin : P1_11 as UART2_TX_A */
    ret = PINMUX_Config(PORT_NUMBER_1, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in PINMUX: P1_11\r\n");
        goto error_pinmux;
    }

    /* Configure GPIO Pin : P1_12 as UART2_CTS_A */
    ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in PINMUX: P1_12\r\n");
        return ret;
    }

    /* Configure GPIO Pin : P1_13 as UART2_RTS_A */
    ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_13, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error in PINMUX: P1_13\r\n");
        return ret;
    }
#endif

    /* Initialize UART driver */
    ret = USARTprintfdrv->Initialize(uart_printf_redirect_callback);
    if(ret)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Power up UART peripheral */
    ret = USARTprintfdrv->PowerControl(ARM_POWER_FULL);
    if(ret)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Configure UART */
    ret =  USARTprintfdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                                   ARM_USART_DATA_BITS_8 |
                                   ARM_USART_PARITY_NONE |
                                   ARM_USART_STOP_BITS_1 |
                                   ARM_USART_FLOW_CONTROL_NONE,
                                   UART_BAUDRATE);
    if(ret)
    {
        return ARM_DRIVER_ERROR;

    }

    /* Enable Transmitter lines */
    ret =  USARTprintfdrv->Control(ARM_USART_CONTROL_TX, 1);
    if(ret)
    {
        return ARM_DRIVER_ERROR;
    }

    return ret;

error_pinmux:
    return ret;
}
#endif /* PRINTF_REDIRECT_UART */

int main()
{
#ifdef PRINTF_REDIRECT_UART
    hardware_init();
#endif
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

/* Define what the initial system looks like.  */
void    tx_application_define(void *first_unused_memory)
{
CHAR *pointer;

    /* Setup the working pointer.  */
    pointer =  (CHAR *)first_unused_memory;

    /* Create the main thread.  */
    tx_thread_create(&main_thread, "Main thread", main_thread_entry, 0,
                     pointer, DEMO_STACK_SIZE,
                     4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    pointer =  pointer + DEMO_STACK_SIZE;

    /* Create the thread responsible for running iperf  */
    tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0,
                     pointer, DEMO_STACK_SIZE,
                     4, 4, TX_NO_TIME_SLICE, TX_DONT_START);
    pointer =  pointer + DEMO_STACK_SIZE;

}

/**
  \fn          void notify_callback(ATCMD_NOTIFY_TYPE notify_type, uint8_t *resp_buf, uint16_t resp_buf_len)
  \brief       Callback routine from the atcmd driver.
  \param[in]   notify_type denotes the type of notification
  \param[in]   resp_buf pointer to response buffer
  \param[in]   resp_buf_len response buffer length
*/
static void notify_callback(ATCMD_NOTIFY_TYPE notify_type, uint8_t *resp_buf, uint16_t resp_buf_len)
{
    switch(notify_type)
    {
    case ATCMD_NOTIFY_CEREG:
       printf("%s\n", resp_buf);
       break;
    case ATCMD_NOTIFY_CRTDCP:
       printf("%s\n", resp_buf);
       break;
    case ATCMD_NOTIFY_CSCON:
       printf("%s\n", resp_buf);
       break;
    case ATCMD_NOTIFY_CGEREP:
       printf("%s\n", resp_buf);
       break;
    case ATCMD_NOTIFY_CEDRXS:
       printf("%s\n", resp_buf);
       break;
    case ATCMD_NOTIFY_CCIOTOPT:
       printf("%s\n", resp_buf);
       break;
    default:
       break;
    }
}

/**
  \fn        ULONG ip_str_to_ulong(char *ip_addr_str)
  \brief     Convert an IP address in "X.X.X.X" form to an unsigned long.
  \param[in] ip_addr_str. IP address in the string form.
*/
static ULONG ip_str_to_ulong(char *ip_addr_str)
{
ULONG ip_addr = 0;
char *token;

    token = strtok(ip_addr_str, ".");

    while (token)
    {
        ip_addr |= atoi(token);
        token = strtok(NULL, ".");
        if (token)
            ip_addr <<= 8;
    }

    return ip_addr;
}

/**
  \function    int16_t wait_for_pin_authentication(void)
  \brief       wait for the pin authentication.
  \note        none
  \param       void
  \retval      execution status
 */
static int16_t wait_for_pin_authentication(void)
{
    ATCMD_RET_STATUS  status;

    do
    {
        memset(buf, 0, MAX_ATCMD_LEN);

        memset(resp_buf, 0, MAX_RESPONSE_BUF_SIZE);

        /* Checking the pin authentication status */
        strcpy((char *)buf, ATCMD_READ_PIN_AUTH_STATUS);

        printf("\r\n%s\n", buf);

        /* Transfer function to send and receive from modem */
        status = ATCMD_Xfer(buf, strlen((const char *)buf) + 1, resp_buf, MAX_RESPONSE_BUF_SIZE);

        if (status == ATCMD_OK_RESPONSE)
        {
            printf("%s\n", resp_buf);
            if (strncmp(ATCMD_PIN_AUTH_READY_STATUS, (const char *)resp_buf, strlen(ATCMD_PIN_AUTH_READY_STATUS)) == 0)
                break;
            else
            {
                tx_thread_sleep(100);
                continue;
            }
        }
        else if ((status == ATCMD_CMD_ERROR) || (status == ATCMD_ERROR_CME) || (status == ATCMD_ERROR_CMS))
        {
            printf("%s\n", resp_buf);
        }
        else
        {
            printf("\r\nTransfer is unsuccessful with status:%d\n", status);

            return ERROR;
        }

    } while(1);

    return SUCCESS;
}

/**
  \function    int16_t enable_phone_functionality(void)
  \brief       Enables phone functionality
  \note        none
  \param       void
  \retval      execution status
 */
static int16_t enable_phone_functionality(void)
{
    ATCMD_RET_STATUS status;

    memset(buf, 0, MAX_ATCMD_LEN);

    memset(resp_buf, 0, MAX_RESPONSE_BUF_SIZE);

    /* Switch on the phone functionality */
    strcpy((char *)buf, ATCMD_ENABLE_PHONE_FUNC);

    printf("\r\n%s\n", buf);

    /* Transfer function to send and receive from modem */
    status = ATCMD_Xfer(buf, strlen((const char *)buf) + 1, resp_buf, MAX_RESPONSE_BUF_SIZE);

    if (status == ATCMD_OK_RESPONSE)
    {
        printf("%s\n", resp_buf);
    }
    else if ((status == ATCMD_CMD_ERROR) || (status == ATCMD_ERROR_CME) || (status == ATCMD_ERROR_CMS))
    {
        printf("%s\n", resp_buf);
    }
    else
    {
        printf("\r\nTransfer is unsuccessful with status:%d\n", status);

        return ERROR;
    }

    return SUCCESS;
}

/**
  \function    int16_t get_ipaddress(ULONG *ip_addr)
  \brief       To get the home ipaddress to which modem is connected.
  \note        none
  \param       ip_addr, storage for the IP address
  \retval      execution status
 */
static int16_t get_ipaddress(ULONG *ip_addr)
{
    ATCMD_RET_STATUS status;
    char             *rx_ptr = NULL;

    memset(buf, 0, MAX_ATCMD_LEN);

    memset(resp_buf, 0, MAX_RESPONSE_BUF_SIZE);

    /* Prepare the ATCMD to retrieve the IP address */
    strcpy((char *)buf, ATCMD_GET_IPADDRESS);

    printf("\r\n%s\n", buf);

    /* Send the ATCMD */
    status = ATCMD_Xfer(buf, strlen((const char *)buf) + 1, resp_buf, MAX_RESPONSE_BUF_SIZE);

    if (status == ATCMD_OK_RESPONSE)
    {
        rx_ptr = strstr((char*) resp_buf, ATCMD_RESP_IPADDRESS_STR);

        if (rx_ptr)
        {
            rx_ptr = strtok((char*)resp_buf, ",");

            rx_ptr = strtok(NULL, "\n");

            strcpy(src_ipaddr, (const char *)rx_ptr);

            printf("\r\nIPADDR = %s\n", src_ipaddr);

            /* Return the IP address in the address provided by the caller */
            *ip_addr = ip_str_to_ulong(src_ipaddr);
        }
    }
    else if ((status == ATCMD_CMD_ERROR) || (status == ATCMD_ERROR_CME) || (status == ATCMD_ERROR_CMS))
    {
        printf("%s \n", resp_buf);
    }
    else
    {
        printf("\r\nTransfer is unsuccessful with status:%d\n", status);

        return ERROR;
    }

    return SUCCESS;
}

/**
  \function    int16_t wait_for_attach(void)
  \brief       wait till the modem is attached.
  \note        none
  \param       void
  \retval      execution status
 */
static int16_t wait_for_attach(void)
{
ATCMD_RET_STATUS  status;
char              *rx_ptr = NULL;
uint8_t           attach_status = 0;

    do
    {
        memset(buf, 0, MAX_ATCMD_LEN);

        memset(resp_buf, 0, MAX_RESPONSE_BUF_SIZE);

        /* Prepare the ATCMD to retrieve the attach status */
        strcpy((char *)buf, ATCMD_READ_CGATT);

        printf("\r\n%s\n", buf);

        /* Send the ATCMD */
        status = ATCMD_Xfer(buf, strlen((const char *)buf) + 1, resp_buf, MAX_RESPONSE_BUF_SIZE);

        if (status == ATCMD_OK_RESPONSE)
        {
            printf("%s\n", resp_buf);

            rx_ptr = strstr((char*) resp_buf, ATCMD_ATTACH_RESP_STR);

            if (rx_ptr)
            {
                rx_ptr = strtok((char*)resp_buf, ":");

                rx_ptr = strtok(NULL, "\n");

                strcpy(attach_status_buf, (const char *)rx_ptr);

                attach_status = atoi(attach_status_buf);
            }
        }
        else if ((status == ATCMD_CMD_ERROR) || (status == ATCMD_ERROR_CME) || (status == ATCMD_ERROR_CMS))
        {
            printf("%s \n", resp_buf);
        }
        else
        {
            return ERROR;
        }

        if (!attach_status)
        {
            printf("\r\nWaiting for modem attach\n");

            /* Sleep here to Send AT command after 1sec */
            tx_thread_sleep(100);
            continue;
        }
        else
        {
            printf("\r\nModem attach Successful\n");
            break;
        }
    } while (1);

    return SUCCESS;
}

/**
  \fn        void main_thread_entry(ULONG thread_input)
  \brief     Thread function for the main thread.
  \param[in] thread_input. Unused.
*/
void main_thread_entry(ULONG thread_input)
{
ATCMD_RET_STATUS status;
uint32_t         local_version, remote_version;
int16_t          ret_status;
UINT  nx_status;

    NX_PARAMETER_NOT_USED(thread_input);

    /* ATCMD initialization */
    status = ATCMD_Init();

    if (status != ATCMD_OK)
    {
        printf("\r\nAT_cmd init was not successful\n");
        goto exit_tx_thread;
    }

    /* Get local version and remote version */
    status = ATCMD_GetVersion(&local_version, &remote_version);

    if (status != ATCMD_OK)
    {
        printf("\r\nFailed to get local and remote version with status :%d\n", status);
        goto exit_tx_thread;
    }

    printf("ATCMD: Local version = %d, remote version = %d\n", local_version, remote_version);

    /* Register the call back function for notification */
    status = ATCMD_Add_NotifyHandler((ATCMD_Notify_Callback_t) notify_callback);

    if (status != ATCMD_OK)
    {
        printf("\r\nFailed to add notify_handler with status :%d\n", status);
        goto exit_tx_thread;
    }

    /* ATCMD sequence :
     * AT+CPIN?\r
     * AT+CFUN=1\r
     * AT+CGATT?\r
     * AT+CGPADDR=5\r
     */

    /* Wait for the PIN Authentication */
    ret_status = wait_for_pin_authentication();

    if (ret_status)
    {
        printf("\r\n PIN auth failed\n");
        goto exit_tx_thread;
    }

    /* Enable the phone functionality */
    ret_status = enable_phone_functionality();

    if (ret_status)
    {
        printf("\r\n Enable phone functionality failed\n");
        goto exit_tx_thread;
    }

    /* Wait for modem attach */
    ret_status = wait_for_attach();

    if (ret_status)
    {
        printf("\r\n Modem Attach failed\n");
        goto exit_tx_thread;
    }

    /* Retrieve the IP address */
    ret_status = get_ipaddress(&server_ip_addr);

    if (ret_status)
    {
        printf("\r\nCould not retrieve the IP address\n");
        goto exit_tx_thread;
    }

    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create a packet pool.  */
    nx_status =  nx_packet_pool_create(&pool_0, "NetX Main Packet Pool", PACKET_SIZE, pool_buffer, POOL_SIZE);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx Packet pool creation failed\n");
        goto exit_tx_thread;
    }

    /* Create an IP instance with static IP */
    nx_status = nx_ip_create(&ip_0, "NetX IP Instance 0", server_ip_addr, 0UL, &pool_0, nx_modem_driver,
                          ip_thread_stack, 2048, 2);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx ip create failed\n");
        goto exit_tx_thread;
    }


    /* Enable IP fragmentation on the IP instance. */
    nx_status = nx_ip_fragment_enable(&ip_0);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx IP fragmentation enable failed\n");
        goto exit_tx_thread;
    }

    /* Enable ICMP traffic */
    nx_status = nxd_icmp_enable(&ip_0);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx ICMP enable failed\n");
        goto exit_tx_thread;
    }

    /* Enable UDP traffic */
    nx_status =  nx_udp_enable(&ip_0);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx UDP enable failed\n");
        goto exit_tx_thread;
    }

    /* Enable TCP processing */
    nx_status =  nx_tcp_enable(&ip_0);

    if (nx_status != NX_SUCCESS)
    {
        printf("\r\n Netx TCP enable failed\n");
        goto exit_tx_thread;
    }

    /* Wake up the iperf thread */
    tx_thread_resume(&thread_0);

    return;

exit_tx_thread:
    printf("\r\n Main thread exiting..\n");
    /* ATCMD modem deinitialization */
    status = ATCMD_DeInit();
    if(status != ATCMD_OK)
    {
        printf("\r\nAT_cmd deinit was not successful with status:%d\n",status);
    }
}

/* Define the iperf thread.  */
void    thread_0_entry(ULONG thread_input)
{
#ifdef FEATURE_NX_IPV6
    tx_thread_sleep(5 * NX_IP_PERIODIC_RATE);
#endif

    /* Call entry function to start iperf test.  */
    nx_iperf_entry(&pool_0, &ip_0, http_stack, HTTP_STACK_SIZE, iperf_stack, IPERF_STACK_SIZE);
}
