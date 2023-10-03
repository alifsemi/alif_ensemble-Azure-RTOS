/*
 * demo_netx.c
 *
 * This program demonstrates the azure-rtos netxduo stack and Alif's netxduo
 * Ethernet driver.
 *
 * The demo creates an UDP echo server and a TCP echo server listening on the
 * standard echo server port (7). Also, ICMP is enabled for the created NetXDuo
 * IP instance. So the system will also respond to incoming ping requests.
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
  \file demo_netx.c
  \brief Implements a demo program using the NetxDuo network stack.
 */

/* Comment out the below line and set SERVER_IP_ADDR to use a static address */
#define DEMO_USES_DHCP

#include   "tx_api.h"
#include   "nx_api.h"

#ifdef DEMO_USES_DHCP
#include   "nxd_dhcp_client.h"
#endif
/* Driver include */
#include "nx_eth_mac.h"

#define     DEMO_STACK_SIZE     2048
#define     PACKET_SIZE         1536

/** \brief The pool size. Allocate a pool to support 24 packets */
#define     POOL_SIZE           ((sizeof(NX_PACKET) + PACKET_SIZE) * 24)

#ifdef DEMO_USES_DHCP
#define SERVER_IP_ADDR          IP_ADDRESS(0, 0, 0, 0)
#else
/** \brief static IP address used by the demo application */
#define SERVER_IP_ADDR          IP_ADDRESS(192, 168, 1, 11)
#endif

/** \brief The port number that the TCP echo server listens on */
#define TCP_SERVER_PORT         7
/** \brief The port number that the UDP echo server listens on */
#define UDP_SERVER_PORT         7

/* Define the ThreadX and NetX object control blocks...  */
TX_THREAD               dhcp_client_thread;
TX_THREAD               udp_server_thread;
TX_THREAD               tcp_server_thread;

NX_PACKET_POOL          pool_0;
NX_IP                   ip_0;

#ifdef DEMO_USES_DHCP
NX_DHCP                 dhcp_client;
#endif

/* \brief The pool buffer area. Allocated in the "eth_buf" section */
UCHAR                   pool_buffer[POOL_SIZE]__attribute__((section("eth_buf")));

NX_UDP_SOCKET           udp_socket;
NX_TCP_SOCKET           tcp_socket;

ULONG error_counter;

/* Define thread prototypes.  */
void dhcp_client_thread_entry(ULONG thread_input);
void udp_server_thread_entry(ULONG thread_input);
void tcp_server_thread_entry(ULONG thread_input);

/* Define TCP callbacks */
void tcp_server_connect_received(NX_TCP_SOCKET *server_socket, UINT port);
void tcp_server_disconnect_received(NX_TCP_SOCKET *server_socket);

int main()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}


/* Define what the initial system looks like.  */
void    tx_application_define(void *first_unused_memory)
{
CHAR *pointer;
UINT  status;

    /* Setup the working pointer.  */
    pointer =  (CHAR *)first_unused_memory;

    /* Create the dhcp client thread */
    tx_thread_create(&dhcp_client_thread, "DHCP client thread", dhcp_client_thread_entry, 0,
                     pointer, DEMO_STACK_SIZE,
                     4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    pointer =  pointer + DEMO_STACK_SIZE;

    /* Create the main thread.  */
    tx_thread_create(&udp_server_thread, "UDP echo server thread", udp_server_thread_entry, 0,
                     pointer, DEMO_STACK_SIZE,
                     4, 4, TX_NO_TIME_SLICE, TX_DONT_START);

    pointer =  pointer + DEMO_STACK_SIZE;

    tx_thread_create(&tcp_server_thread, "TCP echo server thread", tcp_server_thread_entry, 0,
                     pointer, DEMO_STACK_SIZE,
                     3, 3, TX_NO_TIME_SLICE, TX_DONT_START);

    pointer =  pointer + DEMO_STACK_SIZE;

    /* Initialize the NetX system.  */
    nx_system_initialize();

    /* Create a packet pool.  */
    status =  nx_packet_pool_create(&pool_0, "NetX Main Packet Pool", PACKET_SIZE, pool_buffer, POOL_SIZE);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Create an IP instance with static IP */
    status = nx_ip_create(&ip_0, "NetX IP Instance 0", SERVER_IP_ADDR, 0xFFFFFF00UL, &pool_0, nx_eth_driver,
                          pointer, 2048, 1);
    pointer =  pointer + 2048;

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Enable IP fragmentation on the IP instance. */
    status = nx_ip_fragment_enable(&ip_0);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Enable ARP and supply ARP cache memory for the IP Instance */
    status =  nx_arp_enable(&ip_0, (void *)pointer, 1024);
    pointer = pointer + 1024;

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Enable ICMP traffic */
    status = nxd_icmp_enable(&ip_0);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Enable UDP traffic */
    status =  nx_udp_enable(&ip_0);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Enable TCP processing */
    status =  nx_tcp_enable(&ip_0);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }
}

/**
  \fn        void dhcp_client_thread_entry(ULONG thread_input)
  \brief     Thread function for the DHCP client thread.
  \param[in] thread_input. Unused.
*/
void dhcp_client_thread_entry(ULONG thread_input)
{
UINT status;
ULONG actual_status;
ULONG ip_address;
ULONG network_mask;

    NX_PARAMETER_NOT_USED(thread_input);

    /* Let the IP thread execute */
    tx_thread_sleep(NX_IP_PERIODIC_RATE);

    do
    {
        /* Get the link status. */
        status = nx_ip_status_check(&ip_0, NX_IP_LINK_ENABLED,
                    &actual_status, NX_IP_PERIODIC_RATE);
        printf("Waiting for Link up...\n");
    } while (status != NX_SUCCESS);

#ifdef DEMO_USES_DHCP
    status = nx_dhcp_create(&dhcp_client, &ip_0, "dhcp-client");

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    /*
     * Instruct the DHCP client to use our default packet pool.
     * Note that for this to work, NX_DHCP_CLIENT_USER_CREATE_PACKET_POOL should
     * be defined in nxd_dhcp_client.h.
     */
    status = nx_dhcp_packet_pool_set(&dhcp_client, &pool_0);

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    /* Start the DHCP Client.  */
    nx_dhcp_start(&dhcp_client);

    /* Wait for DHCP to assign the IP address. */
    do
    {
        /* Check for address resolution.  */
        status = nx_ip_status_check(&ip_0, NX_IP_ADDRESS_RESOLVED,
                    &actual_status, 10 * NX_IP_PERIODIC_RATE);
        printf("Waiting for IP address resolution...\n");

    } while (status != NX_SUCCESS);
#endif

    /*
     * We have successfully received an IP address (if DHCP is enabled),
     * retrieve and print this info.
     */
    status = nx_ip_address_get(&ip_0, &ip_address, &network_mask);

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    printf("IP Address: %lu.%lu.%lu.%lu Network Mask: %lu.%lu.%lu.%lu\n",
                (ip_address >> 24) & 0xff, (ip_address >> 16) & 0xff,
                (ip_address >> 8) & 0xff, ip_address & 0xff,
                (network_mask >> 24) & 0xff, (network_mask >> 16) & 0xff,
                (network_mask >> 8) & 0xff, network_mask & 0xff);

    /* Wake up TCP/UDP server threads */
    tx_thread_resume(&udp_server_thread);
    tx_thread_resume(&tcp_server_thread);
}

/**
  \fn        void udp_server_thread_entry(ULONG thread_input)
  \brief     Thread function for the UDP echo server thread.
  \param[in] thread_input. Unused.
*/
void udp_server_thread_entry(ULONG thread_input)
{
UINT status;
NX_PACKET *rx_packet;
ULONG ip_address;
UINT protocol;
UINT port;
UINT interface_index;
NXD_ADDRESS ipv4_address;

    NX_PARAMETER_NOT_USED(thread_input);

    /* Let the IP thread execute */
    tx_thread_sleep(NX_IP_PERIODIC_RATE);

    /* Create a UDP socket.  */
    status = nx_udp_socket_create(&ip_0, &udp_socket, "UDP Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, 0x80, 5);

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    /* Bind the UDP socket to the echo server port.  */
    status =  nx_udp_socket_bind(&udp_socket, UDP_SERVER_PORT, TX_WAIT_FOREVER);

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    while (1)
    {
        /* Receive a UDP packet.  */
        status =  nx_udp_socket_receive(&udp_socket, &rx_packet, TX_WAIT_FOREVER);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }

        /* Extract client information from UDP packet interface.*/
        status = nx_udp_packet_info_extract(rx_packet, &ip_address,
                            &protocol, &port, &interface_index);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }

        /* Set up the client address */
        ipv4_address.nxd_ip_version = NX_IP_VERSION_V4;
        ipv4_address.nxd_ip_address.v4 = ip_address;

        /* Send back the received packet */
        status =  nxd_udp_socket_send(&udp_socket, rx_packet, &ipv4_address, port);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }
    }
}

/**
  \fn        void tcp_server_thread_entry(ULONG thread_input)
  \brief     Thread function for the TCP echo server thread.
  \param[in] thread_input. Unused.
*/
void tcp_server_thread_entry(ULONG thread_input)
{
UINT       status;
NX_PACKET *packet_ptr;
ULONG      actual_status;

    NX_PARAMETER_NOT_USED(thread_input);

    /* Wait 1 second for the IP thread to finish its initilization. */
    tx_thread_sleep(NX_IP_PERIODIC_RATE);

    /* Ensure the IP instance has been initialized.  */
    status =  nx_ip_status_check(&ip_0, NX_IP_INITIALIZE_DONE, &actual_status, NX_IP_PERIODIC_RATE);

    if (status != NX_SUCCESS)
    {
        error_counter++;
        return;
    }

    /* Create a TCP socket */
    status =  nx_tcp_socket_create(&ip_0, &tcp_socket, "Server Socket",
                                   NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 100,
                                   NX_NULL, tcp_server_disconnect_received);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Setup this thread to listen.  */
    status =  nx_tcp_server_socket_listen(&ip_0, TCP_SERVER_PORT, &tcp_socket, 5, tcp_server_connect_received);

    if (status != NX_SUCCESS)
    {
        error_counter++;
    }

    /* Loop to create and establish connections.  */
    while (1)
    {
        /* Accept a client socket connection.  */
        status =  nx_tcp_server_socket_accept(&tcp_socket, NX_WAIT_FOREVER);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }

        while (1)
        {

            /* Receive a TCP message from the socket.  */
            status =  nx_tcp_socket_receive(&tcp_socket, &packet_ptr, NX_WAIT_FOREVER);

            if (status != NX_SUCCESS)
            {
                /* If status == NX_NOT_CONNECTED, that just means that the client has disconnected */
                if (status != NX_NOT_CONNECTED)
                {
                    error_counter++;
                }
                break;
            }

            /* Send the packet back */
            status =  nx_tcp_socket_send(&tcp_socket, packet_ptr, NX_IP_PERIODIC_RATE);

            if (status != NX_SUCCESS)
            {
                error_counter++;
                break;
            }
        }

        /* Disconnect the server socket.  */
        status =  nx_tcp_socket_disconnect(&tcp_socket, 10 * NX_IP_PERIODIC_RATE);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }

        /* Unaccept the server socket.  */
        status =  nx_tcp_server_socket_unaccept(&tcp_socket);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }

        /* Setup server socket for listening again.  */
        status =  nx_tcp_server_socket_relisten(&ip_0, TCP_SERVER_PORT, &tcp_socket);

        if (status != NX_SUCCESS)
        {
            error_counter++;
            break;
        }
    }
}

/**
  \fn        void tcp_server_connect_received(NX_TCP_SOCKET *socket_ptr, UINT port)
  \brief     TCP connect received callback function.
  \param[in] socket_ptr. The socket on which the connection request has been received.
  \param[in] port. The port on which the connection request has been received.
*/
void tcp_server_connect_received(NX_TCP_SOCKET *socket_ptr, UINT port)
{
    /* Check for proper socket and port.  */
    if ((socket_ptr != &tcp_socket) || (port != TCP_SERVER_PORT))
    {
        error_counter++;
    }
}

/**
  \fn        void tcp_server_disconnect_received(NX_TCP_SOCKET *socket)
  \brief     TCP disconnect received callback function.
  \param[in] socket_ptr. The socket on which the disconnect event has been received.
*/
void tcp_server_disconnect_received(NX_TCP_SOCKET *socket)
{
    /* Check for proper disconnected socket.  */
    if (socket != &tcp_socket)
    {
        error_counter++;
    }
}
