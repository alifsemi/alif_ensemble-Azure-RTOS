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
 * @file     nx_eqos_network_driver_private.h
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    Private header file for NetXDuo network driver for ETH MAC.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef NX_EQOS_NETWORK_DRIVER_PRIVATE_H
#define NX_EQOS_NETWORK_DRIVER_PRIVATE_H

#include "eqos_hw.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * enum DRIVER_STATE.
 * Different states that the  driver can be in.
 */
typedef enum {
    DRIVER_ATTACHED = 0x1,     /**< Driver has been attached to an IP instance. */
    DRIVER_INITIALIZED = 0x2,  /**< Driver has been initialized. */
    DRIVER_ENABLED = 0x4,      /**< Driver has been enabled. */
} DRIVER_STATE;

/**
  \struct MAC_ADDRESS
  \brief  Represents the 48bit MAC address.
*/
typedef struct MAC_ADDRESS_STRUCT
{
    ULONG nx_mac_address_msw;
    ULONG nx_mac_address_lsw;
} MAC_ADDRESS;

/**
  \struct NX_EQOS_DRIVER
  \brief  Represents the driver instance.
*/
typedef struct NX_EQOS_DRIVER_STRUCT
{
    UINT          nx_driver_state;          /**< The driver state */

    ULONG         nx_driver_total_rx_packets; /**< Total number of packets received */

    ULONG         nx_driver_total_tx_packets; /**< Total number of packets transmitted */

    ULONG         nx_driver_tx_packets_dropped; /**< Number of tx packets dropped because of unavailability of DMA descs */

    ULONG         nx_driver_allocation_errors; /**< Total packet allocation errors */

    NX_INTERFACE *nx_driver_interface_ptr;  /**< Pointer to the NetX interface structure */

    NX_IP        *nx_driver_ip_ptr;         /**< Pointer to the NetX IP instance attached to this driver */

    EQOS_DEV     *nx_driver_dev;            /**< Pointer to \ref EQOS_DEV owned by this driver */

    MAC_ADDRESS   nx_driver_mac_address;    /**< MAC Address */

    TX_TIMER      nx_driver_phy_poll_timer; /**< ThreadX timer for polling the link status */

    TX_THREAD     nx_driver_phy_poll_thread; /**< ThreadX thread for polling the link status */

    TX_EVENT_FLAGS_GROUP   nx_driver_phy_events; /**< ThreadX event flags group polling the link status */

    UCHAR         nx_driver_link_speed;   /**< Current link speed */

    UCHAR         nx_driver_link_duplex;  /**< Current link duplex information */

} NX_EQOS_DRIVER;

static VOID nx_eth_driver_mac_config(NX_EQOS_DRIVER *nx_eqos_driver, UINT link_info);
static VOID nx_eth_driver_enable(NX_EQOS_DRIVER *nx_eqos_driver);
static VOID nx_eth_driver_disable(NX_EQOS_DRIVER *nx_eqos_driver);

#ifdef  __cplusplus
}
#endif

#endif /* NX_EQOS_NETWORK_DRIVER_PRIVATE_H */
