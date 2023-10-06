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
 * @file     nx_eth_mac.h
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    Public header file for NetXDuo network driver for ETH MAC.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef NX_ETH_MAC_H
#define NX_ETH_MAC_H

#include "tx_api.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/** \brief 10mbps Link speed, return value for NX_LINK_GET_SPEED direct driver command */
#define NX_DRIVER_LINK_SPEED_10         (1 << 0)
/** \brief 100mbps Link speed, return value for NX_LINK_GET_SPEED direct driver command */
#define NX_DRIVER_LINK_SPEED_100        (1 << 1)

/** \brief Half duplex link duplex type, return value for NX_LINK_GET_DUPLEX_TYPE direct driver command */
#define NX_DRIVER_LINK_DUPLEX_HALF      (1 << 0)
/** \brief Full duplex link duplex type, return value for NX_LINK_GET_DUPLEX_TYPE direct driver command */
#define NX_DRIVER_LINK_DUPLEX_FULL      (1 << 1)

/**
  \fn VOID nx_eth_driver(NX_IP_DRIVER *driver_req_ptr)
  \brief The main entry point into the driver.
  \param[in] driver_req_ptr Pointer to the driver request structure.
 */
VOID nx_eth_driver(NX_IP_DRIVER *driver_req_ptr);

#ifdef  __cplusplus
}
#endif

#endif /* NX_ETH_MAC_H */
