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
 * @file     nx_eth_phy_ctrl.h
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    Private headerfile for the NetXDuo PHY driver.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef NX_ETH_PHY_CTRL_H
#define NX_ETH_PHY_CTRL_H

#include "nx_eth_phy.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
  \struct PHY_CTRL
  \brief  Private data structure used by the PHY driver to represent a PHY.
*/
typedef struct PHY_CTRL_STRUCT {
    uint32_t device_id; /**< PHY Device ID */
    phy_read_t read;    /**< Read function pointer to access PHY registers */
    phy_write_t write;  /**< Write function pointer to access PHY registers */
    uint8_t id;         /**< The PHY address */
    uint8_t an_status;  /**< Auto negotiation status */
    uint8_t flags;      /**< flags, used to keep track of the PHY state */
} PHY_CTRL;

/* phy flags */
#define PHY_INITIALIZED			0x1 /**< The PHY has been initialized */
#define AN_ENABLED              0x1 /**< User has enabled auto negotiation */

#ifdef  __cplusplus
}
#endif
#endif /* NX_ETH_PHY_CTRL_H */
