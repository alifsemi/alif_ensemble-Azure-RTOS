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
 * @file     nx_eth_phy.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    NetxDuo Ethernet PHY driver.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "nx_eth_phy.h"
#include "nx_eth_phy_ctrl.h"

/* The PHY control object */
static PHY_CTRL phy;

/** \fn UINT nx_phy_init(uint8_t phy_id, phy_read_t read_fn, phy_write_t write_fn);
    \brief Initialize the PHY found at \a phy_id using MDIO read/write functions \a read_fn and \a write_fn.
    \param phy_id The phy address.
    \param read_fn The read function pointer for MDIO reads.
    \param write_fn The write function pointer for MDIO writes.
    \return The status of operation (NX_SUCCESS, NX_INVALID_PARAMETERS or NX_NOT_SUCCESSFUL)
*/
UINT nx_phy_init(uint8_t phy_id, phy_read_t read_fn, phy_write_t write_fn)
{
	uint16_t lower_id = 0, upper_id = 0;

	if ((!read_fn) || (!write_fn))
		return NX_INVALID_PARAMETERS;

	/* the PHY address is a 5bit value */
	if (phy_id > 31)
		return NX_INVALID_PARAMETERS;

	phy.id = phy_id;
	phy.read = read_fn;
	phy.write = write_fn;
	phy.flags = PHY_INITIALIZED;

	/*
	 * Read the phy id registers and make sure a phy is available
	 * at the provided address.
	 */
	if (!phy.read(phy.id, PHY_REG_PHYID2, &lower_id)) {
        /* A valid PHYID will not be all zeros or all ones */
        if ((lower_id != (uint16_t) ~0U) && (lower_id != (uint16_t) 0U)) {
            if (!phy.read(phy.id, PHY_REG_PHYID1, &upper_id)) {
                phy.device_id = (((uint32_t) upper_id) << 16) | (lower_id & 0xFFF0);
#ifdef NX_DEBUG
                printf("Found PHY, ID = %x\n", phy.device_id);
#endif
			    return NX_SUCCESS;
            }
		}
	}
	return NX_NOT_SUCCESSFUL;
}

/** \fn UINT nx_phy_setmode(UINT mode)
    \brief Set the link mode of a previously initialized PHY to \a mode.
    \param mode A bitmask of Speed/Duplex/Autonegotiation settings.
    \return NX_INVALID_INTERFACE if PHY has not been initialized or status of the operation(NX_SUCCESS/NX_NOT_SUCCESSFUL).
*/
UINT nx_phy_setmode(UINT mode)
{
	uint16_t val = 0;

	if (phy.flags != PHY_INITIALIZED)
		return NX_INVALID_INTERFACE;

	switch (mode & ETH_PHY_SPEED_MASK) {
	case ETH_PHY_SPEED_10:
		break;
	case ETH_PHY_SPEED_100:
		val |= PHY_CONTROL_SPEED_SELECT;
		break;
	}

	switch (mode & ETH_PHY_DUPLEX_MASK) {
	case ETH_PHY_DUPLEX_HALF:
		break;
	case ETH_PHY_DUPLEX_FULL:
		val |= PHY_CONTROL_DUPLEX_MODE;
		break;
	}

	if (mode & ETH_PHY_AUTONEGOTIATE) {
		val |= PHY_CONTROL_AN_ENABLE | PHY_CONTROL_RESTART_AN;
        phy.an_status = AN_ENABLED;
    }

	return (phy.write(phy.id, PHY_REG_CONTROL, val));
}

/** \fn UINT nx_phy_get_link_status(VOID)
    \brief Get the current link status.
    \return ETH_PHY_LINK_UP or ETH_PHY_LINK_DOWN.
    \warning Returns ETH_PHY_LINK_DOWN for a PHY interface that is not initialized/configured.
*/
UINT nx_phy_get_link_status(void)
{
	uint16_t val = 0;

	if (phy.flags == PHY_INITIALIZED) {
		phy.read(phy.id, PHY_REG_STATUS, &val);
	}

	if (val & PHY_STATUS_LINK_STATUS)
		return ETH_PHY_LINK_UP;
	else
		return ETH_PHY_LINK_DOWN;
}

/** \fn UINT nx_phy_get_link_info(VOID)
    \brief Get the current link speed/duplex information.
    \return Bitmask of current speed/duplex information.
    \warning Should be called only if the link has been initialized/configured AND is up.
*/
UINT nx_phy_get_link_info(void)
{
	uint16_t val = 0;
	uint32_t link_info = 0;
    uint32_t timeout = 10;

    /*
     * If the user has enabled AN, make sure we wait until AN completes
     * before retrieving the link information.
     */
    if ((phy.flags == PHY_INITIALIZED) && (phy.an_status == AN_ENABLED)) {
        phy.read(phy.id, PHY_REG_STATUS, &val);

        do {
            if (val & PHY_STATUS_AN_COMPLETE)
                break;
            else {
                tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
                phy.read(phy.id, PHY_REG_STATUS, &val);
            }
        } while (timeout--);

        if (!timeout) {
#ifdef NX_DEBUG
            printf("NetX PHY driver: Autonegotiation timed out\n");
#endif
            return link_info;
        }
    }


    if (phy.device_id == DEVICE_ID_KSZ8081RNB) {
	    if (phy.flags == PHY_INITIALIZED) {
		    phy.read(phy.id, PHY_CONTROL_1, &val);
	    }

	    if (val & PHY_CONTROL1_SPEED_100)
		    link_info |= ETH_PHY_SPEED_100;
	    else
		    link_info |= ETH_PHY_SPEED_10;

	    if (val & PHY_CONTROL1_DUPLEX_FULL)
		    link_info |= ETH_PHY_DUPLEX_FULL;
	    else
		    link_info |= ETH_PHY_DUPLEX_HALF;
    } else {

        /*
         * For PHYs like RTL8201F, the result of autonegotiation gets reflected
         * in the CONTROL register. If autonegotiation is not enabled, we read back
         * the speed/duplex settings configured by the user.
         */
	    if (phy.flags == PHY_INITIALIZED) {
		    phy.read(phy.id, PHY_REG_CONTROL, &val);
	    }

        if (val & PHY_CONTROL_SPEED_SELECT)
            link_info |= ETH_PHY_SPEED_100;
        else
            link_info |= ETH_PHY_SPEED_10;

        if (val & PHY_CONTROL_DUPLEX_MODE)
            link_info |= ETH_PHY_DUPLEX_FULL;
        else
            link_info |= ETH_PHY_DUPLEX_HALF;
    }

	return link_info;
}
