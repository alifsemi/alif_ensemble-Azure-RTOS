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
 * @file     nx_eth_mac.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     2-July-2021
 * @brief    NetXDuo network interface driver for ETH MAC.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "nx_api.h"
#include "tx_api.h"

#include "nx_eth_phy.h"
#include "nx_eth_user.h"
#include "nx_eth_mac.h"
#include "nx_eth_mac_private.h"
#include "mac_hw.h"

#include "sys_ctrl_eth.h"
#include "system_utils.h"
#include "pinconf.h"

#define NX_LINK_MTU         1514 /**< The link MTU. Adds the size for physical header to the IP MTU. */
#define NX_ETHERNET_SIZE    14   /**< Size of the Ethernet header. */

/* Define Ethernet address format.  This is prepended to the incoming IP
   and ARP/RARP messages.  The frame beginning is 14 bytes, but for speed
   purposes, we are going to assume there are 16 bytes free in front of the
   prepend pointer and that the prepend pointer is 32-bit aligned.

    Byte Offset     Size            Meaning

        0           6           Destination Ethernet Address
        6           6           Source Ethernet Address
        12          2           Ethernet Frame Type, where:

                                        0x0800 -> IP Datagram
                                        0x0806 -> ARP Request/Reply
                                        0x0835 -> RARP request reply

        42          18          Padding on ARP and RARP messages only.  */

#define NX_ETHERNET_IP      0x0800 /**< EtherType for IPV4. */
#define NX_ETHERNET_ARP     0x0806 /**< EtherType for ARP. */
#define NX_ETHERNET_RARP    0x8035 /**< EtherType for RARP. */
#define NX_ETHERNET_IPV6    0x86DD /**< EtherType for IPV6. */

#define RX_EVENT            0x1 /**< Receive Event, set from the ISR. */
#define TX_EVENT            0x2 /**< Transmit complete Event, set from the ISR. */

#define TX_PHY_POLL_EVENT   0x1 /**< Event set by the phy polling timer. */

/** \brief Used to flag events from the ISR. */
static volatile uint8_t mac_events;

/** \brief Higher bytes of the MAC address. */
ULONG phys_address_msw = MAC_ADDRESS_HIGH;

/** \brief Lower bytes of the MAC address. */
ULONG phys_address_lsw = MAC_ADDRESS_LOW;

/** \brief MAC instance used */
static MAC_DEV mac0 = {
    .base_addr = ETH_BASE,
    .irq = ETH_SBD_IRQ_IRQn,
    .config = {
        .auto_neg_ctrl = ETH_AN_CONFIG,
        .speed = ETH_SPEED_CONFIG,
        .duplex = ETH_DUPLEX_CONFIG,
    },
    .irq_priority = RTE_ETH_MAC_IRQ_PRIORITY,
};

/** \brief Driver instance */
static NX_MAC_DRIVER nx_mac0_driver = {
    .nx_driver_dev = &mac0,
};

#define PHY_POLL_THREAD_STACKSIZE   512 /**< Stack size for the ThreadX thread used for link status polling. */

/** \brief Rx DMA Descriptors */
static DMA_DESC rx_dma_descs[RX_DESC_COUNT]__attribute__((section("eth_buf"))) __attribute((aligned(16)));

/** \brief Tx DMA Descriptors */
static DMA_DESC tx_dma_descs[TX_DESC_COUNT]__attribute__((section("eth_buf"))) __attribute((aligned(16)));

/** \brief NX_PACKET pointers, used to keep track of packets used during rx */
static NX_PACKET *rx_nx_packet_descs[RX_DESC_COUNT];

/** \brief NX_PACKET pointers, used to keep track of packets used during tx */
static NX_PACKET *tx_nx_packet_descs[TX_DESC_COUNT];

/** \brief static stack space for the ThreadX thread used for link status polling. */
static UCHAR phy_poll_thread_stack[PHY_POLL_THREAD_STACKSIZE];

/**
  \fn          static UINT pin_mux_configure(VOID)
  \brief       Configure  the pin mux.
  \return      The result of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
*/
static UINT pin_mux_configure(VOID)
{
int32_t ret;
uint32_t val;

    ret = pinconf_set(ETH_RXD0_PORT, ETH_RXD0_PIN, ETH_RXD0_PIN_FUNCTION, PADCTRL_READ_ENABLE);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_RXD1_PORT, ETH_RXD1_PIN, ETH_RXD1_PIN_FUNCTION, PADCTRL_READ_ENABLE);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_CRSDV_PORT, ETH_CRSDV_PIN, ETH_CRSDV_FUNCTION, PADCTRL_READ_ENABLE);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_RST_PORT, ETH_RST_PIN, ETH_RST_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_TXD0_PORT, ETH_TXD0_PIN, ETH_TXD0_PIN_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_TXD1_PORT, ETH_TXD1_PIN, ETH_TXD1_PIN_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_TXEN_PORT, ETH_TXEN_PIN, ETH_TXEN_PIN_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_IRQ_PORT, ETH_IRQ_PIN, ETH_IRQ_PIN_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_REFCLK_PORT, ETH_REFCLK_PIN, ETH_REFCLK_PIN_FUNCTION, PADCTRL_READ_ENABLE);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_MDIO_PORT, ETH_MDIO_PIN, ETH_MDIO_PIN_FUNCTION, PADCTRL_READ_ENABLE);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    ret = pinconf_set(ETH_MDC_PORT, ETH_MDC_PIN, ETH_MDC_PIN_FUNCTION, 0);
    if (ret)
        return NX_NOT_SUCCESSFUL;

    return NX_SUCCESS;
}

/**
  \fn        static VOID set_mac_addr(MAC_DEV *dev)
  \brief     Write the MAC address into the  MAC's registers.
  \param[in] dev The \ref MAC_DEV to act upon.
*/
static VOID set_mac_addr(MAC_DEV *dev)
{
ULONG val;

    val = ((phys_address_lsw & 0xFFU) << 8) | ((phys_address_lsw >> 8) & 0xFFU);

    val |= MAC_ADDR_HIGH_AE;

    mac_write_reg(dev, MAC_ADDR_HIGH(0), val);

    val = (((phys_address_lsw >> 16) & 0xFFU) << 24) |
          (((phys_address_lsw >> 24) & 0xFFU) << 16) |
          ((phys_address_msw & 0xFFU) << 8) |
          ((phys_address_msw >> 8) & 0xFFU);

    mac_write_reg(dev, MAC_ADDR_LOW(0), val);
}

/**
  \fn        static UINT init_rx_descs(MAC_DEV *dev)
  \brief     Initialize the Rx DMA descriptors and the DMA ring registers.
  \param[in] dev The \ref MAC_DEV to act upon.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
*/
static UINT init_rx_descs(MAC_DEV *dev)
{
uint32_t i, last_rx_desc;
DMA_DESC *desc;
NX_IP *ip_ptr = nx_mac0_driver.nx_driver_ip_ptr;
NX_PACKET *rx_packet;
UINT status = NX_SUCCESS;

    for (i = 0; i < RX_DESC_COUNT; i++)
    {
        desc = &dev -> rx_descs[i];

        status = nx_packet_allocate(ip_ptr -> nx_ip_default_packet_pool, &rx_packet,
                                    NX_RECEIVE_PACKET, TX_NO_WAIT);

        /* Check if the allocation was successful. */
        if (status)
        {
            /* Release previously allocated packets. */
            while (i)
            {
                i--;
                nx_packet_release(dev -> rx_nx_packets[i]);
            }

            nx_mac0_driver.nx_driver_allocation_errors++;

            return status;
        }

        /* the packet buffers are 8 byte aligned. Add 2 byte padding to get 2 byte
         * alignment for eth header so that IP header starts on a 4byte boundary */
        desc -> des0 = (uint32_t) (LocalToGlobal(rx_packet -> nx_packet_prepend_ptr + 2));

        dev -> rx_nx_packets[i] = rx_packet;

        desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN | RDES3_BUFFER1_VALID_ADDR;
    }

    mac_write_reg(dev, DMA_CHAN_RX_BASE_ADDR(0), (uint32_t) (LocalToGlobal((dev -> rx_descs))));
    mac_write_reg(dev, DMA_CHAN_RX_RING_LEN(0), RX_DESC_COUNT - 1 );

    last_rx_desc = (uint32_t) (LocalToGlobal(&(dev -> rx_descs[RX_DESC_COUNT - 1])));
    mac_write_reg(dev, DMA_CHAN_RX_END_ADDR( 0 ), last_rx_desc);

    SCB_CleanDCache_by_Addr((uint32_t *) dev -> rx_descs, RX_DESC_COUNT * sizeof(DMA_DESC));

    return status;
}

/**
  \fn        static VOID init_tx_descs(MAC_DEV *dev)
  \brief     Initialize the Tx DMA descriptors and the DMA ring registers.
  \param[in] dev The \ref MAC_DEV to act upon.
*/
static VOID init_tx_descs(MAC_DEV *dev)
{
UINT i;

    for (i = 0; i < TX_DESC_COUNT; i++ )
        dev -> tx_descs[i] = (DMA_DESC) {0, 0, 0, 0};

    mac_write_reg(dev, DMA_CHAN_TX_BASE_ADDR(0), (uint32_t) (LocalToGlobal((dev -> tx_descs))));
    mac_write_reg(dev, DMA_CHAN_TX_RING_LEN(0), TX_DESC_COUNT - 1 );
    SCB_CleanDCache_by_Addr((uint32_t *) dev -> tx_descs, TX_DESC_COUNT * sizeof(DMA_DESC));
}

/**
  \fn        static UINT init_descs(MAC_DEV *dev)
  \brief     Allocate area for the DMA descriptors and initialize them..
  \param[in] dev The \ref MAC_DEV to act upon.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
*/
static UINT init_descs(MAC_DEV *dev)
{
UINT status;

    dev -> tx_descs = tx_dma_descs;
    dev -> rx_descs = rx_dma_descs;

    dev -> rx_nx_packets = rx_nx_packet_descs;
    dev -> tx_nx_packets = tx_nx_packet_descs;

    status = init_rx_descs(dev);

    if (status)
    {
        return status;
    }

    init_tx_descs(dev);

    return NX_SUCCESS;
}

/**
  \fn        static UINT mac_hw_init(MAC_DEV *dev)
  \brief     Initialize the MAC IP block.
  \param[in] dev The \ref MAC_DEV to act upon.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
*/
static UINT mac_hw_init(MAC_DEV *dev)
{
UINT val, status;
UINT timeout = 10;

#ifdef MTL_HAS_MULTIPLE_QUEUES
UINT tx_fifo_sz, rx_fifo_sz, tqs, rqs;
#endif

    /* Soft reset the logic */
    val = mac_read_reg(dev, DMA_BUS_MODE);
    val |= DMA_BUS_MODE_SFT_RESET;
    mac_write_reg(dev, DMA_BUS_MODE, val);

    do
    {
	    tx_thread_sleep(1);
        val = mac_read_reg(dev, DMA_BUS_MODE);
        timeout--;
    } while((val & DMA_BUS_MODE_SFT_RESET) && timeout);

    if (timeout == 0)
    {
        return NX_NOT_SUCCESSFUL;
    }

    /* Configure MTL Tx Q0 Operating mode */
    val = mac_read_reg(dev, MTL_CHAN_TX_OP_MODE(0));
    val &= ~MTL_OP_MODE_TXQEN_MASK;
    val |= MTL_OP_MODE_TXQEN | MTL_OP_MODE_TSF;
    mac_write_reg(dev, MTL_CHAN_TX_OP_MODE(0), val);

    /* Configure MTL RX Q0 operating mode */
    val = mac_read_reg(dev, MTL_CHAN_RX_OP_MODE(0));
    val |= MTL_OP_MODE_RSF | MTL_OP_MODE_FEP | MTL_OP_MODE_FUP;
    mac_write_reg(dev, MTL_CHAN_RX_OP_MODE(0), val);

#ifdef MTL_HAS_MULTIPLE_QUEUES
    /*
     *  configure the MTL T/Rx Q0 sizes by finding out the configured
     * Tx RX FIFO sizes. Note that this is not needed if the IP is
     * configured to have only one queue (each for Tx and Rx).
     */

    val = mac_read_reg(dev, MAC_HW_FEATURE1);

    tx_fifo_sz = (val >> MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
                MAC_HW_FEATURE1_TXFIFOSIZE_MASK;

    rx_fifo_sz = (val >> MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
                MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

    /*
     * FIFO sizes are encoded as log2(fifo_size) - 7 in the above field
     * and tqs and rqs needs to be programmed in blocks of 256 bytes.
     */
    tqs = (128 << tx_fifo_sz) / 256 - 1;
    rqs = (128 << rx_fifo_sz) / 256 - 1;

    val = mac_read_reg(dev, MTL_CHAN_TX_OP_MODE(0));
    val &= ~(MTL_TXQ0_OPERATION_MODE_TQS_MASK << MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    val |= (tqs <<  MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    mac_write_reg(dev, MTL_CHAN_TX_OP_MODE(0), val);

    val = mac_read_reg(dev, MTL_CHAN_RX_OP_MODE(0));
    val &= ~(MTL_RXQ0_OPERATION_MODE_RQS_MASK << MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    val |= (rqs << MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    mac_write_reg(dev, MTL_CHAN_RX_OP_MODE(0), val);
#endif

    /* Enable MAC RXQ */
    val = mac_read_reg(dev, MAC_RXQ_CTRL0);
    val &= ~(MAC_RXQ_CTRL0_RXQ0EN_MASK <<
                        MAC_RXQ_CTRL0_RXQ0EN_SHIFT);
    val |= (MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB <<
        MAC_RXQ_CTRL0_RXQ0EN_SHIFT);
    mac_write_reg(dev, MAC_RXQ_CTRL0, val);

    /* Configure RXQ control1 for routing multicast/broadcast queues
     * Note that by default, Q0 will be used.
     */
    val = mac_read_reg(dev, MAC_RXQ_CTRL1);
    val |= MAC_RXQCTRL_MCBCQEN;
    mac_write_reg(dev, MAC_RXQ_CTRL1, val);

    /* Configure tx and rx flow control */
    val = mac_read_reg(dev, MAC_QX_TX_FLOW_CTRL(0));
    val |= 0xffff << MAC_Q0_TX_FLOW_CTRL_PT_SHIFT | MAC_Q0_TX_FLOW_CTRL_TFE;
    mac_write_reg(dev, MAC_QX_TX_FLOW_CTRL(0), val);

    val = mac_read_reg(dev, MAC_RX_FLOW_CTRL);
    val |= MAC_RX_FLOW_CTRL_RFE;
    mac_write_reg(dev, MAC_RX_FLOW_CTRL, MAC_RX_FLOW_CTRL_RFE);

    val = mac_read_reg(dev, MAC_CONFIG);
    val &= ~MAC_CONFIG_JE;
    val |= MAC_CONFIG_DM;
#ifdef NX_ENABLE_INTERFACE_CAPABILITY
    /* Enable L3/L4 Rx checksum offloading */
    val |= MAC_CONFIG_IPC;
#endif
    mac_write_reg(dev, MAC_CONFIG, val);

    val = mac_read_reg(dev, MAC_PACKET_FILTER);
    val |= MAC_PACKET_FILTER_PM;
    mac_write_reg(dev, MAC_PACKET_FILTER, val);

    /* Configure the DMA block */
    val = mac_read_reg(dev, DMA_CHAN_RX_CONTROL(0));
    val |= 16 << DMA_CH0_TX_CONTROL_TXPBL_SHIFT;
    mac_write_reg(dev, DMA_CHAN_TX_CONTROL(0), val);

    val = mac_read_reg(dev, DMA_CHAN_RX_CONTROL(0));
    val |= 16 << DMA_CH0_RX_CONTROL_RXPBL_SHIFT;
    val |= dev -> rx_buf_size << DMA_CH0_RX_CONTROL_RBSZ_SHIFT;
    val |= DMA_CH0_RX_CONTROL_RPF;
    mac_write_reg( dev, DMA_CHAN_RX_CONTROL(0), val);

    val = mac_read_reg(dev, DMA_SYS_BUS_MODE);
    val |= DMA_SYSBUS_MODE_BLEN4 | DMA_SYSBUS_MODE_BLEN8 |
            DMA_SYSBUS_MODE_BLEN16;

    val |= 3 << DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT;
    val |= 1 << DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT;
    val |= DMA_SYSBUS_MODE_ONEKBBE;

    mac_write_reg(dev, DMA_SYS_BUS_MODE, val);

    status = init_descs(dev);

    if (status)
    {
        return status;
    }

    set_mac_addr(dev);

    return 0;
}

/**
  \fn static UINT mac_dev_phy_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data,
                            MAC_DEV *dev)
  \brief MAC Hardware instance specific utility function for \ref mac_phy_read().
  \param[in] phy_addr The PHY address.
  \param[in] reg_addr Register address.
  \param[out] data Address to store the read value.
  \param[in] dev Pointer to the \ref MAC_DEV.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
 */
static UINT mac_dev_phy_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data,
                            MAC_DEV *dev)
{
uint32_t val, timeout = 5;

    val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
            (reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
            (MAC_MDIO_ADDR_GOC_READ << MAC_MDIO_ADDR_GOC_SHIFT) |
            (MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT)|
            MAC_MDIO_ADDR_GB;

    mac_write_reg(dev, MAC_MDIO_ADDR, val);

    do
    {
        if (!(mac_read_reg(dev, MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB))
        {
            *data = mac_read_reg(dev, MAC_MDIO_DATA);
            break;
        }
	    tx_thread_sleep(1);
        timeout--;
    } while (timeout);

    if (timeout == 0)
    {
        return NX_NOT_SUCCESSFUL;
    }

    return NX_SUCCESS;
}

/**
  \fn static UINT mac_dev_phy_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data,
                            MAC_DEV *dev)
  \brief MAC Hardware instance specific utility function for \ref mac_phy_read().
  \param[in] phy_addr The PHY address.
  \param[in] reg_addr Register address.
  \param[in] data Value to write.
  \param[in] dev Pointer to the \ref MAC_DEV.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
 */
static UINT mac_dev_phy_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data,
                            MAC_DEV *dev)
{
uint32_t val, timeout = 5;

    val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
                (reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
                (MAC_MDIO_ADDR_GOC_WRITE << MAC_MDIO_ADDR_GOC_SHIFT) |
                (MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) |
                MAC_MDIO_ADDR_GB;

    mac_write_reg(dev, MAC_MDIO_DATA, data);
    mac_write_reg(dev, MAC_MDIO_ADDR, val);

    do {
        if(!(mac_read_reg(dev, MAC_MDIO_ADDR) & MAC_MDIO_ADDR_GB))
        {
            break;
        }
	    tx_thread_sleep(1);
        timeout--;
    } while(timeout);

    if (timeout == 0)
    {
        return NX_NOT_SUCCESSFUL;
    }

    return NX_SUCCESS;
}

/**
  \fn static UINT mac_phy_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief Read a PHY register over MDIO.
  \param[in] phy_addr The PHY address.
  \param[in] reg_addr Register address.
  \param[out] data Address where read data should be stored.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
 */
static UINT mac_phy_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
{
    return mac_dev_phy_read(phy_addr, reg_addr, data, &mac0);
}

/**
  \fn static UINT mac_phy_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief Write a PHY register over MDIO.
  \param[in] phy_addr The PHY address.
  \param[in] reg_addr Register address.
  \param[in] data The data to be written.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
 */
static UINT mac_phy_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    return mac_dev_phy_write(phy_addr, reg_addr, data, &mac0);
}

/**
  \fn static void nx_timer_handler(ULONG arg)
  \brief Timer handler for polling the link status.
  \param[in] arg Unused argument.
 */
static void nx_timer_handler(ULONG arg)
{
    /* Unused argument */
    (void) arg;

    tx_event_flags_set(&nx_mac0_driver.nx_driver_phy_events, TX_PHY_POLL_EVENT, TX_OR);
}

/**
  \fn static UINT nx_eth_driver_output(NX_PACKET *packet_ptr)
  \brief Program the DMA registers to schedule the Tx for outgoing packets..
  \param[in] packet_ptr The NX_PACKET to be sent out.
 */
static UINT nx_eth_driver_output(NX_PACKET *packet_ptr)
{
UINT packet_length, cur_idx;
MAC_DEV *dev = &mac0;
DMA_DESC *desc;
UINT desc_count = 0;
NX_PACKET *current_packet;
TX_INTERRUPT_SAVE_AREA;

    cur_idx = dev -> cur_tx_desc_id;

    current_packet = packet_ptr;

    while (current_packet) {
        desc = &dev -> tx_descs[cur_idx];

        SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

        /* check whether the descriptor is available */
        if ((desc -> des3) & TDES3_OWN)
        {
            /* descriptor not available */
            return NX_NOT_SUCCESSFUL;
        }

        packet_length = current_packet->nx_packet_append_ptr - current_packet->nx_packet_prepend_ptr;

        desc -> des2 = TDES2_INTERRUPT_ON_COMPLETION | packet_length;

        desc -> des0 = (uint32_t) (LocalToGlobal((current_packet -> nx_packet_prepend_ptr)));

        /* reset DES3 */
        desc -> des3 = 0x0;

        SCB_CleanDCache_by_Addr((uint32_t *) current_packet -> nx_packet_prepend_ptr, packet_length);

        /* check if this is the last packet in the chain */
        if (!current_packet -> nx_packet_next)
        {
            /* save the original (head) packet pointer */
            dev -> tx_nx_packets[cur_idx] = packet_ptr;

            desc -> des3 = TDES3_LAST_DESCRIPTOR;
        }

        if (current_packet == packet_ptr)
        {
            /* If this is the first packet in the chain, set FD */
            desc -> des3 |= TDES3_FIRST_DESCRIPTOR;

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
            /* Enable L3/L4 checksum insertion for this packet chain */
            desc->des3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);
#endif
        }

        current_packet =  current_packet -> nx_packet_next;

        cur_idx++;

        cur_idx %= TX_DESC_COUNT;

        desc_count++;
    }

    /* update current desc id */
    dev -> cur_tx_desc_id = cur_idx;

    TX_DISABLE

    dev -> tx_descs_in_use += desc_count;

    TX_RESTORE

    /* starting with cur_idx - 1, hand over all the used descs to the DMA */
    for (; desc_count > 0U; desc_count--)
    {
        /* go from the last descriptor to the first used in this chain */
        if (cur_idx == 0)
            cur_idx = TX_DESC_COUNT - 1;
        else
            cur_idx--;

        desc = &dev -> tx_descs[cur_idx];

        desc -> des3 |= TDES3_OWN;

        SCB_CleanDCache_by_Addr((uint32_t *) desc, sizeof(DMA_DESC));
    }

    /* move the tail pointer */
    mac_write_reg(dev, DMA_CHAN_TX_END_ADDR(0),
               (uint32_t) (LocalToGlobal(&(dev -> tx_descs[dev -> cur_tx_desc_id]))));

    /* increment the count of transmitted packets */
    nx_mac0_driver.nx_driver_total_tx_packets++;

    return NX_SUCCESS;
}

/**
  \fn static void eth_irqhandler(MAC_DEV *dev)
  \brief MAC IP instance specific routine to assist the ISR.
  \param[in] dev Pointer to the \ref MAC_DEV.
 */
static void eth_irqhandler(MAC_DEV *dev)
{
UINT stat, ch0_stat;

    stat = mac_read_reg(dev, DMA_STATUS);

    if (stat & DMA_STATUS_CHAN0)
    {
        ch0_stat = mac_read_reg(dev, DMA_CHAN_STATUS(0));

        mac_write_reg(dev, DMA_CHAN_STATUS(0),
                ch0_stat & (DMA_CHAN_STATUS_NIS | DMA_CHAN_STATUS_RI | DMA_CHAN_STATUS_TI |
                                             DMA_CHAN_STATUS_AIS | DMA_CHAN_STATUS_RBU));

        if (ch0_stat & DMA_CHAN_STATUS_RI)
        {
            mac_events |= RX_EVENT;
        }

        if (ch0_stat & DMA_CHAN_STATUS_TI)
        {
            mac_events |= TX_EVENT;
        }

        if (mac_events)
        {
            /* request deferred processing to complete rx or tx */
            _nx_ip_driver_deferred_processing(nx_mac0_driver.nx_driver_ip_ptr);
        }
    }
}

/**
  \fn void ETH_SBD_IRQHandler(void)
  \brief The interrupt handler.
 */
void ETH_SBD_IRQHandler(void)
{
    eth_irqhandler(&mac0);
}

/**
 \fn static void update_config(void)
 \To update the configurations when link is up
 */
static UINT nx_update_config(void)
{
UINT info, ret = NX_SUCCESS, mode = 0;
MAC_DEV *dev = nx_mac0_driver.nx_driver_dev;

    if (dev -> config.auto_neg_ctrl == AN_DISABLE)
    {
        /* AN disabled, configure the phy with user provided settings */
        if (dev -> config.speed == ETH_SPEED_10M)
            mode |= ETH_PHY_SPEED_10;
        else
            mode |= ETH_PHY_SPEED_100;

        if (dev -> config.duplex == ETH_DUPLEX_FULL)
            mode |= ETH_PHY_DUPLEX_FULL;
        else
            mode |= ETH_PHY_DUPLEX_HALF;

    }
    else
    {
        mode |= ETH_PHY_AUTONEGOTIATE;
    }

    ret = nx_phy_setmode(mode);

    if (ret != NX_SUCCESS)
	    return ret;

	nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up = NX_TRUE;
	/*
     *  Get the link info and program the MAC with the the new settings.
	 */
	info = nx_phy_get_link_info();

	/* Update the MAC configuration */
	nx_eth_driver_disable(&nx_mac0_driver);
	nx_eth_driver_mac_config(&nx_mac0_driver, info);
	nx_eth_driver_enable(&nx_mac0_driver);
	return ret;
}
/**
  \fn static void nx_eth_driver_receive(NX_MAC_DRIVER *nx_mac_driver)
  \brief Receive and push the packets up the stack.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static void nx_eth_driver_receive(NX_MAC_DRIVER *nx_mac_driver)
{
UINT packet_type, cur_idx;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;
DMA_DESC *desc;
UINT status = NX_SUCCESS;
NX_PACKET *rx_packet, *new_packet, *last_packet, *packet;
NX_IP *ip_ptr;
NX_INTERFACE *interface_ptr;
UINT desc_count = 1;
UINT first_idx, last_idx;
UINT total_length, cur_length, accumulated_length = 0, length;
INT cur_count, i;

    ip_ptr = nx_mac_driver -> nx_driver_ip_ptr;
    interface_ptr = nx_mac_driver -> nx_driver_interface_ptr;

    cur_idx = dev -> cur_rx_desc_id;
    desc = &dev -> rx_descs[cur_idx];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

    while(!(desc -> des3 & RDES3_OWN))
    {
        accumulated_length = 0;
        desc_count = 1;

        cur_idx = dev -> cur_rx_desc_id;

        first_idx = last_idx = cur_idx;

        /* Find out the last descriptor for the received frame */
        while (!((desc -> des3) & RDES3_LAST_DESCRIPTOR))
        {
            if (desc -> des3 & RDES3_OWN)
            {
                /* reception is still in progress for this chain, return */
                return;
            }
            last_idx++;
            last_idx %= RX_DESC_COUNT;
            desc_count++;
            desc =  &dev -> rx_descs[last_idx];
            SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));
        }

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
        /* retrieve the last descriptor in this packet chain */
        desc = &dev -> rx_descs[last_idx];

        /* Check if the COE has reported any errors */
        if ((desc -> des1 & RDES1_IP_HDR_ERROR) || (desc -> des1 & RDES1_IP_CSUM_ERROR))
        {
            /*
             * There are checksum errors, do not process this packet chain. Program
             * the DMA to reuse buffers that are already allocated to the descriptors
             */
            for (i = 0; i < desc_count; i++)
            {
                desc = &dev -> rx_descs[cur_idx];

                desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                               RDES3_BUFFER1_VALID_ADDR;

                SCB_CleanDCache_by_Addr((uint32_t *) &dev -> rx_descs[cur_idx], sizeof(DMA_DESC));

                cur_idx++;

                cur_idx %= RX_DESC_COUNT;
            }

            mac_write_reg(dev, DMA_CHAN_RX_END_ADDR(0),
                   (uint32_t) (LocalToGlobal(&(dev -> rx_descs[last_idx]))));

            dev -> cur_rx_desc_id = cur_idx;

            /* Point to the next descriptor after this packet chain */
            desc = &dev -> rx_descs[dev -> cur_rx_desc_id];

            SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

            /* Go and process the next descriptor */
            continue;
        }
#endif
        /* Get the stored NX_PACKET for the first descriptor */
        rx_packet = dev -> rx_nx_packets[cur_idx];

        /* Get the stored NX_PACKET for the last descriptor in chain */
        last_packet = dev -> rx_nx_packets[last_idx];

        /* For the first packet in chain, adjust the prepend ptr */
        rx_packet -> nx_packet_prepend_ptr += 2;

        /* Get the total packet length from the last descriptor */
        total_length = ((dev -> rx_descs[last_idx]).des3 & 0x7fff) - 4;

        rx_packet -> nx_packet_length = total_length;

        /* Now process all the packets in the chain except the last one */
        for (cur_count = 0; cur_count < (desc_count - 1); cur_count++)
        {
            /* Allocate a new packet to be inserted into the descriptor */
            status = nx_packet_allocate(ip_ptr -> nx_ip_default_packet_pool, &new_packet,
                                    NX_RECEIVE_PACKET, TX_NO_WAIT);

            if (status != NX_SUCCESS)
            {
                /* Allocation failure, we cannot process this packet chain */
                nx_mac_driver -> nx_driver_allocation_errors++;

                /*
                 * If the allocation did fail for the first packet in the chain, we just program all the
                 * DMA descriptors in the chain to reuse the buffers already allocated to them.
                 * But if we fail in the middle of the processing of a chain, we need to release the chain
                 * that we have already assembled back to the packet pool and then program the remaining
                 * DMA descriptors to reuse the buffers already allocated to them.
                 */

                if (cur_idx != first_idx)
                {
                    /* 'packet' points to the last NX_PACKET that we successfully processed in this chain */

                    /* break the chain */
                    packet -> nx_packet_next = NX_NULL;

                    /* release the chain of packets that we have prepared until now */

                    /* Clean off the Ethernet header.  */
                    rx_packet -> nx_packet_prepend_ptr =  rx_packet -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

                    /* Adjust the packet length.  */
                    rx_packet -> nx_packet_length =  rx_packet -> nx_packet_length - NX_ETHERNET_SIZE;

                    /* Release the packet. */
                    nx_packet_release(rx_packet);
                }

                /* Now program the descriptors to reuse the previously configured packets */
                for (i = cur_count; i < desc_count; i++)
                {
                    desc = &dev -> rx_descs[cur_idx];

                    desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                                    RDES3_BUFFER1_VALID_ADDR;

                    SCB_CleanDCache_by_Addr((uint32_t *) &dev -> rx_descs[cur_idx], sizeof(DMA_DESC));

                    cur_idx++;

                    cur_idx %= RX_DESC_COUNT;
                }
                mac_write_reg(dev, DMA_CHAN_RX_END_ADDR(0),
                   (uint32_t) (LocalToGlobal(&(dev -> rx_descs[last_idx]))));

                dev -> cur_rx_desc_id = cur_idx;

                /* Point to the next descriptor after this packet chain */
                desc = &dev -> rx_descs[dev -> cur_rx_desc_id];

                SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));

                break;
            }

            /* Retrieve the NX_PACKET pointer for this descriptor */
            packet = dev -> rx_nx_packets[cur_idx];

            desc = &dev -> rx_descs[cur_idx];

            /* Find out the total number of bytes received for the current packet */
            cur_length = desc -> des3 & 0x7fff;

            /* subtract the accumulated length before this to get the length of this buffer */
            length = cur_length - accumulated_length;

            /* update accumulated length */
            accumulated_length = cur_length;

            /* Adjust the append pointer */
            packet -> nx_packet_append_ptr = packet -> nx_packet_prepend_ptr + length;

            SCB_InvalidateDCache_by_Addr(packet -> nx_packet_prepend_ptr, length);

            /* Prepare the descriptor for reception with the newly allocated packet.
            * the packet buffers are 8 byte aligned add 2 byte padding to get 2 byte
             * alignment for eth header so that IP header starts on a 4byte boundary */
            desc -> des0 = (uint32_t) (LocalToGlobal((new_packet -> nx_packet_prepend_ptr + 2)));

            dev -> rx_nx_packets[cur_idx] = new_packet;

            desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                            RDES3_BUFFER1_VALID_ADDR;

            SCB_CleanDCache_by_Addr((uint32_t *) &dev -> rx_descs[cur_idx], sizeof(DMA_DESC));

            /* increment cur_idx to point to the next descriptor */
            cur_idx++;

            cur_idx %= RX_DESC_COUNT;

            /* Link the packet in the chain */
            packet -> nx_packet_next = dev -> rx_nx_packets[cur_idx];

            /* Create a link to the last packet */
            packet -> nx_packet_last = last_packet;
        }

        if (status != NX_SUCCESS)
        {
            /*
             * we had an allocation failure and are breaking out of the for loop
             * handling all the packets other than the last packet, desc now points
             * the next descriptor after this packet chain, try to process it.
             */
            continue;
        }

        /* Only the last packet is remaining now, process it */

        /* Allocate a packet for the last descriptor */
        status = nx_packet_allocate(ip_ptr -> nx_ip_default_packet_pool, &new_packet,
                                    NX_RECEIVE_PACKET, TX_NO_WAIT);

        if (status != NX_SUCCESS)
        {
            /* Allocation failure, we cannot process this chain */
            nx_mac_driver -> nx_driver_allocation_errors++;

            /* 'packet' points to the last packet in the chain that we successfully processed */

            /* break the chain */
            packet -> nx_packet_next = NX_NULL;

            /* release the chain of packets that we have prepared until now */

            /* Clean off the Ethernet header.  */
            rx_packet -> nx_packet_prepend_ptr =  rx_packet -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

            /* Adjust the packet length.  */
            rx_packet -> nx_packet_length =  rx_packet -> nx_packet_length - NX_ETHERNET_SIZE;

            /* Release the packet */
            nx_packet_release(rx_packet);

            /* Now, program the last descriptor to reuse the old buffer */
            desc = &dev -> rx_descs[cur_idx];

            desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                                       RDES3_BUFFER1_VALID_ADDR;

            SCB_CleanDCache_by_Addr((uint32_t *) &dev -> rx_descs[cur_idx], sizeof(DMA_DESC));

            cur_idx++;

            cur_idx %= RX_DESC_COUNT;

            dev -> cur_rx_desc_id = cur_idx;

            mac_write_reg(dev, DMA_CHAN_RX_END_ADDR(0),
                   (uint32_t) (LocalToGlobal(&(dev -> rx_descs[last_idx]))));

            /* Try to process the next descriptor */
            desc = &dev -> rx_descs[dev -> cur_rx_desc_id];

            SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));
            continue;
        }

        /* mark end of the chain */
        last_packet -> nx_packet_next = NX_NULL;

        /* Find out length for the last packet */
        length = total_length - accumulated_length;

        last_packet -> nx_packet_append_ptr = last_packet -> nx_packet_prepend_ptr + length;

        SCB_InvalidateDCache_by_Addr(last_packet -> nx_packet_prepend_ptr, length);

        /* Now pickup the forwarding information from the first packet and send the chain up the stack */
        /* Pickup the packet header to determine where the packet needs to be sent.  */
        packet_type =  (((UINT)(*(rx_packet -> nx_packet_prepend_ptr + 12))) << 8) |
                        ((UINT)(*(rx_packet -> nx_packet_prepend_ptr + 13)));

        /* Setup interface pointer.  */
        rx_packet -> nx_packet_address.nx_packet_interface_ptr = interface_ptr;

        /* Clean off the Ethernet header.  */
        rx_packet -> nx_packet_prepend_ptr =  rx_packet -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

        /* Adjust the packet length.  */
        rx_packet -> nx_packet_length =  rx_packet -> nx_packet_length - NX_ETHERNET_SIZE;

        /* Increment the count of total received packets */
        nx_mac_driver -> nx_driver_total_rx_packets++;

        /* Route the incoming packet according to its ethernet type.  */
        if ((packet_type == NX_ETHERNET_IP) || (packet_type == NX_ETHERNET_IPV6))
        {
            /* Route to the ip receive function.  */
#ifdef NX_DEBUG_PACKET
            printf("NetX Ethernet Driver IP Packet Receive - %s\n", ip_ptr -> nx_ip_name);
#endif
            _nx_ip_packet_receive(ip_ptr, rx_packet);
        }
#ifndef NX_DISABLE_IPV4
        else if (packet_type == NX_ETHERNET_ARP)
        {
            /* Route to the ARP receive function.  */
#ifdef NX_DEBUG
            printf("NetX Ethernet Driver ARP Receive - %s\n", ip_ptr -> nx_ip_name);
#endif
            _nx_arp_packet_deferred_receive(ip_ptr, rx_packet);
        }
        else if (packet_type == NX_ETHERNET_RARP)
        {
            /* Route to the RARP receive function.  */
#ifdef NX_DEBUG
            printf("NetX Ethernet Driver RARP Receive - %s\n", ip_ptr -> nx_ip_name);
#endif
            _nx_rarp_packet_deferred_receive(ip_ptr, rx_packet);
        }
#endif /* !NX_DISABLE_IPV4  */
        else
        {
            /* Invalid packet type */
            nx_packet_release(rx_packet);
        }

        desc = &dev -> rx_descs[last_idx];

        /* The packet buffers are 8 byte aligned. Add 2 byte padding to get 2 byte
         * alignment for eth header so that IP header starts on a 4byte boundary */
        desc -> des0 = (uint32_t)(LocalToGlobal(new_packet -> nx_packet_prepend_ptr + 2));

        dev -> rx_nx_packets[last_idx] = new_packet;

        desc -> des3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                            RDES3_BUFFER1_VALID_ADDR;

        SCB_CleanDCache_by_Addr((uint32_t *) &dev -> rx_descs[last_idx], sizeof(DMA_DESC));

        mac_write_reg(dev, DMA_CHAN_RX_END_ADDR(0),
                   (uint32_t) (LocalToGlobal(&(dev -> rx_descs[last_idx]))));

        /* Move on to the next descriptor */
        dev -> cur_rx_desc_id = last_idx + 1;
        dev -> cur_rx_desc_id %= RX_DESC_COUNT;
        desc = &dev -> rx_descs[dev -> cur_rx_desc_id];

        SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));
    }
    tx_timer_deactivate(&nx_mac0_driver.nx_driver_phy_poll_timer);

    /* Modify the polling period */
    tx_timer_change(&nx_mac0_driver.nx_driver_phy_poll_timer, NX_DRIVER_PHY_POLLING_INTERVAL_LONG * TX_TIMER_TICKS_PER_SECOND,
               NX_DRIVER_PHY_POLLING_INTERVAL_LONG * TX_TIMER_TICKS_PER_SECOND);

    tx_timer_activate(&nx_mac0_driver.nx_driver_phy_poll_timer);


    if(nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up == NX_FALSE)
    {
        nx_update_config();

        /* Notify the stack of the link status change */
        _nx_ip_driver_link_status_event(nx_mac0_driver.nx_driver_ip_ptr,
                                   nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_index);
     }

}

/**
  \fn static void nx_eth_driver_process_tx_complete(NX_MAC_DRIVER *nx_mac_driver)
  \brief Process the Tx complete interrupt.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static void nx_eth_driver_process_tx_complete(NX_MAC_DRIVER *nx_mac_driver)
{
DMA_DESC *desc;
NX_PACKET *tx_packet;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;
uint32_t cur_idx = dev -> cur_tx_desc_id;
uint32_t release_desc_id =  dev -> tx_release_desc_id;
uint32_t descs_in_use =  dev -> tx_descs_in_use;

    /* retrieve the descriptor to start clearing */
    desc = &dev -> tx_descs[release_desc_id];

    SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));


    while ((desc -> des3 & TDES3_OWN) == 0)
    {
        if (release_desc_id == cur_idx && !descs_in_use)
        {
            /* we have reached the last unused desc, exit the loop */
            break;
        }

        if (desc -> des3 & TDES3_LAST_DESCRIPTOR)
        {
            /* Retrieve the NX_PACKET pointer */
            tx_packet = dev -> tx_nx_packets[release_desc_id];

            /* Remove the Ethernet header */
            tx_packet -> nx_packet_prepend_ptr =  tx_packet -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

            /* Adjust the packet length.  */
            tx_packet -> nx_packet_length =  tx_packet -> nx_packet_length - NX_ETHERNET_SIZE;

            /* Now that the Ethernet frame has been removed, release the packet.  */
            nx_packet_transmit_release(tx_packet);
        }

        /* move on to the next descriptor to clear */
        descs_in_use--;
        release_desc_id++;
        release_desc_id %= TX_DESC_COUNT;

        desc = &dev -> tx_descs[release_desc_id];

        SCB_InvalidateDCache_by_Addr(desc, sizeof(DMA_DESC));
    }

    dev -> tx_release_desc_id = release_desc_id;
    dev -> tx_descs_in_use = descs_in_use;
}

/**
  \fn static void nx_eth_driver_deferred_processing(NX_MAC_DRIVER *nx_mac_driver)
  \brief Process events deferred from elsewhere..
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static void nx_eth_driver_deferred_processing(NX_MAC_DRIVER *nx_mac_driver)
{
    if (mac_events | RX_EVENT)
    {
        mac_events &= ~RX_EVENT;
        nx_eth_driver_receive(nx_mac_driver);
    }

    if (mac_events | TX_EVENT)
    {
        mac_events &= ~TX_EVENT;
        nx_eth_driver_process_tx_complete(nx_mac_driver);
    }
}

/**
  \fn static VOID nx_eth_driver_mac_config(NX_MAC_DRIVER *nx_mac_driver, UINT link_info)
  \brief Update MAC speed/duplex configuration.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
  \param[in] link_info Speed/duplex information.
 */
static VOID nx_eth_driver_mac_config(NX_MAC_DRIVER *nx_mac_driver, UINT link_info)
{
UINT val;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;

    val = mac_read_reg(dev, MAC_CONFIG);
    val &= ~(MAC_CONFIG_FES |
            MAC_CONFIG_DM);

    switch (link_info & ETH_PHY_SPEED_MASK)
    {
    case ETH_PHY_SPEED_10:
        /*Nothing to do here as FES is already cleared*/
        nx_mac_driver -> nx_driver_link_speed = NX_DRIVER_LINK_SPEED_10;
        break;
    case ETH_PHY_SPEED_100:
        nx_mac_driver -> nx_driver_link_speed = NX_DRIVER_LINK_SPEED_100;
        val |= MAC_CONFIG_FES;
        break;
    }

    switch (link_info & ETH_PHY_DUPLEX_MASK)
    {
    case ETH_PHY_DUPLEX_FULL:
        nx_mac_driver -> nx_driver_link_duplex = NX_DRIVER_LINK_DUPLEX_FULL;
        val |= MAC_CONFIG_DM;
        break;
    case ETH_PHY_DUPLEX_HALF:
        nx_mac_driver -> nx_driver_link_duplex = NX_DRIVER_LINK_DUPLEX_HALF;
        /* Nothing to do here as DM is already cleared */
        break;
    }

    mac_write_reg(dev, MAC_CONFIG, val);
}

/**
  \fn staitc VOID nx_eth_driver_enable(NX_MAC_DRIVER *nx_mac_driver)
  \brief Enable the ethernet interface.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static VOID nx_eth_driver_enable(NX_MAC_DRIVER *nx_mac_driver)
{
UINT val;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;

    /* Enable DMA channel, MAC tx/rx and interrupts */
    val = mac_read_reg(dev, DMA_CHAN_INTR_ENA(0));
    val |= DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_NIE |
        DMA_CHAN_INTR_ENA_TIE | DMA_CHAN_INTR_ENA_AIE | DMA_CHAN_INTR_ENA_RBUE;
    mac_write_reg(dev, DMA_CHAN_INTR_ENA(0), val);

    val = mac_read_reg(dev, DMA_CHAN_TX_CONTROL(0));
    val |= DMA_CH0_TX_CONTROL_ST;
    mac_write_reg( dev, DMA_CHAN_TX_CONTROL(0), val);

    val = mac_read_reg(dev, DMA_CHAN_RX_CONTROL(0));
    val |= DMA_CH0_RX_CONTROL_SR;
    mac_write_reg(dev, DMA_CHAN_RX_CONTROL(0), val);

    val = mac_read_reg(dev, MAC_CONFIG);
    val |=  MAC_CONFIG_RE | MAC_CONFIG_TE;
    mac_write_reg(dev, MAC_CONFIG, val);
}

/**
  \fn staitc VOID nx_eth_driver_disable(NX_MAC_DRIVER *nx_mac_driver)
  \brief Disable the ethernet interface.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static VOID nx_eth_driver_disable(NX_MAC_DRIVER *nx_mac_driver)
{
UINT val;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;

    /* Disable DMA channel, MAC tx/rx and interrupts */
    val = mac_read_reg(dev, DMA_CHAN_INTR_ENA(0));
    val &= ~(DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_NIE |
        DMA_CHAN_INTR_ENA_TIE | DMA_CHAN_INTR_ENA_AIE | DMA_CHAN_INTR_ENA_RBUE);
    mac_write_reg(dev, DMA_CHAN_INTR_ENA(0), val);

    val = mac_read_reg(dev, DMA_CHAN_TX_CONTROL(0));
    val &= ~DMA_CH0_TX_CONTROL_ST;
    mac_write_reg( dev, DMA_CHAN_TX_CONTROL(0), val);

    val = mac_read_reg(dev, DMA_CHAN_RX_CONTROL(0));
    val &= ~DMA_CH0_RX_CONTROL_SR;
    mac_write_reg(dev, DMA_CHAN_RX_CONTROL(0), val);

    val = mac_read_reg(dev, MAC_CONFIG);
    val &=  ~(MAC_CONFIG_RE | MAC_CONFIG_TE);
    mac_write_reg(dev, MAC_CONFIG, val);
}

/**
  \fn static void nx_phy_poll(ULONG arg)
  \brief Thread function for polling the link status.
  \param[in] arg Unused argument.
 */
static void nx_phy_poll(ULONG arg)
{
UINT status, ret, info;
ULONG events;
MAC_DEV *dev = nx_mac0_driver.nx_driver_dev;

    /* Unused argument */
    (void) arg;

    while (1)
    {

        /* Wait for a phy poll event */
        status = tx_event_flags_get(&nx_mac0_driver.nx_driver_phy_events, TX_PHY_POLL_EVENT,
                                 TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
        if (status == TX_SUCCESS)
        {
            if (nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up == NX_FALSE)
            {
                status = nx_phy_get_link_status();

                if (status == ETH_PHY_LINK_UP)
                {
                    /* Link is up, restart AN if it is enabled in user configuration */
                    ret = nx_update_config();
                    if( ret != NX_SUCCESS)
                        continue;

                    tx_timer_deactivate(&nx_mac0_driver.nx_driver_phy_poll_timer);

                    /* Modify the polling period */
                    tx_timer_change(&nx_mac0_driver.nx_driver_phy_poll_timer,
                                            NX_DRIVER_PHY_POLLING_INTERVAL_LONG * TX_TIMER_TICKS_PER_SECOND,
                                             NX_DRIVER_PHY_POLLING_INTERVAL_LONG * TX_TIMER_TICKS_PER_SECOND);

                    tx_timer_activate(&nx_mac0_driver.nx_driver_phy_poll_timer);


                    /* Notify the stack of the link status change */
                    _nx_ip_driver_link_status_event(nx_mac0_driver.nx_driver_ip_ptr,
                                             nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_index);
                }
            }
            else
            {
                status = nx_phy_get_link_status();

                if (status == ETH_PHY_LINK_DOWN)
                {
                    nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up = NX_FALSE;

                    tx_timer_deactivate(&nx_mac0_driver.nx_driver_phy_poll_timer);

                    /* Modify the polling period */
                    tx_timer_change(&nx_mac0_driver.nx_driver_phy_poll_timer,
                                            NX_DRIVER_PHY_POLLING_INTERVAL_SHORT * TX_TIMER_TICKS_PER_SECOND,
                                             NX_DRIVER_PHY_POLLING_INTERVAL_SHORT * TX_TIMER_TICKS_PER_SECOND);

                    tx_timer_activate(&nx_mac0_driver.nx_driver_phy_poll_timer);

                    /* Notify the stack of the link status change */
                    _nx_ip_driver_link_status_event(nx_mac0_driver.nx_driver_ip_ptr,
                                             nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_index);
                }
            }
        }
    }
}

/**
  \fn static UINT configure_phy(MAC_DEV *dev)
  \brief Initialize and configure the PHY and the link.
  \param[in] dev Pointer to the \ref MAC_DEV.
  \return    The status of the operation(NX_SUCCESS or NX_NOT_SUCCESSFUL).
 */
static UINT configure_phy(MAC_DEV *dev)
{
UINT ret, status;
UINT info;
UINT mode = 0;

    ret = nx_phy_init(ETH_PHY_ADDRESS, mac_phy_read, mac_phy_write);

    if (ret != NX_SUCCESS)
        return ret;

    if (dev -> config.auto_neg_ctrl == AN_DISABLE) {
        /* AN disabled, configure the phy with user provided settings */
        if (dev -> config.speed == ETH_SPEED_10M)
            mode |= ETH_PHY_SPEED_10;
        else
            mode |= ETH_PHY_SPEED_100;

        if (dev -> config.duplex == ETH_DUPLEX_FULL)
            mode |= ETH_PHY_DUPLEX_FULL;
        else
            mode |= ETH_PHY_DUPLEX_HALF;
    }
    else
    {
        mode |= ETH_PHY_AUTONEGOTIATE;
    }


    /* check the link status */
    status = nx_phy_get_link_status();

    if (status == ETH_PHY_LINK_UP)
    {
        ret = nx_phy_setmode(mode);

        if (ret != NX_SUCCESS)
            return ret;

        info = nx_phy_get_link_info();

        /* Update the MAC configuration */
        nx_eth_driver_disable(&nx_mac0_driver);
        nx_eth_driver_mac_config(&nx_mac0_driver, info);
        nx_eth_driver_enable(&nx_mac0_driver);

        nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up = NX_TRUE;
    }
    else
    {
        nx_mac0_driver.nx_driver_interface_ptr -> nx_interface_link_up = NX_FALSE;
    }

    return NX_SUCCESS;
}

/**
  \fn static UINT nx_eth_driver_init(NX_MAC_DRIVER *nx_mac_driver)
  \brief Initialize the ethernet interface.
  \param[in] nx_mac_driver Pointer to the \ref NX_MAC_DRIVER.
 */
static UINT nx_eth_driver_init(NX_MAC_DRIVER *nx_mac_driver)
{
UINT status = NX_SUCCESS;
NX_IP *ip_ptr = nx_mac_driver -> nx_driver_ip_ptr;
NX_PACKET_POOL *pool = ip_ptr -> nx_ip_default_packet_pool;
MAC_DEV *dev = nx_mac_driver -> nx_driver_dev;

    /* Find out the pool payload size and align it to 8 bytes as needed
     * by DMA. Also as the DMA will not use 2 bytes in the beginning of the
     * buffer(for IP header alignment requirements), subtract this before
     * aligning to 8 bytes */
    dev -> rx_buf_size = ((pool -> nx_packet_pool_payload_size) - 2) & ~7U;

    status = pin_mux_configure();

    if (status)
    {
        return status;
    }

    /* Enable Ethernet peripheral clock */
    enable_eth_periph_clk();

    status = mac_hw_init(dev);

    if (status)
    {
        return status;
    }

    status  = configure_phy(dev);

    if (status)
    {
        return status;
    }

    /* Clear pending interrupt requests */
    NVIC_ClearPendingIRQ(dev -> irq);
    /* Set priority and enable interrupts */
    NVIC_SetPriority(dev -> irq, dev -> irq_priority);
    NVIC_EnableIRQ(dev -> irq);

    /* Create event flags to signal the PHY poll events */
    status = tx_event_flags_create(&(nx_mac_driver -> nx_driver_phy_events),
                                     "nx eth driver phy poll events");

    if (status != TX_SUCCESS)
    {
        goto err_event_flags;
    }

    /* Create the timer to poll the PHY status, but do not start it yet */
    status = tx_timer_create(&(nx_mac_driver -> nx_driver_phy_poll_timer), "nx eth driver phy poll timer",
                nx_timer_handler, 0, NX_DRIVER_PHY_POLLING_INTERVAL_SHORT,
                NX_DRIVER_PHY_POLLING_INTERVAL_SHORT, TX_NO_ACTIVATE);

    if (status != TX_SUCCESS)
    {
        goto err_timer_create;
    }

    /* Create a thread to do the actual polling of the link status */
    status = tx_thread_create(&(nx_mac_driver -> nx_driver_phy_poll_thread), "nx eth driver phy poll thread",
                        nx_phy_poll, 0, phy_poll_thread_stack, PHY_POLL_THREAD_STACKSIZE,
                     PHY_POLL_THREAD_PRIORITY, PHY_POLL_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        goto err_thread_create;
    }

    return NX_SUCCESS;

err_thread_create:
    tx_timer_delete(&(nx_mac_driver -> nx_driver_phy_poll_timer));
err_timer_create:
    tx_event_flags_delete(&(nx_mac_driver -> nx_driver_phy_events));
err_event_flags:
    NVIC_DisableIRQ(dev -> irq);

    return NX_NOT_SUCCESSFUL;
}

/**
  \fn VOID nx_eth_driver(NX_IP_DRIVER *driver_req_ptr)
  \brief The main entry point into the driver.
  \param[in] driver_req_ptr Pointer to the driver request structure.
 */
VOID nx_eth_driver(NX_IP_DRIVER *driver_req_ptr)
{
NX_IP        *ip_ptr;
NX_PACKET    *packet_ptr;
ULONG        *ethernet_frame_ptr;
NX_INTERFACE *interface_ptr;
UINT          interface_index;
UINT          status;

    /* Setup the IP pointer from the driver request.  */
    ip_ptr =  driver_req_ptr -> nx_ip_driver_ptr;

    /* Default to successful return.  */
    driver_req_ptr -> nx_ip_driver_status =  NX_SUCCESS;

    /* Setup interface pointer.  */
    interface_ptr = driver_req_ptr -> nx_ip_driver_interface;

    /* Obtain the index number of the network interface. */
    interface_index = interface_ptr -> nx_interface_index;

    /* Make sure we have already attached, if the command is not to attach. */
    if (driver_req_ptr -> nx_ip_driver_command != NX_LINK_INTERFACE_ATTACH)
    {
        if ((nx_mac0_driver.nx_driver_state & DRIVER_ATTACHED) == 0)
        {
            driver_req_ptr -> nx_ip_driver_status = NX_INVALID_INTERFACE;
            return;
        }
    }

    /* Process according to the driver request type in the IP control
       block.  */
    switch (driver_req_ptr -> nx_ip_driver_command)
    {

    case NX_LINK_INTERFACE_ATTACH:
    {

        if (nx_mac0_driver.nx_driver_state & DRIVER_ATTACHED)
        {
            /* the driver is already in use */
            driver_req_ptr -> nx_ip_driver_status = NX_INVALID_INTERFACE;
        }
        else
        {
            /* record the interface pointer */
            nx_mac0_driver.nx_driver_interface_ptr = interface_ptr;

            /* record the IP instance pointer */
            nx_mac0_driver.nx_driver_ip_ptr = ip_ptr;

            nx_mac0_driver.nx_driver_mac_address.nx_mac_address_msw = phys_address_msw;
            nx_mac0_driver.nx_driver_mac_address.nx_mac_address_lsw = phys_address_lsw;

            /* change the drivers state to ATTACHED */
            nx_mac0_driver.nx_driver_state |= DRIVER_ATTACHED;
        }
        break;
    }

    case NX_LINK_INTERFACE_DETACH:
    {
        /* Zero out the driver instance. */
        memset(&nx_mac0_driver, 0, sizeof(NX_MAC_DRIVER));

        break;
    }

    case NX_LINK_INITIALIZE:
    {

#ifdef NX_DEBUG
        printf("NetX Ethernet Driver Initialization - %s\n", ip_ptr -> nx_ip_name);
        printf("  IP Address =%08X\n", ip_ptr -> nx_ip_address);
#endif
        if ((nx_mac0_driver.nx_driver_state & DRIVER_ATTACHED) == 0)
        {
            /* The driver has not been attached to an IP instance yet */
            driver_req_ptr -> nx_ip_driver_status = NX_INVALID_INTERFACE;
        }
        else if ((nx_mac0_driver.nx_driver_state & DRIVER_INITIALIZED) == 0)
        {
            status = nx_eth_driver_init(&nx_mac0_driver);

            if (status != NX_SUCCESS)
            {
                driver_req_ptr -> nx_ip_driver_status =  status;
                break;
            }

            /* The nx_interface_ip_mtu_size should be the MTU for the IP payload.
                For regular Ethernet, the IP MTU is 1500. */
            nx_ip_interface_mtu_set(ip_ptr, interface_index, (NX_LINK_MTU - NX_ETHERNET_SIZE));

            nx_ip_interface_physical_address_set(ip_ptr, interface_index,
                                             nx_mac0_driver.nx_driver_mac_address.nx_mac_address_msw,
                                             nx_mac0_driver.nx_driver_mac_address.nx_mac_address_lsw, NX_FALSE);

            /* Indicate to the stack that IP to physical mapping is required.  */
            nx_ip_interface_address_mapping_configure(ip_ptr, interface_index, NX_TRUE);

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
            /* Let the stack know of our capabilities */
            interface_ptr -> nx_interface_capability_flag = NX_INTERFACE_CAPABILITY_IPV4_TX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_IPV4_RX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_TCP_TX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_TCP_RX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_UDP_TX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_UDP_RX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_ICMPV4_TX_CHECKSUM |
                                                            NX_INTERFACE_CAPABILITY_ICMPV4_RX_CHECKSUM;
#endif
            nx_mac0_driver.nx_driver_state |= DRIVER_INITIALIZED;
        }

        break;
    }

    case NX_LINK_UNINITIALIZE:
    {
        if (nx_mac0_driver.nx_driver_state & DRIVER_INITIALIZED)
        {
            tx_timer_delete(&nx_mac0_driver.nx_driver_phy_poll_timer);

            /* Disable Ethernet peripheral clock */
            disable_eth_periph_clk();

            NVIC_DisableIRQ(nx_mac0_driver.nx_driver_dev -> irq);

            /* Zero out the driver instance. Note that this will reset the driver_state fields */
            memset(&nx_mac0_driver, 0, sizeof(NX_MAC_DRIVER));
        }
        break;
    }

    case NX_LINK_ENABLE:
    {
        if ((nx_mac0_driver.nx_driver_state & DRIVER_INITIALIZED) == 0)
        {
            /* We cannot enable an uninitialized interface */
            driver_req_ptr -> nx_ip_driver_status = NX_INVALID_INTERFACE;
        }
        else if ((nx_mac0_driver.nx_driver_state & DRIVER_ENABLED) == 0)
        {
            nx_eth_driver_enable(&nx_mac0_driver);

            /* initial status of nx_interface_link_up would already be available, start the
             * timer to poll the link status frequently */
            status = tx_timer_activate(&nx_mac0_driver.nx_driver_phy_poll_timer);

            if (status != TX_SUCCESS)
            {
                driver_req_ptr -> nx_ip_driver_status = NX_NOT_SUCCESSFUL;
            }
            else
            {
                nx_mac0_driver.nx_driver_state |= DRIVER_ENABLED;
#ifdef NX_DEBUG
                printf("NetX Ethernet Driver Link Enabled - %s\n", ip_ptr -> nx_ip_name);
#endif
            }
        }
        break;
    }

    case NX_LINK_DISABLE:
    {
        if (nx_mac0_driver.nx_driver_state & DRIVER_ENABLED)
        {
            nx_eth_driver_disable(&nx_mac0_driver);

            status = tx_timer_deactivate(&nx_mac0_driver.nx_driver_phy_poll_timer);

            if (status != TX_SUCCESS)
            {
                driver_req_ptr -> nx_ip_driver_status = NX_NOT_SUCCESSFUL;
            }
            else
            {
                interface_ptr -> nx_interface_link_up =  NX_FALSE;

                nx_mac0_driver.nx_driver_state &= ~DRIVER_ENABLED;

#ifdef NX_DEBUG
                printf("NetX Ethernet Driver Link Disabled - %s\n", ip_ptr -> nx_ip_name);
#endif
            }
        }
        break;
    }

    case NX_LINK_PACKET_SEND:
    case NX_LINK_PACKET_BROADCAST:
    case NX_LINK_ARP_SEND:
    case NX_LINK_ARP_RESPONSE_SEND:
    case NX_LINK_RARP_SEND:
    {
        packet_ptr =  driver_req_ptr -> nx_ip_driver_packet;

        if (interface_ptr -> nx_interface_link_up == NX_FALSE)
        {
            /* The link is not enabled, release the packet */
            driver_req_ptr -> nx_ip_driver_status = NX_NOT_ENABLED;
            nx_packet_transmit_release(packet_ptr);
            break;
        }

        /* Place the ethernet frame at the front of the packet.  */
        /* Adjust the prepend pointer.  */
        packet_ptr -> nx_packet_prepend_ptr =  packet_ptr -> nx_packet_prepend_ptr - NX_ETHERNET_SIZE;

        /* Adjust the packet length.  */
        packet_ptr -> nx_packet_length =  packet_ptr -> nx_packet_length + NX_ETHERNET_SIZE;

        /* Setup the ethernet frame pointer to build the ethernet frame.  Backup another 2
           bytes to get 32-bit word alignment.  */
        /*lint -e{927} -e{826} suppress cast of pointer to pointer, since it is necessary  */
        ethernet_frame_ptr =  (ULONG *)(packet_ptr -> nx_packet_prepend_ptr - 2);

        /* Build the ethernet frame.  */
        *ethernet_frame_ptr     =  driver_req_ptr -> nx_ip_driver_physical_address_msw;
        *(ethernet_frame_ptr + 1) =  driver_req_ptr -> nx_ip_driver_physical_address_lsw;
        *(ethernet_frame_ptr + 2) =  (interface_ptr -> nx_interface_physical_address_msw << 16) |
            (interface_ptr -> nx_interface_physical_address_lsw >> 16);
        *(ethernet_frame_ptr + 3) =  (interface_ptr -> nx_interface_physical_address_lsw << 16);

        if (driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_SEND)
        {
            *(ethernet_frame_ptr + 3) |= NX_ETHERNET_ARP;
        }
        else if (driver_req_ptr -> nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND)
        {
            *(ethernet_frame_ptr + 3) |= NX_ETHERNET_ARP;
        }
        else if (driver_req_ptr -> nx_ip_driver_command == NX_LINK_RARP_SEND)
        {
            *(ethernet_frame_ptr + 3) |= NX_ETHERNET_RARP;
        }
        else if (packet_ptr -> nx_packet_ip_version == 4)
        {
            *(ethernet_frame_ptr + 3) |= NX_ETHERNET_IP;
        }
        else
        {
            *(ethernet_frame_ptr + 3) |= NX_ETHERNET_IPV6;
        }

        /* Endian swapping if NX_LITTLE_ENDIAN is defined. */
        NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr));
        NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 1));
        NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 2));
        NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 3));
#ifdef NX_DEBUG_PACKET
        printf("NetX Ethernet Driver Packet Send - %s\n", ip_ptr -> nx_ip_name);
#endif
        /* Send the packet */
        status  = nx_eth_driver_output(packet_ptr);

        if (status != NX_SUCCESS)
        {
            driver_req_ptr -> nx_ip_driver_status = status;

            /* Remove the Ethernet header */
            packet_ptr -> nx_packet_prepend_ptr =  packet_ptr -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

            /* Adjust the packet length.  */
            packet_ptr -> nx_packet_length =  packet_ptr -> nx_packet_length - NX_ETHERNET_SIZE;

            nx_mac0_driver.nx_driver_tx_packets_dropped++;

            /* Now that the Ethernet frame has been removed, release the packet.  */
            nx_packet_transmit_release(packet_ptr);
        }
        break;
    }

    case NX_LINK_MULTICAST_JOIN:
    {
        /* The MAC is configured to receive all multicast packets, nothing to be done */
        break;
    }

    case NX_LINK_MULTICAST_LEAVE:
    {
        /* The MAC is configured to receive all multicast packets, nothing to be done */
        break;
    }

    case NX_LINK_GET_STATUS:
    {
        /* Return the link status in the supplied return pointer.  */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) =  ip_ptr -> nx_ip_interface[0].nx_interface_link_up;
        break;
    }

    case NX_LINK_GET_SPEED:
    {
        /* Return the link's line speed in the supplied return pointer. */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = nx_mac0_driver.nx_driver_link_speed;
        break;
    }

    case NX_LINK_GET_DUPLEX_TYPE:
    {
        /* Return the link's duplex in the supplied return pointer. */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = nx_mac0_driver.nx_driver_link_duplex;
        break;
    }

    case NX_LINK_GET_ERROR_COUNT:
    {
        /* Get the total error count. Unsupported feature.  */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = 0;
        break;
    }

    case NX_LINK_GET_RX_COUNT:
    {
        /* Return the total number of packets received in the supplied pointer. */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = nx_mac0_driver.nx_driver_total_tx_packets;
        break;
    }

    case NX_LINK_GET_TX_COUNT:
    {
        /* Return the total number of packets transmitted in the supplied pointer. */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = nx_mac0_driver.nx_driver_total_tx_packets;
        break;
    }

    case NX_LINK_GET_ALLOC_ERRORS:
    {
        /* Return the allocation errors in the supplied return pointer. */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = nx_mac0_driver.nx_driver_allocation_errors;
        break;
    }

    case NX_LINK_DEFERRED_PROCESSING:
    {
        /* Call the deferred processing routine */
        nx_eth_driver_deferred_processing(&nx_mac0_driver);
        break;
    }

    case NX_LINK_SET_PHYSICAL_ADDRESS:
    {
	    /* Set the physical address.  */
        nx_mac0_driver.nx_driver_mac_address.nx_mac_address_msw = driver_req_ptr -> nx_ip_driver_physical_address_msw;
        nx_mac0_driver.nx_driver_mac_address.nx_mac_address_lsw = driver_req_ptr -> nx_ip_driver_physical_address_lsw;
        break;
    }

#ifdef NX_ENABLE_INTERFACE_CAPABILITY
    case NX_INTERFACE_CAPABILITY_GET:
    {
        /* Return the capability of the Ethernet controller in the supplied return pointer. Unsupported feature.  */
        *(driver_req_ptr -> nx_ip_driver_return_ptr) = 0;
        break;
    }

    case NX_INTERFACE_CAPABILITY_SET:
    {
        /* Set the capability of the Ethernet controller. Unsupported feature.  */
        break;
    }
#endif /* NX_ENABLE_INTERFACE_CAPABILITY  */

    default:

        /* Invalid driver request.  */

        /* Return the unhandled command status.  */
        driver_req_ptr -> nx_ip_driver_status =  NX_UNHANDLED_COMMAND;

#ifdef NX_DEBUG
        printf("NetX Ethernet Driver Received invalid request - %s\n", ip_ptr -> nx_ip_name);
#endif
        break;
    }
}
