/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     USB_initialize.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This file  initializes and configures the USB controller in device mode
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "usbd_spec.h"

/**
 *\fn           usbd_initialize_eps
 *\brief        This function initializes the physical endpoints.
 *\param[in]    drv  pointer to the USB driver context structure
 *\return       none
 **/

void usbd_initialize_eps(USB_DRIVER *drv)
{
    uint8_t i;
    uint8_t phy_ep;

    for (i = 0U; i < drv->out_eps; i++) {
        phy_ep                             = (i << 1U) | USB_DIR_OUT;
        drv->eps[phy_ep].phy_ep            = phy_ep;
        drv->eps[phy_ep].ep_dir            = USB_DIR_OUT;
        drv->eps[phy_ep].ep_resource_index = 0U;
    }
    for (i = 0U; i < drv->in_eps; i++) {
        phy_ep                             = (i << 1U) | USB_DIR_IN;
        drv->eps[phy_ep].phy_ep            = phy_ep;
        drv->eps[phy_ep].ep_dir            = USB_DIR_IN;
        drv->eps[phy_ep].ep_resource_index = 0U;
    }
}

/**
 *\fn           usbd_set_speed
 *\brief        This function sets the speed of the USB controller.
 *\param[in]    drv  pointer to the USB driver context structure
 *\return       none
 **/

void usbd_set_speed(USB_DRIVER *drv)
{
    uint32_t reg;

    reg              = drv->regs->DCFG;
    reg             &= ~(USB_DCFG_SPEED_MASK);
    reg             |= USB_DCFG_HIGHSPEED;
    drv->regs->DCFG  = reg;
}

/**
 *\fn           usbd_verify_ip_core
 *\brief        This function reads the controller core id and check if it's valid core or not.
 *\param[in]    drv  pointer to the USB driver context structure
 *\return       On success returns true, else false
 **/

bool usbd_verify_ip_core(USB_DRIVER *drv)
{
    uint32_t reg;

    reg = drv->regs->GSNPSID;
    /* Read the USB core ID and followed by revision number */
    if ((reg & USB_GSNPSID_MASK) == 0x55330000) {
        /* Detected USB controller IP */
        drv->revision = reg;
    } else {
#ifdef DEBUG
        printf("Detected wrong IP\n");
#endif
        return false;
    }

    return true;
}

/**
 *\fn           usbd_get_eps
 *\brief        This function obtains the number of IN and OUT endpoints.
 *\param[in]    drv  pointer to controller context structure
 *\return       none
 **/

void usbd_get_eps(USB_DRIVER *drv)
{
    uint32_t reg;

    reg          = drv->regs->GHWPARAMS3;
    drv->in_eps  = USB_IN_EPS(reg);
    drv->out_eps = (USB_NUM_EPS(reg) - drv->in_eps);
}

/**
 *\fn           usbd_configure_event_buffer_registers
 *\brief        This function configures the event buffer registers
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/

void usbd_configure_event_buffer_registers(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf              = drv->event_buf;
    event_buf->lpos        = 0;
    drv->regs->GEVNTADRLO0 = LocalToGlobal((void *) LOWER_32_BITS(event_buf->buf));
    drv->regs->GEVNTADRHI0 = 0;
    drv->regs->GEVNTSIZ0   = event_buf->length;
    drv->regs->GEVNTCOUNT0 = 0;
}

/**
 *\fn           usbd_device_soft_reset
 *\brief        This function performs a soft reset of the USB device controller
 *\param[in]    drv  pointer to the controller context structure
 *\return       On success 0, else error
 **/
int32_t usbd_device_soft_reset(USB_DRIVER *drv)
{
    uint32_t reg;
    int32_t  timeout;

    /* If Controller is in host mode, return success */
    if (drv->dr_role == USB_GCTL_PRTCAP_HOST) {
        return USB_SUCCESS;
    }
    reg = drv->regs->DCTL;
    SET_BIT(reg, USB_DCTL_CSFTRST);
    drv->regs->DCTL = reg;
    timeout         = USB_DCTL_CSFTRST_TIMEOUT;
    /* Wait for Soft Reset to be completed.  */
    do {
        reg = drv->regs->DCTL;
        if (!(reg & USB_DCTL_CSFTRST)) {
            return USB_SUCCESS;
        }
    } while (--timeout);

    if (timeout == 0) {
        return USB_CORE_SFTRST_TIMEOUT_ERROR;
    }

    return USB_SUCCESS;
}

/**
 *\fn           usbd_set_default_config
 *\brief        This function sets the default USB configuration
 *\param[in]    drv pointer to the controller context structure
 *\return       none
 **/

void usbd_set_default_config(USB_DRIVER *drv)
{
#ifdef USB_HOST
    drv->dr_mode = USB_DR_MODE_HOST;
    drv->dr_role = USB_GCTL_PRTCAP_HOST;
#else
    drv->dr_mode = USB_DR_MODE_PERIPHERAL;
#endif
    /* default setting the phy mode */
    drv->hsphy_mode = USB_PHY_INTERFACE_MODE_UTMIW;
    /* default value for frame length  */
    drv->fladj      = USB_GFLADJ_DEFAULT_VALUE;
}
/**
 *\fn           usbd_read_hw_params
 *\brief        This function reads the controller hardware parameters
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/

void usbd_read_hw_params(USB_DRIVER *drv)
{
    USB_HWPARAMS *parms = &drv->hwparams;

    parms->hwparams0    = drv->regs->GHWPARAMS0;
    parms->hwparams1    = drv->regs->GHWPARAMS1;
    parms->hwparams2    = drv->regs->GHWPARAMS2;
    parms->hwparams3    = drv->regs->GHWPARAMS3;
    parms->hwparams4    = drv->regs->GHWPARAMS4;
    parms->hwparams5    = drv->regs->GHWPARAMS5;
    parms->hwparams6    = drv->regs->GHWPARAMS6;
    parms->hwparams7    = drv->regs->GHWPARAMS7;
    parms->hwparams8    = drv->regs->GHWPARAMS8;
}

/**
 *\fn           usbd_validate_mode
 *\brief        This function validates the controller mode
 *\param[in]    drv  pointer to the controller context structure
 *\return       On success 0, else error
 **/
int32_t usbd_validate_mode(USB_DRIVER *drv)
{
    USB_DR_MODE mode;
    uint32_t    hw_mode;

    mode    = drv->dr_mode;
    hw_mode = USB_GHWPARAMS0_MODE(drv->hwparams.hwparams0);
    switch (hw_mode) {

    case USB_GHWPARAMS0_MODE_DEVICE:
        mode = USB_DR_MODE_PERIPHERAL;
        break;
    case USB_GHWPARAMS0_MODE_HOST:
        mode = USB_DR_MODE_HOST;
        break;
    default:
#ifdef USB_HOST
        mode = USB_DR_MODE_HOST;
#else
        mode = USB_DR_MODE_PERIPHERAL;
#endif
    }
    if (mode != drv->dr_mode) {
#ifdef DEBUG
        printf("mode mismatched\n");
#endif
        drv->dr_mode = mode;
        return USB_MODE_MISMATCH;
    }

    return USB_SUCCESS;
}

/**
 *\fn           usbd_configure_usb2_phy
 *\brief        This function configures the USB 2.0 PHY settings
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/
void usbd_configure_usb2_phy(USB_DRIVER *drv)
{
    uint32_t reg;

    /*Configure Global USB2 PHY Configuration Register */
    reg = drv->regs->GUSB2PHYCFG0;

    /* The PHY must not be enabled for auto-resume in device mode.
     * Therefore, the field GUSB2PHYCFG[15] (ULPIAutoRes) must be written with '0'
     *  during the power-on initialization in case the reset value is '1'.
     */
    if (reg & USB_GUSB2PHYCFG_ULPIAUTORES) {
        reg &= ~USB_GUSB2PHYCFG_ULPIAUTORES;
    }

    /* Select the HS PHY interface */
    switch (USB_GHWPARAMS3_HSPHY_IFC(drv->hwparams.hwparams3)) {
    case USB_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI:
        reg &= ~USB_GUSB2PHYCFG_ULPI_UTMI;
        break;
    case USB_GHWPARAMS3_HSPHY_IFC_ULPI:
        /* FALLTHROUGH */
    default:
        break;
    }
    /* Enable PHYIF  */
    switch (drv->hsphy_mode) {
    case USB_PHY_INTERFACE_MODE_UTMI:
        reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK);
        reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
               USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
        break;
    case USB_PHY_INTERFACE_MODE_UTMIW:
        reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK |
                 USB_GUSB2PHYCFG_ULPI_UTMI);
        reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
               USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT) | USB_GUSB2PHYCFG_SUSPHY;
        break;
    default:
        break;
    }
    drv->regs->GUSB2PHYCFG0 = reg;
}
/**
 *\fn           usbd_configure_global_control_reg
 *\brief        This function configures the global control register of the USB controller
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/
void usbd_configure_global_control_reg(USB_DRIVER *drv)
{
    uint32_t reg;

    reg = drv->regs->GCTL;
    SET_BIT(reg, USB_GCTL_DSBLCLKGTNG);
    drv->regs->GCTL = reg;

    if (drv->dr_mode == USB_DR_MODE_HOST) {
        reg = drv->regs->GUCTL;
        SET_BIT(reg, USB_GUCTL_HSTINAUTORETRY);
        drv->regs->GUCTL = reg;
    }
}
/**
 *\fn           usbd_configure_burst_transfer
 *\brief        This function configures the burst transfer settings for the USB controller
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/
void usbd_configure_burst_transfer(USB_DRIVER *drv)
{
    uint32_t reg;

    /* Configure burst transfer settings in the global bus configuration register */
    reg = drv->regs->GSBUSCFG0;
    SET_BIT(reg, USB_GSBUSCFG0_INCRBRSTENA | USB_GSBUSCFG0_INCR32BRSTENA);
    drv->regs->GSBUSCFG0 = reg;
}
/**
 *\fn           usbd_configure_fladj_register
 *\brief        This function configures the frame length adjustment (FLADJ) register
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 **/
void usbd_configure_fladj_register(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t frame_legth;

    reg         = drv->regs->GFLADJ;
    frame_legth = reg & USB_GFLADJ_30MHZ_MASK;
    if (frame_legth != drv->fladj) {
        reg               &= ~USB_GFLADJ_30MHZ_MASK;
        reg               |= USB_GFLADJ_30MHZ_SDBND_SEL | drv->fladj;
        drv->regs->GFLADJ  = reg;
    }
}
/**
 *\fn           usbd_set_port_capability
 *\brief        This function sets the USB controller's port capability (device/host).
 *\param[in]    drv   pointer to the controller context structure
 *\param[in]    mode  USB port capability mode (DEVICE/HOST)
 *\return       none
 **/
void usbd_set_port_capability(USB_DRIVER *drv, uint32_t mode)
{
    uint32_t reg;

    reg              = drv->regs->GCTL;
    reg             &= ~(USB_GCTL_PRTCAPDIR(USB_GCTL_PRTCAP_OTG));
    /* Set the mode   */
    reg             |= USB_GCTL_PRTCAPDIR(mode);
    drv->regs->GCTL  = reg;
    drv->dr_role     = mode;
}

/**
 *\fn           usbd_cleanup_event_buffer
 *\brief        This function clear the event buffer register.
 *\param[in]    drv pointer to the controller context structure
 *\return       none
 */
void usbd_cleanup_event_buffer(USB_DRIVER *drv)
{
    USBD_EVENT_BUFFER *event_buf;

    event_buf              = drv->event_buf;
    event_buf->lpos        = 0;
    drv->regs->GEVNTADRLO0 = 0;
    drv->regs->GEVNTADRHI0 = 0;
    drv->regs->GEVNTSIZ0   = USB_GEVNTSIZ_INTMASK | USB_GEVNTSIZ_SIZE(0);
    drv->regs->GEVNTCOUNT0 = 0;
}

/**
 *\fn           usbd_reset_phy_and_core
 *\brief        This function resets both the PHY and core of the USB controller
 *\param[in]    drv  pointer to the controller context structure
 *\return       none
 */
void usbd_reset_phy_and_core(USB_DRIVER *drv)
{
    uint32_t reg;

    /* Before Resetting PHY, put Core in Reset */
    reg = drv->regs->GCTL;
    SET_BIT(reg, USB_GCTL_CORESOFTRESET);
    drv->regs->GCTL = reg;

    /* Assert USB2 PHY reset */
    reg             = drv->regs->GUSB2PHYCFG0;
    SET_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
    drv->regs->GUSB2PHYCFG0 = reg;
    sys_busy_loop_us(50000);

    /* Clear USB2 PHY reset */
    reg = drv->regs->GUSB2PHYCFG0;
    CLEAR_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
    drv->regs->GUSB2PHYCFG0 = reg;
    sys_busy_loop_us(50000);

    /* Take Core out of reset state after PHYS are stable*/
    reg = drv->regs->GCTL;
    CLEAR_BIT(reg, USB_GCTL_CORESOFTRESET);
    drv->regs->GCTL = reg;
}

/**
 *\fn           usbd_configure_initial_ep_commands
 *\brief        This function configures the initial endpoint commands.
 *\param[in]    drv pointer to the controller context structure
 *\return       none
 */

void usbd_configure_initial_ep_commands(USB_DRIVER *drv)
{
    uint32_t reg;
    /* Check controller USB2 phy config before issuing DEPCMD always */
    usbd_usb2phy_config_check(drv);

    /* Issuing DEPSTARTCFG Command to ep_resource[0] (for EP 0/CONTROL/) */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD = USB_DEPCMD_DEPSTARTCFG | USB_DEPCMD_CMDACT;
    do {
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP0 OUT */
    usbd_usb2phy_config_check(drv);

    /* Issuing DEPCFG Command to ep_resource[0] (for EP 0 OUT/CONTROL/)
     *bit[8] = XferComplete, bit[10] = XferNotReady *
     *bit[25]: Endpoint direction:
     *  0: OUT
     *  1: IN
     * bit[29:26] = Endpoint number
     * Physical endpoint 0 (EP0) must be allocated for control endpoint 0 OUT.
     * Physical endpoint 1 (EP1) must be allocated for control endpoint 0 IN.
     */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR1 = USB_DEPCMD_EP0_OUT_PAR1;
    /* bit[25:22] = Burst size and Burst size =0 for Control endpoint
     * bit[21:17] = FIFONum, and FIFONum = 0;
     * bit[13:3] =  Maximum Packet Size (MPS)
     * bit[2:1] = Endpoint Type (EPType)
     *  00: Control
     *  01: Isochronous
     *  10: Bulk
     *  11: Interrupt
     */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP0_OUT_PAR0;
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD     = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
    do {
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP1 IN */
    usbd_usb2phy_config_check(drv);
    /* Issuing DEPCFG Command to ep_resource[1] (for EP 1 IN/CONTROL/) */
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR1 = USB_DEPCMD_EP1_IN_PAR1;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP1_IN_PAR0;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMD     = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[1].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP0 OUT */
    usbd_usb2phy_config_check(drv);
    /* Issuing Transfer Resource command for each initialized endpoint */
    drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
    drv->regs->USB_ENDPNT_CMD[0].DEPCMD     = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[0].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

    /* Check usb2phy config before issuing DEPCMD for EP1 IN */
    usbd_usb2phy_config_check(drv);
    drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
    drv->regs->USB_ENDPNT_CMD[1].DEPCMD     = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
    do {
        /* Read the device endpoint Command register.  */
        reg = drv->regs->USB_ENDPNT_CMD[1].DEPCMD;
    } while ((reg & USB_DEPCMD_CMDACT) != 0);
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);

}

/**
 *\fn           usbd_enable_device_events
 *\brief        This function enables the device specific events.
 *\param[in]    drv pointer to the controller context structure
 *\return       none
 */

void usbd_enable_device_events(USB_DRIVER *drv)
{
    uint32_t reg;

    /* enable USB Reset, Connection Done, and USB/Link State Change events */
    reg = drv->regs->DEVTEN;
    SET_BIT(reg, USB_DEV_DISSCONNEVTEN);
    SET_BIT(reg, USB_DEV_USBRSTEVTEN);
    SET_BIT(reg, USB_DEV_CONNECTDONEEVTEN);
    drv->regs->DEVTEN = reg;
}

/**
 *\fn           usbd_start
 *\brief        This function will start the USB controller and enables the interrupts.
 *\param[in]    drv pointer to the controller context structure
 *\return       success
 */

uint32_t usbd_start(USB_DRIVER *drv)
{
    uint32_t reg;
    int32_t  retries;

    /* Starting controller . */
    reg = drv->regs->DCTL;
    SET_BIT(reg, USB_DCTL_START);
    drv->regs->DCTL = reg;
    retries         = USB_DCTL_START_TIMEOUT;
    do {
        reg = drv->regs->DSTS;
        if (!(reg & USB_DSTS_DEVCTRLHLT)) {
            break;
        }
    } while (--retries);
    if (retries == 0) {
        return USB_CONTROLLER_INIT_FAILED;
    }
    /* Enable the interrupts */
    NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
    NVIC_EnableIRQ(USB_IRQ_IRQn);

    return USB_SUCCESS;
}
