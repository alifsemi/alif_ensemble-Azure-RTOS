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
 * @file     demo_netx_usb_cdc_ecm.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     11-July-2024
 * @brief    USB device CDC-ECM.
 * @Note     None.
 *******************************************************************************/

#include "nx_api.h"
#ifdef DEMO_USES_DHCP
#include "nxd_dhcp_server.h"
#endif
#include "ux_dcd.h"
#include "ux_device_class_cdc_ecm.h"

#include "se_services_port.h"
#include "app_utils.h"

#if defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_stdout.h"
#include "retarget_init.h"
#endif /* RTE_Compiler_IO_STDOUT */

#define ONE_KB             1024
#define UX_DEMO_NS_SIZE    (32 * ONE_KB)
#define UX_DEMO_STACK_SIZE (1 * ONE_KB)

#define CFG_PACKET_SZ      (1514)
#define CFG_PACKETS        (25)
#define CFG_IP_SZ          (2048)
#define CFG_ARP_SZ         (512)
#define CFG_AUTOIP_SZ      (512)
#define CFG_DHCP_SZ        (NX_DHCP_SERVER_THREAD_STACK_SIZE)

#define NX_IP_PRIORITY                    3
#define UX_DEVICE_DEFAULT_CFG_NUM         1
#define UX_DEVICE_DEFAULT_INTERFACE_NUM   0

#define PACKET_POOL_SIZE   ((CFG_PACKET_SZ + sizeof(NX_PACKET)) * CFG_PACKETS)

#define CFG_HOST_ADDR      (IP_ADDRESS(192, 168, 11, 1))
#define CFG_HOST_MASK      (IP_ADDRESS(255, 255, 255, 0))

#ifdef DEMO_USE_IPERF
/* Stack areas for http and iperf */
#define HTTP_STACK_SIZE    2048
#define IPERF_STACK_SIZE   2048
#endif

static NX_PACKET_POOL pool;
static NX_IP          ip;

#ifdef DEMO_USES_DHCP
static NX_DHCP_SERVER dhcps;
static UCHAR          dhcp_stack[CFG_DHCP_SZ] ATTR_ALIGN(8);
#endif

static VOID ux_device_cdc_ecm_activate(VOID *cdc_ecm_instance);
static VOID ux_device_cdc_ecm_deactivate(VOID *cdc_ecm_instance);
static VOID error_handler(uint32_t status, uint32_t lineNumber);

#ifdef DEMO_USE_IPERF
static UCHAR http_stack[HTTP_STACK_SIZE];
static UCHAR iperf_stack[IPERF_STACK_SIZE];
#endif

static uint8_t                          dma_buf[UX_DEMO_NS_SIZE] ATTR_SECTION("usb_dma_buf");
static uint8_t                          ip_stack[CFG_IP_SZ] ATTR_ALIGN(8);
static uint8_t                          arp_cache[CFG_ARP_SZ];
static uint8_t                          net_mem[PACKET_POOL_SIZE];
static UX_SLAVE_CLASS_CDC_ECM_PARAMETER cdc_ecm_parameter;

static uint8_t local_mac[6]                  = {0x00, 0x1e, 0x58, 0x41, 0xB8, 0x78};
static uint8_t remote_mac[6]                 = {0x00, 0x1e, 0x58, 0x41, 0xB8, 0x79};

static uint8_t device_framework_full_speed[] = {

    /* Device descriptor 18 bytes               */
    0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02, 0x03, 0x01,

    /* Configuration  descriptor 9 bytes        */
    0x09, 0x02, 0x4f, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor         */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x06, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x00, 0x00,

    /* Header Functional Descriptor 5 bytes     */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* Union Functional Descriptor 5 bytes      */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Ethernet Networking Functional Descriptor 13 bytes                   */
    0x0D, 0x24, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0xEA, 0x05, 0x00, 0x00, 0x00,

    /* High Speed Notification endpoint(interrupt) descriptor 7 bytes            */
    0x07, 0x05, 0x83, 0x03, 0x10, 0x00, 0x8,

    /* Data Class Interface Descriptor Alternate Setting 0, 0 endpoints. 9 bytes */
    0x09, 0x04, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,

    /* Data Class Interface Descriptor Alternate Setting 1, 1 endpoint. 9 bytes  */
    0x09, 0x04, 0x01, 0x01, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* BULK IN Endpoint descriptor 7 bytes      */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,

    /* BULK OUT Endpoint descriptor 7 bytes     */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00

};
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED sizeof(device_framework_full_speed)

static uint8_t device_framework_high_speed[] = {

    /* Device descriptor 18 bytes               */
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x25, 0x05, 0xa7, 0xa4,
    0x00, 0x01, 0x01, 0x02, 0x03, 0x01,

    /* device qualifier descriptor              */
    0x0a, 0x06, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x01, 0x00,

    /* Configuration  descriptor 9 bytes        */
    0x09, 0x02, 0x58, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x01,

    /* Interface association descriptor         */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x06, 0x00, 0x00,

    /* Communication Class Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x06, 0x00, 0x00,

    /* Header Functional Descriptor 5 bytes     */
    0x05, 0x24, 0x00, 0x10, 0x01,

    /* Union Functional Descriptor 5 bytes      */
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Call Ethernet Networking Functional Descriptor 13 bytes         */
    0x0D, 0x24, 0x0F, 0x04, 0x00, 0x00, 0x00, 0x00, 0xEA, 0x05, 0x00, 0x00,  0x00,

    /* High Speed Notification endpoint(interrupt) descriptor 7 bytes  */
    0x07, 0x05, 0x83, 0x03, 0x10, 0x00, 0x8,

    /* Data Class Interface Descriptor Alternate Setting 0,  9 bytes   */
    0x09, 0x04, 0x01, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,

    /* Data Class Interface Descriptor Alternate Setting 1, 9 bytes    */
    0x09, 0x04, 0x01, 0x01, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* BULK IN Endpoint descriptor 7 bytes      */
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00,

    /* BULK OUT Endpoint descriptor 7 bytes     */
    0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00
};
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED         sizeof(device_framework_high_speed)

/* String Device Framework :
 * Byte 0 and 1 : containing the language ID : 0x0904 for US
 * Byte 2       : containing the index of the descriptor
 * Byte 3       : containing the length of the descriptor string
 */

static uint8_t string_framework[] = {

    0x09, 0x04, /* language id  */
    0x01,       /* Index */
    0x11,       /*  bLength */
    'A',  'l',  'i', 'f', 'S', 'e', 'm', 'i', 'c', 'o', 'n', 'd', 'u', 'c', 't', 'o', 'r',

    0x09, 0x04, /* language id  */
    0x02,       /* Index */
    0x06,       /* bLength */
    'D',  'e',  'v', 'k', 'i', 't',

    0x09, 0x04, /* language id  */
    0x03,       /* Index   */
    0x04,       /* bLength */
    '1',  '2',  '0', '0',

    0x09, 0x04, /* language id  */
    0x04,       /* Index */
    0x0C,       /* bLength */
    '0',  '0',  '1', 'e', '5', '8', '4', '1', 'b', '8', '7', '9'};
#define STRING_FRAMEWORK_LENGTH sizeof(string_framework)

/* Multiple languages are supported on the device, to add
 * a language besides english, the unicode language code must
 * be appended to the language_id_framework array and the length
 * adjusted accordingly.
 */

static uint8_t language_id_framework[] = {

    /* English. */
    0x09,
    0x04
};
#define LANGUAGE_ID_FRAMEWORK_LENGTH sizeof(language_id_framework)

int main(void)
{
    uint32_t      error_code;
    uint32_t      service_error_code;
    run_profile_t runp;

    /* Initialize the SE services */
    se_services_port_init();
    /* Enable the USB clock */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_20M, /* clock_enable_t */
                                              true,          /* bool enable    */
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    runp.phy_pwr_gating |= USB_PHY_MASK;
    runp.memory_blocks   = SRAM0_MASK | MRAM_MASK;

    /* Set the current run configuration to SE */
    error_code           = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    int32_t ret;

    ret = stdout_init();
    if (ret != ARM_DRIVER_OK) {
        WAIT_FOREVER_LOOP
    }
#endif

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();

    return 0;
}

/* Define what the initial system looks like.  */
void tx_application_define(void *first_unused_memory)
{
    UX_PARAMETER_NOT_USED(first_unused_memory);
    uint32_t status;

    printf("Started USBx cdc-ecm app\n");
    /* Initialize USBX Memory */
    status = ux_system_initialize(dma_buf, UX_DEMO_NS_SIZE, UX_NULL, 0x00);

    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Initialize the usbx device stack  */
    status = ux_device_stack_initialize(device_framework_high_speed,
                                        DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                        device_framework_full_speed,
                                        DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                        string_framework,
                                        STRING_FRAMEWORK_LENGTH,
                                        language_id_framework,
                                        LANGUAGE_ID_FRAMEWORK_LENGTH,
                                        UX_NULL);

    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Setting cdc acm activation and deactivation functionality */
    cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_activate   = ux_device_cdc_ecm_activate;
    cdc_ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate = ux_device_cdc_ecm_deactivate;
    /* Copy the local mac ID. */
    _ux_utility_memory_copy(cdc_ecm_parameter.ux_slave_class_cdc_ecm_parameter_local_node_id,
                            local_mac,
                            sizeof(local_mac));
    /* Copy the remote node id */
    _ux_utility_memory_copy(cdc_ecm_parameter.ux_slave_class_cdc_ecm_parameter_remote_node_id,
                            remote_mac,
                            sizeof(remote_mac));

    /* Register the class driver as CDC-ECM  */
    status = ux_device_stack_class_register(_ux_system_slave_class_cdc_ecm_name,
                                            ux_device_class_cdc_ecm_entry,
                                            UX_DEVICE_DEFAULT_CFG_NUM,
                                            UX_DEVICE_DEFAULT_INTERFACE_NUM,
                                            (VOID *) &cdc_ecm_parameter);
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Initialize the NetX system.  */
    nx_system_initialize();
    /* USBX network driver  */
    status = ux_network_driver_init();
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* USB dwc3 driver Initialization  */
    status = ux_dcd_initialize();
    if (status != UX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Create a packet pool.  */
    status = nx_packet_pool_create(&pool,
                                   "CDC-ECM packet pool",
                                   CFG_PACKET_SZ,
                                   net_mem,
                                   sizeof(net_mem));
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Create an IP instance with for CDC-ECM  */
    status = nx_ip_create(&ip,
                          "CDC-ECM IP instance",
                          CFG_HOST_ADDR,
                          CFG_HOST_MASK,
                          &pool,
                          _ux_network_driver_entry,
                          ip_stack,
                          CFG_IP_SZ,
                          NX_IP_PRIORITY);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Enable IP fragmentation on the IP instance. */
    status = nx_ip_fragment_enable(&ip);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Enable ARP and supply ARP cache for the IP Instance */
    status = nx_arp_enable(&ip, arp_cache, CFG_ARP_SZ);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Enable TCP processing */
    status = nx_tcp_enable(&ip);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Enable UDP traffic */
    status = nx_udp_enable(&ip);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* Enable ICMP traffic */
    status = nx_icmp_enable(&ip);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
#ifdef DEMO_USES_DHCP
    /* Create DHCP server  */
    status = nx_dhcp_server_create(&dhcps, &ip, dhcp_stack, CFG_DHCP_SZ, "CDC-ECM DHCPS", &pool);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    uint32_t added = 0;
    /*  DHCP Server IP address list create   */
    status         = nx_dhcp_create_server_ip_address_list(&dhcps,
                                                           0,
                                                           CFG_HOST_ADDR + 1,
                                                           CFG_HOST_ADDR + 10,
                                                           &added);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* set interface network parameters  */
    status = nx_dhcp_set_interface_network_parameters(&dhcps,
                                                      0,
                                                      CFG_HOST_MASK,
                                                      CFG_HOST_ADDR,
                                                      CFG_HOST_ADDR);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
    /* DHCP server start      */
    status = nx_dhcp_server_start(&dhcps);
    if (status != NX_SUCCESS) {
        error_handler(status, __LINE__);
    }
#endif
#ifdef DEMO_USE_IPERF
    /* Call entry function to start iperf test.  */
    nx_iperf_entry(&pool, &ip, http_stack, HTTP_STACK_SIZE, iperf_stack, IPERF_STACK_SIZE);
#endif
}
/* CDC-ECM Activation callback  */
static VOID ux_device_cdc_ecm_activate(VOID *cdc_ecm_instance)
{
    UX_PARAMETER_NOT_USED(cdc_ecm_instance);
}
/* CDC-ECM Deactivation callback  */
static VOID ux_device_cdc_ecm_deactivate(VOID *cdc_ecm_instance)
{
    UX_PARAMETER_NOT_USED(cdc_ecm_instance);
}

static VOID error_handler(uint32_t status, uint32_t lineNumber)
{
    uint32_t      error_code;
    uint32_t      service_error_code;
    run_profile_t runp;

    printf("In ERROR handler error 0x%X occurred at %d\n", status, lineNumber);
    /* Disable the USB clock  */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle,
                                              CLKEN_CLK_20M,  /* clock_enable_t */
                                              false,          /* bool disable   */
                                              &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    runp.phy_pwr_gating &= ~USB_PHY_MASK;

    /* Set the current run configuration to SE */
    error_code           = SERVICES_set_run_cfg(se_services_s_handle, &runp, &service_error_code);
    if (error_code) {
        WAIT_FOREVER_LOOP
    }
    WAIT_FOREVER_LOOP
}
