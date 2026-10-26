/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     demo_mipi_csi2_camera_threadx.c
 * @author   Prasanna Ravi and Chandra Bhushan Singh
 * @email    prasanna.ravi@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     04-May-2023
 * @brief    TestApp to verify a MIPI CSI2 Camera Sensor
 *            (ARX3A0, MT9M114, or OV5675) with
 *            Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

//* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Cpi Driver */
#include "RTE_Components.h"
#include "RTE_Device.h"
#include "Driver_CPI.h"
#if defined(RTE_Compiler_IO_STDOUT)
#include "retarget_stdout.h"
#endif  /* RTE_Compiler_IO_STDOUT */

/* PINMUX Driver */
#include "pinconf.h"

/* SE Services */
#include "se_services_port.h"

/* Camera  Driver instance 0 */
extern ARM_DRIVER_CPI Driver_CPI;
static ARM_DRIVER_CPI *CAMERAdrv = &Driver_CPI;

void camera_demo_thread_entry(ULONG thread_input);

/* Define the ThreadX object control blocks...  */
#define DEMO_STACK_SIZE                            1024
#define DEMO_BYTE_POOL_SIZE                        9120

TX_THREAD                                          camera_thread;
TX_BYTE_POOL                                       byte_pool_0;
UCHAR                                              memory_area[DEMO_BYTE_POOL_SIZE];
TX_EVENT_FLAGS_GROUP                               camera_event_flags;

/* Camera Sensor Selection
 * Supports: ARX3A0, MT9M114, OV5675
 *
 * Selection is driven by the RTE components enabled in RTE_Components.h.
 * Enable the corresponding CAMERA_SENSOR driver in the RTE configuration.
 *
 *   - RTE_Drivers_CAMERA_SENSOR_ARX3A0  (560x560,  IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_MT9M114 (1280x720, IPI-16 RAW8)
 *   - RTE_Drivers_CAMERA_SENSOR_OV5675  (1296x972, IPI-16 RAW8)
 *
 * Note: Sensors output RAW10, but CPI interface default is IPI-16 RAW8.
 *       CPI color mode can be changed via RTE configuration if needed.
 */

#if defined(RTE_Drivers_CAMERA_SENSOR_ARX3A0)
  #define SELECTED_CAMERA_SENSOR     "ARX3A0"
  #define FRAME_WIDTH                RTE_ARX3A0_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_ARX3A0_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#elif defined(RTE_Drivers_CAMERA_SENSOR_MT9M114)
  #define SELECTED_CAMERA_SENSOR     "MT9M114"
  #define FRAME_WIDTH                 RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_WIDTH
  #define FRAME_HEIGHT                RTE_MT9M114_CAMERA_SENSOR_MIPI_FRAME_HEIGHT

  /* MT9M114 image configuration 0 and 1 uses IPI-16 RAW8 format.
   * RAW8 requires 1 byte per pixel for the framebuffer.
   */
  #if (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 0 ||  \
       RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG == 1)
     #define BYTES_PER_PIXEL 1
  #elif (RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG >= 2 &&  \
        RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG <= 5)
     #define BYTES_PER_PIXEL 2
  #else
     #error "Unsupported RTE_MT9M114_CAMERA_SENSOR_MIPI_IMAGE_CONFIG"
  #endif

#elif defined(RTE_Drivers_CAMERA_SENSOR_OV5675)
  #define SELECTED_CAMERA_SENSOR     "OV5675"
  #define FRAME_WIDTH                RTE_OV5675_CAMERA_SENSOR_FRAME_WIDTH
  #define FRAME_HEIGHT               RTE_OV5675_CAMERA_SENSOR_FRAME_HEIGHT
  #define BYTES_PER_PIXEL 1

#else
  #error  "Enable one RTE_Drivers_CAMERA_SENSOR_* definition"
#endif

/* Camera Sensor configurations
 * Resolution and IPI format based on selected sensor:
 *   - ARX3A0  : 560x560,  IPI-16 RAW8
 *   - MT9M114 : 1280x720, IPI-16 RAW8
 *   - OV5675  : 1296x972, IPI-16 RAW8
 */

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height x bytes per pixel
 */
#define FRAMEBUFFER_POOL_SIZE   \
    ((FRAME_WIDTH) * (FRAME_HEIGHT) * (BYTES_PER_PIXEL))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE] \
        __attribute__((section(".bss.camera_frame_buf")));

/* Captured data is RAW Bayer. Dump framebuffer_pool with a debugger
 * and convert offline if a color image is required.
 */

/* Camera callback events */
typedef enum {
    CAM_CB_EVENT_CAPTURE_STOPPED      = (1 << 0),
    CAM_CB_EVENT_ERROR                = (1 << 1)
}CAMERA_CB_EVENTS;

/**
  \fn          void camera_callback(uint32_t event)
  \brief       Camera isr callback
  \param[in]   event: Camera Event
  \return      none
  */
void camera_callback(uint32_t event)
{
    if(event & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED)
    {
        /* Transfer Success: Capture Stop detected, Wake-up Thread. */
        tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_CAPTURE_STOPPED, TX_OR);
    }

    if(event & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN)
    {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if(event & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN)
    {
        /* Transfer Error: Received FIFO over-run, Wake-up Thread. */
        tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }

    if(event & ARM_CPI_EVENT_ERR_HARDWARE)
    {
        /* Transfer Error: Received Hardware error, Wake-up Thread. */
        tx_event_flags_set(&camera_event_flags, CAM_CB_EVENT_ERROR, TX_OR);
    }
}

/**
  \fn          int i3c_pinmux(void)
  \brief       i3c hardware pin initialization:
  - PIN-MUX configuration
  - PIN-PAD configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int i3c_pinmux(void)
{
    int ret;

    /* Configure GPIO Pin : P7_2 as i2c_sda_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_7, PIN_2, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE |
                                                                  PADCTRL_DRIVER_DISABLED_PULL_UP);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    /* Configure GPIO Pin : P7_3 as i2c_scl_c
     * Pad function: PADCTRL_READ_ENABLE |
     *               PADCTRL_DRIVER_DISABLED_PULL_UP
     */
    ret = pinconf_set(PORT_7, PIN_3, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_READ_ENABLE |
                                                                  PADCTRL_DRIVER_DISABLED_PULL_UP);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: i3c PINMUX and PINPAD failed.\r\n");
        return -1;
    }

    return 0;
}

/**
  \fn          int camera_pinmux(void)
  \brief       Camera hardware pin initialization:
  - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int camera_pinmux()
{
    int ret;

    ret = pinconf_set(PORT_0, PIN_3, PINMUX_ALTERNATE_FUNCTION_6, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: Camera Pin-Mux failed.\r\n");
        return -1;
    }

    return 0;
}

/**
  \fn          int hardware_init(void)
  \brief       - i3c hardware pin initialization:
  - PIN-MUX configuration
  - PIN-PAD configuration
  - Camera hardware pin initialization:
  - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int hardware_init(void)
{
    int ret;

    /* i3c pinmux. */
    ret = i3c_pinmux();
    if(ret != 0)
    {
        printf("\r\n Error in i3c pinmux.\r\n");
        return -1;
    }

    /* Camera pinmux. */
    ret = camera_pinmux();
    if(ret != 0)
    {
        printf("\r\n Error in Camera pinmux.\r\n");
        return -1;
    }

    return 0;
}

/**
  \fn          void camera_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify the selected Camera Sensor with
  Azure RTOS ThreadX as an Operating System.

  This demo thread does:
  - initialize i3c and Camera hardware pins
  using PinMux Driver;
  - initialize DPHY Tx.
  - initialize Camera driver with Camera Resolution
  - capture one frame
  - captured data will be stored in to allocated
  frame buffer address
  - stop Camera capture
  - dump captured RAW Bayer image data from memory address
  using any debugger
  \param[in]   thread_input : thread input
  \return      none
  */
void camera_demo_thread_entry(ULONG thread_input)
{
    INT   ret = 0;
    ULONG actual_events = 0;
    ULONG wait_timer_ticks = 0;
    UINT  service_error_code;
    UINT  error_code;
    run_profile_t runp = {0};

    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> %s Camera Sensor demo with Azure RTOS ThreadX is starting up!!! <<< \r\n",
            SELECTED_CAMERA_SENSOR);
    printf("\r\n \t\t Resolution: %dx%d \r\n", FRAME_WIDTH, FRAME_HEIGHT);


    /* Allocated memory address for Camera frame buffer. */
    printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            FRAMEBUFFER_POOL_SIZE, (uint32_t) framebuffer_pool);

    /* Initialize i3c and Camera hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: CAMERA Hardware Initialize failed.\r\n");
        return;
    }


    /* Initialize the SE services */
    se_services_port_init();

    /* Enable MIPI Clocks */
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, true, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 100MHz clock enable = %d\n", error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, true, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
    {
        printf("SE: MIPI 38.4Mhz(HFOSC) clock enable = %d\n", error_code);
        goto error_disable_100mhz_clk;
    }

    /* Get the current run configuration from SE */
    error_code = SERVICES_get_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if(error_code)
    {
        printf("\r\nSE: get_run_cfg error = %d\n", error_code);
        goto error_disable_hfosc_clk;
    }

    /*
     * Note:
     * This demo uses a specific profile setting that only enables the
     * items it needs. For example, it only requests the RAM regions and
     * peripheral power that are relevant for this demo. If you want to adapt
     * this example for your own use case, you should adjust the profile setting
     * accordingly. You can either add any additional items that you need, or
     * remove the request altogether to use the default setting that turns on
     * almost everything.
     */

    runp.memory_blocks = MRAM_MASK | SRAM0_MASK;

    runp.phy_pwr_gating = MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK | MIPI_RX_DPHY_MASK | LDO_PHY_MASK;

    /* Set the new run configuration */
    error_code = SERVICES_set_run_cfg(se_services_s_handle,
                                      &runp,
                                      &service_error_code);
    if(error_code)
    {
        printf("\r\nSE: set_run_cfg error = %d\n", error_code);
        goto error_disable_hfosc_clk;
    }

    version = CAMERAdrv->GetVersion();
    printf("\r\n Camera driver version api:0x%X driver:0x%X \r\n",version.api, version.drv);

    ret = CAMERAdrv->Initialize(camera_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Initialize failed.\r\n");
        goto error_disable_hfosc_clk;
    }

    /* Power up Camera peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Power Up failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Control configuration for camera controller */
    ret = CAMERAdrv->Control(CPI_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CPI Configuration failed.\r\n");
        goto error_uninitialize_camera;
    }

    /* Control configuration for camera sensor */
    ret = CAMERAdrv->Control(CPI_CAMERA_SENSOR_CONFIGURE, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    /*Control configuration for camera events */
    ret = CAMERAdrv->Control(CPI_EVENTS_CONFIGURE, \
            ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED | \
            ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN | \
            ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN | \
            ARM_CPI_EVENT_ERR_HARDWARE);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA SENSOR Event Configuration failed.\r\n");
        goto error_poweroff_camera;
    }

    printf("\r\n Let's Start Capturing Camera Frame...\r\n");
    ret = CAMERAdrv->CaptureFrame(framebuffer_pool);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA Capture Frame failed.\r\n");
        goto error_poweroff_camera;
    }

    /* wait till any event success/error comes in isr callback,
     *  and if event is set then clear that event.
     * if the event flags are not set,
     *  this service suspends for a maximum of 'n' timer-ticks.
     */
    wait_timer_ticks = TX_TIMER_TICKS_PER_SECOND * 5;
    ret = tx_event_flags_get(&camera_event_flags, \
            CAM_CB_EVENT_CAPTURE_STOPPED      |   \
            CAM_CB_EVENT_ERROR,                   \
            TX_OR_CLEAR,                          \
            &actual_events,                       \
            wait_timer_ticks);
    if (ret != TX_SUCCESS)
    {
        printf("Error: CAMERA tx_event_flags_get failed.\n");
        goto error_poweroff_camera;
    }

    if(!(actual_events & CAM_CB_EVENT_CAPTURE_STOPPED) && (actual_events & CAM_CB_EVENT_ERROR))
    {
        /* Error: Camera Capture Frame failed. */
        printf("\r\n \t\t >> Error: CAMERA Capture Frame failed. \r\n");
        goto error_poweroff_camera;
    }

    /* Okay, we have received Success: Camera Capture Frame VSYNC detected.
     * now stop Camera Capture.
     */
    ret = CAMERAdrv->Stop();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: CAMERA stop Capture failed.\r\n");
        goto error_poweroff_camera;
    }

    /* How to dump captured RAW Bayer image data from memory address?
     *  1)To dump memory using ARM DS(Development Studio) and Ulink Pro Debugger
     *
     *  Use below command in "Commands" tab:
     *   dump binary memory path_with_filename.fileformat starting_address ending_address
     *
     *   example:(update user directory name)
     *    dump binary memory /home/user/camera_dump/cam_image0_<sensor>_<width>x<height>.bin 0x8000000 0x804C8FF
     *
     *   2)To dump memory using Trace32
     *  Use below command in "Commands" tab:
     *   data.save.binary path_with_filename.fileformat starting_address--ending_address
     *
     *   example:(update user directory name)
     *    data.save.binary /home/user/camera_dump/cam_image0_<sensor>_<width>x<height>.bin 0x8000000--0x804C8FF
     *
     *   This commands will dump memory from staring address to ending address
     *   and store it in to given path with filename.
     *   Convert the RAW Bayer dump offline if a color image is required.
     *
     */
    printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 :");
    printf("\n  Use below commands in Commands tab: update user directory name \r\n");

    printf("Ulink:\n   dump binary memory /home/user/camera_dump/cam_image0_%s_%dx%d.bin 0x%X 0x%X \r\n", \
            SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT, \
            (uint32_t) framebuffer_pool, (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));
    printf("T32:\n   data.save.binary /home/user/camera_dump/cam_image0_%s_%dx%d.bin 0x%X--0x%X \r\n", \
            SELECTED_CAMERA_SENSOR, FRAME_WIDTH, FRAME_HEIGHT, \
            (uint32_t) framebuffer_pool, (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));

    printf("\n  This command will dump memory from staring address to ending address \r");
    printf("\n  and store it in to given path with filename.\r\n\r\n");

    printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured RAW Bayer image data from memory address using any debugger!!!\r\n");

error_poweroff_camera:
    /* Power off CAMERA peripheral */
    ret = CAMERAdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CAMERA Power OFF failed.\r\n");

error_uninitialize_camera:
    /* Un-initialize CAMERA driver */
    ret = CAMERAdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
        printf("\r\n Error: CAMERA Uninitialize failed.\r\n");

error_disable_hfosc_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, false, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
        printf("SE: MIPI 38.4Mhz(HFOSC)  clock disable = %d\n", error_code);

error_disable_100mhz_clk:
    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, false, &service_error_code);
    if(error_code != SERVICES_REQ_SUCCESS)
        printf("SE: MIPI 100MHz clock disable = %d\n", error_code);

    printf("\r\n XXX Camera demo thread is exiting XXX...\r\n");

    /* wait forever */
    while(1);
}

/* Define main entry point.  */
int main()
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
    CHAR *pointer  = TX_NULL;
    UINT status    = 0;

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    status = tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte pool\n");
        return;
    }

    /* Create the event flags group used by i2c thread */
    status = tx_event_flags_create(&camera_event_flags, "event flags I2C");
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create event flags\n");
        goto error_delete_byte_pool;
    }

    /* Allocate the stack for thread.  */
    status = tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create byte allocate\n");
        goto error_delete_event_flag;
    }

    /* Create the main thread.  */
    status = tx_thread_create(&camera_thread, "Camera_thread", camera_demo_thread_entry, 0,
            pointer, DEMO_STACK_SIZE,
            1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("ERROR: Could not create thread \n");
        goto error_release_allocated_byte;
    }

    return; //SUCCESS

error_release_allocated_byte:
    status = tx_byte_release((VOID *) pointer);
    if (status != TX_SUCCESS)
        printf("ERROR: Could not release byte allocate\n");

error_delete_event_flag:
    status = tx_event_flags_delete(&camera_event_flags);
    if (status != TX_SUCCESS)
        printf("ERROR: Could not delete event flags\n");

error_delete_byte_pool:
    status = tx_byte_pool_delete(&byte_pool_0);
    if (status != TX_SUCCESS)
        printf("ERROR: Could not delete byte pool\n");

    while(1);
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
