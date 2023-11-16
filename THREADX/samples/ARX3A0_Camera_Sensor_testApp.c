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
 * @file     ARX3A0_Camera_Sensor_testApp.c
 * @author   Prasanna Ravi and Chandra Bhushan Singh
 * @email    prasanna.ravi@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     04-May-2023
 * @brief    TestApp to verify ARX3A0 Camera Sensor with
 *            Azure RTOS ThreadX as an Operating System.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

//* System Includes */
#include <stdio.h>
#include "tx_api.h"

/* Cpi Driver */
#include "RTE_Components.h"
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

/* @Note: ARX3A0 Camera Sensor configurations
 *        are directly borrowed from ARX3A0 Camera Sensor drivers,
 *        for detail refer ARX3A0 driver.
 *
 * Selected ARX3A0 Camera Sensor configurations:
 *   - Interface     : MIPI CSI2
 *   - Resolution    : 560X560
 *   - Output Format : RAW Bayer10
 */

/* ARX3A0 Camera Sensor Resolution. */
#define ARX3A0_CAMERA_RESOLUTION_560x560           0
#define ARX3A0_CAMERA_RESOLUTION                   ARX3A0_CAMERA_RESOLUTION_560x560

#if (ARX3A0_CAMERA_RESOLUTION == ARX3A0_CAMERA_RESOLUTION_560x560)
#define FRAME_WIDTH                               (560)
#define FRAME_HEIGHT                              (560)
#endif

/* Allocate Camera frame buffer memory using memory pool section in
 *  Linker script (sct scatter) file.
 */

/* pool size for Camera frame buffer:
 *  which will be frame width x frame height
 */
#define FRAMEBUFFER_POOL_SIZE                     ((FRAME_WIDTH) * (FRAME_HEIGHT))

/* pool area for Camera frame buffer.
 *  Allocated in the "camera_frame_buf" section.
 */
uint8_t framebuffer_pool[FRAMEBUFFER_POOL_SIZE] \
        __attribute__((section("camera_frame_buf")));

/* (optional)
 * if required convert captured image data format to any other image format.
 *
 *  - for ARX3A0 Camera sensor,
 *    selected Bayer output format:
 *    in-order to get the color image,
 *    Bayer format must be converted in to RGB format.
 *    User can use below provided
 *    "Open-Source" code for Bayer to RGB Conversion
 *    which uses DC1394 library.
 */
/* Enable image conversion Bayer to RGB. */
#define IMAGE_CONVERSION_BAYER_TO_RGB_EN         1

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN

/* @Note: Bayer to RGB configurations
 *        are directly borrowed from "Open-Source" code for
 *        Bayer to RGB Conversion, for detail refer bayer2rgb.c.
 *
 * Selected Bayer to RGB configurations:
 *   - converted image format : tiff
 *   - bpp bit per pixel      : 8-bit
 */
#define TIFF_HDR_NUM_ENTRY                       8
#define TIFF_HDR_SIZE                            10 + TIFF_HDR_NUM_ENTRY * 12

/* bpp bit per pixel
 *  Valid parameters are:
 *   -  8-bit
 *   - 16-bit
 */
#define BITS_PER_PIXEL_8_BIT                     8
#define BITS_PER_PIXEL                           BITS_PER_PIXEL_8_BIT

/* pool size for Camera frame buffer for Bayer to RGB conversion:
 *   which will be frame width x frame height x (bpp / 8) * 3 + tiff header(106 Bytes).
 */
#define BAYER_TO_RGB_BUFFER_POOL_SIZE   \
    ( (FRAME_WIDTH) * (FRAME_HEIGHT) * (BITS_PER_PIXEL / 8) * 3 + TIFF_HDR_SIZE )

/* pool area for Camera frame buffer for Bayer to RGB conversion.
 *  Allocated in the "camera_frame_bayer_to_rgb_buf" section.
 */
uint8_t bayer_to_rgb_buffer_pool[BAYER_TO_RGB_BUFFER_POOL_SIZE] \
        __attribute__((section("camera_frame_bayer_to_rgb_buf")));

/* Optional:
 *  Camera Image Conversions
 */
typedef enum {
    BAYER_TO_RGB_CONVERSION   = (1 << 0),
}IMAGE_CONVERSION;

#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

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

/* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
/**
  \fn          int camera_image_conversion(IMAGE_CONVERSION image_conversion,
  uint8_t  *src,
  uint8_t  *dest,
  uint32_t  frame_width,
  uint32_t  frame_height)
  \brief       Convert image data from one format to any other image format.
  - Supported conversions
  - Bayer(RAW) to RGB Conversion
  - User can use below provided
  "Open-Source" Bayer to RGB Conversion code
  which uses DC1394 library.
  - This code will,
  - Add header for tiff image format
  - Convert RAW Bayer to RGB depending on
  - bpp bit per pixel 8/16 bit
  - DC1394 Color Filter
  - DC1394 Bayer interpolation methods
  - Output image size will be
  - width x height x (bpp / 8) x 3 + tiff header(106 Bytes)
  \param[in]   image_conversion : image conversion methods \ref IMAGE_CONVERSION
  \param[in]   src              : Source address, Pointer to already available
  image data Address
  \param[in]   dest             : Destination address,Pointer to Address,
  where converted image data will be stored.
  \param[in]   frame_width      : image frame width
  \param[in]   frame_height     : image frame height
  \return      success          :  0
  \return      failure          : -1
  */
int camera_image_conversion(IMAGE_CONVERSION  image_conversion,
        uint8_t *src, uint8_t *dest, uint32_t frame_width,
        uint32_t frame_height)
{
    /* Bayer to RGB Conversion. */
    extern int32_t bayer_to_RGB(uint8_t  *src,   uint8_t  *dest,   \
             uint32_t  width, uint32_t  height);

    int ret = 0;

    switch(image_conversion)
    {
        case BAYER_TO_RGB_CONVERSION:
        {
            printf("\r\n Start Bayer to RGB Conversion: \r\n");
            printf("\t Frame Buffer Addr: 0x%X \r\n \t Bayer_to_RGB Addr: 0x%X\n", \
                    (uint32_t) src, (uint32_t) dest);
            ret = bayer_to_RGB(src, dest, frame_width, frame_height);
            if(ret != 0)
            {
                printf("\r\n Error: CAMERA image conversion: Bayer to RGB failed.\r\n");
                return -1;
            }
            break;
        }

        default:
        {
            return -1;
        }
    }

    return 0;
}
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

/**
  \fn          void camera_demo_thread_entry(ULONG thread_input)
  \brief       TestApp to verify ARX3A0 Camera Sensor with
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
  - (optional)
  -if required convert captured image format
  in to any other image format
  - for ARX3A0 Camera sensor,
  - selected Bayer output format;
  - in-order to get the color image,
  Bayer format must be converted in to RGB format.
  - User can use below provided
  "Open-Source" code for Bayer to RGB Conversion
  which uses DC1394 library.
  - dump captured/converted image data from memory address
  using any debugger
  - display image
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

    printf("\r\n \t\t >>> ARX3A0 Camera Sensor demo with Azure RTOS ThreadX is starting up!!! <<< \r\n");


    /* Allocated memory address for
     *   - Camera frame buffer and
     *   - (Optional) Camera frame buffer for Bayer to RGB Conversion
     */
    printf("\n \t frame buffer        pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            FRAMEBUFFER_POOL_SIZE, (uint32_t) framebuffer_pool);

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    printf("\n \t bayer_to_rgb buffer pool size: 0x%0X  pool addr: 0x%0X \r\n ", \
            BAYER_TO_RGB_BUFFER_POOL_SIZE, (uint32_t) bayer_to_rgb_buffer_pool);
#endif

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

    /* (optional)
     * if required convert captured image data format to any other image format.
     *  - for ARX3A0 Camera sensor,
     *     selected Bayer output format:
     *      in-order to get the color image,
     *       Bayer format must be converted in to RGB format.
     *       User can use below provided
     *        "Open-Source" code for Bayer to RGB Conversion
     *        which uses DC1394 library.
     */
    /* Check if image conversion Bayer to RGB is Enabled? */
#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    ret = camera_image_conversion(BAYER_TO_RGB_CONVERSION,
            framebuffer_pool,
            bayer_to_rgb_buffer_pool,
            FRAME_WIDTH,
            FRAME_HEIGHT);
    if(ret != 0)
    {
        printf("\r\n Error: CAMERA image conversion failed.\r\n");
        return;
    }
#endif /* end of IMAGE_CONVERSION_BAYER_TO_RGB_EN */

    /* How to dump captured/converted image data from memory address?
     *  1)To dump memory using ARM DS(Development Studio) and Ulink Pro Debugger
     *
     *  Use below command in "Commands" tab:
     *   dump binary memory path_with_filename.fileformat starting_address ending_address
     *
     *   example:(update user directory name)
     *    dump binary memory /home/user/camera_dump/cam_image0_560p.bin 0x8000000 0x804C8FF
     *
     *   Bayer to RGB:
     *    dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_560p.tif 0x8000000 0x80E5B69
     *
     *   2)To dump memory using Trace32
     *  Use below command in "Commands" tab:
     *   data.save.binary path_with_filename.fileformat starting_address--ending_address
     *
     *   example:(update user directory name)
     *    data.save.binary /home/user/camera_dump/cam_image0_560p.bin 0x8000000--0x804C8FF
     *
     *   Bayer to RGB:
     *    data.save.binary /home/user/camera_dump/cam_image0_Bayer_to_RGB_560p.tif 0x8000000--0x80E5B69
     *
     *   This commands will dump memory from staring address to ending address
     *   and store it in to given path with filename.
     *
     *
     */
    printf("\n To dump memory using ARM DS with Ulink Pro Debugger or Trace32 :");
    printf("\n  Use below commands in Commands tab: update user directory name \r\n");

#if IMAGE_CONVERSION_BAYER_TO_RGB_EN
    printf("Ulink:\n   dump binary memory /home/user/camera_dump/cam_image0_Bayer_to_RGB_560p.tif 0x%X 0x%X \r\n", \
            (uint32_t) bayer_to_rgb_buffer_pool, (uint32_t) (bayer_to_rgb_buffer_pool + BAYER_TO_RGB_BUFFER_POOL_SIZE - 1));
    printf("T32:\n   data.save.binary /home/user/camera_dump/cam_image0_Bayer_to_RGB_560p.tif 0x%X--0x%X \r\n", \
            (uint32_t) bayer_to_rgb_buffer_pool, (uint32_t) (bayer_to_rgb_buffer_pool + BAYER_TO_RGB_BUFFER_POOL_SIZE - 1));

#else
    printf("Ulink:\n   dump binary memory /home/user/camera_dump/cam_image0_560p.bin 0x%X 0x%X \r\n", \
            (uint32_t) framebuffer_pool, (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));
    printf("T32:\n   data.save.binary /home/user/camera_dump/cam_image0_560p.bin 0x%X--0x%X \r\n", \
            (uint32_t) framebuffer_pool, (uint32_t) (framebuffer_pool + FRAMEBUFFER_POOL_SIZE - 1));
#endif

    printf("\n  This command will dump memory from staring address to ending address \r");
    printf("\n  and store it in to given path with filename.\r\n\r\n");

    printf("\r\n\r\n XXX Camera demo thread is halting here! XXX...\r\n");
    printf("\r\n Now User can dump captured/converted image data from memory address using any debugger!!!\r\n");

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
