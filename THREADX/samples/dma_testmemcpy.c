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
 * @file     dma_testmemcpy.c
 * @author   Sudhir Sreedharan
 * @email    sudhir@alifsemi.com
 * @version  V1.0.0
 * @date     15-Mar-2023
 * @brief    ThreadX demo app for memcpy
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include <stdio.h>
#include <tx_api.h>
#include <Driver_DMA.h>
#include <string.h>
#include <RTE_Components.h>
#include CMSIS_device_header

/* Enable Debug logs */
#define DMA_DEBUG
//#define DMA_DEBUG_GET_STATUS

/* Enable the DMA controller to test */
//#define TEST_DMA0
#if (M55_HP)
#define TEST_DMA1
#endif
#if defined (M55_HE)
#define TEST_DMA2
#endif


#define DMA0 0                  /* DMA0 */
#define DMA1 1                  /* DMA1 */
#define DMA2 2                  /* DMA2 */

/* To test a particular BS, BLEN and transfer len */
bool test_point = true;

#define DMA_BS       BS_BYTE_8
#define DMA_BLEN     16
#define DMA_XFER_LEN 1000


#if defined (M55_HE)
/* TCM size is less in RTSS_HE */
#define MAX_TRANSFER_LEN (1024)               /* Total Number of bytes */
#else
#define MAX_TRANSFER_LEN (130*1024)           /* Total Number of bytes */
#endif

/*
 * Add extra space at the end of the buffer. This will help to identify if
 * the DMA copies extra bytes
 */
#define ACTUAL_BUFF_SIZE (MAX_TRANSFER_LEN + 1024)

uint8_t  src_buff[ACTUAL_BUFF_SIZE];
uint8_t  dst_buff[ACTUAL_BUFF_SIZE];

#define DMA_MEMCPY_THREAD_STACK_SIZE      1024
#define DMA_MEMCPY_THREAD_THRD_PRIO       0
#define DMA_MEMCPY_THREAD_PREEMPT_THRSHLD DMA_MEMCPY_THREAD_THRD_PRIO

TX_THREAD            dma_memcpy_thrd;
char                 dma_memcpy_thread_stack[DMA_MEMCPY_THREAD_STACK_SIZE];
TX_EVENT_FLAGS_GROUP dma_memcpy_event_flags;


#define DMA_SEND_COMPLETE_EVENT (1 << 0)
#define DMA_ABORT_EVENT         (1 << 1)


/**
  \fn          void CreateBuffer(void *buf, uint32_t size)
  \brief       Create src buffer to trasmit
  \param[in]   buf Pointer to the buffer
  \param[in]   size Buffer Size
*/
void CreateBuffer(void *buf, uint32_t size) {
    uint8_t *ptr = (uint8_t*)buf;
    uint32_t cnt;

    for (cnt = 0; cnt < size; cnt++)
        ptr[cnt] = cnt + 1;
}

/**
  \fn          void CompareBuffers (uint32_t len)
  \brief       Compare the src and dst buffers
*/
int32_t CompareBuffers (uint32_t len)
{
    int32_t  ret;
    uint32_t cnt;

    ret = memcmp(src_buff, dst_buff, len);

    if(ret == 0) {
        for (cnt = len; cnt < ACTUAL_BUFF_SIZE; cnt++) {
            if (dst_buff[cnt] != 0 ) {
#ifdef DMA_DEBUG
                printf(" DMA Memcpy COPIED MORE BYTES\n");
#endif
                return -1;
            }
        }
    }

    return ret;


}

/**
  \fn          void dma_cb (uint32_t event, int8_t peri_num)
  \brief       Callback routine from the dma driver
  \param[in]   event Event for which the callback has been called
  \param[in]   peri_num Peripheral number
*/
void dma_cb (uint32_t event, int8_t peri_num)
{
    (void)peri_num;

    if(event & ARM_DMA_EVENT_COMPLETE)
    {
        tx_event_flags_set(&dma_memcpy_event_flags, DMA_SEND_COMPLETE_EVENT, TX_OR);
    }


    if(event & ARM_DMA_EVENT_ABORT)
    {
        tx_event_flags_set(&dma_memcpy_event_flags, DMA_ABORT_EVENT, TX_OR);
    }
}

/**
  \fn          void dma_memcpy_thread_entry (ULONG arg)
  \brief       DMA Thread to handle transmission
  \param[in]   argument Pointer to the argument
*/
void dma_memcpy_thread_entry (ULONG arg)
{
      ARM_DRIVER_VERSION   version;
      ARM_DRIVER_DMA       *dma_drv;
      ARM_DMA_CAPABILITIES cap;
      uint32_t             status;
#ifdef DMA_DEBUG
      uint32_t             count = 0;
#endif
      DMA_Handle_Type      handle;
      ARM_DMA_PARAMS       params;
      ARM_DMA_BS_Type      bs;
      uint8_t              blen;
      int32_t              ret;
      uint32_t             len;
      ULONG                actual_flags;


      (void)arg;
      extern ARM_DRIVER_DMA ARM_Driver_DMA_(DMA1);
      extern ARM_DRIVER_DMA ARM_Driver_DMA_(DMA2);
      extern ARM_DRIVER_DMA ARM_Driver_DMA_(DMA0);

#ifdef TEST_DMA0
      dma_drv = &ARM_Driver_DMA_(DMA0);
#elif defined(TEST_DMA1)
      dma_drv = &ARM_Driver_DMA_(DMA1);
#else //DMA2
      dma_drv = &ARM_Driver_DMA_(DMA2);
#endif

      /* Verify the DMA API version for compatibility*/
      version = dma_drv->GetVersion ();
#ifdef DMA_DEBUG
      printf ("DMA API version = %d\n", version.api);
#else
      (void)version;
#endif

      /* Verify if DMA protocol is supported */
      cap = dma_drv->GetCapabilities ();
      if (!cap.mem_to_mem)
          return;

      /* Initializes DMA interface */
      status = dma_drv->Initialize ();
      if(status) {
#ifdef DMA_DEBUG
          printf ("DMA init status = %d\n", status);
#endif
          return;
      }

      /* Power control for DMA */
      status = dma_drv->PowerControl(ARM_POWER_FULL);
      if(status) {
#ifdef DMA_DEBUG
          printf ("DMA Enable Power failed = %d\n", status);
#endif
          return;
      }

      /* Allocate handle for DMA */
      status = dma_drv->Allocate(&handle);
      if(status) {
#ifdef DMA_DEBUG
          printf ("DMA channel allocation failed = %d\n", status);
#endif
          return;
      }

      params.peri_reqno = -1;
      params.dir = ARM_DMA_MEM_TO_MEM;
      params.cb_event   = dma_cb;
      params.src_addr = &src_buff;
      params.dst_addr = &dst_buff;

#ifdef DMA_DEBUG
      printf ("DMA MEMCPY START CNT = %ul\n", ARM_PMU_Get_CCNTR());
#endif
      for (len = 1; len < MAX_TRANSFER_LEN; len++) {
          for (bs = BS_BYTE_1; bs < BS_BYTE_16; bs++) {
              for (blen = 1; blen < 17; blen++) {

                  /*
                   * Skip through the bs, blen and len values and run only for the
                   * requested parameteres
                   */
                  if(test_point) {
                      if ((bs != DMA_BS)  || (blen != DMA_BLEN) || (len != DMA_XFER_LEN))
                          continue;
                  }

#ifdef  DMA_DEBUG
                  printf(" DMA Memcpy STARTED for BS=%d, BLEN = %d\n", bs, blen);
#endif
                  params.burst_size = bs;
                  params.burst_len  = blen;
                  params.num_bytes  = len;

                  /* Start transfer */
                  status = dma_drv->Start(&handle, &params);
                  if(status || (handle < 0)) {
#ifdef DMA_DEBUG
                      printf(" DMA Memcpy FAILED for BS=%d, BLEN = %d, status = %d\n", bs, blen, status);
#endif
                      continue;
                  }

#ifdef TEST_DMA_STOP
                  /* Test Case for Stop functionality in the middle of a operation */
                  status = dma_drv->Stop(&handle);
                  if(status || (handle < 0)) {
#ifdef DMA_DEBUG
                      printf(" DMA STop FAILED for BS=%d, BLEN = %d, status = %d\n", bs, blen, status);
#endif
                      while(1);
                  }
#endif
#ifdef DMA_DEBUG_GET_STATUS
                  while(1) {
                      status = dma_drv->GetStatus(&handle, &count);
                      printf ("DMA count = %d\n", count);

                      if (status || (count == len))
                          break;
                  }
#endif

                  /* wait for the dma callback */
                  tx_event_flags_get(&dma_memcpy_event_flags,
                                     DMA_SEND_COMPLETE_EVENT|DMA_ABORT_EVENT,
                                     TX_OR_CLEAR,
                                     &actual_flags,
                                     TX_WAIT_FOREVER);

                  /* Now the buffer is ready, compare it */
                  ret = CompareBuffers (len);
                  if(ret) {
#ifdef DMA_DEBUG
                      printf(" DMA Memcpy *FAILED* for BS=%d, BLEN = %d xfer_len=%d\n", bs, blen, len);
#endif
                      while(1);
                  }
                  else {
#ifdef DMA_DEBUG
                          printf(" DMA Memcpy SUCCESS for BS=%d, BLEN = %d xfer_len=%d\n", bs, blen, len);
#endif
                  }

                  if(!test_point)
                      memset (dst_buff, 0 , ACTUAL_BUFF_SIZE);

              }
          }
      }

#ifdef DMA_DEBUG
      printf ("DMA MEMCPY STOP CNT = %d\n", ARM_PMU_Get_CCNTR());
#endif

      status = dma_drv->DeAllocate(&handle);
      if(status) {
#ifdef DMA_DEBUG
          printf ("DMA DeAllocate = %d\n", status);
#endif
          while(1);
      }

      /* Power control for DMA */
      status = dma_drv->PowerControl(ARM_POWER_OFF);
      if(status) {
#ifdef DMA_DEBUG
          printf ("DMA PowerOff failed = %d\n", status);
#endif
          while(1);
      }

#ifdef DMA_DEBUG
      printf(" DMA TEST COMPLETED \n");
#endif
}

/**
  \fn          void tx_application_define(void *first_unused_memory)
  \brief       Initialization function for Application defines
  \param[in]   first_unused_memory memory pointer
*/
void tx_application_define(void *first_unused_memory)
{
    UINT status;

    (void)first_unused_memory;

    memset (dst_buff, 0 , ACTUAL_BUFF_SIZE);

    RTSS_CleanDCache_by_Addr (dst_buff, ACTUAL_BUFF_SIZE);

    /* Create random data for the DMA Source data*/
    CreateBuffer((void*)src_buff, ACTUAL_BUFF_SIZE);

    status = tx_thread_create(&dma_memcpy_thrd,
                              "DMA MEMCPY Thread",
                              dma_memcpy_thread_entry,
                              0,
                              &dma_memcpy_thread_stack,
                              DMA_MEMCPY_THREAD_STACK_SIZE,
                              DMA_MEMCPY_THREAD_THRD_PRIO,
                              DMA_MEMCPY_THREAD_PREEMPT_THRSHLD,
                              TX_NO_TIME_SLICE,
                              TX_AUTO_START);
    if(status)
    {
#ifdef DMA_DEBUG
        printf("\r\nCouldn't create TX Thread\n. Exiting..\n");
#endif
        while(1);
    }

    status = tx_event_flags_create(&dma_memcpy_event_flags, "DMA MEMCPY EVENT FLAGS");
    if(status)
    {
#ifdef DMA_DEBUG
        printf("\r\nCouldn't create Events Flags\n. Exiting..\n");
#endif
        while(1);
    }
}

/**
  \fn          int main (void)
  \brief       Application Main
  \return      int application exit status
*/
int main (void)
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}
