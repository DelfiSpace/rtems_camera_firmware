#include "frame_handler.h"
#include "frame_retrieve.h"
#include <stm32l4r9xx.h>
#define RESET_BLOCK_LOCATION 0
#define RESET_PAGE_LOCATION 0

// Define semaphore
#define FRAME_EVENT RTEMS_EVENT_0
extern rtems_id frame_handler_tid;

u32 image_struct_header = {IMAGE_NAND_STR_HEAD};
u32 image_struct_closer = {IMAGE_NAND_STR_CLOS};

/* local declarations */
void init_timer(void);
double calculateElapsedTime(uint16_t startCounter, uint16_t endCounter);

/**
 * --------------------------------------------------------------------------- *
 *       INTERRUPT SERVICE ROUTINE
 * --------------------------------------------------------------------------- *
 */

// Register the interrupt
rtems_status_code register_dcmi_frame_isr(void) {
  rtems_status_code status;

  // Define the interrupt vector
  rtems_vector_number dcmi_interrupt_vector_n = DCMI_IRQn;

  struct dcmi_isr_arg DCMI_frame_isr_arguments;

  // Register the ISR
  status =
      rtems_interrupt_handler_install(dcmi_interrupt_vector_n, // Vector number
                                      "DCMI_frame", // Name for the ISR
                                      RTEMS_INTERRUPT_UNIQUE,   // Flags
                                      DCMI_frame_isr,           // ISR
                                      &DCMI_frame_isr_arguments // ISR argument
      );
  return status;
}

/* ---- this is the function called as dcmi isr handler---- */

rtems_isr DCMI_frame_isr(void *void_args) {
  IFP(uart_write_buf(USART2, "trigger frame isr      \n\r", 25));
  rtems_status_code status_s_release;
  status_s_release = rtems_event_send(frame_handler_tid, FRAME_EVENT);
  if (status_s_release != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "sem release unsuccessful\n\r", 25);
  }

  DCMI->ICR |= DCMI_ICR_FRAME_ISC_Msk;
}

rtems_task DCMI_frame_handler(rtems_task_argument void_args) {

  volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE];

  /* generate buffer context */
  // TODO: move in a constructor
  struct dcmi_buffer_context dcmi_buffer_ctx = {0};
  dcmi_buffer_ctx.buffer_head_ptr = dcmi_dma_buffer + IMG_METADATA_MAX_WSIZE;
  dcmi_buffer_ctx.buffer_tail_ptr =
      &dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE - 1];
  dcmi_buffer_ctx.buff_current_circ_ptr = (u8 *)dcmi_buffer_ctx.buffer_head_ptr;

  init_timer(); // XXX: temporary for testing
  struct jpeg_image image2write = {0};
  struct nand_addr last_addr = {0};
  u32 tick_acq;
  u32 tock_acq;
  u32 tick_wait;
  u32 tock_wait;

  /* enable dcmi vsync interrupt */
  DCMI->IER |= DCMI_IER_FRAME_IE;

  uart_write_buf(USART2, "- Starting capture -    \n\r", 25);
  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;
  while (1) {
    GPIOE->ODR |= GPIO_ODR_OD12;
    DCMI->CR |= DCMI_CR_CAPTURE; // XXX: just for snapshot testing

    /* acquire the semaphore */
    rtems_event_set frame_event_s;
    rtems_status_code status_s_acquire;
    status_s_acquire = rtems_event_receive(FRAME_EVENT, RTEMS_WAIT,
                                           RTEMS_NO_TIMEOUT, &frame_event_s);
    tock_wait = TIM6->CNT & TIM_CNT_CNT_Msk;
    tick_acq = TIM6->CNT & TIM_CNT_CNT_Msk;
    IFP(uart_write_buf(USART2, "handling the frame\n\r", 20));
    GPIOE->ODR &= ~GPIO_ODR_OD12;

    /* ------------------- */
    /* METADATA HANDLING   */
    /* ------------------- */
    /* get properties of the image in the buffer */
    GPIOE->ODR |= GPIO_ODR_OD15;
    int testvar = fast_buffer_analyze(&dcmi_buffer_ctx);
    GPIOE->ODR &= ~GPIO_ODR_OD15;

    if (testvar) {

      /* determine the number of pages necessary to store the image*/
      memset(&image2write, 0x0, sizeof(image2write));

      image2write.num_pages = dcmi_buffer_ctx.img_size / MT29_PAGE_SIZE + 1;

      u32 last_image_page_index = args->image_storage_struct->num_pages - 1;

      /* last address checks */
      if (last_image_page_index <= MAX_PAGES_IMAGE) {
        /* get last used address */
        last_addr =
            args->image_storage_struct->nand_addr[last_image_page_index];
      } else {
        /* if number of pages out of bounds */
        last_addr.block = RESET_BLOCK_LOCATION;
        last_addr.page = RESET_PAGE_LOCATION;
        last_addr.column = 0x0;

        args->image_storage_struct->id = 0;
        args->image_storage_struct->timestamp = 0;
        uart_write_buf(USART2, "Reset addr: max pages   \n\r", 25);
      }
      /* checks bounds for the nand addresses */
      if (last_addr.block > MT29_MAX_BLOCKn ||
          last_addr.page > MT29_MAX_PAGEn) {
        /* if out of bounds return to reset valuesS */
        uart_write_buf(USART2, "Reset addr: out of bound\n\r", 25);
        last_addr.block = RESET_BLOCK_LOCATION;
        last_addr.page = RESET_PAGE_LOCATION;
        last_addr.column = 0x0;

        args->image_storage_struct->id = 0;
        args->image_storage_struct->timestamp = 0;
      }

      /* generate nand addresses for the used pages*/
      image2write.nand_addr[0] = get_next_nand_addr(last_addr);
      image2write.id = args->image_storage_struct->id + 1;
      image2write.timestamp = 0; /* TODO: */

      for (int i = 1; i < image2write.num_pages; i++) {
        image2write.nand_addr[i] =
            get_next_nand_addr(image2write.nand_addr[i - 1]);
        IFP(char temp_str[50]; int n;
            n = sprintf(temp_str, "image page %d: %d,%d (b/p)\r\n", i,
                        image2write.nand_addr[i].block,
                        image2write.nand_addr[i].page);
            uart_write_buf(USART2, temp_str, n));
      }
      /* ------------------- */
      /* BEGIN BUF OVERWRITE */
      /* ------------------- */

      size_t str_s = sizeof(image2write);
      size_t head_s = sizeof(image_struct_header);
      size_t clos_s = sizeof(image_struct_closer);

#define OW_HEADER (dcmi_buffer_ctx.img_head_ptr - str_s - head_s - clos_s)
#define OW_STRUCT (dcmi_buffer_ctx.img_head_ptr - str_s - clos_s)
#define OW_CLOSER (dcmi_buffer_ctx.img_head_ptr - clos_s)

      memcpy(OW_HEADER, &image_struct_header, sizeof(image_struct_header));
      memcpy(OW_STRUCT, &image2write, str_s);
      memcpy(OW_CLOSER, &image_struct_closer, sizeof(image_struct_closer));

      /* ------------------- */
      /* BEGIN MEM WRITE     */
      /* ------------------- */
#ifdef PROGRAMG_NAND
      /* write pages to the respective nand addressess */
      u32 data_padding[10];

      u32 *circ_ptr = (u32 *)OW_HEADER;
      mspi_transfer(args->mspi_interface, args->mspi_device.write_unlock, NULL);

      for (int i = 0; i < image2write.num_pages; i++) {
        if (circ_ptr + MT29_PAGE_W_SIZE > dcmi_buffer_ctx.buffer_tail_ptr) {
          // copy the first part of the image to the beginning of the frame
          // u32 delta = dcmi_buffer_ctx.buffer_tail_ptr - circ_ptr;
          u32 delta = dcmi_buffer_ctx.buffer_tail_ptr - circ_ptr + 1;
          memcpy(dcmi_buffer_ctx.buffer_head_ptr - delta, circ_ptr,
                 (delta) * 4);
          circ_ptr = dcmi_buffer_ctx.buffer_head_ptr - delta;
        }

        args->mspi_interface.data_ptr = data_padding;
        mspi_transfer(args->mspi_interface, args->mspi_device.write_enable,
                      NULL);

        args->mspi_interface.data_ptr = circ_ptr;
        mspi_transfer(args->mspi_interface, args->mspi_device.page_load_QUAD,
                      &image2write.nand_addr[i]);

        args->mspi_interface.data_ptr = data_padding;
        mspi_transfer(args->mspi_interface, args->mspi_device.page_program,
                      &image2write.nand_addr[i]);

        args->mspi_interface.data_ptr = data_padding;
        mspi_transfer(args->mspi_interface, args->mspi_device.wait_oip,
                      &image2write.nand_addr[i]);
        /* NOTE: if you have time in the cycle add a readback to ensure that
         * data in memory is ok */

        circ_ptr += MT29_PAGE_W_SIZE;
      }
#endif

      /* sets last image as current image*/
      *args->image_storage_struct = image2write;
      /* dirty the header and closer of the dma buffer */
      *dcmi_buffer_ctx.img_head_ptr = 0x00;
      *dcmi_buffer_ctx.img_tail_ptr = 0x00;
      /* add here timing check*/
      IFP(uart_write_buf(USART2, "DONE!\n\r", 7));

      /* print time */
      tock_acq = TIM6->CNT & TIM_CNT_CNT_Msk;
      char temp_timestr[20];
      int n;
      n = sprintf(temp_timestr, "acq:%f\r\n",
                  calculateElapsedTime(tick_acq, tock_acq));
      uart_write_buf(USART2, temp_timestr, n);
      n = sprintf(temp_timestr, "wit:%f\r\n",
                  calculateElapsedTime(tick_wait, tock_wait));
      uart_write_buf(USART2, temp_timestr, n);
    } else {
      uart_write_buf(USART2, "NOIMG\n\r", 7);
    }
    tick_wait = TIM6->CNT & TIM_CNT_CNT_Msk;
  }
}

/**
 * ---------------------------------------------------------------------------
 * * IMAGE STORAGE HANDLING METHODS
 * ---------------------------------------------------------------------------
 * *
 */

void get_image_storage_status(void *void_args) {
  /* iterate over the static memory and copies the image structures that are
   * found in the ram registry */

  volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;

  u32 page_buffer[MT29_PAGE_W_SIZE];

  struct nand_addr page_nand_addr;
  page_nand_addr.column = 0;

  size_t str_s = sizeof(*args->image_storage_struct);

  volatile u32 found_image_n = 0;
  extern int search_block_limit;
  for (int i = 0; i < search_block_limit; i++) {
    for (int j = 0; j < MT29_MAX_PAGEn; j++) {

      page_nand_addr.block = i;
      page_nand_addr.page = j;
      char temp_str[20];
      int n;
      n = sprintf(temp_str, "ck: %d %d\r\n", i, j);
      uart_write_buf(USART2, temp_str, n);

      mspi_transfer(args->mspi_interface, args->mspi_device.page_read_from_nand,
                    &page_nand_addr);
      mspi_transfer(args->mspi_interface, args->mspi_device.wait_oip,
                    &page_nand_addr);

      memset(page_buffer, 0x0, MT29_PAGE_SIZE);
      args->mspi_interface.data_ptr = &page_buffer[0];
      mspi_transfer(args->mspi_interface,
                    args->mspi_device.page_read_from_cache_QUAD,
                    &page_nand_addr);

      volatile int k = 0;
      for (; k < MT29_PAGE_W_SIZE; k++) {
        if (page_buffer[k] == image_struct_header &&
            page_buffer[k + str_s / 4 + 1] == image_struct_closer) {
          memcpy(args->image_storage_struct, &page_buffer[k + 1], str_s);
          uart_write_buf(USART2, "found an image\n\r", 16);
#ifdef IF_DOWNLOAD
          retrieve_image(args);
#endif /* ifdef IF_DOWNLOAD */
          found_image_n++;
        }
      }
    }
  }
}
u32 fast_buffer_analyze(struct dcmi_buffer_context *b_ctx) {
  b_ctx->img_head_ptr = (u8 *)b_ctx->buffer_head_ptr;
  b_ctx->img_tail_ptr = (u8 *)b_ctx->buffer_tail_ptr;
  b_ctx->img_size = b_ctx->img_tail_ptr - b_ctx->img_head_ptr;
  return 1;
}

u32 dcmi_buffer_analyze(struct dcmi_buffer_context *b_ctx) {
  /* determines the head and the tail of the image in buffer,
   * calculates the size of the image
   */
#define HEADn1 0x1
#define HEADn2 0x2
#define TAILn1 0x10
#define EOBn1 0x40
#define EOBn2 0x80
#define FOUND_HEADn1 (status_reg & HEADn1)
#define FOUND_HEADn2 (status_reg & HEADn2)
#define FOUND_TAILn1 (status_reg & TAILn1)
#define FOUND_EOBn1 (status_reg & EOBn1)
#define FOUND_EOBn2 (status_reg & EOBn2)
  enum ret_en { working, NOIMG, IMG };

  volatile u8 status_reg = {0};
  int i = 0;
  volatile u8 ret = {0};

#define CHK_PTR (u8 *)b_ctx->buff_current_circ_ptr
  u8 *initial_ptr = CHK_PTR;

  // NOTE: will fail if the image header/closer is in the middle
  // of the end of the buffer (ff end of buffer, d8 beginning f.expl.)

  char temp_str[22];
  int n;
  n = sprintf(temp_str, "%x, chkptr\r\n", CHK_PTR);
  uart_write_buf(USART2, temp_str, n);
  while (ret == 0) {

    /* check if the check pointer is over the tail */
    if ((CHK_PTR + i) > (u8 *)b_ctx->buffer_tail_ptr + 3) {
      if (FOUND_EOBn1) {
        /* if the end of the buffer is found again, no image found */
        status_reg |= EOBn2;
        ret = NOIMG;
      }
      b_ctx->buff_current_circ_ptr = (u8 *)b_ctx->buffer_head_ptr;
      status_reg |= EOBn1;
      i = 0;
    }
    /* if head is found again, no image found*/
    if (FOUND_HEADn1 && *(CHK_PTR + i) == 0xFF && *(CHK_PTR + i + 1) == 0xD8) {
      status_reg |= HEADn2;
      ret = NOIMG;
    }
    /* search for head */
    if ((~status_reg & HEADn1) && *(CHK_PTR + i) == 0xFF &&
        *(CHK_PTR + i + 1) == 0xD8) {
      b_ctx->img_head_ptr = CHK_PTR + i;
      status_reg |= HEADn1;
    }
    /* search for tail if head has been found*/
    if (FOUND_HEADn1 && *(CHK_PTR + i) == 0xFF && *(CHK_PTR + i + 1) == 0xD9) {
      b_ctx->img_tail_ptr = CHK_PTR + i;
      status_reg |= TAILn1;
      ret = IMG;
    }
    i++;
  }

  if (ret == IMG) {
    if (b_ctx->img_tail_ptr > b_ctx->img_head_ptr) {
      b_ctx->img_size = b_ctx->img_tail_ptr - b_ctx->img_head_ptr;
    }
    if (b_ctx->img_head_ptr > b_ctx->img_tail_ptr) {
      b_ctx->img_size = (u8 *)b_ctx->buffer_tail_ptr + 3 - b_ctx->img_head_ptr;
      b_ctx->img_size += b_ctx->img_tail_ptr - (u8 *)b_ctx->buffer_head_ptr;
    }
    IFP(uart_write_buf(USART2, "+\n\r", 3));
    b_ctx->buff_current_circ_ptr = b_ctx->buff_current_circ_ptr + i;
    return 1;
  }
  IFP(uart_write_buf(USART2, "-\n\r", 3); char temp_str[22]; int n;
      n = sprintf(temp_str, "%x,%x\r\n", status_reg, ret);
      uart_write_buf(USART2, temp_str, n));

  return 0;
}

struct nand_addr get_next_nand_addr(struct nand_addr addr) {
  addr.page++;
  if (addr.page == MT29_MAX_PAGEn) {
    addr.page = 0;
    addr.block++;
    if (addr.block == MT29_MAX_BLOCKn) {
      addr.block = 0;
    }
  }
  return addr;
}

/* WARN: temporary, for testing */
void init_timer(void) {
  // clock the APB1 bus
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  spin(1);
// set PSC
#define DIVII 2000
  TIM6->PSC &= TIM_PSC_PSC_Msk;
  TIM6->PSC |= DIVII - 1;

  // set ARR
  TIM6->ARR |= 0xFFFF; // 65536

  // set control register
  TIM6->CR1 |= TIM_CR1_CEN;
}

/* TODO: harmonize with bsp frequency */
#define SYSTICK 120000000 // Hz
double calculateElapsedTime(uint16_t startCounter, uint16_t endCounter) {
  uint32_t tickDifference;

  // Handle counter wrap-around

  if (endCounter >= startCounter) {
    tickDifference = endCounter - startCounter;
  } else {
    // Counter has wrapped around
    tickDifference = (0xFFFF - startCounter) + endCounter + 1;
  }

  // Calculate time in seconds
  double timeElapsed = (double)tickDifference * DIVII / (SYSTICK);

  return timeElapsed;
}
