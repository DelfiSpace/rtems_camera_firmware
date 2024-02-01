#include "frame_handler.h"
#include "stm32l4r9_module_mspi_mt29.h"
#include <stm32l4r9_module_mspi.h>

const u32 image_struct_header[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_struct_closer[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_padding = 0x0;

#define RESET_BLOCK 1
#define RESET_PAGE 0

// Define semaphore
extern rtems_id fa_semaphore_id; // frame available

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
// #define DCMI_POLLING

rtems_isr DCMI_frame_isr(void *void_args) {
  uart_write_buf(USART2, "trigger frame isr      \n\r", 25); // XXX:
#ifndef DCMI_POLLING
  volatile rtems_status_code status_s_release;
  status_s_release = rtems_semaphore_release(fa_semaphore_id);
  if (status_s_release != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "sem release unsuccessful\n\r", 25); // XXX:
  }

#endif /* DCMI_POLLING */
  DCMI->ICR |= DCMI_ICR_FRAME_ISC_Msk;
}

rtems_task DCMI_frame_handler(rtems_task_argument void_args) {

  volatile rtems_status_code status_s_acquire;
  volatile rtems_status_code ir_rs = {0};
  ir_rs = register_dcmi_frame_isr();
  while (1) {

#ifndef DCMI_POLLING
    /* enable dcmi vsync interrupt */
    DCMI->IER |= DCMI_IER_FRAME_IE;

    /* set dcmi capture flag */
    DCMI->CR |= DCMI_CR_CAPTURE;
    /* acquire the semaphore */
    status_s_acquire =
        rtems_semaphore_obtain(fa_semaphore_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    uart_write_buf(USART2, "handling the frame.....         \n\r",
                   34); // XXX:
#endif                  /* DCMI_POLLING */

#ifdef DCMI_POLLING
    /* enable dcmi vsync interrupt */
    DCMI->IER |= DCMI_IER_FRAME_IE;

    /* set dcmi capture flag */
    DCMI->CR |= DCMI_CR_CAPTURE;
    /* poll for the frame complete flag */
    while (~(DCMI->RISR) & DCMI_RIS_FRAME_RIS_Msk) {
    }
#endif /* DCMI_POLLING */

    struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
    /* dcmi dma buffer from dcmi bsp module */
    /* the size of the dma buffer must be larger by at least str_s
     * this is becouse when the image head is in the first bytes of the
     * memory, you will overwrite a memory space that would be before the
     * strict buffer
     */

    extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE];

    volatile struct dcmi_buffer_context dcmi_buffer_ctx = {
        .buffer_head_ptr = dcmi_dma_buffer + IMG_METADATA_MAX_WSIZE};

    /* get properties of the image in the buffer */
    if (dcmi_buffer_analyze(&dcmi_buffer_ctx)) {

      struct jpeg_image image_ws = {.id = args->last_image_index + 1,
                                    .timestamp = 0};

      /* determine the number of pages necessary to store the image*/
      volatile struct jpeg_image image2write;
      image2write.num_pages = dcmi_buffer_ctx.img_size / MT29_PAGE_SIZE + 1;

      volatile struct nand_addr last_addr = {0};
      volatile u32 last_image_page_index =
          args->image_storage_struct->num_pages;
      /* check for number of pages */
      if (last_image_page_index <= MAX_PAGES_IMAGE) {
        /* get last used address */
        last_addr =
            args->image_storage_struct->nand_addr[last_image_page_index];
      } else {
        /* if number of pages out of bounds */
        last_addr.block = RESET_BLOCK;
        last_addr.page = RESET_PAGE;
        last_addr.column = 0x0;
      }
      /* checks bounds for the nand addresses */
      if (last_addr.block == 0 || last_addr.block > MT29_MAX_BLOCKn ||
          last_addr.page > MT29_MAX_PAGEn) {
        /* if out of bounds return to reset valuesS */
        last_addr.block = RESET_BLOCK;
        last_addr.page = RESET_PAGE;
        last_addr.column = 0x0;
      }

      /* generate nand addresses for the used pages*/
      image2write.nand_addr[0] = get_next_nand_addr(last_addr);

      for (int i = 1; i < image2write.num_pages; i++) {
        image2write.nand_addr[i] =
            get_next_nand_addr(image2write.nand_addr[i - 1]);
      }

      size_t str_s = sizeof(image2write);
      volatile u8 *overwrite_ptr = dcmi_buffer_ctx.img_head_ptr - str_s;
      /* overwrite the dcmi_nand buffer with the structure information */
      memcpy(overwrite_ptr, &image2write, str_s);

      /* write pages to the respective nand addressess */
      int i = 0;
      args->mspi_interface.data_ptr = (u32 *)overwrite_ptr;
      while (overwrite_ptr < dcmi_buffer_ctx.img_tail_ptr) {
        mspi_transfer(args->mspi_interface, args->mspi_device.page_load_QUAD,
                      &image2write.nand_addr[i]);
        mspi_transfer(args->mspi_interface, args->mspi_device.page_program,
                      &image2write.nand_addr[i]);
        mspi_transfer(args->mspi_interface, args->mspi_device.wait_oip,
                      &image2write.nand_addr[i]);

        args->mspi_interface.data_ptr += MT29_PAGE_W_SIZE;
        i++;
      }
    }
    /* resets peripheral interrupt */
    uart_write_buf(USART2, "reser frame handler             \n\r",
                   34); // XXX:
                        //
  }
}

/**
 * ---------------------------------------------------------------------------
 * * IMAGE STORAGE HANDLING METHODS
 * ---------------------------------------------------------------------------
 * *
 */

// WARNING: to be tested
void get_image_storage_status(struct mspi_interface octospi,
                              struct mspi_device mt29,
                              struct jpeg_image *image_storage_struct) {
  /* iterate over the static memory and copies the image structures that are
   * found in the ram registry */
  u32 page_buffer[MT29_PAGE_W_SIZE];

  struct nand_addr page_nand_addr;
  page_nand_addr.column = 0;

  size_t str_s = sizeof(*image_storage_struct);

  int i = 0;
  int j = 0;
  u32 found_image_n = 0;

  for (; i < MT29_MAX_BLOCKn; i++) {
    for (; j < MT29_MAX_PAGEn; j++) {

      page_nand_addr.block = i;
      page_nand_addr.page = j;

      mspi_transfer(octospi, mt29.page_read_from_nand, &page_nand_addr);
      memset(page_buffer, 0x0, MT29_PAGE_SIZE);
      mspi_transfer(octospi, mt29.wait_oip, &page_nand_addr);

      octospi.data_ptr = &page_buffer[0];
      mspi_transfer(octospi, mt29.page_read_from_cache_QUAD, &page_nand_addr);

      int k = 0;
      for (; k < MT29_PAGE_W_SIZE; k++) {
        if (page_buffer[k] == image_struct_header[0] &&
            page_buffer[k + 1] == image_struct_header[1] &&
            page_buffer[k + 2] == image_struct_header[2] &&
            page_buffer[k + 3] == image_struct_header[3] &&
            page_buffer[k + str_s] == image_struct_closer[0] &&
            page_buffer[k + str_s + 1] == image_struct_closer[1] &&
            page_buffer[k + str_s + 2] == image_struct_closer[2] &&
            page_buffer[k + str_s + 3] == image_struct_closer[3]) {
          memcpy((image_storage_struct), &page_buffer[k + 4], str_s);
          found_image_n++;
        }
      }
    }
  }
}

// FIX: you need to explore the buffer in a circular manner, becouse
// otherwhise you would always find only the first image, but expecially in
// jpeg there are going to be more
u32 dcmi_buffer_analyze(struct dcmi_buffer_context *dcmi_buffer_ctx) {
  /*
   * data_ptr is the start of the buffer.
   * the size of the buffer is in a macro
   * you can instead define a buffer object, passed to this function
   */

  u8 *check_ptr = (u8 *)dcmi_buffer_ctx->buffer_head_ptr;
  u8 *head_ptr = {0};
  u8 *tail_ptr = {0};
  u8 ptr_areset = {0};
  u32 img_size = {0};

  int i = 0;
  for (i = 0; i < MAX_DMA_TRS_SIZE * 4; i++) {
    // get location of the jpeg header (FFD8)
    if (*(check_ptr + i) == 0xFF && *(check_ptr + i + 1) == 0xD8)
      head_ptr = check_ptr + i;
    ptr_areset |= 1 << 0;
    // get location of the jpeg closer (FFD9)
    if ((check_ptr + i) > head_ptr && *(check_ptr + i) == 0xFF &&
        *(check_ptr + i + 1) == 0xD9)
      tail_ptr = check_ptr + i;
    ptr_areset |= 1 << 1;
  }
  // get location of the last non dirty memory
  // (to get size of the jpeg image)
  if (ptr_areset == 0b11) {
    img_size = tail_ptr - head_ptr;
    dcmi_buffer_ctx->img_head_ptr = head_ptr;
    dcmi_buffer_ctx->img_tail_ptr = tail_ptr;
    dcmi_buffer_ctx->img_size = img_size;
    return 1; // frame found
  }
  return 0; // frame not found
}

struct nand_addr get_next_nand_addr(struct nand_addr addr) {
  addr.page++;
  if (addr.page == MT29_MAX_PAGEn) {
    addr.page = 0;
    addr.block++;
    if (addr.block == MT29_MAX_BLOCKn) {
      addr.block = 1; // restart from block 1 rather than from block 0
    }
  }
  return addr;
}

u32 retrieve_image(struct dcmi_isr_arg isr_ctx,
                   struct jpeg_image *image_storage_struct) {
  /*read image data from the nand pages and store it in a temporary buffer */

  /*allocate stack temp buffer */
  /* this is terminally huge in stack.... and is allocated even if the fun is
   * not used */
  /* therefore the dma buffer is used */
  /* CAUTION: the function will clear the dma buffer and shall be used only
   * when acquisition has been completed */
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE];
  uint32_t *tmp_buffer = dcmi_dma_buffer;
  isr_ctx.mspi_interface.data_ptr = tmp_buffer;
  /*copy in buffer all the pages content*/
  for (int i = 0; i < image_storage_struct->num_pages; i++) {

    mspi_transfer(isr_ctx.mspi_interface,
                  isr_ctx.mspi_device.page_read_from_nand,
                  &image_storage_struct->nand_addr[i]);

    mspi_transfer(isr_ctx.mspi_interface, isr_ctx.mspi_device.wait_oip,
                  &image_storage_struct->nand_addr[i]);

    mspi_transfer(isr_ctx.mspi_interface,
                  isr_ctx.mspi_device.page_read_from_cache_QUAD,
                  &image_storage_struct->nand_addr[i]);
    isr_ctx.mspi_interface.data_ptr += MT29_PAGE_W_SIZE;
  }

  /* generate context for temporary buffer */
  struct dcmi_buffer_context tmp_buffer_context = {
      .buffer_head_ptr = tmp_buffer}; // beginning of the extended buffer
  /*run buffer analyze to isolate the imagedata */
  dcmi_buffer_analyze(&tmp_buffer_context);

  /* TODO: add here stuff to do with the image in the temporary buffer */
}
