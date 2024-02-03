#include "frame_handler.h"
#include "frame_retrieve.h"
#define RESET_BLOCK_LOCATION 1
#define RESET_PAGE_LOCATION 0

// Define semaphore
#define FRAME_EVENT RTEMS_EVENT_0
extern rtems_id frame_handler_tid;

u32 image_struct_header = {IMAGE_NAND_STR_HEAD};
u32 image_struct_closer = {IMAGE_NAND_STR_CLOS};

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
  uart_write_buf(USART2, "trigger frame isr      \n\r", 25); // XXX:
  volatile rtems_status_code status_s_release;
  status_s_release = rtems_event_send(frame_handler_tid, FRAME_EVENT);
  if (status_s_release != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "sem release unsuccessful\n\r", 25); // XXX:
  }

  DCMI->ICR |= DCMI_ICR_FRAME_ISC_Msk;
}

rtems_task DCMI_frame_handler(rtems_task_argument void_args) {

  /* enable dcmi vsync interrupt */
  DCMI->IER |= DCMI_IER_FRAME_IE;

  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;
  while (1) {

    /* acquire the semaphore */
    rtems_event_set frame_event_s;
    volatile rtems_status_code status_s_acquire;
    status_s_acquire = rtems_event_receive(FRAME_EVENT, RTEMS_WAIT,
                                           RTEMS_NO_TIMEOUT, &frame_event_s);
    uart_write_buf(USART2, "handling the frame!!!!!         \n\r", 34);

    volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;

    extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE];

    volatile struct dcmi_buffer_context dcmi_buffer_ctx = {
        .buffer_head_ptr = dcmi_dma_buffer + IMG_METADATA_MAX_WSIZE,
        .buffer_tail_ptr =
            dcmi_dma_buffer + sizeof(dcmi_dma_buffer) - IMG_METADATA_MAX_WSIZE};

    /* get properties of the image in the buffer */
    if (dcmi_buffer_analyze(&dcmi_buffer_ctx)) {

      struct jpeg_image image_ws = {.id = args->last_image_index + 1,
                                    .timestamp = 0};

      /* determine the number of pages necessary to store the image*/
      volatile struct jpeg_image image2write;
      image2write.num_pages = dcmi_buffer_ctx.img_size / MT29_PAGE_SIZE + 1;

      volatile struct nand_addr last_addr = {0};
      volatile u32 last_image_page_index =
          args->image_storage_struct->num_pages - 1;

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
      }
      /* checks bounds for the nand addresses */
      if (last_addr.block == 0 || last_addr.block > MT29_MAX_BLOCKn ||
          last_addr.page > MT29_MAX_PAGEn) {
        /* if out of bounds return to reset valuesS */
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

      char temp_str[50]; // XXX: debug
      for (int i = 1; i < image2write.num_pages; i++) {
        image2write.nand_addr[i] =
            get_next_nand_addr(image2write.nand_addr[i - 1]);
        /*
         * (debug) Print the addresses:
         */
        int n;
        n = sprintf(temp_str, "image page %d: %d,%d (b/p)\r\n", i,
                    image2write.nand_addr[i].block,
                    image2write.nand_addr[i].page);
        uart_write_buf(USART2, temp_str, n);
        /*
         */
      }

      size_t str_s = sizeof(image2write);
      size_t head_s = sizeof(image_struct_header);
      size_t clos_s = sizeof(image_struct_closer);

#define OW_HEADER (dcmi_buffer_ctx.img_head_ptr - str_s - head_s - clos_s)
#define OW_STRUCT (dcmi_buffer_ctx.img_head_ptr - str_s - clos_s)
#define OW_CLOSER (dcmi_buffer_ctx.img_head_ptr - clos_s)

      memcpy(OW_HEADER, &image_struct_header, sizeof(image_struct_header));
      memcpy(OW_STRUCT, &image2write, str_s);
      memcpy(OW_CLOSER, &image_struct_closer, sizeof(image_struct_closer));

      /* write pages to the respective nand addressess */
      u32 data_padding[10];
      int i = 0;

      u32 *circ_ptr = (u32 *)OW_HEADER;
#ifdef PROGRAMG_NAND
      mspi_transfer(args->mspi_interface, args->mspi_device.write_unlock, NULL);
#endif
      while (circ_ptr < (u32 *)dcmi_buffer_ctx.img_tail_ptr) {
#ifdef PROGRAMG_NAND

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
#endif
        /* NOTE: if you have time in the cycle add a readback to ensure that
         * data in memory is ok */

        circ_ptr += MT29_PAGE_W_SIZE;
        // you are not overwriting the
        i++;
      }

      /* sets last image as current image*/
      *args->image_storage_struct = image2write;
      /* dirty the header and closer of the dma buffer */
      *dcmi_buffer_ctx.img_head_ptr = 0x00;
      *dcmi_buffer_ctx.img_tail_ptr = 0x00;
      /* add here timing check*/
      /* WARN: not sure this works*/
      if (DCMI->MISR) {
        uart_write_buf(USART2, "TOO SLOW\n\r", 10);
      }
    }
    uart_write_buf(USART2, "!!!!! end of routine\n\r", 22);
  }
}

/**
 * ---------------------------------------------------------------------------
 * * IMAGE STORAGE HANDLING METHODS
 * ---------------------------------------------------------------------------
 * *
 */

#define IF_DOWNLOAD
void get_image_storage_status(void *void_args) {
  /* iterate over the static memory and copies the image structures that are
   * found in the ram registry */

  volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
  /*
  struct mspi_interface octospi = args->mspi_interface;
  struct mspi_device mt29 = args->mspi_device;
  struct jpeg_image *image_storage_struct = args->image_storage_struct;
  */

  volatile u32 page_buffer[MT29_PAGE_W_SIZE];

  struct nand_addr page_nand_addr;
  page_nand_addr.column = 0;

  size_t str_s = sizeof(*args->image_storage_struct);

  volatile u32 found_image_n = 0;

// #define SEARCH_BLOCK_LIMIT MT29_MAX_BLOCKn
#define SEARCH_BLOCK_LIMIT 5
  for (int i = 0; i < SEARCH_BLOCK_LIMIT; i++) {
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
  /* XXX: before exiting clear the dcmi_dma_buffer*/
  /* FIX: at the moment does not change the values that are passed as inputs */
}

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
    if (*(check_ptr + i) == 0xFF && *(check_ptr + i + 1) == 0xD8) {
      head_ptr = check_ptr + i;
      ptr_areset |= 1 << 0;
    }
    // get location of the jpeg closer (FFD9)
    if ((check_ptr + i) > head_ptr && ptr_areset == 1) {
      if (*(check_ptr + i) == 0xFF && *(check_ptr + i + 1) == 0xD9) {
        tail_ptr = check_ptr + i;
        ptr_areset |= 1 << 1;
      }
    }
  }
  // get location of the last non dirty memory
  // (to get size of the jpeg image)
  if (ptr_areset == 0b11 && tail_ptr > head_ptr) {
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
      addr.block = 0; // restart from block 1 rather than from block 0
    }
  }
  return addr;
}
