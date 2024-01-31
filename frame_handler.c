#include "frame_handler.h"
#include "stm32l4r9_module_mspi_mt29.h"
#include <stm32l4r9_module_mspi.h>

const u32 image_struct_header[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_struct_closer[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_padding = 0x0;
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
                                      DCMI_frame_isr_handler,   // ISR
                                      &DCMI_frame_isr_arguments // ISR argument
      );
  return status;
}
/* ---- this is the function called as dcmi isr handler---- */
rtems_isr DCMI_frame_isr_handler(void *void_args) {
  struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
  uart_write_buf(USART2, "rtems interrupt received frame  \n\r", 34); // XXX:
  /* dcmi dma buffer from dcmi bsp module */
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_BYTESIZE];
  /* the size of the dma buffer must be larger by at least str_s
   * this is becouse when the image head is in the first bytes of the memory,
   * you will overwrite a memory space that would be before the strict buffer */

#ifdef SKIP

  // FIX: no now what you consider the start of the buffer in the
  // dcmi_buffer_ctx:
  // - It is the start of the actual dcmi dma buffer (so the memory managed by
  // the dma channel)
  // - It needs to be derived from the pointer of the buffer by adding the size
  // of the prebuffer (IMG_METADATA_MAX_BYTESIZE)

  struct dcmi_buffer_context dcmi_buffer_ctx = {
      .buffer_head_ptr = dcmi_dma_buffer + IMG_METADATA_MAX_BYTESIZE};

  /* get properties of the image in the buffer */
  dcmi_buffer_analyze(&dcmi_buffer_ctx);

  struct jpeg_image image_ws = {.id = args->last_image_index + 1,
                                .timestamp = 0};

  /* determine the number of pages necessary to store the image*/
  struct jpeg_image image2write;
  image2write.num_pages = dcmi_buffer_ctx.img_size / MT29_PAGE_SIZE + 1;

  /* get first free page index */
  u32 last_image_page_index =
      args->image_storage_struct[args->last_image_index].num_pages - 1;
  //(from number of used pages to index)

  /* get last used address */
  struct nand_addr last_addr =
      args->image_storage_struct[args->last_image_index]
          .nand_addr[last_image_page_index];

  /* generate nand addresses for the used pages*/
  image2write.nand_addr[0] = get_next_nand_addr(last_addr);
  for (int i = 1; i < image2write.num_pages; i++) {
    image2write.nand_addr[i] = get_next_nand_addr(image2write.nand_addr[i - 1]);
  }
  // FIX: add error/waring in case pages would be more than MAX_PAGES_IMAGE
  // FIX: a routine to handle errors must be implemented

  size_t str_s = sizeof(image2write);
  u8 *overwrite_ptr = dcmi_buffer_ctx.img_head_ptr - str_s;
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
#endif /* ifndef SKIP */

  /* resets peripheral interrupt */
  DCMI->ICR &= ~(DCMI_ICR_FRAME_ISC_Msk);
}

/**
 * --------------------------------------------------------------------------- *
 *       IMAGE STORAGE HANDLING METHODS
 * --------------------------------------------------------------------------- *
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
          memcpy((image_storage_struct + found_image_n), &page_buffer[k + 4],
                 str_s);
          found_image_n++;
        }
      }
    }
  }
}

u32 find_last_image_index(struct jpeg_image *image_storage_struct) {
  /* finds in the ram image structure registry the instance that has the largest
   * id value.
   * determines the first empty nand address after the image in question.
   */
  u32 max_id = 0;
  int max_index = 0;

  for (int i = 0; i < MAX_N_STORABLE_FRAMES; ++i) {
    if (image_storage_struct[i].id > max_id) {
      max_id = image_storage_struct[i].id;
      max_index = i;
    }
  }

  return max_index;
}

// FIX: you need to explore the buffer in a circular manner, becouse otherwhise
// you would always find only the first image, but expecially in jpeg there are
// going to be more
void dcmi_buffer_analyze(struct dcmi_buffer_context *dcmi_buffer_ctx) {
  /*
   * data_ptr is the start of the buffer.
   * the size of the buffer is in a macro
   * you can instead define a buffer object, passed to this function
   */

  u8 *check_ptr = (u8 *)dcmi_buffer_ctx->buffer_head_ptr;
  u8 *head_ptr;
  u8 *tail_ptr;
  // ptr_set: [0]:head, [1]:tail
  u8 ptr_areset = {0};
  u32 img_size = {0};

  int i = 0;
  for (i = 0; i < MAX_DMA_TRS_SIZE * 2; i++) {
    // get location of the jpeg header (FFD8)
    if (*(check_ptr + i) == 0xFF && *(check_ptr + i + 1) == 0xD8)
      head_ptr = check_ptr + i;
    ptr_areset |= 1 << 0;
    // get location of the jpeg closer (FFD9)
    if (*(check_ptr + i) == 0xFF && *(check_ptr + i + 1) == 0xD9)
      tail_ptr = check_ptr + i;
    ptr_areset |= 1 << 1;
  }
  // get location of the last non dirty memory
  // (to get size of the jpeg image)
  if (ptr_areset == 0b11)
    img_size = tail_ptr - head_ptr;
  dcmi_buffer_ctx->img_head_ptr = head_ptr;
  dcmi_buffer_ctx->img_tail_ptr = tail_ptr;
  dcmi_buffer_ctx->img_size = img_size;
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

u32 retrieve_image(struct dcmi_isr_arg isr_ctx,
                   struct jpeg_image *image_storage_struct) {
  /*read image data from the nand pages and store it in a temporary buffer */

  /*allocate stack temp buffer */
  /* this is terminally huge in stack.... and is allocated even if the fun is
   * not used */
  /* therefore the dma buffer is used */
  /* CAUTION: the function will clear the dma buffer and shall be used only when
   * acquisition has been completed */
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_BYTESIZE];
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
