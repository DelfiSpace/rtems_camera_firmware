#include "frame_handler.h"
#include "stm32l4r9_module_mspi_mt29.h"
#include <stm32l4r9_module_mspi.h>

const u32 image_struct_header[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_struct_closer[4] = {IMAGE_NAND_STR_HEAD};
const u32 image_padding = 0x0;
/**
 * --------------------------------------------------------------------------- *
 *       INTERRUPTS
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

// Implementation of the ISR
rtems_isr DCMI_frame_isr_handler(rtems_vector_number vector) {
  uart_write_buf(USART2, "rtems interrupt received frame  \n\r", 34); // XXX:

  /* write test buffer to the memory */
  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* TODO: add here data handling procedure */

  /* read buffer from memory */
  DCMI->ICR &= ~(DCMI_ICR_FRAME_ISC_Msk);
}

/**
 * --------------------------------------------------------------------------- *
 *       IMAGE STORAGE METHODS
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

/* ---- this is the function called as dcmi isr handler---- */
struct jpeg_image generate_image_structure(struct dcmi_isr_arg arg) {

  /* dcmi dma buffer from dcmi bsp module */
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE];

  struct dcmi_buffer_context dcmi_buffer_ctx = {.buffer_head_ptr =
                                                    dcmi_dma_buffer};

  /* get properties of the image in the buffer */
  dcmi_buffer_analyze(&dcmi_buffer_ctx);

  struct jpeg_image image_ws = {.id = arg.last_image_index + 1, .timestamp = 0};

  /* determine the number of pages necessary to store the image*/
  struct jpeg_image image2write;
  image2write.num_pages = dcmi_buffer_ctx.img_size / MT29_PAGE_SIZE + 1;

  /* get first free page index */
  u32 last_image_page_index =
      arg.image_storage_struct[arg.last_image_index].num_pages - 1;
  //(from number of used pages to index)

  /* get last used address */
  struct nand_addr last_addr = arg.image_storage_struct[arg.last_image_index]
                                   .nand_addr[last_image_page_index];

  /* generate nand addresses for the used pages*/
  image2write.nand_addr[0] = get_next_nand_addr(last_addr);
  for (int i = 1; i < image2write.num_pages; i++) {
    image2write.nand_addr[i] = get_next_nand_addr(image2write.nand_addr[i - 1]);
  }
  // FIX: add error/waring in case pages would be more than MAX_PAGES_IMAGE

  /* overwrite the dcmi_nand buffer with the structure information */

  /* prepare the pointer to pass to the nand write fuction */

  /* write pages to the respective nand addressess */

  /* profit */
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
#ifdef IGNORE

void nand_write(struct jpeg_image, u8 *head_ptr, u32 num);
void image_allocate(void);

void image_allocate(void) {
  /* analyzes the buffer to find the relevant data pointers
   * fills the image structure with the nand organization information
   */
}

/* NOTE: you need to generate a structure that contains the image and write
 * all the structure to the memory in an agnostic way
 */

void image_write(struct jpeg_image img, u8 *head_ptr, u32 num) {
  /* num is the number of bytes to be transferred */
  struct mspi_device mt29 = mspi_device_constr();
  u8 *tail_ptr = head_ptr + num;

  struct mspi_interface octospi1;
  octospi1.interface_select = 0x01;

  octospi1.data_ptr = (u32 *)head_ptr;

  while (head_ptr + MT29_PAGE_SIZE < tail_ptr) {
    // TODO: set data pointer to head_ptr in device_context
    mspi_transfer(octospi1, mt29.page_load_QUAD, nand_address);
    head_ptr = head_ptr + MT29_PAGE_W_SIZE;
  }
  /* add zeros to the buffer, intentional buffer overrite */
  memset(head_ptr, 0x0, tail_ptr - head_ptr);
  /* perform last transfer */
  mspi_transfer(octospi1, mt29.page_load_QUAD, nand_address);
}

/* you can define a priori the addressess that you need to write to
 *
 * i think this is application code, therefore it does not need to support
 * generalities
 *
 * Lets generate the device context in the
 *
 * you can
 * */

#endif /* ifdef IGNORE                                                         \
        */
