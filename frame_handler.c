#include "frame_handler.h"
#include "stm32l4r9_module_mspi_mt29.h"

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

  // Register the ISR
  status =
      rtems_interrupt_handler_install(dcmi_interrupt_vector_n, // Vector number
                                      "DCMI_frame", // Name for the ISR
                                      RTEMS_INTERRUPT_UNIQUE, // Flags
                                      DCMI_frame_isr_handler, // ISR
                                      NULL                    // ISR argument
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

// FIX: you need to explore the buffer in a circular manner, becouse otherwhise
// you would always find only the first image
// probably you need to create a dcmi_buffer_context object
//
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

/* ---- ---- */
struct jpeg_image imwrite_struct_gen(void) {
  /* from dcmi bsp module */
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE];

  struct dcmi_buffer_context dcmi_buffer_ctx;
  dcmi_buffer_ctx.buffer_head_ptr = dcmi_dma_buffer;

  /* obtain image in buffer characteristics */
  dcmi_buffer_analyze(&dcmi_buffer_ctx);

  /* getting status from reading the memory */
  /* you should do this before enabling the acuisition, during application
   * initialization */
  // structure to yeet to the nand
  struct jpeg_image image_ws;
  image_ws.id = 0;        /* todo */
  image_ws.timestamp = 0; /* todo */
}

void imwrite_storagestatus_get(struct jpeg_image *image_storage_registry) {
  /* iteratively:
   * make a flowchart
   */
}

#ifdef IGNORE

void nand_write(struct jpeg_image, u8 *head_ptr, u32 num);
void image_allocate(void);

void image_allocate(void) {
  /* analyzes the buffer to find the relevant data pointers
   * fills the image structure with the nand organization information
   */
}

/* NOTE: you need to generate a structure that contains the image and write all
 * the structure to the memory in an agnostic way
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
