#include "frame_retrieve.h"
#include <stm32l4r9_module_dcmi.h>

__attribute__((used)) void
image_downloader_hook(volatile struct dcmi_isr_arg *args,
                      volatile struct dcmi_buffer_context buffer_context) {
  /* this is a hook for the gdb image download script
   * must not be optimized
   */
  spin(1);
}

u32 retrieve_image(volatile void *void_args) {
  /* iterate over the static memory and if image metadata is found, retrieve
   * the relative image data and provide an hook for the gdb script to download
   * from the buffer */

  volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
  extern uint32_t dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE];
  /* clear buffer */
  memset(dcmi_dma_buffer, 0x0, sizeof(dcmi_dma_buffer));

  u32 data_padding[10];
  u32 *circ_ptr = dcmi_dma_buffer;
  /*copy in buffer all the pages content*/
  for (int i = 0; i < args->image_storage_struct->num_pages; i++) {

    args->mspi_interface.data_ptr = data_padding;
    mspi_transfer(args->mspi_interface, args->mspi_device.page_read_from_nand,
                  &(args->image_storage_struct->nand_addr[i]));

    args->mspi_interface.data_ptr = data_padding;
    mspi_transfer(args->mspi_interface, args->mspi_device.wait_oip,
                  &(args->image_storage_struct->nand_addr[i]));

    args->mspi_interface.data_ptr = circ_ptr;
    mspi_transfer(args->mspi_interface,
                  args->mspi_device.page_read_from_cache_QUAD,
                  &(args->image_storage_struct->nand_addr[i]));

    circ_ptr += MT29_PAGE_W_SIZE;
  }

  /* generate context for the buffer buffer */
  // TODO: transform in a constructor
  struct dcmi_buffer_context buffer_context = {0};
  buffer_context.buffer_head_ptr = dcmi_dma_buffer;
  buffer_context.buffer_tail_ptr =
      &dcmi_dma_buffer[MAX_DMA_TRS_SIZE + IMG_METADATA_MAX_WSIZE - 1];
  buffer_context.buff_current_circ_ptr = (u8 *)buffer_context.buffer_head_ptr;
  /*run buffer analyze to isolate the imagedata */
  u32 valid = dcmi_buffer_analyze(&buffer_context);

  if (valid) {
    /* note: whithout the print part it is going to be optimized out (dead code
     * optimization)*/
    char temp_str[100];
    int n;
    n = sprintf(temp_str,
                "image downloading ptr %x, %x, %x (head/tail/size)\r\n",
                buffer_context.img_head_ptr, buffer_context.img_tail_ptr,
                buffer_context.img_size);
    uart_write_buf(USART2, temp_str, n);
    image_downloader_hook(args, buffer_context);
    return 1;
  } else {
    return 0;
  }
  return 0;
}
