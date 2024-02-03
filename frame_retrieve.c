#include "frame_retrieve.h"

__attribute__((used)) void
image_downloader_hook(volatile struct dcmi_isr_arg *args,
                      volatile struct dcmi_buffer_context buffer_context) {
  /* this is a hook for the gdb image download script
   * must not be optimized
   */
  spin(1);
}

u32 retrieve_image(volatile void *void_args) {

  /* iterate over the static memory and copies the image structures that are
   * found in the ram registry */

  volatile struct dcmi_isr_arg *args = (struct dcmi_isr_arg *)void_args;
  /*read image data from the nand pages and store it in a temporary
   * buffer */

  /*allocate stack temp buffer */
  /* this is terminally huge in stack.... and is allocated even if the
   * fun is not used */
  /* therefore the dma buffer is used */
  /* CAUTION: the function will clear the dma buffer and shall be used
   * only when acquisition has been completed */
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
  volatile struct dcmi_buffer_context buffer_context = {.buffer_head_ptr =
                                                            dcmi_dma_buffer};
  /*run buffer analyze to isolate the imagedata */
  u32 valid = dcmi_buffer_analyze(&buffer_context);

  if (valid) {
    /* whithout this part it is going to be optimized out */
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
