#pragma once
#include <rtems.h>
#include <rtems/irq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>

#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_uart.h>

#define MAX_PAGES_IMAGE 20

#define IMAGE_NAND_STR_HEAD 0xF0CACC1A
#define IMAGE_NAND_STR_CLOS 0xFEEDC0DE

/* this was wayy to much to fit in ram
 * -> do a report of the initial implementaton and than the following
 *  solution
 *  XXX: Important fot the thesis
 */
#define MAX_N_STORABLE_FRAMES 1

struct dcmi_buffer_context {
  u8 *img_head_ptr;
  u8 *img_tail_ptr;
  size_t img_size;

  u8 *buff_current_circ_ptr;
  u32 *buffer_head_ptr;
};

struct jpeg_image {

  u32 id;
  u32 timestamp;

  size_t num_pages;
  struct nand_addr nand_addr[MAX_PAGES_IMAGE];
};

struct dcmi_isr_arg {
  struct jpeg_image *image_storage_struct;
  u32 last_image_index;
  struct mspi_interface mspi_interface;
  struct mspi_device mspi_device;
};

/* isr prototypes */
rtems_status_code register_dcmi_frame_isr(void);
rtems_isr DCMI_frame_isr(void *arg);
rtems_task DCMI_frame_handler(rtems_task_argument argument);

u32 dcmi_buffer_analyze(struct dcmi_buffer_context *dcmi_buffer_ctx);
u32 *dcmi_get_buffer_ptr(void);

void get_image_storage_status(struct mspi_interface octospi,
                              struct mspi_device mt29,
                              struct jpeg_image *image_storage_struct);

u32 find_last_image_index(struct jpeg_image *image_storage_struct);
struct nand_addr get_next_nand_addr(struct nand_addr addr);