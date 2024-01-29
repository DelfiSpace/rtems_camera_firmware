#pragma once
#include <rtems.h>
#include <rtems/irq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>

#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_uart.h>

/* isr prototypes */
rtems_isr DCMI_frame_isr_handler(rtems_vector_number vector);
rtems_status_code register_dcmi_frame_isr(void);

#define MAX_PAGES_IMAGE 10

#define IMAGE_NAND_STR_HEAD 0xF0CACC1A
#define IMAGE_NAND_STR_CLOS 0xFEEDC0DE

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

void dcmi_buffer_analisis(struct jpeg_image *, u32 *);
u32 *dcmi_get_buffer_ptr(void);

void imwrite_storagestatus_get(struct jpeg_image *);
