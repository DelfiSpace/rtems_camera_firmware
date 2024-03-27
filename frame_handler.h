#pragma once
#include <rtems.h>
#include <rtems/irq.h>
#include <rtems/rtems/event.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9xx.h>
#include <string.h>

#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_uart.h>

#define MAX_PAGES_IMAGE 64

#define IMAGE_NAND_STR_HEAD 0xF0CACC1A
#define IMAGE_NAND_STR_CLOS 0xFEEDC0DE

#define PROGRAMG_NAND
#define CLEAR_NAND
#define IF_DOWNLOAD

// #define SEARCH_BLOCK_LIMIT MT29_MAX_BLOCKn
#define SEARCH_BLOCK_LIMIT 150

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
  u32 *buffer_tail_ptr;
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

/* main functional methods */
rtems_task DCMI_frame_handler(rtems_task_argument argument);
void get_image_storage_status(void *args);

/* accessory methods */
u32 fast_buffer_analyze(struct dcmi_buffer_context *b_ctx);
u32 dcmi_buffer_analyze(struct dcmi_buffer_context *dcmi_buffer_ctx);
u32 *dcmi_get_buffer_ptr(void);
struct nand_addr get_next_nand_addr(struct nand_addr addr);

/*
 * Interesting for the thesis:
 * have the images stored as a linked list in memory.
 * this way the image metadata contain also a next and previous
 * in terms of nand addresses.
 * It is therefore possible to go more efficiently for a precise image, given a
 * set of assumptions. It could also be inteiesting to analyze the complexity of
 * each of the options.
 *
 */

#ifndef IFP
// static u8 frame_handler_doprint = {0};
static u8 frame_handler_doprint = 1;
#define IFP(codeBlock)                                                         \
  do {                                                                         \
    if (frame_handler_doprint) {                                               \
      codeBlock;                                                               \
    }                                                                          \
  } while (0)
#endif
