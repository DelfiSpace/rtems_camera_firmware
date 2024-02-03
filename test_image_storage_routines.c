#include "test_image_storage_routines.h"

/* local function declarations */
void imst_test_case_1(void);

void image_storage_test_routines(void) { imst_test_case_1(); }

/* test case 1:
 * objective: get latest image metadata from static memory
 *            get image jpeg data from memory
 *            use the debug interface to download the image
 */

void imst_test_case_1(void) {
  struct jpeg_image latest_image;
  struct mspi_interface octospi1 = {.interface_select = 0x01};
  struct mspi_device mt29 = mspi_device_constr();

  struct dcmi_isr_arg image_storage_context = {.image_storage_struct =
                                                   &latest_image,
                                               .mspi_interface = octospi1,
                                               .mspi_device = mt29};
  get_image_storage_status(&image_storage_context);
  // latest_image does now contain the metadata of the latest image
  retrieve_image(&image_storage_context);
}
