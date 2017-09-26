/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.

**/

//
// Include files
//

#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bl_common.h>
#include <console.h>
#include <debug.h>
#include <mmio.h>
#include <string.h>
#include <platform.h>
#include <platform_def.h>
#include <platform_private.h>

#include <console_printf.h>
#include "Altera_Hps_Socal.h"
#include "Handoff.h"

#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
int verify_handoff_image (handoff *hoff_ptr, handoff *reverse_hoff_ptr)
{
	int i;
	uint32_t *buffer;


	memcpy (reverse_hoff_ptr, hoff_ptr, sizeof(handoff));
	buffer = (uint32_t*)reverse_hoff_ptr;

	/* convert big indian to little indian */
	for(i = 0; i < sizeof(handoff) / 4; i++) {
		buffer[i] = SWAP_UINT32(buffer[i]);
	}

	/* check for magic header */
	if (reverse_hoff_ptr->header_magic != HANDOFF_MAGIC_HEADER) {
		console_printf("invalid magic header\n");
		return -1;
	}
	/* check for magic pinmux sel header */
	if (reverse_hoff_ptr->pinmux_sel_magic != HANDOFF_MAGIC_PINMUX_SEL)
		return -1;
	/* check for magic pinmux ioctrl header */
	if (reverse_hoff_ptr->pinmux_io_magic != HANDOFF_MAGIC_IOCTLR)
		return -1;
	/* check reverse_hoff_ptrfor magic pinmux fpga header */
	if (reverse_hoff_ptr->pinmux_fpga_magic != HANDOFF_MAGIC_FPGA)
		return -1;
	/* check for magic pinmux io delay header */
	if (reverse_hoff_ptr->pinmux_delay_magic != HANDOFF_MAGIC_IODELAY)
		return -1;
	return 0;
}
