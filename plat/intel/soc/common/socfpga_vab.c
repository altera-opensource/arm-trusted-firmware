/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2019-2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <common/debug.h>
#include <common/tbbr/tbbr_img_def.h>
#include <drivers/delay_timer.h>
#include <errno.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <tools_share/firmware_image_package.h>

#include "socfpga_mailbox.h"
#include "socfpga_vab.h"

/**************************************************
*
* Fixme: Still need to create code for VAB calling
*
**************************************************/


static size_t get_img_size(uint8_t *img_buf, size_t img_buf_sz)
{
	uint8_t *img_buf_end = img_buf + img_buf_sz;
	uint32_t cert_sz = get_unaligned_le32(img_buf_end - sizeof(uint32_t));
	uint8_t *p = img_buf_end - cert_sz - sizeof(uint32_t);

	/* Ensure p is pointing within the img_buf */
	if (p < img_buf || p > (img_buf_end - VAB_CERT_HEADER_SIZE))
		return 0;

	// if (get_unaligned_le32(p) == SDM_CERT_MAGIC_NUM)
		// return (size_t)(p - img_buf);

	return 0;
}



int socfpga_vendor_authentication(void **p_image, size_t *p_size)
{
	int retry_count = 20;
	uint8_t hash384[FCS_SHA384_WORD_SIZE];
	uint64_t img_addr, mbox_data_addr;
	uint32_t img_sz, mbox_data_sz;
	uint8_t *cert_hash_ptr, *mbox_relocate_data_addr;
	uint32_t resp = 0, resp_len = 1;
	int ret = 0;

	img_addr = (uintptr_t)*p_image;
	img_sz = get_img_size((uint8_t *)img_addr, *p_size);

	if (!img_sz) {
		NOTICE("VAB certificate not found in image!\n");
		return -ENOENT;
	}

	if (!IS_BYTE_ALIGNED(img_sz, sizeof(uint32_t))) {
		NOTICE("Image size (%d bytes) not aliged to 4 bytes!\n",img_sz);
		return -ENOEXEC;
	}

	/* Generate HASH384 from the image */
	sha384_csum_wd((uint8_t *)img_addr, img_sz, hash384, CHUNKSZ_PER_WD_RESET);
	cert_hash_ptr = (uint8_t *)(img_addr + img_sz + VAB_CERT_MAGIC_OFFSET + VAB_CERT_FIT_SHA384_OFFSET);

	/*
	 * Compare the SHA384 found in certificate against the SHA384
	 * calculated from image
	 */
	if (memcmp(hash384, cert_hash_ptr, FCS_SHA384_WORD_SIZE)) {
		NOTICE("SHA384 does not match!\n");
		return -EKEYREJECTED;
	}


	mbox_data_addr = img_addr + img_sz - sizeof(uint32_t);
	/* Size in word (32bits) */
	mbox_data_sz = (BYTE_ALIGN(*p_size - img_sz, sizeof(uint32_t))) >> 2;

	NOTICE("mbox_data_addr = %lx    mbox_data_sz = %d\n",mbox_data_addr, mbox_data_sz);


	memcpy(mbox_relocate_data_addr, (uint8_t *)mbox_data_addr, mbox_data_sz * sizeof(uint32_t));
	*(uint32_t *)mbox_relocate_data_addr = 0;

	do {
		/* Invoke SMC call to ATF to send the VAB certificate to SDM */
		ret  = mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_VAB_SRC_CERT, (uint32_t *)mbox_relocate_data_addr, mbox_data_sz, 0, &resp, &resp_len);

		/* If SDM is not available, just delay 50ms and retry again */
		if (ret == MBOX_RESP_ERR(0x1FF)) {
			mdelay(50);
		}
		else {
			break;
		}
	} while (--retry_count);

	/* Free the relocate certificate memory space */
	zeromem((void *)&mbox_relocate_data_addr, sizeof(uint32_t));


	/* Exclude the size of the VAB certificate from image size */
	*p_size = img_sz;

	if (ret) {
		/*
		 * Unsupported mailbox command or device not in the
		 * owned/secure state
		 */
		if (ret == MBOX_RESP_ERR(0x85)) {
			/* SDM bypass authentication */
			NOTICE("%s 0x%lx (%d bytes)\n","Image Authentication bypassed at address\n",img_addr, img_sz);
			return 0;
		}
		NOTICE("VAB certificate authentication failed in SDM\n");

		if (ret == MBOX_RESP_ERR(0x1FF)) {
			NOTICE("Operation timed out\n");
			return -ETIMEDOUT;
		} else if (ret == MBOX_WRONG_ID) {
			NOTICE("No such process\n");
			return -ESRCH;
		}
		return -EKEYREJECTED;
	} else {
		/* If Certificate Process Status has error */
		if (resp) {
			NOTICE("VAB certificate execution format error\n");
			return -ENOEXEC;
		}
	}

	NOTICE("%s 0x%lx (%d bytes)\n","Image Authentication passed at address", img_addr, img_sz);
	return ret;

}

static uint32_t get_unaligned_le32(const void *p)
{
	return 0;
}

void sha384_csum_wd(const unsigned char *input, unsigned int ilen,
		unsigned char *output, unsigned int chunk_sz)
{
#if 0
	sha512_context ctx;
#if defined(CONFIG_HW_WATCHDOG) || defined(CONFIG_WATCHDOG)
	const unsigned char *end;
	unsigned char *curr;
	int chunk;
#endif

	sha384_starts(&ctx);

#if defined(CONFIG_HW_WATCHDOG) || defined(CONFIG_WATCHDOG)
	curr = (unsigned char *)input;
	end = input + ilen;
	while (curr < end) {
		chunk = end - curr;
		if (chunk > chunk_sz)
			chunk = chunk_sz;
		sha384_update(&ctx, curr, chunk);
		curr += chunk;
		WATCHDOG_RESET();
	}
#else
	sha384_update(&ctx, input, ilen);
#endif

	sha384_finish(&ctx, output);
#endif
}