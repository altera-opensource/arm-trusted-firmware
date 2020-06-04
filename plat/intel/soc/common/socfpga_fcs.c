/*
 * Copyright (c) 2020, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <lib/mmio.h>

#include "socfpga_fcs.h"
#include "socfpga_mailbox.h"
#include "socfpga_sip_svc.h"

static bool is_size_4_bytes_aligned(uint32_t size)
{
	if (size % MBOX_WORD_BYTE) {
		return false;
	} else {
		return true;
	}
}

uint32_t intel_fcs_random_number_gen(uint64_t addr, uint64_t *ret_size,
					uint32_t *mbox_error)
{
	int status;
	unsigned int i;
	uint32_t random_data[FCS_RANDOM_WORD_SIZE] = {0U};

	if (!is_address_in_ddr_range(addr, FCS_RANDOM_BYTE_SIZE)) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	status = mailbox_send_cmd(MBOX_JOB_ID, MBOX_FCS_RANDOM_GEN, NULL, 0U,
			CMD_CASUAL, random_data, FCS_RANDOM_WORD_SIZE);

	if (status < 0) {
		*mbox_error = -status;
		return INTEL_SIP_SMC_STATUS_ERROR;
	}

	if (status != FCS_RANDOM_WORD_SIZE) {
		*mbox_error = ~0U;
		return INTEL_SIP_SMC_STATUS_ERROR;
	}

	*ret_size = FCS_RANDOM_BYTE_SIZE;

	for (i = 0U; i < FCS_RANDOM_WORD_SIZE; i++) {
		mmio_write_32(addr, random_data[i]);
		addr += MBOX_WORD_BYTE;
	}

	return INTEL_SIP_SMC_STATUS_OK;
}

uint32_t intel_fcs_send_cert(uint64_t addr, uint64_t size,
					uint32_t *send_id)
{
	int status;

	if (!is_address_in_ddr_range(addr, size)) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	if (!is_size_4_bytes_aligned(size)) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	status = mailbox_send_cmd_async(send_id, MBOX_CMD_VAB_SRC_CERT,
				(uint32_t *)addr, size / MBOX_WORD_BYTE,
				CMD_DIRECT);

	if (status < 0) {
		return INTEL_SIP_SMC_STATUS_ERROR;
	}

	return INTEL_SIP_SMC_STATUS_OK;
}

uint32_t intel_fcs_get_provision_data(uint64_t addr, uint64_t *ret_size,
					uint32_t *mbox_error)
{
	int status;
	unsigned int i;
	uint32_t provision_data[FCS_PROV_DATA_WORD_SIZE] = {0U};

	if (!is_address_in_ddr_range(addr, FCS_PROV_DATA_BYTE_SIZE)) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	status = mailbox_send_cmd(MBOX_JOB_ID, MBOX_FCS_GET_PROVISION, NULL, 0U,
			CMD_CASUAL, provision_data, FCS_PROV_DATA_WORD_SIZE);

	if (status < 0) {
		*mbox_error = -status;
		return status;
	}

	*ret_size = status * MBOX_WORD_BYTE;

	for (i = 0U; i < status; i++) {
		mmio_write_32(addr, provision_data[i]);
		addr += MBOX_WORD_BYTE;
	}

	return INTEL_SIP_SMC_STATUS_OK;
}

uint32_t intel_fcs_cryption(uint32_t mode, uint32_t src_addr,
		uint32_t src_size, uint32_t dst_addr,
		uint32_t dst_size, uint32_t *send_id)
{
	int status;
	uint32_t cmd;

	fcs_crypt_payload payload = {
		FCS_CRYPTION_DATA_0,
		src_addr,
		src_size,
		dst_addr,
		dst_size };

	if (!is_address_in_ddr_range(src_addr, src_size) ||
		!is_address_in_ddr_range(dst_addr, dst_size)) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	if (!is_size_4_bytes_aligned(sizeof(fcs_crypt_payload))) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	if (mode) {
		cmd = MBOX_FCS_ENCRYPT_REQ;
	} else {
		cmd = MBOX_FCS_DECRYPT_REQ;
	}

	status = mailbox_send_cmd_async(send_id, cmd, (uint32_t *) &payload,
				sizeof(fcs_crypt_payload) / MBOX_WORD_BYTE,
				CMD_INDIRECT);
	inv_dcache_range(dst_addr, dst_size);

	if (status < 0) {
		return INTEL_SIP_SMC_STATUS_REJECTED;
	}

	return INTEL_SIP_SMC_STATUS_OK;
}
