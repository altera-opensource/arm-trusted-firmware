/*
 * Copyright (c) 2020, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SOCFPGA_FCS_H
#define SOCFPGA_FCS_H

/* FCS Definitions */

#define FCS_RANDOM_WORD_SIZE		8U
#define FCS_PROV_DATA_WORD_SIZE		44U

#define FCS_RANDOM_BYTE_SIZE		FCS_RANDOM_WORD_SIZE * 4U
#define FCS_PROV_DATA_BYTE_SIZE		FCS_PROV_DATA_WORD_SIZE * 4U

#define FCS_MODE_DECRYPT		0x0
#define FCS_MODE_ENCRYPT		0x1
#define FCS_ENCRYPTION_DATA_0		0x10100
#define FCS_DECRYPTION_DATA_0		0x10102
#define FCS_OWNER_ID_OFFSET		0xC

#define PSGSIGMA_TEARDOWN_MAGIC		0xB852E2A4
#define	PSGSIGMA_SESSION_ID_ONE		0x1
#define PSGSIGMA_UNKNOWN_SESSION	0xFFFFFFFF

#define	RESERVED_AS_ZERO		0x0
/* FCS Single cert */

#define FCS_BIG_CNTR_SEL		0x1

#define FCS_SVN_CNTR_0_SEL		0x2
#define FCS_SVN_CNTR_1_SEL		0x3
#define FCS_SVN_CNTR_2_SEL		0x4
#define FCS_SVN_CNTR_3_SEL		0x5

#define FCS_BIG_CNTR_VAL_MAX		495U
#define FCS_SVN_CNTR_VAL_MAX		64U

/* FCS Payload Structure */

typedef struct fcs_encrypt_payload_t {
	uint32_t first_word;
	uint32_t src_addr;
	uint32_t src_size;
	uint32_t dst_addr;
	uint32_t dst_size;
} fcs_encrypt_payload;

typedef struct fcs_decrypt_payload_t {
	uint32_t first_word;
	uint32_t owner_id[2];
	uint32_t src_addr;
	uint32_t src_size;
	uint32_t dst_addr;
	uint32_t dst_size;
} fcs_decrypt_payload;

typedef struct psgsigma_teardown_msg_t {
	uint32_t reserved_word;
	uint32_t magic_word;
	uint32_t session_id;
} psgsigma_teardown_msg;

typedef struct fcs_cntr_set_preauth_payload_t {
	uint32_t first_word;
	uint32_t counter_value;
} fcs_cntr_set_preauth_payload;

/* Functions Definitions */

uint32_t intel_fcs_random_number_gen(uint64_t addr, uint64_t *ret_size,
				uint32_t *mbox_error);
uint32_t intel_fcs_send_cert(uint64_t addr, uint64_t size,
				uint32_t *send_id);
uint32_t intel_fcs_get_provision_data(uint64_t addr, uint64_t *fcs_size,
				uint32_t *mbox_error);
uint32_t intel_fcs_cntr_set_preauth(uint8_t counter_type,
				int32_t counter_value,
				uint32_t test_bit,
				uint32_t *mbox_error);
uint32_t intel_fcs_encryption(uint32_t src_addr, uint32_t src_size,
				uint32_t dst_addr, uint32_t dst_size,
				uint32_t *send_id);

uint32_t intel_fcs_decryption(uint32_t src_addr, uint32_t src_size,
				uint32_t dst_addr, uint32_t dst_size,
				uint32_t *send_id);

int intel_fcs_sigma_teardown(uint32_t session_id, uint32_t *mbox_error);
int intel_fcs_chip_id(uint32_t *id_low, uint32_t *id_high, uint32_t *mbox_error);
int intel_fcs_attestation_subkey(uint64_t src_addr, uint32_t src_size,
				uint64_t dst_addr, uint32_t *dst_size,
				uint32_t *mbox_error);
int intel_fcs_get_measurement(uint64_t src_addr, uint32_t src_size,
				uint64_t dst_addr, uint32_t *dst_size,
				uint32_t *mbox_error);

#endif /* SOCFPGA_FCS_H */
