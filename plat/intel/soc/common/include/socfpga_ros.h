/*
 * Copyright (c) 2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SOCFPGA_ROS_H
#define SOCFPGA_ROS_H

#include <arch_helpers.h>
#include <lib/utils_def.h>

/** status response*/
#define ROS_RET_OK		(0x00U)
#define ROS_RET_INVALID		(0x01U)
#define ROS_RET_NOT_RSU_MODE	(0x02U)
#define ROS_QSPI_READ_ERROR	(0x03U)
#define ROS_SPT_BAD_MAGIC_NUM	(0x04U)
#define ROS_SPT_CRC_ERROR	(0x05U)
#define ROS_IMAGE_INDEX_ERR	(0x06U)

#define ADDR_64(h, l)			(((((unsigned long)(h)) & 0xffffffff) << 32) | \
						(((unsigned long)(l)) & 0xffffffff))

#define RSU_GET_SPT_RESP_SIZE		(4U)

#define RSU_STATUS_RES_SIZE		(9U)

#define SPT_MAGIC_NUMBER		(0x57713427U)
#define SPT_VERSION			(0U)
#define SPT_FLAG_RESERVED		(1U)
#define SPT_FLAG_READONLY		(2U)

#define SPT_MAX_PARTITIONS		(127U)
#define SPT_PARTITION_NAME_LENGTH	(16U)
#define SPT_RSVD_LENGTH			(4U)
#define SPT_SIZE			(4096U)

#define FACTORY_IMAGE			"FACTORY_IMAGE"
#define FACTORY_SSBL			"FACTORY_IM.SSBL"
#define SSBL_PREFIX			".SSBL"

typedef struct {
	uint32_t magic_number;
	uint32_t version;
	uint32_t partitions;
	uint32_t checksum;
	uint32_t __RSVD[SPT_RSVD_LENGTH];

	struct {
		char name[SPT_PARTITION_NAME_LENGTH];
		uint64_t offset;
		uint32_t length;
		uint32_t flags;
	} partition[SPT_MAX_PARTITIONS];
} __packed spt_table_t;

uint32_t ros_qspi_get_ssbl_offset(unsigned long *offset);

#endif /* SOCFPGA_ROS_H */
