/*
 * Copyright (c) 2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* system header files*/
#include <assert.h>
#include <endian.h>
#include <string.h>

/* Cadense qspi driver*/
#include <qspi/cadence_qspi.h>

/* CRC function header */
#include <common/tf_crc32.h>

/* Mailbox driver*/
#include <socfpga_mailbox.h>

#include <socfpga_ros.h>

static uint32_t get_spt(spt_table_t *spt_buf);
static uint32_t load_and_check_spt(spt_table_t *spt_buf, size_t offset);
static uint32_t get_current_image_index(spt_table_t *spt_buf, uint32_t *i);
static void swap_bits(char *data, int len);

spt_table_t spt;

uint32_t ros_qspi_get_ssbl_offset(unsigned long *offset)
{
	if (offset == NULL) {
		return ROS_RET_INVALID;
	}

	uint32_t ret, img_index;
	char ssbl_name[SPT_PARTITION_NAME_LENGTH];

	ret = get_spt(&spt);
	if (ret != ROS_RET_OK) {
		return ret;
	}

	ret = get_current_image_index(&spt, &img_index);
	if (ret != ROS_RET_OK) {
		return ret;
	}

	if (strncmp(spt.partition[img_index].name, FACTORY_IMAGE, SPT_PARTITION_NAME_LENGTH) == 0) {
		strlcpy(ssbl_name, FACTORY_SSBL, SPT_PARTITION_NAME_LENGTH);
	} else {
		strlcpy(ssbl_name, spt.partition[img_index].name, SPT_PARTITION_NAME_LENGTH);
		strlcat(ssbl_name, SSBL_PREFIX, SPT_PARTITION_NAME_LENGTH);
	}

	for (uint32_t index = 0; index < spt.partitions; index++) {
		if (strncmp(spt.partition[index].name, ssbl_name, SPT_PARTITION_NAME_LENGTH) == 0) {
			*offset = spt.partition[index].offset;
			NOTICE("ROS: Corresponding SSBL is at 0x%08lx\n", *offset);
			return ROS_RET_OK;
		}
	}

	return ROS_IMAGE_INDEX_ERR;
}

static uint32_t get_spt(spt_table_t *spt_buf)
{
	if (spt_buf == NULL) {
		return ROS_RET_INVALID;
	}

	uint32_t ret;
	uint32_t spt_offset[RSU_GET_SPT_RESP_SIZE];

	/* Get SPT offset from SDM via mailbox commands */
	ret = mailbox_rsu_get_spt_offset(spt_offset, RSU_GET_SPT_RESP_SIZE);
	if (ret != MBOX_RET_OK) {
		WARN("ROS: Not booted in RSU mode\n");
		return ROS_RET_NOT_RSU_MODE;
	}

	/* Print the SPT table addresses */
	VERBOSE("ROS: SPT0 0x%08lx\n", ADDR_64(spt_offset[0], spt_offset[1]));
	VERBOSE("ROS: SPT1 0x%08lx\n", ADDR_64(spt_offset[2], spt_offset[3]));

	/* Load and validate SPT0*/
	ret = load_and_check_spt(spt_buf, ADDR_64(spt_offset[2], spt_offset[3]));
	if (ret != ROS_RET_OK) {
		/* Load and validate SPT1*/
		ret = load_and_check_spt(spt_buf, ADDR_64(spt_offset[0], spt_offset[1]));
		if (ret != ROS_RET_OK) {
			WARN("Both SPT tables are unusable\n");
			return ret;
		}
	}

	return ROS_RET_OK;
}

static uint32_t get_current_image_index(spt_table_t *spt_buf, uint32_t *img_index)
{
	if (spt_buf == NULL || img_index == NULL) {
		return ROS_RET_INVALID;
	}

	uint32_t ret;
	unsigned long current_image;
	uint32_t rsu_status[RSU_STATUS_RES_SIZE];

	ret = mailbox_rsu_status(rsu_status, RSU_STATUS_RES_SIZE);
	if (ret != MBOX_RET_OK) {
		return ROS_RET_NOT_RSU_MODE;
	}

	current_image = ADDR_64(rsu_status[1], rsu_status[0]);
	NOTICE("ROS: Current image is at 0x%08lx\n", current_image);

	*img_index = 0;
	for (uint32_t index = 0 ; index < spt.partitions; index++) {
		if (spt_buf->partition[index].offset == current_image) {
			*img_index = index;
			break;
		}
	}

	if (*img_index == 0) {
		return ROS_IMAGE_INDEX_ERR;
	}

	return ROS_RET_OK;
}

static uint32_t load_and_check_spt(spt_table_t *spt_ptr, size_t offset)
{

	if (spt_ptr == NULL || offset == 0) {
		return ROS_RET_INVALID;
	}

	int ret;
	uint32_t calc_crc;
	static spt_table_t spt_data;

	ret = cad_qspi_read(spt_ptr, offset, SPT_SIZE);
	if (ret != 0) {
		return ROS_QSPI_READ_ERROR;
	}

	if (spt_ptr->magic_number != SPT_MAGIC_NUMBER) {
		return ROS_SPT_BAD_MAGIC_NUM;
	}

	memcpy(&spt_data, spt_ptr, SPT_SIZE);
	spt_data.checksum = 0;
	swap_bits((char *)&spt_data, SPT_SIZE);

	calc_crc = tf_crc32(0, (uint8_t *)&spt_data, SPT_SIZE);
	if (bswap32(spt_ptr->checksum) != calc_crc) {
		return ROS_SPT_CRC_ERROR;
	}

	NOTICE("ROS: SPT table at 0x%08lx is verified\n", offset);
	return ROS_RET_OK;
}

static void swap_bits(char *data, int len)
{
	int x, y;
	char tmp;

	for (x = 0; x < len; x++) {
		tmp = 0;
		for (y = 0; y < 8; y++) {
			tmp <<= 1;
			if (data[x] & 1) {
				tmp |= 1;
			}
			data[x] >>= 1;
		}
		data[x] = tmp;
	}
}
