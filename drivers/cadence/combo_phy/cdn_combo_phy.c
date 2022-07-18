/*
 * Copyright (c) 2022-2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/utils.h>

#include "cdn_combo_phy.h"

void temp(handoff *hoff_ptr)
{
	hoff_ptr->peripheral_pwr_gate_array = 0x40;

}

int dfi_select(handoff *hoff_ptr)
{
	uint32_t data = 0;
	handoff reverse_handoff_ptr;

	temp(&reverse_handoff_ptr);

	if (((reverse_handoff_ptr.peripheral_pwr_gate_array) & PERIPHERAL_SDMMC_MASK) ==
		0U) {
		ERROR("SDMMC/NAND is not set properly\n");
		return -ENXIO;
	}

	mmio_setbits_32(SOCFPGA_SYSMGR(DFI_INTF),
		(((reverse_handoff_ptr.peripheral_pwr_gate_array) &
		PERIPHERAL_SDMMC_MASK) >> PERIPHERAL_SDMMC_OFFSET));
	data = mmio_read_32(SOCFPGA_SYSMGR(DFI_INTF));
	if ((data & DFI_INTF_MASK) != (((reverse_handoff_ptr.peripheral_pwr_gate_array) &
		PERIPHERAL_SDMMC_MASK) >> PERIPHERAL_SDMMC_OFFSET)) {
		ERROR("DFI is not set properly\n");
		return -ENXIO;
	}

	return 0;
}

int sdmmc_write_phy_reg(uint32_t phy_reg_addr, uint32_t phy_reg_addr_value,
			uint32_t phy_reg_data, uint32_t phy_reg_data_value)
{
	uint32_t data = 0;
	uint32_t value = 0;

	/* Get PHY register address, write HRS04*/
	value = mmio_read_32(phy_reg_addr);
	value &= ~PHY_REG_ADDR_MASK;
	value |= phy_reg_addr_value;
	mmio_write_32(phy_reg_addr, value);
	data = mmio_read_32(phy_reg_addr);
	if ((data & PHY_REG_ADDR_MASK) != phy_reg_addr_value) {
		ERROR("PHY_REG_ADDR is not set properly\n");
		return -ENXIO;
	}

	/* Get PHY register data, write HRS05 */
	value &= ~PHY_REG_DATA_MASK;
	value |= phy_reg_data_value;
	mmio_write_32(phy_reg_data, value);
	data = mmio_read_32(phy_reg_data);
	if (data != phy_reg_data_value) {
		ERROR("PHY_REG_DATA is not set properly\n");
		return -ENXIO;
	}

	return 0;
}

int sd_card_detect(void)
{
	uint32_t value = 0;

	/* Card detection */
	do {
		value = mmio_read_32(SDMMC_CDN(SRS09));
	/* Wait for card insertion. SRS09.CI = 1 */
	} while ((value & (1 << CI)) == 0);

	if ((value & (1 << CI)) == 0) {
		ERROR("Card does not detect\n");
		return -ENXIO;
	}

	return 0;
}

int emmc_card_reset(void)
{
	uint32_t _status = 0;

	/* Reset embedded card */
	mmio_write_32(SDMMC_CDN(SRS10), (7 << BVS) | (1 << BP) | _status);
	udelay(68000); // ~200us
	udelay(680);
	mmio_write_32(SDMMC_CDN(SRS10), (7 << BVS) | (0 << BP));
	udelay(340); // ~1us

	/* Turn on supply voltage */
	/* BVS = 7, BP = 1, BP2 only in UHS2 mode */
	mmio_write_32(SDMMC_CDN(SRS10), (7 << BVS) | (1 << BP) | _status);

	return 0;
}
