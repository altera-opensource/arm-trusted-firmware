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
#include <drivers/cadence/cdns_combo_phy.h>
#include <drivers/cadence/cdns_sdmmc.h>
#include <lib/mmio.h>
#include <lib/utils.h>

#include "agilex5_pinmux.h"

extern struct cdns_sdmmc_combo_phy sdmmc_combo_phy_reg;
extern struct cdns_sdmmc_sdhc sdmmc_sdhc_reg;

/* Will romove once ATF driver is developed */
void sdmmc_pin_config(void)
{
	/* temp use base + addr. Official must change to common method */
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x00, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x04, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x08, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x0C, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x10, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x14, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x18, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x1C, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x20, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x24, 0x0);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x28, 0x0);
}

int sdmmc_init(handoff *hoff_ptr)
{
	int result = 0;

	/* SDMMC pin mux configuration */
	sdmmc_pin_config();
	cdns_set_sdmmc_var(&sdmmc_combo_phy_reg, &sdmmc_sdhc_reg);
	result = cdns_sd_host_init(&sdmmc_combo_phy_reg, &sdmmc_sdhc_reg);

	return result;
}





