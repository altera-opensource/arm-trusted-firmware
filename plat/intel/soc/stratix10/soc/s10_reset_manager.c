/*
 * Copyright (c) 2019, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <arch_helpers.h>
#include <assert.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <drivers/console.h>
#include <errno.h>
#include <lib/mmio.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include "s10_mailbox.h"
#include "s10_reset_manager.h"
#include "s10_system_manager.h"

void deassert_peripheral_reset(void)
{
	mmio_clrbits_32(S10_RSTMGR_PER1MODRST,
			S10_RSTMGR_PER1MODRST_WATCHDOG0 |
			S10_RSTMGR_PER1MODRST_WATCHDOG1 |
			S10_RSTMGR_PER1MODRST_WATCHDOG2 |
			S10_RSTMGR_PER1MODRST_WATCHDOG3 |
			S10_RSTMGR_PER1MODRST_L4SYSTIMER0 |
			S10_RSTMGR_PER1MODRST_L4SYSTIMER1 |
			S10_RSTMGR_PER1MODRST_SPTIMER0 |
			S10_RSTMGR_PER1MODRST_SPTIMER1 |
			S10_RSTMGR_PER1MODRST_I2C0 |
			S10_RSTMGR_PER1MODRST_I2C1 |
			S10_RSTMGR_PER1MODRST_I2C2 |
			S10_RSTMGR_PER1MODRST_I2C3 |
			S10_RSTMGR_PER1MODRST_I2C4 |
			S10_RSTMGR_PER1MODRST_UART0 |
			S10_RSTMGR_PER1MODRST_UART1 |
			S10_RSTMGR_PER1MODRST_GPIO0 |
			S10_RSTMGR_PER1MODRST_GPIO1);

	mmio_clrbits_32(S10_RSTMGR_PER0MODRST,
			S10_RSTMGR_PER0MODRST_EMAC0OCP |
			S10_RSTMGR_PER0MODRST_EMAC1OCP |
			S10_RSTMGR_PER0MODRST_EMAC2OCP |
			S10_RSTMGR_PER0MODRST_USB0OCP |
			S10_RSTMGR_PER0MODRST_USB1OCP |
			S10_RSTMGR_PER0MODRST_NANDOCP |
			S10_RSTMGR_PER0MODRST_SDMMCOCP |
			S10_RSTMGR_PER0MODRST_DMAOCP);

	mmio_clrbits_32(S10_RSTMGR_PER0MODRST,
			S10_RSTMGR_PER0MODRST_EMAC0 |
			S10_RSTMGR_PER0MODRST_EMAC1 |
			S10_RSTMGR_PER0MODRST_EMAC2 |
			S10_RSTMGR_PER0MODRST_USB0 |
			S10_RSTMGR_PER0MODRST_USB1 |
			S10_RSTMGR_PER0MODRST_NAND |
			S10_RSTMGR_PER0MODRST_SDMMC |
			S10_RSTMGR_PER0MODRST_DMA |
			S10_RSTMGR_PER0MODRST_SPIM0 |
			S10_RSTMGR_PER0MODRST_SPIM1 |
			S10_RSTMGR_PER0MODRST_SPIS0 |
			S10_RSTMGR_PER0MODRST_SPIS1 |
			S10_RSTMGR_PER0MODRST_EMACPTP |
			S10_RSTMGR_PER0MODRST_DMAIF0 |
			S10_RSTMGR_PER0MODRST_DMAIF1 |
			S10_RSTMGR_PER0MODRST_DMAIF2 |
			S10_RSTMGR_PER0MODRST_DMAIF3 |
			S10_RSTMGR_PER0MODRST_DMAIF4 |
			S10_RSTMGR_PER0MODRST_DMAIF5 |
			S10_RSTMGR_PER0MODRST_DMAIF6 |
			S10_RSTMGR_PER0MODRST_DMAIF7);

}

void config_hps_hs_before_warm_reset(void)
{
	uint32_t or_mask = 0;

	or_mask |= S10_RSTMGR_HDSKEN_SDRSELFREFEN;
	or_mask |= S10_RSTMGR_HDSKEN_FPGAHSEN;
	or_mask |= S10_RSTMGR_HDSKEN_ETRSTALLEN;
	or_mask |= S10_RSTMGR_HDSKEN_L2FLUSHEN;
	or_mask |= S10_RSTMGR_HDSKEN_L3NOC_DBG;
	or_mask |= S10_RSTMGR_HDSKEN_DEBUG_L3NOC;

	mmio_setbits_32(S10_RSTMGR_HDSKEN, or_mask);
}

int socfpga_bridges_enable(void)
{
	uint32_t status, poll_idle;
	uint32_t time_out = 1000;

	status = intel_mailbox_get_config_status(MBOX_CONFIG_STATUS);

	if (!status) {
		/* Clear idle request */
		mmio_setbits_32(S10_SYSMGR_CORE(SYSMGR_NOC_IDLEREQ_CLR), ~0);

		/* De-assert all bridges */
		mmio_clrbits_32(S10_RSTMGR_BRGMODRST, ~0);

		/* Wait until idle ack becomes 0 */
		poll_idle = S10_SYSMGR_CORE(SYSMGR_NOC_IDLEACK);
		for (int i = 0; i < time_out; i++) {
			if (!(mmio_read_32(poll_idle)))
				return 0;
		}
		return -ETIMEDOUT;
	}
	return status;
}

int socfpga_bridges_disable(void)
{
	uint32_t poll_idle, idle_data;
	uint32_t time_out;

	/* Set idle request */
	mmio_write_32(S10_SYSMGR_CORE(SYSMGR_NOC_IDLEREQ_SET), ~0);

	/* Enable NOC timeout */
	mmio_setbits_32(SYSMGR_NOC_TIMEOUT, 1);

	/* Wait until each idle ack bit toggle to 1 */
	poll_idle = S10_SYSMGR_CORE(SYSMGR_NOC_IDLEACK);
	for (time_out = 1000; time_out > 0; time_out--) {
		idle_data = mmio_read_32(poll_idle) & IDLE_DATA_MASK;
		if (idle_data == IDLE_DATA_MASK)
			break;
	}
	if (!time_out)
		return -ETIMEDOUT;

	/* Wait until each idle status bit toggle to 1 */
	poll_idle = S10_SYSMGR_CORE(SYSMGR_NOC_IDLESTATUS);
	for (time_out = 1000; time_out > 0; time_out--) {
		idle_data = mmio_read_32(poll_idle) & IDLE_DATA_MASK;
		if (idle_data == IDLE_DATA_MASK)
			break;
	}
	if (!time_out)
		return -ETIMEDOUT;

	/* Assert all bridges */
	mmio_setbits_32(S10_RSTMGR_BRGMODRST,
		~(BRGMODRST_DDRSCH_MASK | BRGMODRST_FPGA2SOC_MASK));

	/* Disable NOC timeout */
	mmio_clrbits_32(S10_SYSMGR_CORE(SYSMGR_NOC_TIMEOUT), 1);

	return 0;
}
