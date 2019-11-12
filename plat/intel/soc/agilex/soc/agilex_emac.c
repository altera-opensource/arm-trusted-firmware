/*
 * Copyright (c) 2019, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <mmio.h>
#include <platform_def.h>
#include <agilex_reset_manager.h>
#include <agilex_system_manager.h>

void agx_emac_init()
{
	mmio_setbits_32(AGX_RSTMGR_PER0MODRST,
		AGX_RSTMGR_PER0MODRST_EMAC0 |
		AGX_RSTMGR_PER0MODRST_EMAC1 |
		AGX_RSTMGR_PER0MODRST_EMAC2);

	mmio_clrsetbits_32(AGX_SYSMGR_CORE(SYSMGR_CORE_EMAC_0),
		SYSMGR_CORE_EMAC_PHY_INTF_SEL_MSK,
		EMAC0_PHY_MODE);
	mmio_clrsetbits_32(AGX_SYSMGR_CORE(SYSMGR_CORE_EMAC_1),
		SYSMGR_CORE_EMAC_PHY_INTF_SEL_MSK,
		EMAC1_PHY_MODE);
	mmio_clrsetbits_32(AGX_SYSMGR_CORE(SYSMGR_CORE_EMAC_2),
		SYSMGR_CORE_EMAC_PHY_INTF_SEL_MSK,
		EMAC2_PHY_MODE);

	mmio_clrbits_32(AGX_SYSMGR_CORE(SYSMGR_CORE_FPGAINTF_EN_3),
		SYSMGR_CORE_FPGAINTF_EN_3_EMAC_MSK(0) |
		SYSMGR_CORE_FPGAINTF_EN_3_EMAC_MSK(1) |
		SYSMGR_CORE_FPGAINTF_EN_3_EMAC_MSK(2));

	mmio_clrbits_32(AGX_RSTMGR_PER0MODRST,
		AGX_RSTMGR_PER0MODRST_EMAC0 |
		AGX_RSTMGR_PER0MODRST_EMAC1 |
		AGX_RSTMGR_PER0MODRST_EMAC2);
}
