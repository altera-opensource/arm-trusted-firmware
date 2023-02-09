/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2019-2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLAT_SOCFPGA_DEF_H
#define PLAT_SOCFPGA_DEF_H

#include <platform_def.h>
#include "agilex5_system_manager.h"
#include "agilex5_memory_controller.h"

/* Platform Setting */
#define PLATFORM_MODEL						PLAT_SOCFPGA_AGILEX5
#define BOOT_SOURCE							BOOT_SOURCE_QSPI
#define SOCFPGA_GIC_V3						1
#define XLAT_TABLES_V2						U(1)

/* FPGA config helpers */
#define INTEL_SIP_SMC_FPGA_CONFIG_ADDR		0x400000
#define INTEL_SIP_SMC_FPGA_CONFIG_SIZE		0x2000000

/* QSPI Setting */
#define CAD_QSPIDATA_OFST			0x10900000
#define CAD_QSPI_OFFSET				0x108d2000

/* Register Mapping */
#define SOCFPGA_CCU_NOC_REG_BASE			0x1c000000
#define SOCFPGA_F2SDRAMMGR_REG_BASE			0x18001000

#define SOCFPGA_MMC_REG_BASE				0x10808000
#define SOCFPGA_MEMCTRL_REG_BASE			0x108CC000
#define SOCFPGA_RSTMGR_REG_BASE				0x10d11000
#define SOCFPGA_SYSMGR_REG_BASE				0x10d12000
#define SOCFPGA_PINMUX_REG_BASE				0x10d13000
#define SOCFPGA_NAND_REG_BASE				0x10B80000

#define SOCFPGA_L4_PER_SCR_REG_BASE			0x10d21000
#define SOCFPGA_L4_SYS_SCR_REG_BASE			0x10d21100
#define SOCFPGA_SOC2FPGA_SCR_REG_BASE		0x10d21200
#define SOCFPGA_LWSOC2FPGA_SCR_REG_BASE		0x10d21300

/* Define maximum page size for NAND flash devices */
#define PLATFORM_MTD_MAX_PAGE_SIZE			U(0x1000)


#endif /* PLAT_SOCFPGA_DEF_H */
