/*
 * Copyright (c) 2019-2022, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef AGX_MEMORYCONTROLLER_H
#define AGX_MEMORYCONTROLLER_H

#include "socfpga_plat_def.h"

#define AGX_MPFE_IOHMC_REG_DRAMADDRW			0xf80100a8
#define AGX_MPFE_IOHMC_CTRLCFG0				0xf8010028
#define AGX_MPFE_IOHMC_CTRLCFG1				0xf801002c
#define AGX_MPFE_IOHMC_CTRLCFG2				0xf8010030
#define AGX_MPFE_IOHMC_CTRLCFG3				0xf8010034
#define AGX_MPFE_IOHMC_DRAMADDRW			0xf80100a8
#define AGX_MPFE_IOHMC_DRAMTIMING0			0xf8010050
#define AGX_MPFE_IOHMC_CALTIMING0			0xf801007c
#define AGX_MPFE_IOHMC_CALTIMING1			0xf8010080
#define AGX_MPFE_IOHMC_CALTIMING2			0xf8010084
#define AGX_MPFE_IOHMC_CALTIMING3			0xf8010088
#define AGX_MPFE_IOHMC_CALTIMING4			0xf801008c
#define AGX_MPFE_IOHMC_CALTIMING9			0xf80100a0
#define AGX_MPFE_IOHMC_CALTIMING9_ACT_TO_ACT(x) (((x) & 0x000000ff) >> 0)
#define AGX_MPFE_IOHMC_CTRLCFG1_CFG_ADDR_ORDER(value)	\
						(((value) & 0x00000060) >> 5)

#define AGX_MPFE_HMC_ADP_ECCCTRL1			0xf8011100
#define AGX_MPFE_HMC_ADP_ECCCTRL2			0xf8011104
#define AGX_MPFE_HMC_ADP_RSTHANDSHAKESTAT		0xf8011218
#define AGX_MPFE_HMC_ADP_RSTHANDSHAKESTAT_SEQ2CORE	0x000000ff
#define AGX_MPFE_HMC_ADP_RSTHANDSHAKECTRL		0xf8011214


#define AGX_MPFE_IOHMC_REG_CTRLCFG1			0xf801002c

#define AGX_MPFE_IOHMC_REG_NIOSRESERVE0_OFST		0xf8010110

#define IOHMC_DRAMADDRW_COL_ADDR_WIDTH(x)	(((x) & 0x0000001f) >> 0)
#define IOHMC_DRAMADDRW_ROW_ADDR_WIDTH(x)	(((x) & 0x000003e0) >> 5)
#define IOHMC_DRAMADDRW_CS_ADDR_WIDTH(x)	(((x) & 0x00070000) >> 16)
#define IOHMC_DRAMADDRW_BANK_GRP_ADDR_WIDTH(x)	(((x) & 0x0000c000) >> 14)
#define IOHMC_DRAMADDRW_BANK_ADDR_WIDTH(x)	(((x) & 0x00003c00) >> 10)

#define AGX_MPFE_DDR(x)					(0xf8000000 + x)
#define AGX_MPFE_HMC_ADP_DDRCALSTAT			0xf801100c
#define AGX_MPFE_DDR_MAIN_SCHED				0xf8000400
#define AGX_MPFE_DDR_MAIN_SCHED_DDRCONF			0xf8000408
#define AGX_MPFE_DDR_MAIN_SCHED_DDRTIMING		0xf800040c
#define AGX_MPFE_DDR_MAIN_SCHED_DDRCONF_SET_MSK		0x0000001f
#define AGX_MPFE_DDR_MAIN_SCHED_DDRMODE			0xf8000410
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV		0xf800043c
#define AGX_MPFE_DDR_MAIN_SCHED_READLATENCY		0xf8000414
#define AGX_MPFE_DDR_MAIN_SCHED_ACTIVATE		0xf8000438
#define AGX_MPFE_DDR_MAIN_SCHED_ACTIVATE_FAWBANK_OFST	10
#define AGX_MPFE_DDR_MAIN_SCHED_ACTIVATE_FAW_OFST	4
#define AGX_MPFE_DDR_MAIN_SCHED_ACTIVATE_RRD_OFST	0
#define AGX_MPFE_DDR_MAIN_SCHED_DDRCONF_SET(x)	(((x) << 0) & 0x0000001f)
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSRDTORD_OFST	0
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSRDTORD_MSK	(BIT(0) | BIT(1))
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSRDTOWR_OFST	2
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSRDTOWR_MSK	(BIT(2) | BIT(3))
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSWRTORD_OFST	4
#define AGX_MPFE_DDR_MAIN_SCHED_DEVTODEV_BUSWRTORD_MSK	(BIT(4) | BIT(5))

#define AGX_MPFE_HMC_ADP(x)				(0xf8011000 + (x))
#define AGX_MPFE_HMC_ADP_HPSINTFCSEL			0xf8011210
#define AGX_MPFE_HMC_ADP_DDRIOCTRL			0xf8011008
#define HMC_ADP_DDRIOCTRL				0x8
#define HMC_ADP_DDRIOCTRL_IO_SIZE(x)		(((x) & 0x00000003) >> 0)
#define HMC_ADP_DDRIOCTRL_CTRL_BURST_LENGTH(x)	(((x) & 0x00003e00) >> 9)
#define ADP_DRAMADDRWIDTH				0xe0

#define ACT_TO_ACT_DIFF_BANK(value)		(((value) & 0x00fc0000) >> 18)
#define ACT_TO_ACT(value)			(((value) & 0x0003f000) >> 12)
#define ACT_TO_RDWR(value)			(((value) & 0x0000003f) >> 0)
#define ACT_TO_ACT(value)			(((value) & 0x0003f000) >> 12)

/* timing 2 */
#define RD_TO_RD_DIFF_CHIP(value)		(((value) & 0x00000fc0) >> 6)
#define RD_TO_WR_DIFF_CHIP(value)		(((value) & 0x3f000000) >> 24)
#define RD_TO_WR(value)				(((value) & 0x00fc0000) >> 18)
#define RD_TO_PCH(value)			(((value) & 0x00000fc0) >> 6)

/* timing 3 */
#define CALTIMING3_WR_TO_RD_DIFF_CHIP(value)	(((value) & 0x0003f000) >> 12)
#define CALTIMING3_WR_TO_RD(value)		(((value) & 0x00000fc0) >> 6)

/* timing 4 */
#define PCH_TO_VALID(value)			(((value) & 0x00000fc0) >> 6)

#define DDRTIMING_BWRATIO_OFST				31
#define DDRTIMING_WRTORD_OFST				26
#define DDRTIMING_RDTOWR_OFST				21
#define DDRTIMING_BURSTLEN_OFST				18
#define DDRTIMING_WRTOMISS_OFST				12
#define DDRTIMING_RDTOMISS_OFST				6
#define DDRTIMING_ACTTOACT_OFST				0

#define ADP_DDRIOCTRL_IO_SIZE(x)			(((x) & 0x3) >> 0)

#define DDRMODE_AUTOPRECHARGE_OFST			1
#define DDRMODE_BWRATIOEXTENDED_OFST			0


#define AGX_MPFE_IOHMC_REG_DRAMTIMING0_CFG_TCL(x)	(((x) & 0x7f) >> 0)
#define AGX_MPFE_IOHMC_REG_CTRLCFG0_CFG_MEM_TYPE(x)	(((x) & 0x0f) >> 0)

#define AGX_CCU_CPU0_MPRT_DDR				0xf7004400
#define AGX_CCU_CPU0_MPRT_MEM0				0xf70045c0
#define AGX_CCU_CPU0_MPRT_MEM1A				0xf70045e0
#define AGX_CCU_CPU0_MPRT_MEM1B				0xf7004600
#define AGX_CCU_CPU0_MPRT_MEM1C				0xf7004620
#define AGX_CCU_CPU0_MPRT_MEM1D				0xf7004640
#define AGX_CCU_CPU0_MPRT_MEM1E				0xf7004660
#define AGX_CCU_IOM_MPRT_MEM0				0xf7018560
#define AGX_CCU_IOM_MPRT_MEM1A				0xf7018580
#define	AGX_CCU_IOM_MPRT_MEM1B				0xf70185a0
#define	AGX_CCU_IOM_MPRT_MEM1C				0xf70185c0
#define	AGX_CCU_IOM_MPRT_MEM1D				0xf70185e0
#define	AGX_CCU_IOM_MPRT_MEM1E				0xf7018600

#define AGX_NOC_FW_DDR_SCR				0xf8020200
#define AGX_NOC_FW_DDR_SCR_MPUREGION0ADDR_LIMITEXT	0xf802021c
#define AGX_NOC_FW_DDR_SCR_MPUREGION0ADDR_LIMIT		0xf8020218
#define AGX_NOC_FW_DDR_SCR_NONMPUREGION0ADDR_LIMITEXT	0xf802029c
#define AGX_NOC_FW_DDR_SCR_NONMPUREGION0ADDR_LIMIT	0xf8020298

#define AGX_SOC_NOC_FW_DDR_SCR_ENABLE			0xf8020200
#define AGX_SOC_NOC_FW_DDR_SCR_ENABLESET		0xf8020204
#define AGX_CCU_NOC_DI_SET_MSK				0x10

#define AGX_SYSMGR_CORE_HMC_CLK				0xffd120b4
#define AGX_SYSMGR_CORE_HMC_CLK_STATUS			0x00000001

#define AGX_MPFE_IOHMC_NIOSRESERVE0_NIOS_RESERVE0(x)	(((x) & 0xffff) >> 0)
#define AGX_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_MSK		0x00000003
#define AGX_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_OFST		0
#define AGX_MPFE_HMC_ADP_HPSINTFCSEL_ENABLE		0x001f1f1f
#define AGX_IOHMC_CTRLCFG1_ENABLE_ECC_OFST		7

#define AGX_MPFE_HMC_ADP_ECCCTRL1_AUTOWB_CNT_RST_SET_MSK	0x00010000
#define AGX_MPFE_HMC_ADP_ECCCTRL1_CNT_RST_SET_MSK		0x00000100
#define AGX_MPFE_HMC_ADP_ECCCTRL1_ECC_EN_SET_MSK		0x00000001

#define AGX_MPFE_HMC_ADP_ECCCTRL2_AUTOWB_EN_SET_MSK		0x00000001
#define AGX_MPFE_HMC_ADP_ECCCTRL2_OVRW_RB_ECC_EN_SET_MSK	0x00010000
#define AGX_MPFE_HMC_ADP_ECCCTRL2_RMW_EN_SET_MSK		0x00000100
#define AGX_MPFE_HMC_ADP_DDRCALSTAT_CAL(value)		(((value) & 0x1) >> 0)


#define AGX_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE(x)		(((x) & 0x00003) >> 0)
#define IOHMC_DRAMADDRW_CFG_BANK_ADDR_WIDTH(x)		(((x) & 0x03c00) >> 10)
#define IOHMC_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH(x)	(((x) & 0x0c000) >> 14)
#define IOHMC_DRAMADDRW_CFG_COL_ADDR_WIDTH(x)		(((x) & 0x0001f) >> 0)
#define IOHMC_DRAMADDRW_CFG_CS_ADDR_WIDTH(x)		(((x) & 0x70000) >> 16)
#define IOHMC_DRAMADDRW_CFG_ROW_ADDR_WIDTH(x)		(((x) & 0x003e0) >> 5)

#define AGX_SDRAM_0_LB_ADDR				0x0
#define AGX_DDR_SIZE					0x40000000


/* Macros */
#define SOCFPGA_MEMCTRL_ECCCTRL1					0x008
#define SOCFPGA_MEMCTRL_ERRINTEN					0x010
#define SOCFPGA_MEMCTRL_ERRINTENS					0x014
#define SOCFPGA_MEMCTRL_ERRINTENR					0x018
#define SOCFPGA_MEMCTRL_INTMODE					0x01C
#define SOCFPGA_MEMCTRL_INTSTAT					0x020
#define SOCFPGA_MEMCTRL_DIAGINTTEST					0x024
#define SOCFPGA_MEMCTRL_DERRADDRA					0x02C


#define SOCFPGA_MEMCTRL(_reg)		(SOCFPGA_MEMCTRL_REG_BASE \
						+ (SOCFPGA_MEMCTRL_##_reg))

//int init_hard_memory_controller(void);

#endif
