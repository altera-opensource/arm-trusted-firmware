/*
 * Copyright (c) 2022, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2022-2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CDN_MMC_H
#define CDN_MMC_H

#include "agilex5_pinmux.h"
#include "agilex5_system_manager.h"
#include "socfpga_handoff.h"
#include "socfpga_plat_def.h"
#include "socfpga_reset_manager.h"
#include "../combo_phy/cdn_combo_phy.h"

/* SDMMC Host */
#define SDMMC_REG_BASE		0x10808000
#define SDMMC_SDHC_HRS00	0x0	/* PHY register address */
#define SDMMC_SDHC_HRS04	0x10	/* PHY register address */
#define SDMMC_SDHC_HRS05	0x14	/* PHY register data port */
#define SDMMC_SDHC_HRS06	0x18	/* PHY register data port */
#define SDMMC_SDHC_HRS07	0x1C	/* IO_DELAY_INFO_REG */
#define SDMMC_SDHC_HRS09	0x24	/* PHY reset port */
#define SDMMC_SDHC_HRS10	0x28	/* PHY reset port */
#define SDMMC_SDHC_HRS16	0x40	/* CMD_DATA_OUTPUT */

#define SDHC_REG_MASK		0xFFFFFFFF

/* HRS00 */
#define SWR			0

/* HRS09 */
#define SDHC_PHY_SW_RESET		BIT(0)
#define SDHC_PHY_INIT_COMPLETE		BIT(1)
#define SDHC_EXTENDED_RD_MODE(x)	((x) << 2) //0x1
#define EXTENDED_WR_MODE		3
#define SDHC_EXTENDED_WR_MODE(x)	((x) << 3) //0x1
#define RDCMD_EN			15
#define SDHC_RDCMD_EN(x)		((x) << 15) //0x1
#define SDHC_RDDATA_EN(x)		((x) << 16) //0x1

/* HRS10 */
#define SDHC_HCSDCLKADJ(x)		((x) << 16) //0x1

/* HRS16 */
#define SDHC_WRCMD0_DLY(x)		((x) << 0) //0xf
#define SDHC_WRCMD1_DLY(x)		((x) << 4) //0xf
#define SDHC_WRDATA0_DLY(x)		((x) << 8) //0xf
#define SDHC_WRDATA1_DLY(x)		((x) << 12) //0xf
#define SDHC_WRCMD0_SDCLK_DLY(x)	((x) << 16) //0xf
#define SDHC_WRCMD1_SDCLK_DLY(x)	((x) << 20) //0xf
#define SDHC_WRDATA0_SDCLK_DLY(x)	((x) << 24) //0xf
#define SDHC_WRDATA1_SDCLK_DLY(x)	((x) << 28) //0xf

/* HRS07 */
#define SDHC_IDELAY_VAL(x)		((x) << 0) //0x1f
#define SDHC_RW_COMPENSATE(x)		((x) << 16) //0x1f

/* eMMC_MODE_HS_DDR */
#define SD_ADDR_SECTOR_MODE		1

/* Shared Macros */
#define SDMMC_SDHC(_reg)		(SDMMC_REG_BASE + \
					(SDMMC_SDHC_##_reg))

// eMMC modes
#define eMMC_MODE_HS		1

typedef enum cdn_mmc_device_type {
	SD_DS_ID, /* Identification */
	SD_DS, /* Default speed */
	SD_HS, /* High speed */
	SD_UHS_SDR12, /* Ultra high speed SDR12 */
	SD_UHS_SDR25, /* Ultra high speed SDR25 */
	SD_UHS_SDR50, /* Ultra high speed SDR`50 */
	SD_UHS_SDR104, /* Ultra high speed SDR104 */
	SD_UHS_DDR50, /* Ultra high speed DDR50 */
	EMMC_SDR_BC, /* SDR backward compatible */
	EMMC_SDR, /* SDR */
	EMMC_DDR, /* DDR */
	EMMC_HS200, /* High speed 200Mhz in SDR */
	EMMC_HS400, /* High speed 200Mhz in DDR */
	EMMC_HS400es, /* High speed 200Mhz in SDR with enhanced strobe*/
} cdn_mmc_device_type_t;

typedef struct cdn_mmc_params {
	uintptr_t	reg_base;
	uintptr_t	desc_base;
	size_t		desc_size;
	int		clk_rate;
	int		bus_width;
	unsigned int	flags;
	cdn_mmc_device_type_t	cdn_mmc_dev_type;
} cdn_mmc_params_t;

typedef struct cdn_mmc_device_info {
	unsigned int		sdmclk;			/* Freq in Hz */
	unsigned int		sdclk;			/* Freq in Hz */
	unsigned int		iocell_input_delay;	/* Delay in ps */
	unsigned int		iocell_output_delay;	/* Delay in ps */
	unsigned int		delay_element;		/* Delay in ps */
	cdn_mmc_device_type_t	cdn_mmc_dev_type;	/* Type of MMC */
} cdn_mmc_device_info_t;

typedef struct mmc_sdhc {
	uint32_t	sdhc_extended_rd_mode;
	uint32_t	sdhc_extended_wr_mode;
	uint32_t	sdhc_hcsdclkadj;
	uint32_t	sdhc_idelay_val;
	uint32_t	sdhc_rdcmd_en;
	uint32_t	sdhc_rddata_en;
	uint32_t	sdhc_rw_compensate;
	uint32_t	sdhc_sdcfsh;
	uint32_t	sdhc_sdcfsl;
	uint32_t	sdhc_wrcmd0_dly;
	uint32_t	sdhc_wrcmd0_sdclk_dly;
	uint32_t	sdhc_wrcmd1_dly;
	uint32_t	sdhc_wrcmd1_sdclk_dly;
	uint32_t	sdhc_wrdata0_dly;
	uint32_t	sdhc_wrdata0_sdclk_dly;
	uint32_t	sdhc_wrdata1_dly;
	uint32_t	sdhc_wrdata1_sdclk_dly;
} mmc_sdhc_t;

int wait_ics(uint16_t timeout, uint32_t cdn_srs_res);

void sdmmc_pin_config(void);

void set_mmc_var(mmc_combo_phy_t *mmc_combo_phy_reg, mmc_sdhc_t *mmc_sdhc_reg);

int sd_host_set_clk(cdn_mmc_params_t *cdn_mmc_dev_type_params);

int sdmmc_write_sd_host_reg(uint32_t addr, uint32_t data);

int sd_host_init(mmc_combo_phy_t *mmc_combo_phy_reg, mmc_sdhc_t *mmc_sdhc_reg);

int cdn_mmc_init(handoff *hoff_ptr);

#endif
