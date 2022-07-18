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

#include "cdn_mmc.h"


int wait_ics(uint16_t timeout, uint32_t cdn_srs_res)
{
	uint32_t count = 0;
	uint32_t data = 0;

	/* Wait status command response ready */
	do {
		data = mmio_read_32(cdn_srs_res);
		count++;
		if (count >= timeout) {
			return -ETIMEDOUT;
		}
	} while ((data & (1 << ICS)) == 0);

	return 0;
}

void sdmmc_pin_config(void)
{
	// temp use base + addr. Official must change to common method.
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x00, 0x1);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x04, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x08, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x0C, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x10, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x14, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x18, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x1C, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x20, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x24, 0x3);
	mmio_write_32(AGX5_PINMUX_PIN0SEL+0x28, 0x3);

}

void set_mmc_var(mmc_combo_phy_t *mmc_combo_phy_reg, mmc_sdhc_t *mmc_sdhc_reg)
{
	mmc_combo_phy_reg->cp_clk_wr_delay = 0;
	mmc_combo_phy_reg->cp_clk_wrdqs_delay = 0;
	mmc_combo_phy_reg->cp_data_select_oe_end = 0;
	mmc_combo_phy_reg->cp_dll_bypass_mode = 1;
	mmc_combo_phy_reg->cp_dll_locked_mode = 0;
	mmc_combo_phy_reg->cp_dll_start_point = 0;
	mmc_combo_phy_reg->cp_gate_cfg_always_on = 1;
	mmc_combo_phy_reg->cp_io_mask_always_on = 0;
	mmc_combo_phy_reg->cp_io_mask_end = 0;
	mmc_combo_phy_reg->cp_io_mask_start = 0;
	mmc_combo_phy_reg->cp_rd_del_sel = 52;
	mmc_combo_phy_reg->cp_read_dqs_cmd_delay = 0;
	mmc_combo_phy_reg->cp_read_dqs_delay = 0;
	mmc_combo_phy_reg->cp_sw_half_cycle_shift = 0;
	mmc_combo_phy_reg->cp_sync_method = 1;
	mmc_combo_phy_reg->cp_underrun_suppress = 1;
	mmc_combo_phy_reg->cp_use_ext_lpbk_dqs = 1;
	mmc_combo_phy_reg->cp_use_lpbk_dqs = 1;
	mmc_combo_phy_reg->cp_use_phony_dqs = 1;
	mmc_combo_phy_reg->cp_use_phony_dqs_cmd = 1;

	mmc_sdhc_reg->sdhc_extended_rd_mode = 1;
	mmc_sdhc_reg->sdhc_extended_wr_mode = 1;
	mmc_sdhc_reg->sdhc_hcsdclkadj = 0;
	mmc_sdhc_reg->sdhc_idelay_val = 0;
	mmc_sdhc_reg->sdhc_rdcmd_en = 1;
	mmc_sdhc_reg->sdhc_rddata_en = 1;
	mmc_sdhc_reg->sdhc_rw_compensate = 9;
	mmc_sdhc_reg->sdhc_sdcfsh = 0;
	mmc_sdhc_reg->sdhc_sdcfsl = 1;
	mmc_sdhc_reg->sdhc_wrcmd0_dly = 1;
	mmc_sdhc_reg->sdhc_wrcmd0_sdclk_dly = 0;
	mmc_sdhc_reg->sdhc_wrcmd1_dly = 0;
	mmc_sdhc_reg->sdhc_wrcmd1_sdclk_dly = 0;
	mmc_sdhc_reg->sdhc_wrdata0_dly = 1;
	mmc_sdhc_reg->sdhc_wrdata0_sdclk_dly = 0;
	mmc_sdhc_reg->sdhc_wrdata1_dly = 0;
	mmc_sdhc_reg->sdhc_wrdata1_sdclk_dly = 0;

}

int sd_host_set_clk(cdn_mmc_params_t *cdn_mmc_dev_type_params)
{
	uint32_t ret = 0;
	uint32_t dtcvval, sdclkfsval;

	dtcvval = 0xE;
	sdclkfsval = 0;

	if ((cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_DS) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_UHS_SDR12) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_SDR_BC)) {
		sdclkfsval = 4;
	} else if ((cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_HS) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_UHS_SDR25) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_UHS_DDR50) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_SDR)) {
		sdclkfsval = 2;
	} else if ((cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_UHS_SDR50) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_DDR) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_HS400) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_HS400es)) {
		sdclkfsval = 1;
	} else if ((cdn_mmc_dev_type_params->cdn_mmc_dev_type == SD_UHS_SDR104) ||
		(cdn_mmc_dev_type_params->cdn_mmc_dev_type == EMMC_HS200)) {
		sdclkfsval = 0;
	}

	/* Set sdclk */
	mmio_write_32(SDMMC_CDN(SRS11), 0);
	mmio_write_32(SDMMC_CDN(SRS11), (dtcvval << DTCV) |
		(sdclkfsval << SDCLKFS) | (1 << ICE));
	ret = wait_ics(5000, SDMMC_CDN(SRS11));
	if (ret != 0) {
		ERROR("Wating ICS timeout");
		return ret;
	}

	/* Enable DLL reset */
	mmio_write_32(SDMMC_SDHC(HRS09), mmio_read_32(SDMMC_SDHC(HRS09)) & ~0x00000001);
	/* Set extended_wr_mode */
	mmio_write_32(SDMMC_SDHC(HRS09), (mmio_read_32(SDMMC_SDHC(HRS09)) & 0xFFFFFFF7) | (1 << EXTENDED_WR_MODE));
	/* Release DLL reset */
	mmio_write_32(SDMMC_SDHC(HRS09), mmio_read_32(SDMMC_SDHC(HRS09)) | 1);
	mmio_write_32(SDMMC_SDHC(HRS09), mmio_read_32(SDMMC_SDHC(HRS09)) | (3 << RDCMD_EN));

	do {
		mmio_read_32(SDMMC_SDHC(HRS09));
	} while (~mmio_read_32(SDMMC_SDHC(HRS09)) & (1 << 1)); // phy_init_complete

	mmio_write_32(SDMMC_CDN(SRS11), (dtcvval << DTCV) |
		(sdclkfsval << SDCLKFS) | (1 << ICE) | (1 << SDCE));

	return 0;
}

int sdmmc_write_sd_host_reg(uint32_t addr, uint32_t data)
{
	uint32_t value = 0;

	value = mmio_read_32(addr);
	value &= ~SDHC_REG_MASK;
	value |= data;
	mmio_write_32(addr, value);
	value = mmio_read_32(addr);
	if (value != data) {
		ERROR("SD host address is not set properly\n");
		return -ENXIO;
	}

	return 0;
}

int sd_host_init(mmc_combo_phy_t *mmc_combo_phy_reg, mmc_sdhc_t *mmc_sdhc_reg)
{
	uint32_t data = 0;
	uint32_t count = 0;
	uint32_t value = 0;
	int ret = 0;

	value = mmio_read_32(SDMMC_CDN(SRS11));
	value &= ~(0xFFFF);
	value |= 0x0;
	mmio_write_32(SDMMC_CDN(SRS11), value);
	udelay(1000);

	/* Software reset */
	mmio_write_32(SDMMC_SDHC(HRS00), (1 << SWR));
	/* Wait status command response ready */
	do {
		data = mmio_read_32(SDMMC_SDHC(HRS00));
		count++;
		if (count >= 5000) {
			NOTICE("Software reset is not completed\n");
			return -ETIMEDOUT;
		}
	/* Wait for HRS00.SWR */
	} while ((data & (1 << SWR)) == 1);

	/* Step 1, switch on DLL_RESET */
	value = mmio_read_32(SDMMC_SDHC(HRS09));
	value &= ~SDHC_PHY_SW_RESET;
	mmio_write_32(SDMMC_SDHC(HRS09), value);

	//* Step 2, program PHY_DQS_TIMING_REG */
	value = (CP_USE_EXT_LPBK_DQS(mmc_combo_phy_reg->cp_use_ext_lpbk_dqs))
		| (CP_USE_LPBK_DQS(mmc_combo_phy_reg->cp_use_lpbk_dqs))
		| (CP_USE_PHONY_DQS(mmc_combo_phy_reg->cp_use_phony_dqs))
		| (CP_USE_PHONY_DQS_CMD(mmc_combo_phy_reg->cp_use_phony_dqs_cmd));
	ret = sdmmc_write_phy_reg(SDMMC_SDHC(HRS04), PHY_DQS_TIMING_REG,
			SDMMC_SDHC(HRS05), value);
	if (ret != 0U) {
		ERROR("Program PHY_DQS_TIMING_REG failed\n");
		return ret;
	}

	/* Step 3, program PHY_GATE_LPBK_CTRL_REG */
	value = (CP_SYNC_METHOD(mmc_combo_phy_reg->cp_sync_method))
		| (CP_SW_HALF_CYCLE_SHIFT(mmc_combo_phy_reg->cp_sw_half_cycle_shift))
		| (CP_RD_DEL_SEL(mmc_combo_phy_reg->cp_rd_del_sel))
		| (CP_UNDERRUN_SUPPRESS(mmc_combo_phy_reg->cp_underrun_suppress))
		| (CP_GATE_CFG_ALWAYS_ON(mmc_combo_phy_reg->cp_gate_cfg_always_on));
	ret = sdmmc_write_phy_reg(SDMMC_SDHC(HRS04), PHY_GATE_LPBK_CTRL_REG,
			SDMMC_SDHC(HRS05), value);
	if (ret != 0U) {
		ERROR("Program PHY_GATE_LPBK_CTRL_REG failed\n");
		return -ret;
	}

	/* Step 4, program PHY_DLL_MASTER_CTRL_REG */
	value = (CP_DLL_BYPASS_MODE(mmc_combo_phy_reg->cp_dll_bypass_mode))
		| (CP_DLL_START_POINT(mmc_combo_phy_reg->cp_dll_start_point));
	ret = sdmmc_write_phy_reg(SDMMC_SDHC(HRS04), PHY_DLL_MASTER_CTRL_REG,
			SDMMC_SDHC(HRS05), value);
	if (ret != 0U) {
		ERROR("Program PHY_DLL_MASTER_CTRL_REG failed\n");
		return ret;
	}

	/* Step 5, program PHY_DLL_SLAVE_CTRL_REG */
	value = (CP_READ_DQS_CMD_DELAY(mmc_combo_phy_reg->cp_read_dqs_cmd_delay))
		| (CP_CLK_WRDQS_DELAY(mmc_combo_phy_reg->cp_clk_wrdqs_delay))
		| (CP_CLK_WR_DELAY(mmc_combo_phy_reg->cp_clk_wr_delay))
		| (CP_READ_DQS_DELAY(mmc_combo_phy_reg->cp_read_dqs_delay));
	ret = sdmmc_write_phy_reg(SDMMC_SDHC(HRS04), PHY_DLL_SLAVE_CTRL_REG,
			SDMMC_SDHC(HRS05), value);
	if (ret != 0U) {
		ERROR("Program PHY_DLL_SLAVE_CTRL_REG failed\n");
		return ret;
	}

	/* Step 6, program PHY_CTRL_REG */
	mmio_write_32(SDMMC_SDHC(HRS04), PHY_CTRL_REG);
	value = mmio_read_32(SDMMC_SDHC(HRS05));
	/* phony_dqs_timing=0 */
	value &= ~(CP_PHONY_DQS_TIMING_MASK << CP_PHONY_DQS_TIMING_SHIFT);
	mmio_write_32(SDMMC_SDHC(HRS05), value);

	/* Step 7, switch off DLL_RESET */
	do {
		value = mmio_read_32(SDMMC_SDHC(HRS09));
		value |= SDHC_PHY_SW_RESET;
		mmio_write_32(SDMMC_SDHC(HRS09), value);
		value = mmio_read_32(SDMMC_SDHC(HRS09));
	/* Step 8, polling PHY_INIT_COMPLETE */
	} while ((value & SDHC_PHY_INIT_COMPLETE) != SDHC_PHY_INIT_COMPLETE);

	/* Step 9, program PHY_DQ_TIMING_REG */
	value = (CP_IO_MASK_ALWAYS_ON(mmc_combo_phy_reg->cp_io_mask_always_on))
		| (CP_IO_MASK_END(mmc_combo_phy_reg->cp_io_mask_end))
		| (CP_IO_MASK_START(mmc_combo_phy_reg->cp_io_mask_start))
		| (CP_DATA_SELECT_OE_END(mmc_combo_phy_reg->cp_data_select_oe_end));
	ret = sdmmc_write_phy_reg(SDMMC_SDHC(HRS04), PHY_DQ_TIMING_REG,
			SDMMC_SDHC(HRS05), value);
	if (ret != 0U) {
		ERROR("Program PHY_DQ_TIMING_REG failed\n");
		return ret;
	}

	/* Step 10, program HRS09, register 42 */
	value = (SDHC_RDDATA_EN(mmc_sdhc_reg->sdhc_rddata_en))
		| (SDHC_RDCMD_EN(mmc_sdhc_reg->sdhc_rdcmd_en))
		| (SDHC_EXTENDED_WR_MODE(mmc_sdhc_reg->sdhc_extended_wr_mode))
		| (SDHC_EXTENDED_RD_MODE(mmc_sdhc_reg->sdhc_extended_rd_mode));
	ret = sdmmc_write_sd_host_reg(SDMMC_SDHC(HRS09), value);
	if (ret != 0U) {
		ERROR("Program HRS09 failed\n");
		return ret;
	}

	/* Step 11, program HRS10, register 43 */
	value = (SDHC_HCSDCLKADJ(mmc_sdhc_reg->sdhc_hcsdclkadj));
	ret = sdmmc_write_sd_host_reg(SDMMC_SDHC(HRS10), value);
	if (ret != 0U) {
		ERROR("Program HRS10 failed\n");
		return ret;
	}

	/* Step 12, program HRS16, register 48 */
	value = (SDHC_WRDATA1_SDCLK_DLY(mmc_sdhc_reg->sdhc_wrdata1_sdclk_dly))
		| (SDHC_WRDATA0_SDCLK_DLY(mmc_sdhc_reg->sdhc_wrdata0_sdclk_dly))
		| (SDHC_WRCMD1_SDCLK_DLY(mmc_sdhc_reg->sdhc_wrcmd1_sdclk_dly))
		| (SDHC_WRCMD0_SDCLK_DLY(mmc_sdhc_reg->sdhc_wrcmd0_sdclk_dly))
		| (SDHC_WRDATA1_DLY(mmc_sdhc_reg->sdhc_wrdata1_dly))
		| (SDHC_WRDATA0_DLY(mmc_sdhc_reg->sdhc_wrdata0_dly))
		| (SDHC_WRCMD1_DLY(mmc_sdhc_reg->sdhc_wrcmd1_dly))
		| (SDHC_WRCMD0_DLY(mmc_sdhc_reg->sdhc_wrcmd0_dly));
	ret = sdmmc_write_sd_host_reg(SDMMC_SDHC(HRS16), value);
	if (ret != 0U) {
		ERROR("Program HRS16 failed\n");
		return ret;
	}

	/* Step 13, program HRS07, register 40 */
	value = (SDHC_RW_COMPENSATE(mmc_sdhc_reg->sdhc_rw_compensate))
		| (SDHC_IDELAY_VAL(mmc_sdhc_reg->sdhc_idelay_val));
	ret = sdmmc_write_sd_host_reg(SDMMC_SDHC(HRS07), value);
	if (ret != 0U) {
		ERROR("Program HRS07 failed\n");
		return ret;
	}

	ret = sd_card_detect();
	if (ret != 0U) {
		ERROR("SD card does not detect\n");
		return ret;
	}

	ret = emmc_card_reset();
	if (ret != 0U) {
		ERROR("eMMC card reset failed\n");
		return ret;
	}

	cdn_mmc_params_t cdn_mmc_dev_type_params;

	ret = sd_host_set_clk(&cdn_mmc_dev_type_params);
	if (ret != 0U) {
		ERROR("SD host controller clock set failed\n");
		return ret;
	}

	/* Enable flags */
	mmio_write_32(SDMMC_CDN(SRS13), 0xFFFFFFFF);

	return 0;
}

int sd_host_send_cmd(uint32_t srs3, uint32_t cmd_arg, char use_intr)
{
	uint32_t status = 0;

	/* Clear all flags */
	mmio_write_32(SDMMC_CDN(SRS12), 0xFFFFFFFF);
	/* Set command argument */
	mmio_write_32(SDMMC_CDN(SRS02), cmd_arg);

	if (use_intr != 0U) {
		/* Enable interrupts*/
		mmio_write_32(SDMMC_CDN(SRS14), (1 << CC_IE) | (1 << TC_IE) |
			(1 << DMAINT_IE) | (1 << EINT));
		/* Execute command */
		mmio_write_32(SDMMC_CDN(SRS03), srs3);
		status = 0;
	} else {
		/* Enable interrupts*/
		mmio_write_32(SDMMC_CDN(SRS14), 0x00000000);
		/* Execute command */
		mmio_write_32(SDMMC_CDN(SRS03), srs3);
		udelay(30000); // 30us
		do {
			status = mmio_read_32(SDMMC_CDN(SRS12));
		/* Exit loop when CC (command complete) flag is set to 1 */
		} while ((status & ((1 << CC) | (1 << EINT))) == 0);
	}

	/* SRS12: Error or normal interrupt status */
	status &= 0xFFFF8000;

	if (status != 0U) {
		ERROR("SD host controller send command failed, SRS12 = %x\n", status);
		return -1;
	}

	return 0;
}

int emmc_card_init(char low_voltage_en)
{
	uint32_t value = 0;
	uint32_t sd_card_rca = 0;
	int ret = 0;

	/* CMD0: Reset all cards to IDLE state */
	/* SRS02, SRS03, SRS12 and SRS14 */
	ret = sd_host_send_cmd((0 << CIDX) | (0 << CRCCE) | (0 << RTS), 0x00000000, 0);
	if (ret != 0U) {
		ERROR("SD host controller send command failed\n");
		return -ret;
	}

	/* CMD1: Send operational condition and go to READY state */
	do {
		// 2.7-3.6V OCR bit set to 1
		value = 0x80FF8000;
		if (low_voltage_en == 1) {
			/* 1.8V */
			value |= (1 << 7);
		}

		value |= (SD_ADDR_SECTOR_MODE << 30); // byte or sector address mode
		/* SRS02, SRS03, SRS12 and SRS14 */
		sd_host_send_cmd((1 << CIDX) | (2 << RTS), value, 0);  // voltage and address mode

		/* SRS04: RESP0 */
		value = mmio_read_32(SDMMC_CDN(SRS04));
		NOTICE("Waiting response0. SRS04 value = %x\n", value);
	} while ((value & 0x80000000) == 0);

	/* CMD2: Get CardID and go to IDENT state */
	/* SRS02, SRS03, SRS12 and SRS14 */
	ret = sd_host_send_cmd((2 << CIDX) | (0 << CRCCE) | (1 << RTS), 0x00000000, 0);
	if (ret != 0U) {
		ERROR("SD host controller send command failed\n");
		return -ret;
	}

	/* Return with RESP0, RESP1, RESP2 and RESP3*/
	value = mmio_read_32(SDMMC_CDN(SRS04));
	NOTICE("Waiting response0, SRS04 = %x\n", value);

	/* CMD3: Send RCA to eMMC card and go to STBY */
	sd_card_rca = 0x1234;
	/* SRS02, SRS03, SRS12 and SRS14 */
	ret = sd_host_send_cmd((3 << CIDX) | (3 << CRCCE) | (2 << RTS), sd_card_rca << 16, 0);
	if (ret != 0U) {
		ERROR("SD host controller send command failed\n");
		return -ret;
	}
	/* Return with RESP0*/
	value = mmio_read_32(SDMMC_CDN(SRS04));
	NOTICE("Waiting response0, SRS04 = %x\n", value);

	NOTICE("eMMC Card initialized with RCA number 0x%x\n", sd_card_rca);

	return 0;
}

int cdn_mmc_init(handoff *hoff_ptr)
{
	int ret = 0;

	ret = dfi_select(hoff_ptr);
	if (ret != 0U) {
		ERROR("DFI configuration failed\n");
		return ret;
	}

	sdmmc_pin_config();

	mmc_combo_phy_t mmc_combo_phy_reg;
	mmc_sdhc_t mmc_sdhc_reg;

	set_mmc_var(&mmc_combo_phy_reg, &mmc_sdhc_reg);
	sd_host_init(&mmc_combo_phy_reg, &mmc_sdhc_reg);

	ret = emmc_card_init(1);
	if (ret != 0U) {
		ERROR("eMMC card initialization failed\n");
		return ret;
	}

	return 0;
}
