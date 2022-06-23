/*
 * Copyright (c) 2019-2022, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CLOCKMANAGER_H__
#define __CLOCKMANAGER_H__

#include "socfpga_handoff.h"

#define ALT_CLKMGR				0xffd10000

#define ALT_CLKMGR_CTRL				0x0
#define ALT_CLKMGR_STAT				0x4
#define ALT_CLKMGR_INTRCLR			0x14
#define ALT_CLKMGR_INTRCLR_MAINLOCKLOST_SET_MSK	0x00000004
#define ALT_CLKMGR_INTRCLR_PERLOCKLOST_SET_MSK	0x00000008

#define ALT_CLKMGR_CTRL_BOOTMODE_SET_MSK	0x00000001
#define ALT_CLKMGR_STAT_BUSY_E_BUSY		0x1
#define ALT_CLKMGR_STAT_BUSY(x)			(((x) & 0x00000001) >> 0)
#define ALT_CLKMGR_STAT_MAINPLLLOCKED(x)	(((x) & 0x00000100) >> 8)
#define ALT_CLKMGR_STAT_PERPLLLOCKED(x)		(((x) & 0x00000200) >> 9)

#define ALT_CLKMGR_MAINPLL			0xffd10030
#define ALT_CLKMGR_MAINPLL_EN			0x0
#define ALT_CLKMGR_MAINPLL_BYPASS		0xc
#define ALT_CLKMGR_MAINPLL_MPUCLK		0x18
#define ALT_CLKMGR_MAINPLL_NOCCLK		0x1c
#define ALT_CLKMGR_MAINPLL_CNTR2CLK		0x20
#define ALT_CLKMGR_MAINPLL_CNTR3CLK		0x24
#define ALT_CLKMGR_MAINPLL_CNTR4CLK		0x28
#define ALT_CLKMGR_MAINPLL_CNTR5CLK		0x2c
#define ALT_CLKMGR_MAINPLL_CNTR6CLK		0x30
#define ALT_CLKMGR_MAINPLL_CNTR7CLK		0x34
#define ALT_CLKMGR_MAINPLL_CNTR8CLK		0x38
#define ALT_CLKMGR_MAINPLL_CNTR9CLK		0x3c
#define ALT_CLKMGR_MAINPLL_NOCDIV		0x40
#define ALT_CLKMGR_MAINPLL_PLLGLOB		0x44
#define ALT_CLKMGR_MAINPLL_FDBCK		0x48
#define ALT_CLKMGR_MAINPLL_PLLC0		0x54
#define ALT_CLKMGR_MAINPLL_PLLC1		0x58
#define ALT_CLKMGR_MAINPLL_VCOCALIB		0x5c
#define ALT_CLKMGR_MAINPLL_EN_RESET		0x000000ff
#define ALT_CLKMGR_MAINPLL_FDBCK_MDIV(x)	(((x) & 0xff000000) >> 24)
#define ALT_CLKMGR_MAINPLL_PLLGLOB_PD_SET_MSK	0x00000001
#define ALT_CLKMGR_MAINPLL_PLLGLOB_REFCLKDIV(x)	(((x) & 0x00003f00) >> 8)
#define ALT_CLKMGR_MAINPLL_PLLGLOB_RST_SET_MSK	0x00000002
#define ALT_CLKMGR_MAINPLL_VCOCALIB_HSCNT_SET(x) (((x) << 0) & 0x000000ff)
#define ALT_CLKMGR_MAINPLL_VCOCALIB_MSCNT_SET(x) (((x) << 9) & 0x0001fe00)

#define ALT_CLKMGR_PSRC(x)			(((x) & 0x00030000) >> 16)
#define ALT_CLKMGR_SRC_MAIN			0
#define ALT_CLKMGR_SRC_PER			1

#define ALT_CLKMGR_PLLGLOB_PSRC_EOSC1		0x0
#define ALT_CLKMGR_PLLGLOB_PSRC_INTOSC		0x1
#define ALT_CLKMGR_PLLGLOB_PSRC_F2S		0x2

#define ALT_CLKMGR_PERPLL			0xffd100a4
#define ALT_CLKMGR_PERPLL_EN			0x0
#define ALT_CLKMGR_PERPLL_EN_SDMMCCLK		BIT(5)
#define ALT_CLKMGR_PERPLL_BYPASS		0xc
#define ALT_CLKMGR_PERPLL_CNTR2CLK		0x18
#define ALT_CLKMGR_PERPLL_CNTR3CLK		0x1c
#define ALT_CLKMGR_PERPLL_CNTR4CLK		0x20
#define ALT_CLKMGR_PERPLL_CNTR5CLK		0x24
#define ALT_CLKMGR_PERPLL_CNTR6CLK		0x28
#define ALT_CLKMGR_PERPLL_CNTR7CLK		0x2c
#define ALT_CLKMGR_PERPLL_CNTR8CLK		0x30
#define ALT_CLKMGR_PERPLL_CNTR9CLK		0x34
#define ALT_CLKMGR_PERPLL_GPIODIV		0x3c
#define ALT_CLKMGR_PERPLL_EMACCTL		0x38
#define ALT_CLKMGR_PERPLL_PLLGLOB		0x40
#define ALT_CLKMGR_PERPLL_FDBCK			0x44
#define ALT_CLKMGR_PERPLL_PLLC0			0x50
#define ALT_CLKMGR_PERPLL_PLLC1			0x54
#define ALT_CLKMGR_PERPLL_EN_RESET		0x00000fff
#define ALT_CLKMGR_PERPLL_FDBCK_MDIV(x)		(((x) & 0xff000000) >> 24)
#define ALT_CLKMGR_PERPLL_GPIODIV_GPIODBCLK_SET(x) (((x) << 0) & 0x0000ffff)
#define ALT_CLKMGR_PERPLL_PLLGLOB_PD_SET_MSK	0x00000001
#define ALT_CLKMGR_PERPLL_PLLGLOB_REFCLKDIV(x)	(((x) & 0x00003f00) >> 8)
#define ALT_CLKMGR_PERPLL_PLLGLOB_REFCLKDIV_SET(x) (((x) << 8) & 0x00003f00)
#define ALT_CLKMGR_PERPLL_PLLGLOB_RST_SET_MSK	0x00000002
#define ALT_CLKMGR_PERPLL_VCOCALIB_HSCNT_SET(x) (((x) << 0) & 0x000000ff)
#define ALT_CLKMGR_PERPLL_VCOCALIB_MSCNT_SET(x) (((x) << 9) & 0x0001fe00)
#define ALT_CLKMGR_PERPLL_VCOCALIB		0x58

#define ALT_CLKMGR_INTOSC_HZ			460000000

void config_clkmgr_handoff(handoff *hoff_ptr);
uint32_t get_wdt_clk(void);
uint32_t get_uart_clk(void);
uint32_t get_mmc_clk(void);
uint32_t get_l3_clk(uint32_t ref_clk);
uint32_t get_ref_clk(uint32_t pllglob);
uint32_t get_cpu_clk(void);

#endif
