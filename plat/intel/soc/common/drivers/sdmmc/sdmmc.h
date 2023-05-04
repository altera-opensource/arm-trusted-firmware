/*
 * Copyright (c) 2022-2023, Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SDMMC_H
#define SDMMC_H

#include <lib/mmio.h>
#include "socfpga_handoff.h"


#define PERIPHERAL_SDMMC_MASK		0x60
#define PERIPHERAL_SDMMC_OFFSET		6

#define SOCFPGA_MMC_REG_BASE				0x10808000

/* FUNCTION DEFINATION */
/*
 * @brief Nand controller initialization function
 *
 * @hoff_ptr: Pointer to the hand-off data
 * Return: 0 on success, a negative errno on failure
 */
int sdmmc_init(handoff *hoff_ptr);
void sdmmc_pin_config(void);

#endif
