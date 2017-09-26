/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.

**/

//
// Include files
//
#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bl_common.h>
#include <console.h>
#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <generic_delay_timer.h>
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <platform_private.h>
#include <console_printf.h>

#include "Altera_Hps_Socal.h"
#include "Base.h"
#include "MemoryController.h"

#define DDR_READ_LATENCY_DELAY 40

#define MAX_MEM_CAL_RETRY         3
#define PRE_CALIBRATION_DELAY     1
#define POST_CALIBRATION_DELAY    1
#define TIMEOUT_EMIF_CALIBRATION  100
#define CLEAR_EMIF_DELAY                                 50000
#define CLEAR_EMIF_TIMEOUT                               0x100000
#define TIMEOUT_INT_RESP                                 10000
#define ALT_CCU_NOC_DI_SET_MSK 0x10
#define RemapOnChipRamTo1stOneMB   0

#define DDR_CONFIG(A, B, C, R)	((A<<24)|(B<<16)|(C<<8)|R)

/* The followring are the supported configurations */
uint32_t ddr_config[] = {
	/* DDR_CONFIG(Address order,Bank,Column,Row) */
	/* List for DDR3 or LPDDR3 (pinout order > chip, row, bank, column) */
	DDR_CONFIG(0, 3, 10, 12),
	DDR_CONFIG(0, 3,  9, 13),
	DDR_CONFIG(0, 3, 10, 13),
	DDR_CONFIG(0, 3,  9, 14),
	DDR_CONFIG(0, 3, 10, 14),
	DDR_CONFIG(0, 3, 10, 15),
	DDR_CONFIG(0, 3, 11, 14),
	DDR_CONFIG(0, 3, 11, 15),
	DDR_CONFIG(0, 3, 10, 16),
	DDR_CONFIG(0, 3, 11, 16),
	DDR_CONFIG(0, 3, 12, 15),	/* 0xa */
	/* List for DDR4 only (pinout order > chip, bank, row, column) */
	DDR_CONFIG(1, 3, 10, 14),
	DDR_CONFIG(1, 4, 10, 14),
	DDR_CONFIG(1, 3, 10, 15),
	DDR_CONFIG(1, 4, 10, 15),
	DDR_CONFIG(1, 3, 10, 16),
	DDR_CONFIG(1, 4, 10, 16),
	DDR_CONFIG(1, 3, 10, 17),
	DDR_CONFIG(1, 4, 10, 17),
};

#define DDR_CONFIG_ELEMENTS	(sizeof(ddr_config)/sizeof(uint32_t))

int match_ddr_conf(uint32_t ddr_conf)
{
	int i;
	for (i = 0; i < DDR_CONFIG_ELEMENTS; i++) {
		if (ddr_conf == ddr_config[i])
			return i;
	}
	return 0;
}

int InitHardMemoryController (void)
{
	int Status;

	console_printf ("Initializing Hard Memory Controller\r\n");
    console_printf ("Enable access to DDR register from CPU master\r\n");
    //Enable access to DDR register from CPU master
    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_DDRREG_SPRT_DDRREGSPACE0_0_OFST,
  			   ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE0_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1A_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1B_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1C_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

	mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1D_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1E_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

    //Enable access to DDR register from IO master
    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE0_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1A_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1B_0_OFST,
                 ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1C_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1D_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

    mmio_clrbits_32(ALT_CCU_NOC_OFST +
                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADBASE_MEM_MEM0_SPRT_MEMSPACE1E_0_OFST,
                ALT_CCU_NOC_DI_SET_MSK);

	// this enables nonsecure access to DDR
    // mpu region limit
	mmio_write_32(ALT_SOC_NOC_FW_DDR_SCR_OFST +
	              ALT_SOC_NOC_FW_DDR_SCR_MPUREGION0ADDR_LIMIT_OFST,
				  0xFFFF0000);
    mmio_write_32(ALT_SOC_NOC_FW_DDR_SCR_OFST +
	              ALT_SOC_NOC_FW_DDR_SCR_MPUREGION0ADDR_LIMITEXT_OFST,
				  0x1F);
	// nonmpu region limit
    mmio_write_32(ALT_SOC_NOC_FW_DDR_SCR_OFST +
	              ALT_SOC_NOC_FW_DDR_SCR_NONMPUREGION0ADDR_LIMIT_OFST,
				  0xFFFF0000);
    mmio_write_32(ALT_SOC_NOC_FW_DDR_SCR_OFST +
	              ALT_SOC_NOC_FW_DDR_SCR_NONMPUREGION0ADDR_LIMITEXT_OFST,
				  0x1F);
	// enable mpu and nonmpu bit 0 and bit 8
    mmio_write_32(ALT_SOC_NOC_FW_DDR_SCR_OFST +
	              ALT_SOC_NOC_FW_DDR_SCR_ENABLE_OFST,
				  BIT0 | BIT8);

	console_printf("DDR: Ensure HMC clock is running\r\n");
	// Ensure HMC clock is running
	Status = CheckHmcClock ();
	if (Status) {
		ERROR("DDR: Error as HMC clock not running\r\n");
		return Status;
	}
	console_printf ("De-assert the reset signal of DDR Scheduler in the NOC\r\n");
	// De-assert the reset signal of DDR Scheduler in the NOC
	mmio_clrbits_32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_BRGMODRST_OFST,
             ALT_RSTMGR_BRGMODRST_DDRSCH_SET_MSK);

	console_printf("DDR:  Memory Calibration\r\n");
	// Memory Calibration
	Status = MemoryCalibration ();
	if (Status) {
		ERROR("DDR:  Memory Calibration Failed\r\n");
		return Status;
	}

#ifndef EMULATOR
	// HMC Adaptor is a component between NOC and IO48_HMC which also do ECC checking.
	ConfigureHmcAdaptorRegisters ();
	ConfigureDdrSchedulerControlRegisters ();
#endif
   return 0;
}

int CheckHmcClock (void)
{
    unsigned long      TimeOutCount;
  uint32_t     Data32;

  TimeOutCount = 0;

  // wait until hmc clock is running
  // hmc clock status: hmc clock not running = 0; hmc clock runnin = 1.
  do {
    Data32 = mmio_read_32 (ALT_SYSMGR_CORE_OFST + ALT_SYSMGR_CORE_HMC_CLK_OFST);
    if (Data32 & ALT_SYSMGR_CORE_HMC_CLK_STATUS_SET_MSK)
      break;
    udelay (1);
  } while (++TimeOutCount < 1000);
  if (TimeOutCount >= 1000) {
    return ETIMEDOUT;
  }
  return 0;
}

int ClearEMIF (void)
{
  uint32_t          Data32;
    unsigned long            TimeOutCount;
  // Request HMC to Clear EMIF
  mmio_write_32 (ALT_MPFE_HMC_ADP_OFST +
                 ALT_MPFE_HMC_ADP_RSTHANDSHAKECTRL_OFST,
                 0);
  // Wait for Clear EMIF done.
  TimeOutCount = 0;
  do {
    Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST + ALT_MPFE_HMC_ADP_RSTHANDSHAKESTAT_OFST);
    if ((Data32 & ALT_MPFE_HMC_ADP_RSTHANDSHAKESTAT_SEQ2CORE_SET_MSK) == 0)
      break;
    udelay (CLEAR_EMIF_DELAY);
  } while (++TimeOutCount < CLEAR_EMIF_TIMEOUT);
  if (TimeOutCount >= CLEAR_EMIF_TIMEOUT) {
    return ETIMEDOUT;
  }
  return 0;
}


int MemoryCalibration (void)
{
  int      Status;
  uint32_t          Data32;
  unsigned long            TimeOutCount;
  unsigned long            RetryCount;

  Status = 0;

  console_printf ("\t HMC Calibrating Memory...\r\n");
  udelay (PRE_CALIBRATION_DELAY);
  console_printf ("\t Wait for EMIF NIOS II to set calibration success bit...\r\n");
  //--------------------------------------------------------
  // Wait for EMIF NIOS II to set calibration success bit.
  //--------------------------------------------------------
 RetryCount = 0;
  do {
    if (RetryCount != 0) {
      console_printf ("\t\t Retry DRAM calibration\r\n");
    }

    TimeOutCount = 0;
    do {
      Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST +
                         	  ALT_MPFE_HMC_ADP_DDRCALSTAT_OFST);
      if (ALT_MPFE_HMC_ADP_DDRCALSTAT_CAL_GET(Data32) == 1)
        break;
      udelay (1);
    } while (++TimeOutCount < TIMEOUT_EMIF_CALIBRATION);

	// if fail, clear emif
	if (ALT_MPFE_HMC_ADP_DDRCALSTAT_CAL_GET(Data32) == 0) {
      Status = ClearEMIF();
	  if (Status)
        ERROR ("Failed to clear Emif\r\n");
    } else {
	  break;
	}
  } while (++RetryCount < MAX_MEM_CAL_RETRY);

  if (ALT_MPFE_HMC_ADP_DDRCALSTAT_CAL_GET(Data32) == 0) {
    ERROR ("DRAM calibration fail.\r\n");
	Status = -EIO;
  } else {
    console_printf ("DRAM calibration success.\r\n");
	Status = 0;
  }
  udelay (POST_CALIBRATION_DELAY);

  return Status;
}


void ConfigureDdrSchedulerControlRegisters (void)
{
  uint32_t          Data32;
  // Variable Used by Step 1
  uint32_t          DramAddrOrder;
  uint32_t          DdrConf;
  uint32_t          Bank, Row, Col;
  // Variable Used by Step 2
  uint32_t          RdToMiss;
  uint32_t          WrToMiss;
  uint32_t          BurstLen;
  uint32_t          BurstLenInDdrClockUnit;
  uint32_t          BurstLenInSchedulerClockUnit;
  uint32_t          ActToAct;
  uint32_t          RdToWr;
  uint32_t          WrToRd;
  uint32_t          BwRatio;
  uint32_t          tRTP;
  uint32_t          tRP;
  uint32_t          tRCD;
  uint32_t          RdLatency; // Also used by Step 4
  uint32_t          tWRinClockCycles;
  // Variable Used by Step 3
  uint32_t          BwRatioExtended;
  uint32_t          AutoPreCharge;
  // Variable Used by Step 5
  uint32_t          ActToActDiffBank;
  uint32_t          Faw;
  uint32_t          FawBank;
  // Variable Used by Step 6
  uint32_t          BusRdToRd;
  uint32_t          BusRdToWr;
  uint32_t          BusWrToRd;

  console_printf ("\t Init HPS NOC's DDR Scheduler.\r\n");

  //
  // Step 1 - Init Selector of predefined ddrConf configuration.
  //
  // First, find out the DRAM Address Ordering format
  // where "00" - chip, row, bank(BG, BA), column;
  //       "01" - chip, bank(BG, BA), row, column;
  //       "10" - row, chip, bank(BG, BA), column;
  // Only do this step 1 when "00" or "01", because this DdrConf selector
  // only support "00":Chip_R(n)_B(n)_C(n) and "01":Chip_B(n)_R(n)_C(n)
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CTRLCFG1_OFST);
  DramAddrOrder = ALT_MPFE_IOHMC_REG_CTRLCFG1_CFG_ADDR_ORDER_GET(Data32);

  // Next, get the row/column/bank address width information
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_DRAMADDRW_OFST);

  Col  = ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32);
  Row  = ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32);
  Bank = ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
         ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32);

  // Determine what value to be set for DdrConf
  DdrConf = match_ddr_conf(DDR_CONFIG(DramAddrOrder, Bank, Col, Row));

  // Set the DdrConf register
  if (DdrConf) {
	console_printf("Ddr config is %x \n", DdrConf);

    mmio_clrsetbits_32 (
      ALT_MPFE_DDR_MAIN_SCHED_OFST +
      ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRCONF_OFST,
      ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRCONF_DDRCONF_SET_MSK,
      ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRCONF_DDRCONF_SET(DdrConf)
    );
  } else {
    ERROR ("\t\t Cannot find predefined ddrConf configuration.\r\n");
  }

  // Set HMC Dram Address width
  mmio_write_32(ALT_MPFE_HMC_ADP_OFST + ALT_MPFE_HMC_ADP_DRAMADDRWIDTH_OFST, Data32);
  console_printf("Init DDR timing\r\n");

  //
  // Step 2 - Init DDR Timing for scheduler clock cycles between DRAM commands.
  // Register : ddr_T_main_Scheduler_DdrTiming
  //
  // WRTOMISS formula note:
  // Begin with, WRTOMISS = WL + tWR + tRP + tRCD
  //      Where, WL = RL + BL/2 + 2 - rd-to-wr
  //        and, tWR = 15ns (see JEDEC standard eg. DDR4 = JESD79-4.pdf)
  // The equation is in memory clock units,
  // We need to divide the first part by 2 to make it HMC clock units.
  // A note for BL (on question why is it BL's final value is equal to original BL divide by 4):
  // BL/2 - the 1ST divide by 2 is because data is transferred on both edges DDR clock.
  // (BL/2)/2 - the 2nd divide by 2 again is to convert to scheduler or controller clocks.
  // Final WRTOMISS = ((RL + BL/2 + 2 + tWR) / 2) - rd-to-wr + tRP + tRCD
  //
  // RDTOMISS formula note:
  // Begin with, RDTOMISS = tRTP + tRP + tRCD - BL/2
  // After convert BL to HMC clock units.
  // Final RDTOMISS = tRTP + tRP + tRCD - ((BL/2)/2)
  //
  // For DDR3, Maximum Frequency is 1066 MHz, to get 15ns tWR:
  //   1066MHz is 0.938ns which is close to 1ns, so we can use 15 directly.
  //
  // For DDR4, Maximum Frequency is 1333 MHz, to get 15ns tWR:
  //   1333MHz is 0.75ns, to get 15ns, we need 15/0.75 = 20 clock cycles
  //
  // BURSTLEN = hmc_mmr.ctrlcfg0.cfg_ctrl_burst_length / 2
  // ACTTOACT = hmc_mmr.caltiming0.cfg_t_param_act_to_act
  // RDTOWR   = hmc_mmr.caltiming1.cfg_t_param_rd_to_wr
  // WRTORD   = hmc_mmr.caltiming3.cfg_t_param_wr_to_rd
  // BWRATIO  = ((ecc_hmc.DDRIOCTRL == 0) ? 0 : 1)

  // Calculate the value to be set for DdrTiming register
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_DRAMTIMING0_OFST);
  RdLatency = ALT_MPFE_IOHMC_REG_DRAMTIMING0_CFG_TCL_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING0_OFST);
  ActToAct = ALT_MPFE_IOHMC_REG_CALTIMING0_CFG_T_PARAM_ACT_TO_ACT_GET(Data32);
  tRCD = ALT_MPFE_IOHMC_REG_CALTIMING0_CFG_T_PARAM_ACT_TO_RDWR_GET(Data32);
  ActToActDiffBank = ALT_MPFE_IOHMC_REG_CALTIMING0_CFG_T_PARAM_ACT_TO_ACT_DIFF_BANK_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING1_OFST);
  RdToWr = ALT_MPFE_IOHMC_REG_CALTIMING1_CFG_T_PARAM_RD_TO_WR_GET(Data32);
  BusRdToRd = ALT_MPFE_IOHMC_REG_CALTIMING1_CFG_T_PARAM_RD_TO_RD_DIFF_CHIP_GET(Data32);
  BusRdToWr = ALT_MPFE_IOHMC_REG_CALTIMING1_CFG_T_PARAM_RD_TO_WR_DIFF_CHIP_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING2_OFST);
  tRTP = ALT_MPFE_IOHMC_REG_CALTIMING2_CFG_T_PARAM_RD_TO_PCH_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING3_OFST);
  WrToRd = ALT_MPFE_IOHMC_REG_CALTIMING3_CFG_T_PARAM_WR_TO_RD_GET(Data32);
  BusWrToRd = ALT_MPFE_IOHMC_REG_CALTIMING3_CFG_T_PARAM_WR_TO_RD_DIFF_CHIP_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING4_OFST);
  tRP = ALT_MPFE_IOHMC_REG_CALTIMING4_CFG_T_PARAM_PCH_TO_VALID_GET(Data32);

  Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST + ALT_MPFE_HMC_ADP_DDRIOCTRL_OFST);
  BwRatio = ((ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_GET(Data32) == 0) ? 0 : 1);

  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CTRLCFG0_OFST);
  BurstLen = ALT_MPFE_IOHMC_REG_CTRLCFG0_CFG_CTRL_BURST_LENGTH_GET(Data32);
  BurstLenInDdrClockUnit = BurstLen / 2;
  BurstLenInSchedulerClockUnit = ((BurstLen/2) / 2);

  // tWR = Min. 15ns constant, see JEDEC standard eg. DDR4 is JESD79-4.pdf
  #define tWR_IN_NANOSECONDS 15
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CTRLCFG0_OFST);
  switch (ALT_MPFE_IOHMC_REG_CTRLCFG0_CFG_MEM_TYPE_GET(Data32))
  {
    case 1:
      // DDR4 - 1333MHz
      // 20 (19.995) clock cycles = 15ns
      // Calculate with rounding
      tWRinClockCycles = (((tWR_IN_NANOSECONDS * 1333) % 1000) >= 500) ?
                          ((tWR_IN_NANOSECONDS * 1333) / 1000) + 1 :
                          ((tWR_IN_NANOSECONDS * 1333) / 1000);
      break;
    default:
      // Others - 1066MHz or slower
      // 16 (15.990) clock cycles = 15ns
      // Calculate with rounding
      tWRinClockCycles = (((tWR_IN_NANOSECONDS * 1066) % 1000) >= 500) ?
                          ((tWR_IN_NANOSECONDS * 1066) / 1000) + 1 :
                          ((tWR_IN_NANOSECONDS * 1066) / 1000);
      break;
  }

  RdToMiss = tRTP + tRP + tRCD - BurstLenInSchedulerClockUnit;
  WrToMiss = ((RdLatency + BurstLenInDdrClockUnit + 2 + tWRinClockCycles) / 2) - RdToWr + tRP + tRCD;

  console_printf (
    "\t\t RdLatency = %d\r\n"
    "\t\t ActToAct = %d\r\n"
    "\t\t tRCD = %d\r\n"
    "\t\t ActToActDiffBank = %d\r\n"
    "\t\t BusRdToRd = %d\r\n"
    "\t\t BusRdToWr = %d\r\n"
    "\t\t tRTP = %d\r\n"
    "\t\t WrToRd = %d\r\n"
    "\t\t BusWrToRd = %d\r\n"
    "\t\t tRP = %d\r\n"
    "\t\t BwRatio = %d\r\n"
    "\t\t BurstLen = %d\r\n"
    "\t\t BurstLen in one DDR Clock cycles = %d\r\n"
    "\t\t BurstLen in DDR Scheduler Clock cycles = %d\r\n"
    "\t\t tWR in Nano Seconds = %d\r\n"
    "\t\t tWR in Clock Cycles = %d\r\n"
    "\t\t RdToMiss = %d\r\n"
    "\t\t WrToMiss = %d\r\n",
    RdLatency,
    ActToAct,
    tRCD,
    ActToActDiffBank,
    BusRdToRd,
    BusRdToWr,
    tRTP,
    WrToRd,
    BusWrToRd,
    tRP,
    BwRatio,
    BurstLen,
    BurstLenInDdrClockUnit,
    BurstLenInSchedulerClockUnit,
    tWR_IN_NANOSECONDS,
    tWRinClockCycles,
    RdToMiss,
    WrToMiss);

  // Set the DdrTiming register
  mmio_clrsetbits_32 (
    ALT_MPFE_DDR_MAIN_SCHED_OFST +
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_OFST,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BWRATIO_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTORD_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOWR_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BURSTLEN_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTOMISS_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOMISS_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_ACTTOACT_SET_MSK,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BWRATIO_SET(BwRatio) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTORD_SET(WrToRd) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOWR_SET(RdToWr) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BURSTLEN_SET(BurstLenInSchedulerClockUnit) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTOMISS_SET(WrToMiss) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOMISS_SET(RdToMiss) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_ACTTOACT_SET(ActToAct)
  );

  //
  // Step 3 - Init DDR mode concerning Page Auto Precharge and BW Ratio.
  //
  Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST + ALT_MPFE_HMC_ADP_DDRIOCTRL_OFST);
  BwRatioExtended = ((ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_GET(Data32) == 0) ? 1 : 0);

  AutoPreCharge = 0;

  mmio_clrsetbits_32 (
    ALT_MPFE_DDR_MAIN_SCHED_OFST +
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_OFST,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_BWRATIOEXTENDED_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_AUTOPRECHARGE_SET_MSK,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_BWRATIOEXTENDED_SET(BwRatioExtended) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_AUTOPRECHARGE_SET(AutoPreCharge)
  );

  //
  // Step 4 - Init Read Latency between a read request and the first data response.
  //
  // case 314587: Update DDR Scheduler ReadLatency calculation
  mmio_clrsetbits_32 (
    ALT_MPFE_DDR_MAIN_SCHED_OFST +
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_READLATENCY_OFST,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_READLATENCY_READLATENCY_SET_MSK,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_READLATENCY_READLATENCY_SET(RdLatency / 2 + DDR_READ_LATENCY_DELAY)
  );

  //
  // Step 5 - Init Timing values concerning Activate commands
  //
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CALTIMING9_OFST);
  Faw = ALT_MPFE_IOHMC_REG_CALTIMING9_CFG_T_PARAM_4_ACT_TO_ACT_GET(Data32);

  // Number of Bank of a given device involved in the FAW period.
  FawBank = 1; // always 1 because we always have 4 bank DDR.

  mmio_clrsetbits_32 (
    ALT_MPFE_DDR_MAIN_SCHED_OFST +
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_OFST,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAWBANK_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAW_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_RRD_SET_MSK,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAWBANK_SET(FawBank) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAW_SET(Faw) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_RRD_SET(ActToActDiffBank)
  );

  //
  // Step 6 - Init Timing values concerning Device to Device Data Bus ownership change.
  //
  mmio_clrsetbits_32 (
    ALT_MPFE_DDR_MAIN_SCHED_OFST +
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_OFST,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTORD_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTOWR_SET_MSK |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSWRTORD_SET_MSK,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTORD_SET(BusRdToRd) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTOWR_SET(BusRdToWr) |
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSWRTORD_SET(BusWrToRd)
  );

}


void ConfigureHmcAdaptorRegisters (void)
{
  uint32_t          Data32;
  uint32_t          DramIoWidth;

  console_printf ("\t Init HMC Adaptor.\r\n");

  //
  // Init Register: DDRIOCTRL
  //
  DramIoWidth = ALT_MPFE_IOHMC_REG_NIOSRESERVE0_NIOS_RESERVE0_GET(
				mmio_read_32 (ALT_MPFE_IOHMC_OFST +
                ALT_MPFE_IOHMC_REG_NIOSRESERVE0_OFST));

  // Check if the IO Size value from HMC MMR is legit
  /*
  if ((DramIoWidth >= 16) && (DramIoWidth <= 64))
  {
    // niosreserve(N) registers looks legit
    console_printf ("\t\t DRAM IoSize from HMC_MMR.\r\n");
    DramIoWidth = (DramIoWidth & 0xFF) >> 5; // Convert it to DDRIOCTRL_IO_SIZE format
  } else {
    ERROR ("\t\t ERROR! DRAM IoSize from HMC_MMR_NIOSRESERVE0 does not look legit\r\n");
    DramIoWidth = 1;
  }
  */
  DramIoWidth = (DramIoWidth & 0xFF) >> 5; // Convert it to DDRIOCTRL_IO_SIZE format
  // Program the Dram IO Width
  mmio_clrsetbits_32 (ALT_MPFE_HMC_ADP_OFST +
                   ALT_MPFE_HMC_ADP_DDRIOCTRL_OFST,
                   ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_SET_MSK,
                   ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_SET(DramIoWidth));

  // enable HPS interface to HMC
  mmio_write_32(ALT_MPFE_HMC_ADP_OFST +
                ALT_MPFE_HMC_ADP_HPSINTFCSEL_OFST,
				0x001f1f1f);

  //
  // Does HMC enabled the generation and checking of ECC ?
  //
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CTRLCFG1_OFST);
  if (ALT_MPFE_IOHMC_REG_CTRLCFG1_CFG_CTRL_ENABLE_ECC_GET(Data32) == 1) {
    // Yes, ECC is enabled.
    console_printf ("\t\t ECC is enabled.\r\n");

    //
    // Init Register: ECCCTRL1 and ECCCTRL2
    //

    // ECCCTRL1:Perform counter reset
    mmio_clrsetbits_32 (ALT_MPFE_HMC_ADP_OFST +
                     ALT_MPFE_HMC_ADP_ECCCTRL1_OFST,
                     ALT_MPFE_HMC_ADP_ECCCTRL1_AUTOWB_CNT_RST_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_CNT_RST_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_ECC_EN_SET_MSK,
                     ALT_MPFE_HMC_ADP_ECCCTRL1_AUTOWB_CNT_RST_SET(1) |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_CNT_RST_SET(1) |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_ECC_EN_SET(0));

    // ECCCTRL2: Enable read modify write logic and auto write back correction.
    mmio_clrsetbits_32 (ALT_MPFE_HMC_ADP_OFST +
                     ALT_MPFE_HMC_ADP_ECCCTRL2_OFST,
                     ALT_MPFE_HMC_ADP_ECCCTRL2_OVRW_RB_ECC_EN_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL2_RMW_EN_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL2_AUTOWB_EN_SET_MSK,
                     ALT_MPFE_HMC_ADP_ECCCTRL2_OVRW_RB_ECC_EN_SET(0) |
                     ALT_MPFE_HMC_ADP_ECCCTRL2_RMW_EN_SET(1) |
                     ALT_MPFE_HMC_ADP_ECCCTRL2_AUTOWB_EN_SET(1));

    // ECCCTRL1:Enable the ECC
    mmio_clrsetbits_32 (ALT_MPFE_HMC_ADP_OFST +
                     ALT_MPFE_HMC_ADP_ECCCTRL1_OFST,
                     ALT_MPFE_HMC_ADP_ECCCTRL1_AUTOWB_CNT_RST_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_CNT_RST_SET_MSK |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_ECC_EN_SET_MSK,
                     ALT_MPFE_HMC_ADP_ECCCTRL1_AUTOWB_CNT_RST_SET(0) |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_CNT_RST_SET(0) |
                     ALT_MPFE_HMC_ADP_ECCCTRL1_ECC_EN_SET(1));

  } else {
    // No, ECC is disabled.
    console_printf ("\t\t ECC is disabled.\r\n");
  }

}

unsigned long  GetPhysicalDramSize (void)
{
  uint32_t          Data32;
  unsigned long           RamSize;
  unsigned long            RamAddrWidth;
  unsigned long            RamExtIfIoWidth;

  // Get DRAM external interface IO size
  Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST +
                           ALT_MPFE_HMC_ADP_DDRIOCTRL_OFST);
  switch (ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_GET(Data32))
  {
    case 0:
      RamExtIfIoWidth = 16;
      break;
    case 1:
      RamExtIfIoWidth = 32;
      break;
    case 2:
      RamExtIfIoWidth = 64;
      break;
    default:
      RamExtIfIoWidth = 0;
      break;
  }
  // Get RAM Address Width
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_DRAMADDRW_OFST);
  RamAddrWidth = ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32) +
                 ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32) +
                 ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
                 ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32) +
                 ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32);
  // Calcualte total RAM size in number of bytes
  RamSize = (1 << RamAddrWidth) * (RamExtIfIoWidth / 8);

  return RamSize;
}


unsigned long  GetMpuWindowDramSize (void)
{
  unsigned long  RamSize;
  // Considered the case where MPU DRAM windows does not start at 0
  RamSize = GetPhysicalDramSize() - GetMpuWindowDramBaseAddr();
  return RamSize;
}


unsigned long  GetMpuWindowDramBaseAddr (void)
{
  unsigned long   MpuDramWindowsBottom;
  unsigned long   DramBaseAddr = 0;
  uint32_t Data32;

  Data32 =   mmio_read_32 (ARM_MPUL2_OFST + ARM_MPUL2_ADDR_FILTERING_START_OFST);
  assert(ARM_MPUL2_ADDR_FILTERING_START_EN_GET(Data32) ==
       ARM_MPUL2_ADDR_FILTERING_ENABLED);

  MpuDramWindowsBottom = ARM_MPUL2_ADDR_FILTERING_ADDR_GET(Data32);
  DramBaseAddr = MpuDramWindowsBottom;
  return DramBaseAddr;
}


void DisplayMemoryInfo (void)
{
  uint32_t  Data32;
  unsigned long  RamSize;
  unsigned long    RamAddrWidth;
  unsigned long    RamExtIfIoWidth;
  char*  DramTypeAsciiStrPtr;

  // Display RAM external IO size
  Data32 = mmio_read_32 (ALT_MPFE_HMC_ADP_OFST +
                           ALT_MPFE_HMC_ADP_DDRIOCTRL_OFST);
  console_printf (
    "HMC console_printf:\r\n"
    "\t ECC_HMC_OCP_DDRIOCTL: 0x%x\r\n"
    "\t\t IO Size : ",
    Data32);
  switch (ALT_MPFE_HMC_ADP_DDRIOCTRL_IO_SIZE_GET(Data32))
  {
    case 0:
      RamExtIfIoWidth = 16;
      console_printf ("x16\r\n");
      break;
    case 1:
      RamExtIfIoWidth = 32;
      console_printf ("x32\r\n");
      break;
    case 2:
      RamExtIfIoWidth = 64;
      console_printf ("x64\r\n");
      break;
    default:
      RamExtIfIoWidth = 0;
      console_printf ("Unknown\r\n");
      break;
  }

  // Display RAM Address Width
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_DRAMADDRW_OFST);
  console_printf (
    "\t HMC_MMR_DRAMADDRW: 0x%x\r\n"
    "\t\t column address width\t\t: %d bits\r\n"
    "\t\t row address width\t\t: %d bits\r\n"
    "\t\t bank address width\t\t: %d bits\r\n"
    "\t\t bank group address width\t: %d bits\r\n"
    "\t\t chip select address width\t: %d bits\r\n",
    Data32,
    ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32),
    ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32),
    ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32),
    ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32),
    ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32));

  // Display total RAM size
  RamAddrWidth = ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32) +
                   ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32) +
                   ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
                   ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32) +
                   ALT_MPFE_IOHMC_REG_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32);
  RamSize = (1 << RamAddrWidth) * (RamExtIfIoWidth / 8);
  console_printf ("\t\t Memory Size\t\t\t: %lu\r\n", RamSize);

  // Display RAM Type
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST + ALT_MPFE_IOHMC_REG_CTRLCFG0_OFST);
  console_printf (
    "\t HMC_MMR_CTLCFG0: 0x%x\r\n"
    "\t\t Memory Type\t\t\t: ",
    Data32);
  switch (ALT_MPFE_IOHMC_REG_CTRLCFG0_CFG_MEM_TYPE_GET(Data32))
  {
    case 0:
      DramTypeAsciiStrPtr = "DDR3";
      break;
    case 1:
      DramTypeAsciiStrPtr = "DDR4";
      break;
    case 2:
      DramTypeAsciiStrPtr = "LPDDR3";
      break;
    case 3:
      DramTypeAsciiStrPtr = "RLDRAM3";
      break;
    default:
      DramTypeAsciiStrPtr = "Unknown";
      break;
  }
  console_printf ("%s\r\n",  DramTypeAsciiStrPtr);

  //----------------------------------------------------------------------------------

  // Display IO48_HMC_MMR_NIOSRESERVE0
  Data32 = mmio_read_32 (ALT_MPFE_IOHMC_OFST +ALT_MPFE_IOHMC_REG_NIOSRESERVE0_OFST);
  console_printf ("\t IO48_HMC_MMR_NIOSRESERVE0: 0x%x\r\n", Data32);

   //-----------------------------------------------------------------------------
  console_printf ("DDR Scheduler console_printf:\r\n");

  // Display DDR Scheduler DDRCONF register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRCONF_OFST);
  console_printf ("\t DDRCONF: 0x%x\r\n", Data32);

  // Display DDR Scheduler DDRTIMING register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_OFST);
  console_printf (
    "\t DDRTIMING: 0x%x\r\n"
    "\t\t BwRatio \t: %d\r\n"
    "\t\t WrToRd  \t: %d\r\n"
    "\t\t RdToWr  \t: %d\r\n"
    "\t\t BurstLen\t: %d\r\n"
    "\t\t WrToMiss\t: %d\r\n"
    "\t\t RdToMiss\t: %d\r\n"
    "\t\t ActToAct\t: %d\r\n",
    Data32,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BWRATIO_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTORD_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOWR_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_BURSTLEN_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_WRTOMISS_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_RDTOMISS_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRTIMING_ACTTOACT_GET(Data32));

  // Display DDR Scheduler DDRMODE register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_OFST);
  console_printf (
    "\t DDRMODE: 0x%x\r\n"
    "\t\t BwRatioExtended : %d\r\n"
    "\t\t AutoPreCharge   : %d\r\n",
    Data32,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_BWRATIOEXTENDED_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DDRMODE_AUTOPRECHARGE_GET(Data32));

  // Display DDR Scheduler RDLATENCY register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_READLATENCY_OFST);
  console_printf (
    "\t RDLATENCY: 0x%x (%d)\r\n",
     Data32, ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_READLATENCY_READLATENCY_GET(Data32));

  // Display DDR Scheduler ACTIVATE register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_OFST);
  console_printf (
    "\t ACTIVATE: 0x%x\r\n"
    "\t\t FAW BANK : %d\r\n"
    "\t\t FAW      : %d\r\n"
    "\t\t RRD      : %d\r\n",
    Data32,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAWBANK_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_FAW_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_ACTIVATE_RRD_GET(Data32));

  // Display DDR Scheduler DEVTODEV register
  Data32 = mmio_read_32 (ALT_MPFE_DDR_MAIN_SCHED_OFST + ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_OFST);
  console_printf (
    "\t DEVTODEV: 0x%x\r\n"
    "\t\t BusRdToRd : %d\r\n"
    "\t\t BusRdToWr : %d\r\n"
    "\t\t BusWrToRd : %d\r\n",
    Data32,
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTORD_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSRDTOWR_GET(Data32),
    ALT_MPFE_DDR_MAIN_SCHED_DDR_T_MAIN_SCHEDULER_DEVTODEV_BUSWRTORD_GET(Data32));

  //-----------------------------------------------------------------------------

  console_printf("Memory console_printf:\r\n" "\t Physical DRAM Type  = ");
  if ((RamSize >= (1024*1024*1024)) && ((RamSize % 1024) == 0)) {
    console_printf("%lu GB ",
                RamSize / 1024 / 1024 / 1024);
  } else {
    console_printf ( "%lu MB ",
                RamSize / 1024 / 1024);
  }
  console_printf("%s\r\n"
    "\t Physical DRAM Size  = 0x%lx\r\n",
    DramTypeAsciiStrPtr,
    (  unsigned long )GetPhysicalDramSize()
  );

  console_printf("Memory Map : \r\n");
  console_printf("\t SDRAM 0                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_0_LB_ADDR,                     (  unsigned long ) ALT_SDRAM_0_UB_ADDR );
  console_printf("\t FPGA Bridge AA32 1G        = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_FPGA_BRIDGE_AA32_H2F_1G_LB_ADDR,     (  unsigned long ) ALT_FPGA_BRIDGE_AA32_H2F_1G_UB_ADDR);
  console_printf("\t FPGA Bridge AA32 512M      = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_FPGA_BRIDGE_AA32_H2F_512M_LB_ADDR,  (  unsigned long ) ALT_FPGA_BRIDGE_AA32_H2F_512M_UB_ADDR);
  console_printf("\t CCU NOC                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_CCU_NOC_LB_ADDR,                     (  unsigned long ) ALT_CCU_NOC_UB_ADDR);
  console_printf("\t DDR Registers              = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_MPFE_DDR_MAIN_PRB_LB_ADDR,          (  unsigned long ) ALT_MPFE_F2SDR_MGR_MAIN_SBMGR_UB_ADDR);
  console_printf("\t LW FPGA slaves             = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_FPGA_BRIDGE_LWH2F_LB_ADDR,           (  unsigned long ) ALT_FPGA_BRIDGE_LWH2F_UB_ADDR);
  console_printf("\t Noc Cache Clean            = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_NOC_CACHE_CLEAN_LB_ADDR,            (  unsigned long ) ALT_NOC_CACHE_CLEAN_UB_ADDR);
  console_printf("\t SMMU                       = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SMMU_SECURE_LB_ADDR,                (  unsigned long ) ALT_SMMU_SECURE_UB_ADDR);
  console_printf("\t Peripherals Region         = 0x%lx - 0x%lx\r\n",    (  unsigned long ) 0XFF800000,                                 (  unsigned long )  ALT_OCRAM_LB_ADDR);
  console_printf("\t HPS OCRAM                  = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_OCRAM_LB_ADDR,                        (  unsigned long ) ALT_OCRAM_UB_ADDR);
  console_printf("\t GIC                        = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_GIC_DIST_LB_ADDR,                      (  unsigned long ) 0xfffc7fff);
  console_printf("\t SDRAM 1                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_1_LB_ADDR,                      (  unsigned long ) ALT_SDRAM_1_UB_ADDR);
  console_printf("\t SDRAM 2                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_2_LB_ADDR,                      (  unsigned long ) ALT_SDRAM_2_UB_ADDR);
  console_printf("\t SDRAM 3                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_3_LB_ADDR,                      (  unsigned long ) ALT_SDRAM_3_UB_ADDR);
  console_printf("\t SDRAM 4                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_4_LB_ADDR,                      (  unsigned long ) ALT_SDRAM_4_UB_ADDR);
  console_printf("\t SDRAM 5                    = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_SDRAM_5_LB_ADDR,                      (  unsigned long ) ALT_SDRAM_5_UB_ADDR);
  console_printf("\t FPGA Bridge AA64 1G        = 0x%lx - 0x%lx\r\n",    (  unsigned long ) ALT_FPGA_BRIDGE_AA64_H2F_1G_LB_ADDR,   (  unsigned long ) ALT_FPGA_BRIDGE_AA64_H2F_1G_UB_ADDR);
  console_printf("\t FPGA Bridge AA64 512M      = 0x%lx - 0x%lx\r\n",	   (  unsigned long ) ALT_FPGA_BRIDGE_AA64_H2F_512M_LB_ADDR,   (  unsigned long ) ALT_FPGA_BRIDGE_AA64_H2F_512M_UB_ADDR);
  console_printf("\t FPGA Bridge H2F 2.5G       = 0x%lx - 0x%lx\r\n",    (  unsigned long )ALT_FPGA_BRIDGE_H2F_2_5G_LB_ADDR,        (  unsigned long ) ALT_FPGA_BRIDGE_H2F_2_5G_UB_ADDR);


}
