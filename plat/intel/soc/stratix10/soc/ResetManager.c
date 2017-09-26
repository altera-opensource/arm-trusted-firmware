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
#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <platform_private.h>

#include <console_printf.h>
#include "Altera_Hps_Socal.h"
#include <console_printf.h>

//
// Functions
//

void DeassertPeripheralsReset (void)
{
  console_printf ("Deassert Peripheral from Reset\r\n");

  //
  // De-assert Reset to Peripheral Group 1 components
  //

  mmio_clrbits_32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER1MODRST_OFST,
			//ALT_RSTMGR_PER1MODRST_WATCHDOG0_SET_MSK |
			//ALT_RSTMGR_PER1MODRST_WATCHDOG1_SET_MSK |
			//ALT_RSTMGR_PER1MODRST_WATCHDOG2_SET_MSK |
			//ALT_RSTMGR_PER1MODRST_WATCHDOG3_SET_MSK |
			ALT_RSTMGR_PER1MODRST_L4SYSTIMER0_SET_MSK |
			ALT_RSTMGR_PER1MODRST_L4SYSTIMER1_SET_MSK |
			ALT_RSTMGR_PER1MODRST_SPTIMER0_SET_MSK |
			ALT_RSTMGR_PER1MODRST_SPTIMER1_SET_MSK |
			ALT_RSTMGR_PER1MODRST_I2C0_SET_MSK |
			ALT_RSTMGR_PER1MODRST_I2C1_SET_MSK |
			ALT_RSTMGR_PER1MODRST_I2C2_SET_MSK |
			ALT_RSTMGR_PER1MODRST_I2C3_SET_MSK |
			ALT_RSTMGR_PER1MODRST_I2C4_SET_MSK |
			ALT_RSTMGR_PER1MODRST_UART0_SET_MSK |
			ALT_RSTMGR_PER1MODRST_UART1_SET_MSK |
			ALT_RSTMGR_PER1MODRST_GPIO0_SET_MSK |
			ALT_RSTMGR_PER1MODRST_GPIO1_SET_MSK);

  // De-assert Reset to Peripheral Group 0 components two step:
  // First, all none ECC_OCP components, followed by ECC_OCP components
  // NOTE:
  // When asserting reset, we have to set the ECC OCP last
  // When deasserting reset, we have to set the ECC OCP first

  // De-assert reset of ECC_OCP components
  mmio_clrbits_32 (ALT_RSTMGR_OFST +
               ALT_RSTMGR_PER0MODRST_OFST,
			   ALT_RSTMGR_PER0MODRST_EMAC0OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_EMAC1OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_EMAC2OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_USB0OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_USB1OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_NANDOCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_SDMMCOCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_DMAOCP_SET_MSK);
  // De-assert reset of none ECC_OCP components

  mmio_clrbits_32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER0MODRST_OFST,
			 ALT_RSTMGR_PER0MODRST_EMAC0_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_EMAC1_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_EMAC2_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_USB0_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_USB1_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_NAND_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_SDMMC_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMA_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_SPIM0_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_SPIM1_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_SPIS0_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_SPIS1_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_EMACPTP_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF0_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF1_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF2_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF3_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF4_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF5_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF6_SET_MSK |
			 ALT_RSTMGR_PER0MODRST_DMAIF7_SET_MSK);

}


void ConfigureHpsSubsystemHandshakeBehaviorBeforeWarmReset (void)
{
  uint32_t OrMask;

  console_printf ("Config HPS Handshake\r\n");

  // Enable hardware handshake with other modules before warm reset.
  OrMask = 0;
  OrMask |= ALT_RSTMGR_HDSKEN_SDRSELFREFEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_FPGAHSEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_ETRSTALLEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_L2FLUSHEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_L3NOC_DBG_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_DEBUG_L3NOC_SET_MSK;

  mmio_setbits_32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_HDSKEN_OFST,
            OrMask);
}


void DisplayResetManagerInfo(void)
{
  uint32_t          Data32;

  // Display logged reset request source(s) de-assert the request in the same cycle
  Data32 = mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_STAT_OFST);
  console_printf ( "Reset Request Source(s): 0x%x\r\n",
              Data32);
  if (ALT_RSTMGR_STAT_SDMCOLDRST_GET(Data32) == 1)
    console_printf ("\t SDM Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_SDMWARMRST_GET(Data32) == 1)
    console_printf ("\t SDM Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_SDMLASTPORRST_GET(Data32) == 1)
    console_printf ("\t nSDM last POR Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU0RST_GET(Data32) == 1)
    console_printf ("\t MPU0 Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU1RST_GET(Data32) == 1)
    console_printf ("\t MPU1 Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU2RST_GET(Data32) == 1)
    console_printf ("\t MPU2 Resett \r\n");
  if (ALT_RSTMGR_STAT_MPU3RST_GET(Data32) == 1)
    console_printf ("\t MPU3 Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD0RST_GET(Data32) == 1)
    console_printf ("\t MPU Watchdog 0 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD1RST_GET(Data32) == 1)
    console_printf ("\t MPU Watchdog 1 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD2RST_GET(Data32) == 1)
    console_printf ("\t MPU Watchdog 2 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD3RST_GET(Data32) == 1)
    console_printf ("\t MPU Watchdog 3 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_DEBUGRST_GET(Data32) == 1)
    console_printf ("\t  FPGA Core Debug Reset \r\n");
  if (ALT_RSTMGR_STAT_CSDAPRST_GET(Data32) == 1)
    console_printf ("\t CS DAP Reset \r\n");


  // Display Reset Manager Register
  console_printf ("RST_MGR MPU Status : 0x%x\r\n"
             "RST_MGR MISC Status: 0x%x\r\n"
             "RST_MGR HdSkEn reg : 0x%x\r\n"
             "RST_MGR HdSkAck reg: 0x%x\r\n"
             "RST_MGR Peripherals Reset State:\r\n"
             "\tPER0MODRST: 0x%x\r\n\tPER0M1DRST: 0x%x\r\n",
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPURSTSTAT_OFST),
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MISCSTAT_OFST),
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKEN_OFST),
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKACK_OFST),
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER0MODRST_OFST),
              mmio_read_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER1MODRST_OFST));

}

void EnableHpsAndFpgaBridges (void)
{
  // Clear NOC idle request
  mmio_write_32 (ALT_SYSMGR_CORE_OFST +
              ALT_SYSMGR_CORE_NOC_IDLEREQ_CLR_OFST,
			  ~ALT_SYSMGR_CORE_NOC_IDLEREQ_CLR_RESET);

  // De-assert all the bridge reset
  mmio_write_32 (ALT_RSTMGR_OFST +
              ALT_RSTMGR_BRGMODRST_OFST,
			  ALT_RSTMGR_BRGMODRST_RESET);

  // Poll until all noc idle request ack to 0
  while (mmio_read_32(ALT_SYSMGR_CORE_OFST +
                     ALT_SYSMGR_CORE_NOC_IDLEACK_OFST));
}

void DisableHpsAndFpgaBridges (void)
{
  // Set NOC idle request
  mmio_write_32 (ALT_SYSMGR_CORE_OFST +
              ALT_SYSMGR_CORE_NOC_IDLEREQ_SET_OFST,
			  ~ALT_SYSMGR_CORE_NOC_IDLEREQ_SET_RESET);
  // Enable the NOC timeout
  mmio_setbits_32(ALT_SYSMGR_CORE_OFST +
             ALT_SYSMGR_CORE_NOC_TIMEOUT_OFST,
			 ALT_SYSMGR_CORE_NOC_TIMEOUT_EN_SET_MSK);


  // Poll until all noc idle request ack to 1
  while (mmio_read_32(ALT_SYSMGR_CORE_OFST +
         ALT_SYSMGR_CORE_NOC_IDLEACK_OFST) !=
        (ALT_SYSMGR_CORE_NOC_IDLEACK_SOC2FPGA_SET_MSK |
		 ALT_SYSMGR_CORE_NOC_IDLEACK_LWSOC2FPGA_SET_MSK));

  // Poll until all noc idle status to 1
  while (mmio_read_32(ALT_SYSMGR_CORE_OFST +
                     ALT_SYSMGR_CORE_NOC_IDLESTATUS_OFST) !=
                    (ALT_SYSMGR_CORE_NOC_IDLESTATUS_SOC2FPGA_SET_MSK |
 		             ALT_SYSMGR_CORE_NOC_IDLESTATUS_LWSOC2FPGA_SET_MSK));

  // Assert all bridges reset except DDR schdule
  mmio_setbits_32 (ALT_RSTMGR_OFST +
              ALT_RSTMGR_BRGMODRST_OFST,
			  ALT_RSTMGR_BRGMODRST_SOC2FPGA_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_LWHPS2FPGA_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_FPGA2SOC_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM0_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM1_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM2_SET_MSK
			  );
}