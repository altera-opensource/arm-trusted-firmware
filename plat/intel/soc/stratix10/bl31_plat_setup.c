/*
 * Copyright (c) 2013-2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bl_common.h>
#include <console.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <delay_timer.h>
#include <generic_delay_timer.h>
#include <console_printf.h>
#include <platform_def.h>
#include <platform_private.h>

#include "Base.h"
#include "Altera_Hps_Socal.h"
#include "soc/Ethernet.h"
#include "soc/Handoff.h"
#include "soc/ResetManager.h"
#include "soc/MemoryController.h"
#include "soc/Pinmux.h"
#include "soc/ClockManager.h"
#include "soc/SystemManager.h"
#include "soc/Board.h"
#include "mmc/SdMmc.h"
#include "nand/NandLib.h"

typedef enum {
	BOOT_SOURCE_SDMMC = 0,
    BOOT_SOURCE_FPGA,
    BOOT_SOURCE_NAND,
    BOOT_SOURCE_RSVD,
	BOOT_SOURCE_QSPI,
} BOOT_SOURCE_TYPE;
/*******************************************************************************
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted SRAM
 ******************************************************************************/
unsigned long __RO_START__;
unsigned long __RO_END__;

unsigned long __COHERENT_RAM_START__;
unsigned long __COHERENT_RAM_END__;

/*
 * The next 2 constants identify the extents of the code & RO data region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __RO_START__ and __RO_END__ linker symbols refer to page-aligned addresses.
 */
#define BL31_RO_BASE (unsigned long)(&__RO_START__)
#define BL31_RO_LIMIT (unsigned long)(&__RO_END__)

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols
 * refer to page-aligned addresses.
 */
#define BL31_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL31_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

static entry_point_info_t bl32_image_ep_info;
static entry_point_info_t bl33_image_ep_info;

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL33 corresponds to the non-secure image type
 * while BL32 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	next_image_info = (type == NON_SECURE) ?
			  &bl33_image_ep_info : &bl32_image_ep_info;

	/* None of the images on this platform can have 0x0 as the entrypoint */
	if (next_image_info->pc)
		return next_image_info;
	else
		return NULL;
}

void SerialPortDisplayInfoForTheFirstTime (void)
{
  INFO("Arm Trusted Firmware for S10 SOC FPGA\n");
  // Display System Manager Info
  DisplaySystemManagerInfo ();

  // Display Reset Manager Info
  DisplayResetManagerInfo ();

  // Display PinMux Info
  //DisplayIo48PinMuxInfo ();

  // Display Clock Manager Info
  DisplayClockManagerInfo ();

}

void enable_nonsecure_access (void)
{
	// this enables nonsecure access to UART0
	mmio_write_32(ALT_NOC_FW_L4_PER_SCR_OFST +
                  ALT_NOC_FW_L4_PER_SCR_UART0_OFST,
				  0x1);

    // enables nonsecure MPU accesses to SDMMC
	mmio_write_32(ALT_NOC_FW_L4_PER_SCR_OFST +
                  ALT_NOC_FW_L4_PER_SCR_SDMMC_OFST,
                  0x1);

	// enables nonsecure access to all the emacs
	mmio_write_32(ALT_NOC_FW_L4_PER_SCR_OFST +
	              ALT_NOC_FW_L4_PER_SCR_EMAC0_OFST,
				  0x1 | BIT24 | BIT16);
	mmio_write_32(ALT_NOC_FW_L4_PER_SCR_OFST +
	              ALT_NOC_FW_L4_PER_SCR_EMAC1_OFST,
				  0x1 | BIT24 | BIT16 );
	mmio_write_32(ALT_NOC_FW_L4_PER_SCR_OFST +
	              ALT_NOC_FW_L4_PER_SCR_EMAC2_OFST,
				  0x1 | BIT24 | BIT16 );

	mmio_write_32(ALT_NOC_FW_L4_SYS_SCR_OFST +
	              ALT_NOC_FW_L4_SYS_SCR_EMAC0RX_ECC_OFST,
				  0x1 | BIT24 | BIT16 );
	mmio_write_32(ALT_NOC_FW_L4_SYS_SCR_OFST +
	              ALT_NOC_FW_L4_SYS_SCR_EMAC0TX_ECC_OFST,
				  0x1 | BIT24 | BIT16 );

	// this enables nonsecure access to OCRAM ECC for flash DMA transaction AXI API & MPU
	mmio_write_32(ALT_NOC_FW_L4_SYS_SCR_OFST +
	              ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_OFST,
	              ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_AXI_AP_SET_MSK |
			      ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_MPU_SET_MSK);

	// disable ocram security at CCU, temporary hack
	mmio_clrbits_32(ALT_CCU_NOC_OFST +
	                 ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADMASK_MEM_RAM_SPRT_RAMSPACE0_0_OFST,
					 0x03);
	mmio_clrbits_32(ALT_CCU_NOC_OFST +
	                 ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADMASK_MEM_RAM_SPRT_RAMSPACE0_0_OFST,
					 0x03);
}

BOOT_SOURCE_TYPE GetBootSourceTypeHandoff (handoff *hoff_ptr)
{
  return hoff_ptr->boot_source;
}

BOOT_SOURCE_TYPE GetBootSourceType (void)
{
  return BOOT_SOURCE;
}


/*******************************************************************************
 * Perform any BL3-1 early platform setup. Here is an opportunity to copy
 * parameters passed by the calling EL (S-EL1 in BL2 & S-EL3 in BL1) before they
 * are lost (potentially). This needs to be done before the MMU is initialized
 * so that the memory layout can be used while creating page tables.
 * BL2 has flushed this information to memory, so we are guaranteed to pick up
 * good data.
 ******************************************************************************/
void bl31_early_platform_setup(bl31_params_t *from_bl2,
			       void *plat_params_from_bl2)
{
	BOOT_SOURCE_TYPE  BootSourceType;
#ifndef VIRTUAL_PLATFORM
 #ifndef ENABLE_HANDOFF
	console_printf("handoff is disabled\n");
	ConfigPinMux();
	ConfigureClockManager ();
	BootSourceType = GetBootSourceType();
 #else
	console_printf("verify handoff image\n");
	handoff *handoff_ptr = (handoff*) PLAT_HANDOFF_OFFSET;
	handoff reverse_handoff_ptr;
	if (verify_handoff_image(handoff_ptr, &reverse_handoff_ptr))  {
		console_printf("invalid hand off image\n");
		return;
	}
	ConfigPinMuxHandoff(&reverse_handoff_ptr);
	ConfigureClockManagerHandoff (&reverse_handoff_ptr);
	// Detect Boot Source Type
    BootSourceType = GetBootSourceTypeHandoff (&reverse_handoff_ptr);
 #endif

    // Reset Deassertion through the Reset Manager
	DeassertPeripheralsReset ();
	// Reset manager handshake with other modules before warm reset.
	ConfigureHpsSubsystemHandshakeBehaviorBeforeWarmReset ();

 #ifdef EMULATOR
	// enable pin mux USE FPGA
	console_printf("enable pin mux USE FPGA\n");
	mmio_write_32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_UART0_USEFPGA_OFST, 0x1);
	mmio_write_32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_UART1_USEFPGA_OFST, 0x1);
	mmio_write_32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_SDMMC_USEFPGA_OFST, 0x1);
 #endif
	// enable nonsecure access
	enable_nonsecure_access ();
	// init some emac registers that cannot be done in EL2
	emac_init();

#else
	BootSourceType = GetBootSourceType();
    // Reset Deassertion through the Reset Manager
	DeassertPeripheralsReset ();
#endif
	// init uart console
	console_init(PLAT_UART0_BASE, PLAT_UART_CLOCK, PLAT_BAUDRATE);
	// display console
	SerialPortDisplayInfoForTheFirstTime ();
	// init timer
	INFO ("Init Timer\n");
	plat_delay_timer_init();
	//udelay (1000);
	// init DDR
	INFO ("Init Hard Memory Controller\n");
	InitHardMemoryController ();
	DisplayMemoryInfo ();

    // Flash Device initialization
    switch (BootSourceType)
    {
      case BOOT_SOURCE_NAND:
        // Init NAND
        NandInit ();
        break;
      case BOOT_SOURCE_SDMMC:
        // Init SDMMC
        InitSdMmc();
		// Load next boot image
		LoadBootImageFile (PLAT_NS_IMAGE_NAME, PLAT_NS_IMAGE_OFFSET);
        break;
      case BOOT_SOURCE_RSVD:
      case BOOT_SOURCE_FPGA:
      default:
        // No Flash device.
        INFO ("No Flash Device Available!\r\n");
        break;
    }

	BoardSpecificInitialization ();

#if RESET_TO_BL31
	/* There are no parameters from BL2 if BL31 is a reset vector */
	assert(from_bl2 == NULL);
	assert(plat_params_from_bl2 == NULL);

#ifdef BL32_BASE
	/* Populate entry point information for BL32 */
	SET_PARAM_HEAD(&bl32_image_ep_info,
				PARAM_EP,
				VERSION_1,
				0);
	SET_SECURITY_STATE(bl32_image_ep_info.h.attr, SECURE);
	bl32_image_ep_info.pc = BL32_BASE;
	bl32_image_ep_info.spsr = plat_get_spsr_for_bl32_entry();
#endif /* BL32_BASE */

	/* Populate entry point information for BL33 */
	SET_PARAM_HEAD(&bl33_image_ep_info,
				PARAM_EP,
				VERSION_1,
				0);
	/*
	 * Tell BL31 where the non-trusted software image
	 * is located and the entry state information
	 */
	bl33_image_ep_info.pc = plat_get_ns_image_entrypoint();
	bl33_image_ep_info.spsr = plat_get_spsr_for_bl33_entry();
	SET_SECURITY_STATE(bl33_image_ep_info.h.attr, NON_SECURE);

#else
	/*
	 * Check params passed from BL2 should not be NULL,
	 */
	assert(from_bl2 != NULL);
	assert(from_bl2->h.type == PARAM_BL31);
	assert(from_bl2->h.version >= VERSION_1);
	/*
	 * In debug builds, we pass a special value in 'plat_params_from_bl2'
	 * to verify platform parameters from BL2 to BL31.
	 * In release builds, it's not used.
	 */
	assert(((unsigned long long)plat_params_from_bl2) ==
		ARM_BL31_PLAT_PARAM_VAL);

	/*
	 * Copy BL32 (if populated by BL2) and BL33 entry point information.
	 * They are stored in Secure RAM, in BL2's address space.
	 */
	if (from_bl2->bl32_ep_info)
		bl32_image_ep_info = *from_bl2->bl32_ep_info;
	bl33_image_ep_info = *from_bl2->bl33_ep_info;
#endif
}

/*******************************************************************************
 * Perform any BL3-1 platform setup code
 ******************************************************************************/
void bl31_platform_setup(void)
{
	INFO ("Initialize the gic cpu and distributor interfaces\n");
	/* Initialize the gic cpu and distributor interfaces */
	plat_gic_driver_init();
	plat_arm_gic_init();
}

/*******************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only intializes the mmu in a quick and dirty way.
 ******************************************************************************/
void bl31_plat_arch_setup(void)
{
	plat_configure_mmu_el3(BL31_RO_BASE,
			       (BL31_COHERENT_RAM_LIMIT - BL31_RO_BASE),
			       BL31_RO_BASE,
			       BL31_RO_LIMIT,
			       BL31_COHERENT_RAM_BASE,
			       BL31_COHERENT_RAM_LIMIT);

}

