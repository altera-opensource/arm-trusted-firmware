/*
 * Copyright (c) 2014-2015, ARM Limited and Contributors. All rights reserved.
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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

/*******************************************************************************
 * User configuration
 ******************************************************************************/
//#define EMULATOR
//#define VIRTUAL_PLATFORM
#define ENABLE_HANDOFF
//#define PLAT_SEMIHOSTING_ENABLE

/* Define next boot image name and offset */
#define PLAT_NS_IMAGE_NAME				"PEI.ROM"
#define PLAT_NS_IMAGE_OFFSET			0x50000
#define PLAT_HANDOFF_OFFSET 			0xFFE3F000
#define BOOT_SOURCE                     BOOT_SOURCE_SDMMC

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT			"elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH			aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/
#define PLAT_PRIMARY_CPU			0
#define PLAT_SECONDARY_ENTRY_BASE            0x01f78bf0

/* Size of cacheable stacks */
#define PLATFORM_STACK_SIZE			0x5000

/* PSCI related constant */
#define PLAT_NUM_POWER_DOMAINS		5
#define PLAT_MAX_PWR_LVL		1
#define PLAT_MAX_RET_STATE		1
#define PLAT_MAX_OFF_STATE		2
#define PLATFORM_SYSTEM_COUNT			1
#define PLATFORM_CLUSTER_COUNT			1
#define PLATFORM_CLUSTER0_CORE_COUNT		4
#define PLATFORM_CLUSTER1_CORE_COUNT		0
#define PLATFORM_CORE_COUNT			(PLATFORM_CLUSTER1_CORE_COUNT + \
						PLATFORM_CLUSTER0_CORE_COUNT)
#define PLATFORM_MAX_CPUS_PER_CLUSTER		4

/* Interrupt related constant */

#define ARM_IRQ_SEC_PHY_TIMER		29

#define ARM_IRQ_SEC_SGI_0		8
#define ARM_IRQ_SEC_SGI_1		9
#define ARM_IRQ_SEC_SGI_2		10
#define ARM_IRQ_SEC_SGI_3		11
#define ARM_IRQ_SEC_SGI_4		12
#define ARM_IRQ_SEC_SGI_5		13
#define ARM_IRQ_SEC_SGI_6		14
#define ARM_IRQ_SEC_SGI_7		15

#define TSP_IRQ_SEC_PHY_TIMER		ARM_IRQ_SEC_PHY_TIMER
#define TSP_SEC_MEM_BASE		BL32_BASE
#define TSP_SEC_MEM_SIZE		(BL32_LIMIT - BL32_BASE + 1)
/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/
#define DRAM_BASE				(0x0)
#define DRAM_SIZE				(0x80000000)

#define OCRAM_BASE				(0xFFE00000)
#define OCRAM_SIZE				(0x00100000)

#define MEM64_BASE				(0x0100000000)
#define MEM64_SIZE				(0x1F00000000)

#define DEVICE1_BASE				(0x80000000)
#define DEVICE1_SIZE				(0x60000000)

#define DEVICE2_BASE				(0xF7000000)
#define DEVICE2_SIZE				(0x08E00000)

#define DEVICE3_BASE				(0xFFFC0000)
#define DEVICE3_SIZE				(0x00008000)

#define DEVICE4_BASE				(0x2000000000)
#define DEVICE4_SIZE				(0x0100000000)

/*******************************************************************************
 * BL31 specific defines.
 ******************************************************************************/
/*
 * Put BL3-1 at the top of the Trusted SRAM (just below the shared memory, if
 * present). BL31_BASE is calculated using the current BL3-1 debug size plus a
 * little space for growth.
 */
#define BL31_BASE				(OCRAM_BASE)
#define BL31_LIMIT				(OCRAM_BASE + OCRAM_SIZE)

# define BL32_BASE			     0x60000000
# define BL32_LIMIT			     0x7fffffff

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define ADDR_SPACE_SIZE			(1ull << 48)
#define MAX_XLAT_TABLES			8
#define MAX_MMAP_REGIONS			16

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT			6
#define CACHE_WRITEBACK_GRANULE		(1 << CACHE_WRITEBACK_SHIFT)

#define PLAT_GIC_BASE			(0xFFFC0000)
#define PLAT_GICC_BASE			(PLAT_GIC_BASE + 0x2000)
#define PLAT_GICD_BASE			(PLAT_GIC_BASE + 0x1000)
#define PLAT_GICR_BASE			0

/*******************************************************************************
 * UART related constants
 ******************************************************************************/
#define PLAT_UART0_BASE		(0xFFC02000)
#define PLAT_UART1_BASE		(0xFFC02100)

#ifdef EMULATOR
// emulator
#define PLAT_BAUDRATE			(4800)
#define PLAT_UART_CLOCK		(74000)
# else
#define PLAT_BAUDRATE			(115200)
#define PLAT_UART_CLOCK		(100000000)
#endif

/*******************************************************************************
 * System counter frequency related constants
 ******************************************************************************/
#define PLAT_SYS_COUNTER_FREQ_IN_TICKS	(24000000)
#define PLAT_SYS_COUNTER_FREQ_IN_MHZ	(24)

#define PLAT_ARM_GICD_BASE	PLAT_GICD_BASE
#define PLAT_ARM_GICC_BASE	PLAT_GICC_BASE

/*
 * Define a list of Group 1 Secure and Group 0 interrupts as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define PLAT_ARM_G1S_IRQS	ARM_IRQ_SEC_PHY_TIMER, \
				ARM_IRQ_SEC_SGI_0,	\
				ARM_IRQ_SEC_SGI_1,	\
				ARM_IRQ_SEC_SGI_2,	\
				ARM_IRQ_SEC_SGI_3,	\
				ARM_IRQ_SEC_SGI_4,	\
				ARM_IRQ_SEC_SGI_5,	\
				ARM_IRQ_SEC_SGI_6,	\
				ARM_IRQ_SEC_SGI_7

#define PLAT_ARM_G0_IRQS
#endif /* __PLATFORM_DEF_H__ */

