#
# Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

SOC_S10_DIR		:=	plat/intel/soc/stratix10

PLAT_INCLUDES		:=	-Iinclude/plat/arm/common/		\
                        -Iinclude/plat/common/          \
				-Iinclude/plat/arm/common/aarch64/	    \
				-I${SOC_S10_DIR}/		                \
				-I${SOC_S10_DIR}/include/                \
				-I${SOC_S10_DIR}/include/Socal/

PLAT_BL_COMMON_SOURCES	:=	lib/aarch64/xlat_tables.c		\
				plat/common/aarch64/plat_common.c	\
				plat/common/plat_gic.c

BL31_SOURCES	+=	drivers/arm/cci/cci.c				\
			drivers/arm/gic/arm_gic.c			\
			drivers/arm/gic/gic_v2.c			\
			drivers/arm/gic/gic_v3.c			\
			drivers/console/console.S			\
			drivers/delay_timer/delay_timer.c		\
			drivers/delay_timer/generic_delay_timer.c  \
			drivers/ti/uart/16550_console.S			\
			lib/cpus/aarch64/aem_generic.S			\
			lib/cpus/aarch64/cortex_a53.S			\
			lib/semihosting/semihosting.c			\
			lib/semihosting/aarch64/semihosting_call.S \
			plat/common/aarch64/platform_mp_stack.S		\
			plat/common/plat_psci_common.c			\
			${SOC_S10_DIR}/aarch64/plat_helpers.S		\
			${SOC_S10_DIR}/aarch64/platform_common.c	\
			${SOC_S10_DIR}/bl31_plat_setup.c 		\
			${SOC_S10_DIR}/plat_gicv2.c			\
			${SOC_S10_DIR}/plat_psci.c			\
			${SOC_S10_DIR}/plat_topology.c   \
			${SOC_S10_DIR}/plat_delay_timer.c   \
			${SOC_S10_DIR}/soc/ResetManager.c  \
			${SOC_S10_DIR}/soc/MemoryController.c	\
			${SOC_S10_DIR}/soc/Pinmux.c   \
			${SOC_S10_DIR}/soc/SystemManager.c   \
			${SOC_S10_DIR}/soc/Board.c   \
			${SOC_S10_DIR}/soc/ClockManager.c \
			${SOC_S10_DIR}/soc/Handoff.c \
			${SOC_S10_DIR}/soc/Ethernet.c \
			${SOC_S10_DIR}/mmc/AlteraSdMmcMain.c \
            ${SOC_S10_DIR}/mmc/AlteraSdMmcLib.c \
            ${SOC_S10_DIR}/mmc/SdMmc.c \
			${SOC_S10_DIR}/mmc/Diagnostics.c \
            ${SOC_S10_DIR}/mmc/Mmc.c \
            ${SOC_S10_DIR}/mmc/MmcBlockIo.c \
            ${SOC_S10_DIR}/mmc/MmcDebug.c \
            ${SOC_S10_DIR}/mmc/MmcIdentification.c \
            ${SOC_S10_DIR}/mmc/SdMmcHostProtocol.c \
            ${SOC_S10_DIR}/nand/NandLib.c \
            ${SOC_S10_DIR}/semihosting/console_printf.c

## change default flag value
# indicate the reset vector address can be programmed
PROGRAMMABLE_RESET_ADDRESS	:=	1
# bypass BL1 and BL2
RESET_TO_BL31 			:= 	1
# support GCC extension format
DISABLE_PEDANTIC		:= 1
# disable PSCI compatible mode
ENABLE_PLAT_COMPAT := 0

## add defense enable flag for security check
#Stack execution protection:                               LDFLAGS="-z noexecstack" 
#Data relocation and protection (RELRO):        LDLFAGS="-z relro -z now" 
#Stack-based Buffer Overrun Detection:           CFLAGS=”-fstack-protector-strong”if using GCC 4.9 or newer,
#                                                                                 otherwise CFLAGS="-fstack-protector"
#Position Independent Execution (PIE)              CFLAGS="-fPIE -fPIC" LDFLAGS="-pie" (PIE for executables only)
#Fortify source:                                                       CFLAGS="-O2 -D_FORTIFY_SOURCE=2"
#Format string vulnerabilities:                             CFLAGS="-Wformat -Wformat-security"

TF_CFLAGS	+= -Wformat -Wformat-security -O2 -D_FORTIFY_SOURCE=2 -fPIE -fPIC
LDFLAGS		+= -z noexecstack -z relro -z now -pie
