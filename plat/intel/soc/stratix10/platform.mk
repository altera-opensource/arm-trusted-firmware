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
				-Iinclude/plat/arm/common/aarch64/	\
				-I${SOC_S10_DIR}/include/

PLAT_BL_COMMON_SOURCES	:=	lib/aarch64/xlat_tables.c		\
				plat/common/aarch64/plat_common.c	\
				plat/common/plat_gic.c

BL31_SOURCES	+=	drivers/arm/cci/cci.c				\
			drivers/arm/gic/arm_gic.c			\
			drivers/arm/gic/gic_v2.c			\
			drivers/arm/gic/gic_v3.c			\
			drivers/console/console.S			\
			drivers/delay_timer/delay_timer.c		\
			drivers/ti/uart/16550_console.S			\
			lib/cpus/aarch64/aem_generic.S			\
			lib/cpus/aarch64/cortex_a53.S			\
			plat/common/aarch64/platform_mp_stack.S		\
			${SOC_S10_DIR}/aarch64/plat_helpers.S		\
			${SOC_S10_DIR}/aarch64/platform_common.c	\
			${SOC_S10_DIR}/bl31_plat_setup.c 		\
			${SOC_S10_DIR}/plat_delay_timer.c		\
			${SOC_S10_DIR}/plat_gicv2.c			\
			${SOC_S10_DIR}/plat_pm.c			\
			${SOC_S10_DIR}/plat_topology.c



# Enable workarounds for selected Cortex-A53 erratas.
ERRATA_A53_826319		:=	1
ERRATA_A53_836870		:=	1

# indicate the reset vector address can be programmed
PROGRAMMABLE_RESET_ADDRESS	:=	1
RESET_TO_BL31 			:= 	1
# Disable the PSCI platform compatibility layer
ENABLE_PLAT_COMPAT		:= 	1
