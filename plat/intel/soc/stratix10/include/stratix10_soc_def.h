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

#ifndef __PLAT_DEF_H__
#define __PLAT_DEF_H__


#define PLAT_GIC_BASE			(0xFFFC0000)
#define PLAT_GICC_BASE			(PLAT_GIC_BASE + 0x2000)
#define PLAT_GICD_BASE			(PLAT_GIC_BASE + 0x1000)
#define PLAT_GICR_BASE			0

/*******************************************************************************
 * UART related constants
 ******************************************************************************/
#define PLAT_UART0_BASE		(0xFFC02000)
#define PLAT_UART1_BASE		(0xFFC02100)

#define PLAT_BAUDRATE			(115200)
#define PLAT_UART_CLOCK		(100000000)

/*******************************************************************************
 * System counter frequency related constants
 ******************************************************************************/
#define PLAT_SYS_COUNTER_FREQ_IN_TICKS	(24000000)
#define PLAT_SYS_COUNTER_FREQ_IN_MHZ	(24)

#define PLAT_TMR_SP0_BASE		(0xFFC03000)
#define PLAT_TMR_SP1_BASE		(0xFFC03100)

#endif /* __PLAT_DEF_H__ */
