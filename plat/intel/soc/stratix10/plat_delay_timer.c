/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <arch_helpers.h>
#include <delay_timer.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>

#include <platform_def.h>
/********************************************************************
 * The timer delay function
 ********************************************************************/
static uint32_t plat_get_timer_value(void)
{
	/*
	 * Generic delay timer implementation expects the timer to be a down
	 * counter. We apply bitwise NOT operator to the tick values returned
	 * by read_cntpct_el0() to simulate the down counter. The value is
	 * clipped from 64 to 32 bits.
	 */
	return (uint32_t)(~read_cntpct_el0());
}

static const timer_ops_t plat_timer_ops = {
	.get_timer_value    = plat_get_timer_value,
	.clk_mult           = 1,
	.clk_div	    = PLAT_SYS_COUNTER_FREQ_IN_MHZ,
};

void plat_delay_timer_init(void)
{
	uint64_t cval;
	uint32_t ctl = 0;
	uint32_t counter_freq;

	timer_init(&plat_timer_ops);
	// enable global timer
	mmio_write_32 (0xffd01000, 0x3);
	// set timer counter freq
	// this actually will be done at runtime_svc_init, which is after bl31_platform_setup. However we need it earlier. So that is why do it here.
	counter_freq = plat_get_syscnt_freq2();
	write_cntfrq_el0(counter_freq);

	/* The timer will fire every 0.5 second */
	cval = read_cntpct_el0() + (read_cntfrq_el0() >> 1);
	write_cntps_cval_el1(cval);

	/* Enable the secure physical timer */
	set_cntp_ctl_enable(ctl);
	write_cntps_ctl_el1(ctl);
}
