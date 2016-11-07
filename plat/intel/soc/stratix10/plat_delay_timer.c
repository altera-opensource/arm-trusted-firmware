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
#include <delay_timer.h>
#include <mmio.h>
#include <stratix10_soc_def.h>

#define TIMER1_LOADCNT_OFFSET     (0x000)
#define TIMER1_CURRENTVAL_OFFSET  (0x004)
#define TIMER1_CONTROL_OFFSET     (0x008)

#define TIMER_CTRL_ENABLE         (1 << 0)
#define TIMER_CTRL_DISABLE        (0 << 0)
#define TIMER_CTRL_MODE_USEDEF    (1 << 1)
#define TIMER_CTRL_MODE_FREERUN  ~(1 << 1)
#define TIMER_INTR_MASK           (1 << 2)

/********************************************************************
 * The timer delay function
 ********************************************************************/
uint32_t plat_get_timer_value(void)
{
	return mmio_read_32(PLAT_TMR_SP0_BASE + TIMER1_CURRENTVAL_OFFSET);
}
static const timer_ops_t plat_timer_ops = {
	.get_timer_value    = plat_get_timer_value,
	.clk_mult           = 1,
	.clk_div	    = PLAT_SYS_COUNTER_FREQ_IN_MHZ,
};

void plat_delay_timer_init(void)
{
	timer_init(&plat_timer_ops);

	/* disable timer1 */
	mmio_write_32(PLAT_TMR_SP0_BASE + TIMER1_CONTROL_OFFSET,
		      TIMER_CTRL_DISABLE);

	/* configure the timer for user defined count mode */
	mmio_clrbits_32(PLAT_TMR_SP0_BASE + TIMER1_CONTROL_OFFSET,
			TIMER_CTRL_MODE_FREERUN);

	/* program the timer with new count value */
	mmio_write_32(PLAT_TMR_SP0_BASE + TIMER1_LOADCNT_OFFSET,
		      UINT32_MAX);

	/* enable timer */
	mmio_setbits_32(PLAT_TMR_SP0_BASE + TIMER1_CONTROL_OFFSET,
			TIMER_CTRL_ENABLE);
}
