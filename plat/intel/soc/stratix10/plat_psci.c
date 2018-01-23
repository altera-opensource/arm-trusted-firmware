/*
 * Copyright (c) 2013-2016, ARM Limited and Contributors. All rights reserved.
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

#include <arch_helpers.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
//#include <gicv2.h>
#include <mmio.h>
#include <plat_arm.h>
#include <platform.h>
#include <psci.h>

#include "platform_def.h"
#include "platform_private.h"

#include "Altera_Hps_Socal.h"
#include "soc/ResetManager.h"
#include "soc/s10_mailbox.h"

uintptr_t stratix10_sec_entry;

/*******************************************************************************
 * plat handler called when a CPU is about to enter standby.
 ******************************************************************************/
void plat_cpu_standby(plat_local_state_t cpu_state)
{
	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	VERBOSE("%s: cpu_state: 0x%x\n", __func__, cpu_state);
	dsb();
	wfi();
}

/*******************************************************************************
 * plat handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
int plat_pwr_domain_on(u_register_t mpidr)
{
	unsigned int cpu_id = plat_core_pos_by_mpidr(mpidr);

	VERBOSE("%s: mpidr: 0x%lx\n", __func__, mpidr);

	if (cpu_id == -1)
		return PSCI_E_INTERN_FAIL;

	/* release core reset */
	mmio_clrbits_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPUMODRST_OFST, 1 << cpu_id);
	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * plat handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void plat_pwr_domain_off(const psci_power_state_t *target_state)
{
	unsigned int cpu_id = plat_my_core_pos();

	for (size_t i = 0; i <= PLAT_MAX_PWR_LVL; i++)
		VERBOSE("%s: target_state->pwr_domain_state[%lu]=%x\n",
			__func__, i, target_state->pwr_domain_state[i]);

	/* Prevent interrupts from spuriously waking up this cpu */
	//gicv2_cpuif_disable();

	/* assert core reset */
	mmio_setbits_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPUMODRST_OFST, 1 << cpu_id);
}

/*******************************************************************************
 * plat handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void plat_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	unsigned int cpu_id = plat_my_core_pos();

	for (size_t i = 0; i <= PLAT_MAX_PWR_LVL; i++)
		VERBOSE("%s: target_state->pwr_domain_state[%lu]=%x\n",
			__func__, i, target_state->pwr_domain_state[i]);
	/* assert core reset */
	mmio_setbits_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPUMODRST_OFST, 1 << cpu_id);

}

/*******************************************************************************
 * plat handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void plat_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	unsigned int cpu_id = plat_my_core_pos();

	for (size_t i = 0; i <= PLAT_MAX_PWR_LVL; i++)
		VERBOSE("%s: target_state->pwr_domain_state[%lu]=%x\n",
			__func__, i, target_state->pwr_domain_state[i]);
	/* release core reset */
	mmio_clrbits_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPUMODRST_OFST, 1 << cpu_id);
}

/*******************************************************************************
 * plat handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void plat_pwr_domain_suspend_finish(const psci_power_state_t *target_state)
{
	unsigned int cpu_id = plat_my_core_pos();

	for (size_t i = 0; i <= PLAT_MAX_PWR_LVL; i++)
		VERBOSE("%s: target_state->pwr_domain_state[%lu]=%x\n",
			__func__, i, target_state->pwr_domain_state[i]);

	/* release core reset */
	mmio_clrbits_32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPUMODRST_OFST, 1 << cpu_id);
}

/*******************************************************************************
 * plat handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 plat_system_off(void)
{
	wfi();
	ERROR("System Off: operation not handled.\n");
	/* disable coherency */
	//plat_arm_interconnect_exit_coherency();
	panic();
}

static void __dead2 plat_system_reset(void)
{
	/* disable coherency */
	//plat_arm_interconnect_exit_coherency();
	/* trigger system reset */
    INFO ("assert Peripheral from Reset\r\n");

	DeassertPeripheralsReset();
    mailbox_reset_cold();

	while (1)
		wfi();
}

int plat_validate_power_state(unsigned int power_state,
				psci_power_state_t *req_state)
{
	VERBOSE("%s: power_state: 0x%x\n", __func__, power_state);

	//return arm_validate_power_state(power_state, req_state);
	return PSCI_E_SUCCESS;
}

int plat_validate_ns_entrypoint(unsigned long ns_entrypoint)
{
	VERBOSE("%s: ns_entrypoint: 0x%lx\n", __func__, ns_entrypoint);
	return PSCI_E_SUCCESS;
}

void plat_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	req_state->pwr_domain_state[PSCI_CPU_PWR_LVL] = PLAT_MAX_OFF_STATE;
	req_state->pwr_domain_state[1] = PLAT_MAX_OFF_STATE;
}

/*******************************************************************************
 * Export the platform handlers via plat_arm_psci_pm_ops. The ARM Standard
 * platform layer will take care of registering the handlers with PSCI.
 ******************************************************************************/
const plat_psci_ops_t plat_psci_pm_ops = {
	.cpu_standby = plat_cpu_standby,
	.pwr_domain_on = plat_pwr_domain_on,
	.pwr_domain_off = plat_pwr_domain_off,
	.pwr_domain_suspend = plat_pwr_domain_suspend,
	.pwr_domain_on_finish = plat_pwr_domain_on_finish,
	.pwr_domain_suspend_finish = plat_pwr_domain_suspend_finish,
	.system_off = plat_system_off,
	.system_reset = plat_system_reset,
	.validate_power_state = plat_validate_power_state,
	.validate_ns_entrypoint = plat_validate_ns_entrypoint,
	.get_sys_suspend_power_state = plat_get_sys_suspend_power_state
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const struct plat_psci_ops **psci_ops)
{
	/* Save warm boot entrypoint.*/
	stratix10_sec_entry = sec_entrypoint;

	*psci_ops = &plat_psci_pm_ops;
	return 0;
}
