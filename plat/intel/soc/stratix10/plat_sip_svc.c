/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
#include <debug.h>
#include <mmio.h>
#include <runtime_svc.h>
#include <uuid.h>

/* SMC function IDs for SiP Service queries */
#define SIP_SVC_CALL_COUNT	  0x8200ff00
#define SIP_SVC_UID		      0x8200ff01
#define SIP_SVC_VERSION		  0x8200ff03

/* SiP Service Calls version numbers */
#define SIP_SVC_VERSION_MAJOR	0
#define SIP_SVC_VERSION_MINOR	1

/* Number of SiP Calls implemented */
#define SIP_NUM_CALLS		0x3

/*  SiP Service UUID */
// TODO: Change UUID
DEFINE_SVC_UUID(sip_svc_uid,
		0xe86fc7e2, 0x313e, 0x11e6, 0xb7, 0x0d,
		0x8f, 0x88, 0xee, 0x74, 0x7b, 0x72);

uint64_t plat_sip_handler(uint32_t smc_fid,
				   uint64_t x1,
				   uint64_t x2,
				   uint64_t x3,
				   uint64_t x4,
				   void *cookie,
				   void *handle,
				   uint64_t flags)
{
	ERROR("%s: unhandled SMC (0x%x)\n", __func__, smc_fid);
	SMC_RET1(handle, SMC_UNK);
}

/*
 * This function is responsible for handling all SiP calls from the NS world
 */
uint64_t sip_smc_handler(uint32_t smc_fid,
			 uint64_t x1,
			 uint64_t x2,
			 uint64_t x3,
			 uint64_t x4,
			 void *cookie,
			 void *handle,
			 uint64_t flags)
{
	uint32_t ns;

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);
	if (!ns)
		SMC_RET1(handle, SMC_UNK);

	switch (smc_fid) {
	case SIP_SVC_CALL_COUNT:
		/* Return the number of SiP Service Calls. */
		SMC_RET1(handle, PLAT_SIP_NUM_CALLS);

	case SIP_SVC_UID:
		/* Return UID to the caller */
		SMC_UUID_RET(handle, sip_svc_uid);
		break;

	case SIP_SVC_VERSION:
		/* Return the version of current implementation */
		SMC_RET2(handle, SIP_SVC_VERSION_MAJOR,
			SIP_SVC_VERSION_MINOR);
		break;

	default:
		return plat_sip_handler(smc_fid, x1, x2, x3, x4,
			cookie, handle, flags);
	}
}

/* Define a runtime service descriptor for fast SMC calls */
DECLARE_RT_SVC(
	sip_svc,
	OEN_SIP_START,
	OEN_SIP_END,
	SMC_TYPE_FAST,
	NULL,
	sip_smc_handler
);
