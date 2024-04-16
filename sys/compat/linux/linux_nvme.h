/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2024 Chuck Tuffli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _LINUX_NVME_H_
#define _LINUX_NVME_H_

#include <dev/nvme/nvme.h>
#include <dev/nvme/nvme_private.h>

struct linnvme_ns_dev {
	struct cdev	*cdev;
	STAILQ_ENTRY(linnvme_ns_dev)	link;
};

/*
 * Linux NVMe IOCTL structures and definitions
 */
struct nvme_user_io {
	uint8_t		opcode;
	uint8_t		flags;
	uint16_t	control;
	uint16_t	nblocks;
	uint16_t	rsvd;
	uint64_t	metadata;
	uint64_t	addr;
	uint64_t	slba;
	uint32_t	dsmgmt;
	uint32_t	reftag;
	uint16_t	apptag;
	uint16_t	appmask;
};

struct nvme_passthru_cmd {
	uint8_t		opcode;
	uint8_t		flags;
	uint16_t	rsvd;
	uint32_t	nsid;
	uint32_t	cdw2;
	uint32_t	cdw3;
	uint64_t	metadata;
	uint64_t	addr;
	uint32_t	metadata_len;
	uint32_t	data_len;
	uint32_t	cdw10;
	uint32_t	cdw11;
	uint32_t	cdw12;
	uint32_t	cdw13;
	uint32_t	cdw14;
	uint32_t	cdw15;
	uint32_t	timeout_ms;
	uint32_t	result;
};

int linux_nvme_do_cmd(struct file *, struct nvme_passthru_cmd *, u_long);

#endif /* !_LINUX_NVME_H_ */
