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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/capsicum.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/stat.h>
#include <sys/vnode.h>

#ifdef COMPAT_LINUX32
#include <machine/../linux32/linux.h>
#include <machine/../linux32/linux32_proto.h>
#else
#include <machine/../linux/linux.h>
#include <machine/../linux/linux_proto.h>
#endif

#include <compat/linux/linux_ioctl.h>
#include <compat/linux/linux_nvme.h>

#include <dev/nvme/nvme.h>
#include <dev/nvme/nvme_private.h>

static STAILQ_HEAD(, linnvme_ns_dev) ns_list = STAILQ_HEAD_INITIALIZER(ns_list);
static struct mtx linnvme_mtx;
MTX_SYSINIT(linnvme_mtx_init, &linnvme_mtx, "Linux NVMe lock", MTX_DEF);

MALLOC_DEFINE(M_LINUX_NVME, "linux_nvme", "Memory used for Linux NVMe device");

struct linux_nvme_pt_command {
	struct nvme_pt_command pt;
	int ioctl_cmd;
};

/*
 * Convert from a Linux passthru command to the native format
 */
static int
linux_nvme_cmd_convert(struct nvme_pt_command *npt, struct nvme_passthru_cmd *lpt)
{

	if (!npt || !lpt)
		return (EINVAL);

	npt->cmd.opc = lpt->opcode;
	npt->cmd.nsid = htole32(lpt->nsid);

	npt->cmd.cdw10 = htole32(lpt->cdw10);
	npt->cmd.cdw11 = htole32(lpt->cdw11);
	npt->cmd.cdw12 = htole32(lpt->cdw12);
	npt->cmd.cdw13 = htole32(lpt->cdw13);
	npt->cmd.cdw14 = htole32(lpt->cdw14);
	npt->cmd.cdw15 = htole32(lpt->cdw15);

	npt->buf = (void *)lpt->addr;
	npt->len = lpt->data_len;

	npt->is_read = (lpt->opcode & 0x2) != 0;

	return (0);
}

static int
linux_nvme_cpl_convert(struct nvme_pt_command *npt, struct nvme_passthru_cmd *lpt)
{
	lpt->result = le32toh(npt->cpl.cdw0);
	return (le16toh(npt->cpl.status));
}

int
linux_nvme_do_cmd(struct file *fp, struct nvme_passthru_cmd *lpt, u_long cmd)
{
	struct linux_nvme_pt_command compat = { { { 0 }, }, };
	int err;

	compat.ioctl_cmd = cmd;
	err = linux_nvme_cmd_convert(&compat.pt, lpt);
	if (err == 0) {
		struct cdev *cdev;
		struct nvme_controller *ctrlr;
		uint32_t nsid = le32toh(compat.pt.cmd.nsid);
		uint32_t did, nid;	/* device node ID, namespace ID */
		bool is_admin_cmd = cmd == LINUX_NVME_IOCTL_ADMIN_CMD;

		if ((fp == NULL) || (fp->f_vnode == NULL) ||
		    (fp->f_vnode->v_rdev == NULL))
			return (-1);
		cdev = fp->f_vnode->v_rdev;
		if (cdev == NULL)
			return -1;
		/*
		 * Linux clients will issue Admin commands to namespace devices
		 */
		err = sscanf(cdev->si_name, "nvme%dns%d", &did, &nid);
		switch (err) {
		case 1:
			/* nvmeX */
			ctrlr = cdev->si_drv1;
			break;
		case 2:
			/* nvmeXnsY */
			ctrlr = ((struct nvme_namespace *)(cdev->si_drv1))->ctrlr;
			break;
		default:
			return (-1);
		}
		err = nvme_ctrlr_passthrough_cmd(ctrlr,
		    &compat.pt, nsid,
		    1 /* is_user_buffer */,
		    is_admin_cmd);
		if (err == 0)
			err = linux_nvme_cpl_convert(&compat.pt, lpt);
	}
	return (err);
}

/*
 * Linux NVMe namespace device nodes use a different naming scheme than FreeBSD:
 *     FreeBSD : /dev/nvmeXnsY
 *     Linux   : /dev/nvmeXnY
 *
 * Create alias from the FreeBSD format to the Linux one.
 */
void
linux_dev_nvme_create(void)
{
	devclass_t dc;
	device_t *devlist;
	int dcount = 0;

	dc = devclass_find("nvme");
	if (dc == NULL) {
		return;
	}

	if (devclass_get_devices(dc, &devlist, &dcount)) {
		return;
	}

	for (int i = 0; i < dcount; i++) {
		struct nvme_controller *ctrlr;
		struct linnvme_ns_dev *ns_dev;
		const char *nameunit;
		int nsid;

		ctrlr = device_get_softc(devlist[i]);
		if (ctrlr == NULL)
			continue;

		nameunit = device_get_nameunit(devlist[i]);
		for (nsid = 0; nsid < NVME_MAX_NAMESPACES; nsid++) {
			if (ctrlr->ns[nsid].cdev == NULL)
				continue;

			ns_dev = malloc(sizeof(*ns_dev), M_LINUX_NVME, M_ZERO | M_WAITOK);
			if (ns_dev == NULL)
				continue;

			make_dev_alias_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
			    &ns_dev->cdev, ctrlr->ns[nsid].cdev, "%sn%d", nameunit,
			    ctrlr->ns[nsid].id);

			mtx_lock(&linnvme_mtx);
			STAILQ_INSERT_TAIL(&ns_list, ns_dev, link);
			mtx_unlock(&linnvme_mtx);
		}
	}

	free(devlist, M_TEMP);
}

void
linux_dev_nvme_destroy(void)
{
	struct linnvme_ns_dev *ns_dev;

	while (!STAILQ_EMPTY(&ns_list)) {
		mtx_lock(&linnvme_mtx);
		ns_dev = STAILQ_FIRST(&ns_list);
		STAILQ_REMOVE_HEAD(&ns_list, link);
		mtx_unlock(&linnvme_mtx);

		destroy_dev(ns_dev->cdev);
		free(ns_dev, M_LINUX_NVME);
	}
}

