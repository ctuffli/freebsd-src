/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Raghav Sharma <raghav@freebsd.org>
 * Parts Copyright (c) 2014 Dave Vasilevsky <dave@vasilevsky.ca>
 * Obtained from the squashfuse project
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
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/libkern.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mount.h>
#include <sys/mutex.h>
#include <sys/namei.h>
#include <sys/priv.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/sbuf.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/vnode.h>

#include <fs/squashfs/squashfs.h>
#include <fs/squashfs/squashfs_io.h>
#include <fs/squashfs/squashfs_mount.h>
#include <fs/squashfs/squashfs_inode.h>
#include <fs/squashfs/squashfs_decompressor.h>
#include <fs/squashfs/squashfs_block.h>
#include <fs/squashfs/squashfs_dir.h>

/* Swapendian functions for all types of inodes */
void		swapendian_base_inode(struct sqsh_base_inode *temp);
void		swapendian_reg_inode(struct sqsh_reg_inode *temp);
void		swapendian_lreg_inode(struct sqsh_lreg_inode *temp);
void		swapendian_dir_inode(struct sqsh_dir_inode *temp);
void		swapendian_ldir_inode(struct sqsh_ldir_inode *temp);

/* init functions for all types of inodes */
sqsh_err	sqsh_init_reg_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_lreg_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_dir_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_ldir_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_symlink_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_dev_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_ldev_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_ipc_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);
sqsh_err	sqsh_init_lipc_inode(struct sqsh_mount *ump, struct sqsh_inode *inode);

static	MALLOC_DEFINE(M_SQUASHFSTABLEBLK, "SQUASHFS tab blk", "SQUASHFS table block");

sqsh_err
sqsh_init_table(struct sqsh_table *table, struct sqsh_mount *ump,
	off_t start, size_t each, size_t count)
{
	size_t i;
	size_t nblocks, bread;

	if (count == 0)
		return (SQFS_OK);

	nblocks = sqsh_ceil(each * count, SQUASHFS_METADATA_SIZE);
	bread = nblocks * sizeof(uint64_t);

	table->each = each;
	table->blocks = malloc(bread, M_SQUASHFSTABLEBLK, M_WAITOK);
	if (table->blocks == NULL)
		return (SQFS_ERR);

	if (sqsh_io_read_buf(ump, table->blocks, start, bread) != bread) {
		ERROR("Failed to read data, I/O error");
		sqsh_free_table(table);
		return (SQFS_ERR);
	}

	/* SwapEndian data to host */
	for (i = 0; i < nblocks; ++i)
		table->blocks[i] = le64toh(table->blocks[i]);

	return (SQFS_OK);
}

void
sqsh_free_table(struct sqsh_table *table)
{
	free(table->blocks, M_SQUASHFSTABLEBLK);
	table->blocks = NULL;
}

sqsh_err
sqsh_get_table(struct sqsh_table *table, struct sqsh_mount *ump,
	size_t idx, void *buf)
{
	struct sqsh_block *block;
	size_t pos, bnum, off, data_size;
	off_t bpos;

	pos = idx * table->each;
	bnum = pos / SQUASHFS_METADATA_SIZE;
	off = pos % SQUASHFS_METADATA_SIZE;

	bpos = table->blocks[bnum];
	data_size = 0;
	if (sqsh_metadata_read(ump, bpos, &data_size, &block))
		return (SQFS_ERR);

	memcpy(buf, (char*)(block->data) + off, table->each);
	/* Free block since currently we have no cache */
	sqsh_free_block(block);
	return (SQFS_OK);
}

bool
sqsh_export_ok(struct sqsh_mount *ump)
{
	return (ump->sb.lookup_table_start != SQUASHFS_INVALID_BLK);
}

void
sqsh_metadata_run_inode(struct sqsh_block_run *cur, uint64_t id, off_t base)
{
	cur->block = (id >> 16) + base;
	cur->offset = id & 0xffff;
}

__enum_uint8(vtype)
sqsh_inode_type(int inode_type)
{
	switch (inode_type) {
	case SQUASHFS_DIR_TYPE:
	case SQUASHFS_LDIR_TYPE:
		return (VDIR);
	case SQUASHFS_REG_TYPE:
	case SQUASHFS_LREG_TYPE:
		return (VREG);
	case SQUASHFS_SYMLINK_TYPE:
	case SQUASHFS_LSYMLINK_TYPE:
		return (VLNK);
	case SQUASHFS_BLKDEV_TYPE:
	case SQUASHFS_LBLKDEV_TYPE:
		return (VBLK);
	case SQUASHFS_CHRDEV_TYPE:
	case SQUASHFS_LCHRDEV_TYPE:
		return (VCHR);
	case SQUASHFS_FIFO_TYPE:
	case SQUASHFS_LFIFO_TYPE:
		return (VFIFO);
	case SQUASHFS_SOCKET_TYPE:
	case SQUASHFS_LSOCKET_TYPE:
		return (VSOCK);
	}
	return (VBAD);
}

__enum_uint8(vtype)
sqsh_inode_type_from_id(struct sqsh_mount *ump, uint64_t inode_id)
{
	struct sqsh_block_run cur;
	struct sqsh_base_inode base;
	sqsh_err err;

	sqsh_metadata_run_inode(&cur, inode_id, ump->sb.inode_table_start);

	err = sqsh_metadata_get(ump, &cur, &base, sizeof(base));
	if (err != SQFS_OK)
		return (VBAD);
	swapendian_base_inode(&base);
	return sqsh_inode_type(base.inode_type);
}

sqsh_err
sqsh_verify_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{

	/* check for inode type */
	if (inode->base.inode_type < SQUASHFS_TYPE_MIN_VALID
		|| inode->base.inode_type > SQUASHFS_TYPE_MAX_VALID) {
		TRACE("%s: @%d", __func__, __LINE__);
		return (SQFS_ERR);
	}

	/*
	 * check for inode_number.
	 * The inode numbers are from 1 to the total number of inodes.
	 * Note that 0 is always invalid because we will
	 * always have at least a root inode.
	 */
	if (inode->base.inode_number < SQUASHFS_INODE_MIN_COUNT
		|| inode->base.inode_number > ump->sb.inodes) {
		TRACE("%s: @%d", __func__, __LINE__);
		return (SQFS_ERR);
	}

	/*
	 * If inode type is directory then check for parent inode.
	 * Note that we add +1 because for root inode parent_inode
	 * is total inodes + 1
	 */
	if (inode->base.inode_type == SQUASHFS_DIR_TYPE) {
		if (inode->xtra.dir.parent_inode < SQUASHFS_INODE_MIN_COUNT
			|| inode->xtra.dir.parent_inode > ump->sb.inodes + 1) {
			TRACE("%s: @%d - parent_inode=%u MIN=%u inodes+1=%u", __func__, __LINE__, inode->xtra.dir.parent_inode, SQUASHFS_INODE_MIN_COUNT, ump->sb.inodes + 1);
			return (SQFS_ERR);
		}
	}

	return (SQFS_OK);
}

sqsh_err
sqsh_get_inode_id(struct sqsh_mount *ump, uint16_t idx, uint32_t *id)
{
	uint32_t rid;
	sqsh_err err;

	err = sqsh_get_table(&ump->id_table, ump, idx, &rid);
	if (err != SQFS_OK)
		return (err);
	rid = le32toh(rid);
	*id = rid;
	return (SQFS_OK);
}

sqsh_err
sqsh_export_inode(struct sqsh_mount *ump, uint32_t n, uint64_t *i)
{
	uint64_t r;
	sqsh_err err;

	if (!sqsh_export_ok(ump))
		return (SQFS_ERR);

	err = sqsh_get_table(&ump->export_table, ump, n - 1, &r);
	if (err)
		return (err);
	r = le64toh(r);
	*i = r;
	return (SQFS_OK);
}

uint64_t
sqsh_root_inode(struct sqsh_mount *ump)
{
	return (ump->sb.root_inode);
}

sqsh_err
sqsh_get_inode(struct sqsh_mount *ump, struct sqsh_inode *inode,
	uint64_t id)
{
	struct sqsh_block_run cur;
	sqsh_err err;

	KASSERT(inode != NULL, ("Inode: NULL"));

	memset(inode, 0, sizeof(*inode));
	inode->xattr = SQUASHFS_INVALID_XATTR;

	sqsh_metadata_run_inode(&cur, id, ump->sb.inode_table_start);
	inode->next = cur;

	err = sqsh_metadata_get(ump, &cur, &inode->base, sizeof(inode->base));
	if (err != SQFS_OK) {
		TRACE("%s: @%d", __func__, __LINE__);
		return (err);
	}
	swapendian_base_inode(&inode->base);
	inode->type = sqsh_inode_type(inode->base.inode_type);

	inode->vnode = NULL;
	inode->ump = ump;
	inode->ino_id = id;

	switch (inode->base.inode_type) {
	case SQUASHFS_REG_TYPE: {
		err = sqsh_init_reg_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_LREG_TYPE: {
		err = sqsh_init_lreg_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_DIR_TYPE: {
		err = sqsh_init_dir_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_LDIR_TYPE: {
		err = sqsh_init_ldir_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_SYMLINK_TYPE:
	case SQUASHFS_LSYMLINK_TYPE: {
		err = sqsh_init_symlink_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_BLKDEV_TYPE:
	case SQUASHFS_CHRDEV_TYPE: {
		err = sqsh_init_dev_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_LBLKDEV_TYPE:
	case SQUASHFS_LCHRDEV_TYPE: {
		err = sqsh_init_ldev_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_SOCKET_TYPE:
	case SQUASHFS_FIFO_TYPE: {
		err = sqsh_init_ipc_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	case SQUASHFS_LSOCKET_TYPE:
	case SQUASHFS_LFIFO_TYPE: {
		err = sqsh_init_lipc_inode(ump, inode);
		if (err != SQFS_OK) {
			TRACE("%s: @%d", __func__, __LINE__);
			return (err);
		}
		break;
	}
	default:
		TRACE("%s: @%d", __func__, __LINE__);
		return (SQFS_ERR);
	}

	err = sqsh_verify_inode(ump, inode);

	return (err);
}

sqsh_err
sqsh_init_reg_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_reg_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);
	swapendian_reg_inode(&temp);

	inode->nlink				=	1;
	inode->xtra.reg.start_block	=	temp.start_block;
	inode->size					=	temp.file_size;
	inode->xtra.reg.frag_idx	=	temp.fragment;
	inode->xtra.reg.frag_off	=	temp.offset;

	return (SQFS_OK);
}

sqsh_err
sqsh_init_lreg_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_lreg_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);
	swapendian_lreg_inode(&temp);

	inode->nlink				=	temp.nlink;
	inode->xtra.reg.start_block	=	temp.start_block;
	inode->size					=	temp.file_size;
	inode->xtra.reg.frag_idx	=	temp.fragment;
	inode->xtra.reg.frag_off	=	temp.offset;
	inode->xattr				=	temp.xattr;

	return (SQFS_OK);
}

sqsh_err
sqsh_init_dir_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_dir_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);
	swapendian_dir_inode(&temp);

	inode->nlink					=	temp.nlink;
	inode->xtra.dir.start_block 	=	temp.start_block;
	inode->xtra.dir.offset			=	temp.offset;
	inode->size						=	temp.file_size;
	inode->xtra.dir.idx_count		=	0;
	inode->xtra.dir.parent_inode	=	temp.parent_inode;

	sqsh_dir_init(ump, inode, &inode->xtra.dir.d);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_ldir_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_ldir_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);
	swapendian_ldir_inode(&temp);

	inode->nlink					=	temp.nlink;
	inode->xtra.dir.start_block 	=	temp.start_block;
	inode->xtra.dir.offset			=	temp.offset;
	inode->size						=	temp.file_size;
	inode->xtra.dir.idx_count		=	temp.i_count;
	inode->xtra.dir.parent_inode	=	temp.parent_inode;
	inode->xattr					=	temp.xattr;

	sqsh_dir_init(ump, inode, &inode->xtra.dir.d);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_symlink_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_symlink_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);

	inode->nlink				=	le32toh(temp.nlink);
	inode->size					=	le32toh(temp.symlink_size);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_dev_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_dev_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);

	inode->size				=	0;
	inode->nlink			=	le32toh(temp.nlink);
	temp.rdev				=	le32toh(temp.rdev);
	inode->xtra.dev.major	=	(temp.rdev >> 8) & 0xfff;
	inode->xtra.dev.minor	=	(temp.rdev & 0xff) | ((temp.rdev >> 12) & 0xfff00);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_ldev_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_ldev_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);

	inode->size				=	0;
	inode->nlink			=	le32toh(temp.nlink);
	inode->xattr			=	le32toh(temp.xattr);
	temp.rdev				=	le32toh(temp.rdev);
	inode->xtra.dev.major	=	(temp.rdev >> 8) & 0xfff;
	inode->xtra.dev.minor	=	(temp.rdev & 0xff) | ((temp.rdev >> 12) & 0xfff00);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_ipc_inode(struct sqsh_mount *ump, struct sqsh_inode *inode) {
	struct sqsh_ipc_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);

	inode->size				=	0;
	inode->nlink			=	le32toh(temp.nlink);

	return (SQFS_OK);
}

sqsh_err
sqsh_init_lipc_inode(struct sqsh_mount *ump, struct sqsh_inode *inode)
{
	struct sqsh_lipc_inode temp;
	sqsh_err err;

	err = sqsh_metadata_get(ump, &inode->next, &temp, sizeof(temp));
	if (err != SQFS_OK)
		return (err);

	inode->size				=	0;
	inode->nlink			=	le32toh(temp.nlink);
	inode->xattr			=	le32toh(temp.xattr);

	return (SQFS_OK);
}

void
swapendian_base_inode(struct sqsh_base_inode *temp)
{

	temp->inode_type	=	le16toh(temp->inode_type);
	temp->mode			=	le16toh(temp->mode);
	temp->uid			=	le16toh(temp->uid);
	temp->guid			=	le16toh(temp->guid);
	temp->mtime			=	le32toh(temp->mtime);
	temp->inode_number	=	le32toh(temp->inode_number);
}

void
swapendian_reg_inode(struct sqsh_reg_inode *temp)
{

	temp->inode_type	=	le16toh(temp->inode_type);
	temp->mode			=	le16toh(temp->mode);
	temp->uid			=	le16toh(temp->uid);
	temp->guid			=	le16toh(temp->guid);
	temp->mtime			=	le32toh(temp->mtime);
	temp->inode_number	=	le32toh(temp->inode_number);
	temp->start_block	=	le32toh(temp->start_block);
	temp->fragment		=	le32toh(temp->fragment);
	temp->offset		=	le32toh(temp->offset);
	temp->file_size		=	le32toh(temp->file_size);
}

void
swapendian_lreg_inode(struct sqsh_lreg_inode *temp)
{

	temp->inode_type	=	le16toh(temp->inode_type);
	temp->mode			=	le16toh(temp->mode);
	temp->uid			=	le16toh(temp->uid);
	temp->guid			=	le16toh(temp->guid);
	temp->mtime			=	le32toh(temp->mtime);
	temp->inode_number	=	le32toh(temp->inode_number);
	temp->start_block	=	le64toh(temp->start_block);
	temp->file_size		=	le64toh(temp->file_size);
	temp->sparse		=	le64toh(temp->sparse);
	temp->nlink			=	le32toh(temp->nlink);
	temp->fragment		=	le32toh(temp->fragment);
	temp->offset		=	le32toh(temp->offset);
	temp->xattr			=	le32toh(temp->xattr);
}

void
swapendian_dir_inode(struct sqsh_dir_inode *temp)
{

	temp->inode_type	=	le16toh(temp->inode_type);
	temp->mode			=	le16toh(temp->mode);
	temp->uid			=	le16toh(temp->uid);
	temp->guid			=	le16toh(temp->guid);
	temp->mtime			=	le32toh(temp->mtime);
	temp->inode_number	=	le32toh(temp->inode_number);
	temp->start_block	=	le32toh(temp->start_block);
	temp->nlink			=	le32toh(temp->nlink);
	temp->file_size		=	le16toh(temp->file_size);
	temp->offset		=	le16toh(temp->offset);
	temp->parent_inode	=	le32toh(temp->parent_inode);
}

void
swapendian_ldir_inode(struct sqsh_ldir_inode *temp)
{

	temp->inode_type	=	le16toh(temp->inode_type);
	temp->mode			=	le16toh(temp->mode);
	temp->uid			=	le16toh(temp->uid);
	temp->guid			=	le16toh(temp->guid);
	temp->mtime			=	le32toh(temp->mtime);
	temp->inode_number	=	le32toh(temp->inode_number);
	temp->nlink			=	le32toh(temp->nlink);
	temp->file_size		=	le32toh(temp->file_size);
	temp->start_block	=	le32toh(temp->start_block);
	temp->parent_inode	=	le32toh(temp->parent_inode);
	temp->i_count		=	le16toh(temp->i_count);
	temp->offset		=	le16toh(temp->offset);
	temp->xattr			=	le32toh(temp->xattr);
}
