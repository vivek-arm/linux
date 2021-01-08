// SPDX-License-Identifier: GPL-2.0
/*
 * PASID table management for the IOMMU
 *
 * Copyright (C) 2021 Arm Ltd.
 */
#ifndef __IOMMU_PASID_TABLE_H
#define __IOMMU_PASID_TABLE_H

#include <linux/io-pgtable.h>

#include "arm/arm-smmu-v3/arm-smmu-v3.h"

enum pasid_table_fmt {
	PASID_TABLE_ARM_SMMU_V3,
	PASID_TABLE_NUM_FMTS,
};

/**
 * struct arm_smmu_cfg_info - arm-smmu-v3 specific configuration data
 *
 * @s1_cfg: arm-smmu-v3 stage1 config data
 * @feat_flag: features supported by arm-smmu-v3 implementation
 */
struct arm_smmu_cfg_info {
	struct arm_smmu_s1_cfg	*s1_cfg;
	u32			feat_flag;
};

/**
 * struct iommu_vendor_psdtable_cfg - Configuration data for PASID tables
 *
 * @iommu_dev: device performing the DMA table walks
 * @fmt: The PASID table format
 * @base: DMA address of the allocated table, set by the vendor driver
 * @cfg: arm-smmu-v3 specific config data
 */
struct iommu_vendor_psdtable_cfg {
	struct device		*iommu_dev;
	enum pasid_table_fmt	fmt;
	dma_addr_t		base;
	union {
		struct arm_smmu_cfg_info	cfg;
	} vendor;
};

struct iommu_vendor_psdtable_ops;

/**
 * struct iommu_pasid_table - describes a set of PASID tables
 *
 * @cookie: An opaque token provided by the IOMMU driver and passed back to any
 * callback routine.
 * @cfg: A copy of the PASID table configuration
 * @ops: The PASID table operations in use for this set of page tables
 */
struct iommu_pasid_table {
	void					*cookie;
	struct iommu_vendor_psdtable_cfg	cfg;
	struct iommu_vendor_psdtable_ops	*ops;
};

#define pasid_table_cfg_to_table(pst_cfg) \
	container_of((pst_cfg), struct iommu_pasid_table, cfg)

struct iommu_vendor_psdtable_ops {
	int (*alloc)(struct iommu_vendor_psdtable_cfg *cfg);
	void (*free)(struct iommu_vendor_psdtable_cfg *cfg);
	void (*prepare)(struct iommu_vendor_psdtable_cfg *cfg,
			struct io_pgtable_cfg *pgtbl_cfg, u32 asid);
	int (*write)(struct iommu_vendor_psdtable_cfg *cfg, int ssid,
		     void *cookie);
	void (*sync)(void *cookie, int ssid, bool leaf);
};

static inline int iommu_psdtable_alloc(struct iommu_pasid_table *tbl,
				       struct iommu_vendor_psdtable_cfg *cfg)
{
	if (!tbl->ops->alloc)
		return -ENOSYS;

	return tbl->ops->alloc(cfg);
}

static inline void iommu_psdtable_free(struct iommu_pasid_table *tbl,
				       struct iommu_vendor_psdtable_cfg *cfg)
{
	if (!tbl->ops->free)
		return;

	tbl->ops->free(cfg);
}

static inline int iommu_psdtable_prepare(struct iommu_pasid_table *tbl,
					 struct iommu_vendor_psdtable_cfg *cfg,
					 struct io_pgtable_cfg *pgtbl_cfg,
					 u32 asid)
{
	if (!tbl->ops->prepare)
		return -ENOSYS;

	tbl->ops->prepare(cfg, pgtbl_cfg, asid);
	return 0;
}

static inline int iommu_psdtable_write(struct iommu_pasid_table *tbl,
				       struct iommu_vendor_psdtable_cfg *cfg,
				       int ssid, void *cookie)
{
	if (!tbl->ops->write)
		return -ENOSYS;

	return tbl->ops->write(cfg, ssid, cookie);
}

static inline int iommu_psdtable_sync(struct iommu_pasid_table *tbl,
				      void *cookie, int ssid, bool leaf)
{
	if (!tbl->ops->sync)
		return -ENOSYS;

	tbl->ops->sync(cookie, ssid, leaf);
	return 0;
}

/* A placeholder to register vendor specific pasid layer */
static inline struct iommu_pasid_table *
iommu_register_pasid_table(enum pasid_table_fmt fmt,
			   struct device *dev, void *cookie)
{
	return NULL;
}

#endif /* __IOMMU_PASID_TABLE_H */
