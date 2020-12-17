// SPDX-License-Identifier: GPL-2.0
/*
 * arm-smmu-v3 context descriptor handling library driver
 *
 * Copyright (C) 2021 Arm Ltd.
 */

#include <linux/dma-iommu.h>

#include "arm-smmu-v3.h"
#include "../../iommu-pasid-table.h"

static int arm_smmu_alloc_cd_leaf_table(struct device *dev,
					struct arm_smmu_l1_ctx_desc *l1_desc)
{
	size_t size = CTXDESC_L2_ENTRIES * (CTXDESC_CD_DWORDS << 3);

	l1_desc->l2ptr = dmam_alloc_coherent(dev, size,
					     &l1_desc->l2ptr_dma, GFP_KERNEL);
	if (!l1_desc->l2ptr) {
		dev_warn(dev, "failed to allocate context descriptor table\n");
		return -ENOMEM;
	}
	return 0;
}

static void arm_smmu_write_cd_l1_desc(__le64 *dst,
				      struct arm_smmu_l1_ctx_desc *l1_desc)
{
	u64 val = (l1_desc->l2ptr_dma & CTXDESC_L1_DESC_L2PTR_MASK) |
		  CTXDESC_L1_DESC_V;

	/* See comment in arm_smmu_write_ctx_desc() */
	WRITE_ONCE(*dst, cpu_to_le64(val));
}

static __le64 *arm_smmu_get_cd_ptr(struct iommu_vendor_psdtable_cfg *pst_cfg,
				   u32 ssid)
{
	__le64 *l1ptr;
	unsigned int idx;
	struct device *dev = pst_cfg->iommu_dev;
	struct arm_smmu_cfg_info *cfgi = &pst_cfg->vendor.cfg;
	struct arm_smmu_s1_cfg *s1cfg = cfgi->s1_cfg;
	struct arm_smmu_ctx_desc_cfg *cdcfg = &s1cfg->cdcfg;
	struct arm_smmu_l1_ctx_desc *l1_desc;
	struct iommu_pasid_table *tbl = pasid_table_cfg_to_table(pst_cfg);

	if (s1cfg->s1fmt == STRTAB_STE_0_S1FMT_LINEAR)
		return cdcfg->cdtab + ssid * CTXDESC_CD_DWORDS;

	idx = ssid >> CTXDESC_SPLIT;
	l1_desc = &cdcfg->l1_desc[idx];
	if (!l1_desc->l2ptr) {
		if (arm_smmu_alloc_cd_leaf_table(dev, l1_desc))
			return NULL;

		l1ptr = cdcfg->cdtab + idx * CTXDESC_L1_DESC_DWORDS;
		arm_smmu_write_cd_l1_desc(l1ptr, l1_desc);
		/* An invalid L1CD can be cached */
		if (iommu_psdtable_sync(tbl, tbl->cookie, ssid, false))
			return NULL;
	}
	idx = ssid & (CTXDESC_L2_ENTRIES - 1);
	return l1_desc->l2ptr + idx * CTXDESC_CD_DWORDS;
}

static int arm_smmu_write_ctx_desc(struct iommu_vendor_psdtable_cfg *pst_cfg,
				   int ssid, void *cookie)
{
	/*
	 * This function handles the following cases:
	 *
	 * (1) Install primary CD, for normal DMA traffic (SSID = 0).
	 * (2) Install a secondary CD, for SID+SSID traffic.
	 * (3) Update ASID of a CD. Atomically write the first 64 bits of the
	 *     CD, then invalidate the old entry and mappings.
	 * (4) Quiesce the context without clearing the valid bit. Disable
	 *     translation, and ignore any translation fault.
	 * (5) Remove a secondary CD.
	 */
	u64 val;
	bool cd_live;
	__le64 *cdptr;
	struct arm_smmu_cfg_info *cfgi = &pst_cfg->vendor.cfg;
	struct arm_smmu_s1_cfg *s1cfg = cfgi->s1_cfg;
	struct iommu_pasid_table *tbl = pasid_table_cfg_to_table(pst_cfg);
	struct arm_smmu_ctx_desc *cd = cookie;

	if (WARN_ON(ssid >= (1 << s1cfg->s1cdmax)))
		return -E2BIG;

	cdptr = arm_smmu_get_cd_ptr(pst_cfg, ssid);
	if (!cdptr)
		return -ENOMEM;

	val = le64_to_cpu(cdptr[0]);
	cd_live = !!(val & CTXDESC_CD_0_V);

	if (!cd) { /* (5) */
		val = 0;
	} else if (cd == &quiet_cd) { /* (4) */
		val |= CTXDESC_CD_0_TCR_EPD0;
	} else if (cd_live) { /* (3) */
		val &= ~CTXDESC_CD_0_ASID;
		val |= FIELD_PREP(CTXDESC_CD_0_ASID, cd->asid);
		/*
		 * Until CD+TLB invalidation, both ASIDs may be used for tagging
		 * this substream's traffic
		 */
	} else { /* (1) and (2) */
		cdptr[1] = cpu_to_le64(cd->ttbr & CTXDESC_CD_1_TTB0_MASK);
		cdptr[2] = 0;
		cdptr[3] = cpu_to_le64(cd->mair);

		/*
		 * STE is live, and the SMMU might read dwords of this CD in any
		 * order. Ensure that it observes valid values before reading
		 * V=1.
		 */
		if (iommu_psdtable_sync(tbl, tbl->cookie, ssid, true))
			return -ENOSYS;

		val = cd->tcr |
#ifdef __BIG_ENDIAN
			CTXDESC_CD_0_ENDI |
#endif
			CTXDESC_CD_0_R | CTXDESC_CD_0_A |
			(cd->mm ? 0 : CTXDESC_CD_0_ASET) |
			CTXDESC_CD_0_AA64 |
			FIELD_PREP(CTXDESC_CD_0_ASID, cd->asid) |
			CTXDESC_CD_0_V;

		/* STALL_MODEL==0b10 && CD.S==0 is ILLEGAL */
		if (cfgi->feat_flag & ARM_SMMU_FEAT_STALL_FORCE)
			val |= CTXDESC_CD_0_S;
	}

	/*
	 * The SMMU accesses 64-bit values atomically. See IHI0070Ca 3.21.3
	 * "Configuration structures and configuration invalidation completion"
	 *
	 *   The size of single-copy atomic reads made by the SMMU is
	 *   IMPLEMENTATION DEFINED but must be at least 64 bits. Any single
	 *   field within an aligned 64-bit span of a structure can be altered
	 *   without first making the structure invalid.
	 */
	WRITE_ONCE(cdptr[0], cpu_to_le64(val));
	if (iommu_psdtable_sync(tbl, tbl->cookie, ssid, true))
		return -ENOSYS;

	return 0;
}

static void arm_smmu_prepare_cd(struct iommu_vendor_psdtable_cfg *pst_cfg,
				struct io_pgtable_cfg *pgtbl_cfg, u32 asid)
{
	struct arm_smmu_cfg_info *cfgi = &pst_cfg->vendor.cfg;
	struct arm_smmu_s1_cfg *s1cfg = cfgi->s1_cfg;
	typeof(&pgtbl_cfg->arm_lpae_s1_cfg.tcr) tcr = &pgtbl_cfg->arm_lpae_s1_cfg.tcr;

	s1cfg->cd.asid	= (u16)asid;
	s1cfg->cd.ttbr	= pgtbl_cfg->arm_lpae_s1_cfg.ttbr;
	s1cfg->cd.tcr	= FIELD_PREP(CTXDESC_CD_0_TCR_T0SZ, tcr->tsz) |
			  FIELD_PREP(CTXDESC_CD_0_TCR_TG0, tcr->tg) |
			  FIELD_PREP(CTXDESC_CD_0_TCR_IRGN0, tcr->irgn) |
			  FIELD_PREP(CTXDESC_CD_0_TCR_ORGN0, tcr->orgn) |
			  FIELD_PREP(CTXDESC_CD_0_TCR_SH0, tcr->sh) |
			  FIELD_PREP(CTXDESC_CD_0_TCR_IPS, tcr->ips) |
			  CTXDESC_CD_0_TCR_EPD1 | CTXDESC_CD_0_AA64;
	s1cfg->cd.mair	= pgtbl_cfg->arm_lpae_s1_cfg.mair;
}

static int arm_smmu_alloc_cd_tables(struct iommu_vendor_psdtable_cfg *pst_cfg)
{
	int ret;
	size_t l1size;
	size_t max_contexts;
	struct device *dev = pst_cfg->iommu_dev;
	struct arm_smmu_cfg_info *cfgi = &pst_cfg->vendor.cfg;
	struct arm_smmu_s1_cfg *s1cfg = cfgi->s1_cfg;
	struct arm_smmu_ctx_desc_cfg *cdcfg = &s1cfg->cdcfg;

	max_contexts = 1 << s1cfg->s1cdmax;

	if (!(cfgi->feat_flag & ARM_SMMU_FEAT_2_LVL_CDTAB) ||
	    max_contexts <= CTXDESC_L2_ENTRIES) {
		s1cfg->s1fmt = STRTAB_STE_0_S1FMT_LINEAR;
		cdcfg->num_l1_ents = max_contexts;

		l1size = max_contexts * (CTXDESC_CD_DWORDS << 3);
	} else {
		s1cfg->s1fmt = STRTAB_STE_0_S1FMT_64K_L2;
		cdcfg->num_l1_ents = DIV_ROUND_UP(max_contexts,
						  CTXDESC_L2_ENTRIES);

		cdcfg->l1_desc = devm_kcalloc(dev, cdcfg->num_l1_ents,
					      sizeof(*cdcfg->l1_desc),
					      GFP_KERNEL);
		if (!cdcfg->l1_desc)
			return -ENOMEM;

		l1size = cdcfg->num_l1_ents * (CTXDESC_L1_DESC_DWORDS << 3);
	}

	cdcfg->cdtab = dmam_alloc_coherent(dev, l1size, &cdcfg->cdtab_dma,
					   GFP_KERNEL);
	if (!cdcfg->cdtab) {
		dev_warn(dev, "failed to allocate context descriptor\n");
		ret = -ENOMEM;
		goto err_free_l1;
	}

	return 0;

err_free_l1:
	if (cdcfg->l1_desc) {
		devm_kfree(dev, cdcfg->l1_desc);
		cdcfg->l1_desc = NULL;
	}
	return ret;
}

static void arm_smmu_free_cd_tables(struct iommu_vendor_psdtable_cfg *pst_cfg)
{
	int i;
	size_t size, l1size;
	struct device *dev = pst_cfg->iommu_dev;
	struct arm_smmu_cfg_info *cfgi = &pst_cfg->vendor.cfg;
	struct arm_smmu_s1_cfg *s1cfg = cfgi->s1_cfg;
	struct arm_smmu_ctx_desc_cfg *cdcfg = &s1cfg->cdcfg;

	if (cdcfg->l1_desc) {
		size = CTXDESC_L2_ENTRIES * (CTXDESC_CD_DWORDS << 3);

		for (i = 0; i < cdcfg->num_l1_ents; i++) {
			if (!cdcfg->l1_desc[i].l2ptr)
				continue;

			dmam_free_coherent(dev, size,
					   cdcfg->l1_desc[i].l2ptr,
					   cdcfg->l1_desc[i].l2ptr_dma);
		}
		devm_kfree(dev, cdcfg->l1_desc);
		cdcfg->l1_desc = NULL;

		l1size = cdcfg->num_l1_ents * (CTXDESC_L1_DESC_DWORDS << 3);
	} else {
		l1size = cdcfg->num_l1_ents * (CTXDESC_CD_DWORDS << 3);
	}

	dmam_free_coherent(dev, l1size, cdcfg->cdtab, cdcfg->cdtab_dma);
	cdcfg->cdtab_dma = 0;
	cdcfg->cdtab = NULL;
}

struct iommu_vendor_psdtable_ops arm_cd_table_ops = {
	.alloc	 = arm_smmu_alloc_cd_tables,
	.free	 = arm_smmu_free_cd_tables,
	.prepare = arm_smmu_prepare_cd,
	.write	 = arm_smmu_write_ctx_desc,
	.sync	 = arm_smmu_sync_cd,
};

struct iommu_pasid_table *arm_smmu_register_cd_table(struct device *dev,
						     void *cookie)
{
	struct iommu_pasid_table *tbl;

	tbl = devm_kzalloc(dev, sizeof(tbl), GFP_KERNEL);
	if (!tbl)
		return NULL;

	tbl->cookie = cookie;
	tbl->ops = &arm_cd_table_ops;

	return tbl;
}
