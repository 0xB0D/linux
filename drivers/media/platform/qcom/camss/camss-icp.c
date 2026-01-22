// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm CAMSS ICP (Image Control Processor) Driver
 *
 * Standalone platform driver that boots ICP and exports APIs for BPS/IPE.
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "camss-icp.h"

/* CSR Register Offsets (from CSR base 0xac01000) */
#define ICP_CSR_HW_VERSION		0x00
#define ICP_CSR_TCM_SIZE		0x04
#define ICP_CSR_GP_REG_BASE		0x20
#define ICP_CSR_DBG_STATUS		0x80
#define ICP_CSR_DBG_CTRL		0x84

/* GP register offsets from GP_REG_BASE */
#define GP_REG(n)			((n) * 4)
#define ICP_NUM_GP_REGS			24

/*
 * HFI interface registers - offsets from CSR base
 * CAMX's iface_addr returns csr_base + GP_REG_BASE (0x20)
 * Then uses GEN_PURPOSE_REG(n) = n * 4 as offset from there
 * So actual offset from csr_base = GP_REG_BASE + GP_REG(n)
 * 
 * Evidence: writes to 0x10-0x1f don't stick, but 0x20+ work
 */
#define HFI_REG_FW_VERSION		(ICP_CSR_GP_REG_BASE + GP_REG(1))   /* 0x24 */
#define HFI_REG_HOST_ICP_INIT_REQUEST	(ICP_CSR_GP_REG_BASE + GP_REG(2))   /* 0x28 */
#define HFI_REG_ICP_HOST_INIT_RESPONSE	(ICP_CSR_GP_REG_BASE + GP_REG(3))   /* 0x2C */
#define HFI_REG_SHARED_MEM_PTR		(ICP_CSR_GP_REG_BASE + GP_REG(4))   /* 0x30 */
#define HFI_REG_SHARED_MEM_SIZE		(ICP_CSR_GP_REG_BASE + GP_REG(5))   /* 0x34 */
#define HFI_REG_QTBL_PTR		(ICP_CSR_GP_REG_BASE + GP_REG(6))   /* 0x38 */
#define HFI_REG_SECONDARY_HEAP_PTR	(ICP_CSR_GP_REG_BASE + GP_REG(7))   /* 0x3C */
#define HFI_REG_SECONDARY_HEAP_SIZE	(ICP_CSR_GP_REG_BASE + GP_REG(8))   /* 0x40 */
#define HFI_REG_SFR_PTR			(ICP_CSR_GP_REG_BASE + GP_REG(10))  /* 0x48 - GP9 skipped */
#define HFI_REG_QDSS_IOVA		(ICP_CSR_GP_REG_BASE + GP_REG(11))  /* 0x4C */
#define HFI_REG_QDSS_IOVA_SIZE		(ICP_CSR_GP_REG_BASE + GP_REG(12))  /* 0x50 */
#define HFI_REG_IO_REGION_IOVA		(ICP_CSR_GP_REG_BASE + GP_REG(13))  /* 0x54 */
#define HFI_REG_IO_REGION_SIZE		(ICP_CSR_GP_REG_BASE + GP_REG(14))  /* 0x58 */
#define HFI_REG_IO2_REGION_IOVA		(ICP_CSR_GP_REG_BASE + GP_REG(15))  /* 0x5C */
#define HFI_REG_IO2_REGION_SIZE		(ICP_CSR_GP_REG_BASE + GP_REG(16))  /* 0x60 */
#define HFI_REG_FWUNCACHED_REGION_IOVA	(ICP_CSR_GP_REG_BASE + GP_REG(17))  /* 0x64 */
#define HFI_REG_FWUNCACHED_REGION_SIZE	(ICP_CSR_GP_REG_BASE + GP_REG(18))  /* 0x68 */

/* CIRQ Register Offsets (from CIRQ base 0xac01800) */
#define CIRQ_OB_MASK			0x00
#define CIRQ_OB_CLEAR			0x04
#define CIRQ_OB_STATUS			0x0C
#define CIRQ_HOST2ICPINT		0x124

/* CIRQ status/mask bits */
#define CIRQ_WDT_BITE_WS0		BIT(4)
#define CIRQ_ICP2HOSTINT		BIT(2)

/* Constants */
#define ICP_INIT_REQUEST_SET		1
#define ICP_INIT_RESP_SUCCESS		1
#define ICP_PAS_ID			33
#define HFI_POLL_DELAY_US		100
#define HFI_POLL_TIMEOUT_US		2000000
#define ICP_BOOT_TIMEOUT_MS		2000

/* Fixed IOVA addresses - must match CAMX downstream */
#define ICP_IOVA_SHARED			0x00800000
#define ICP_IOVA_SECHEAP		0x10400000
#define ICP_IOVA_QTBL			0x10500000
#define ICP_IOVA_CMD_Q			0x10600000
#define ICP_IOVA_MSG_Q			0x10700000
#define ICP_IOVA_DBG_Q			0x10800000
#define ICP_IOVA_SFR			0x10900000
#define ICP_IOVA_QDSS			0x10b00000
#define ICP_IOVA_IO1			0x10c00000
#define ICP_IOVA_IO2			0xe0800000

/* Region sizes - must match CAMX downstream */
#define HFI_QTBL_SIZE			SZ_1M
#define HFI_Q_SIZE			SZ_1M
#define HFI_SFR_SIZE			SZ_8K
#define HFI_SHMEM_SIZE			SZ_1M	/* Actual allocation - small control buffer */
#define HFI_SHMEM_REGION_SIZE		(252 * SZ_1M)  /* Reported to FW - full IOVA range */
#define HFI_QDSS_SIZE			SZ_1M
#define HFI_SECHEAP_SIZE		SZ_1M
#define HFI_IO1_SIZE			0xcf400000
#define HFI_IO2_SIZE			0x1e700000
#define HFI_FWUNCACHED_SIZE		(7 * SZ_1M)

/* Queue data sizes */
#define HFI_CMD_Q_DATA_SIZE		4096
#define HFI_MSG_Q_DATA_SIZE		4096
#define HFI_DBG_Q_DATA_SIZE		102400

/* HFI queue types - must match CAMX: CMD=0, MSG=1, DBG=2 */
#define HFI_QUEUE_CMD_TYPE		0
#define HFI_QUEUE_MSG_TYPE		1
#define HFI_QUEUE_DBG_TYPE		2
#define HFI_QUEUE_TABLE_VERSION		0xFFFFFFFF

/* HFI command types */
#define HFI_CMD_SYS_INIT		0x10000001
#define HFI_CMD_SYS_PC_PREP		0x10000002
#define HFI_CMD_SYS_PING		0x10000003

/* HFI message types */
#define HFI_MSG_SYS_INIT_DONE		0x20000001
#define HFI_MSG_SYS_PC_PREP_DONE	0x20000002
#define HFI_MSG_SYS_PING_ACK		0x20000003

/* HFI packet header */
struct hfi_pkt_hdr {
	u32 size;
	u32 pkt_type;
} __packed;

/* HFI SYS_INIT command packet */
struct hfi_cmd_sys_init {
	u32 size;
	u32 pkt_type;
} __packed;

/*
 * HFI Queue Header - CAMX compatible
 * Each field is padded to 64 bytes (16 u32s) for cache line alignment.
 * The firmware expects this exact layout.
 * Total size MUST be 956 bytes (0x3BC) to match CAMX.
 */
struct hfi_queue_header {
	u32 dummy0[15];
	u32 status;
	u32 dummy1[15];
	u32 start_addr;
	u32 dummy2[15];
	u32 type;
	u32 dummy3[15];
	u32 q_size;
	u32 dummy4[15];
	u32 pkt_size;
	u32 dummy5[15];
	u32 pkt_drop_cnt;
	u32 dummy6[15];
	u32 rx_wm;
	u32 dummy7[15];
	u32 tx_wm;
	u32 dummy8[15];
	u32 rx_req;
	u32 dummy9[15];
	u32 tx_req;
	u32 dummy10[15];
	u32 rx_irq_status;
	u32 dummy11[15];
	u32 tx_irq_status;
	u32 dummy12[15];
	u32 read_idx;
	u32 dummy13[15];
	u32 write_idx;
	u32 dummy14[15];  /* 15 elements - structure must be 956 bytes (0x3BC) */
};

struct hfi_queue_table_header {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_queues;
	u32 num_active_queues;
	struct hfi_queue_header queues[];
};

/* Track allocation info for proper cleanup */
struct icp_mem_region {
	void *vaddr;
	dma_addr_t dma_addr;
	dma_addr_t iova;
	size_t size;
};

struct icp_hfi_mem {
	struct icp_mem_region qtbl;
	struct icp_mem_region cmd_q;
	struct icp_mem_region msg_q;
	struct icp_mem_region dbg_q;
	struct icp_mem_region sfr;
	struct icp_mem_region shmem;
	struct icp_mem_region secheap;  /* FwUncached/SecHeap - 7MB at 0x10400000 */
	struct icp_mem_region qdss;     /* QDSS region - 1MB at 0x10b00000 */
};

#define ICP_NUM_CLOCKS 13

static const char * const icp_clock_names[] = {
	"ahb",
	"core", 
	"debug_xo",
	/* BPS clocks */
	"bps_ahb",
	"bps_fast_ahb",
	"bps",
	"cpas_bps",
	/* IPE clocks */
	"ipe_ahb",
	"ipe_nps_fast_ahb",
	"ipe_pps_fast_ahb",
	"ipe_nps",
	"ipe_pps",
	"cpas_ipe",
};

struct camss_icp {
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *cirq_base;
	int irq;

	struct clk_bulk_data clocks[ICP_NUM_CLOCKS];
	int num_clocks;
	struct icc_path *icc_path;

	/* Multiple power domains */
	struct dev_pm_domain_list *pd_list;

	phys_addr_t fw_phys;
	size_t fw_size;
	u32 fw_version;
	bool use_pil;

	struct icp_hfi_mem hfi_mem;
	struct hfi_queue_table_header *qtbl;
	void *cmd_queue;
	void *msg_queue;

	struct mutex lock;
	struct completion msg_complete;
	bool booted;
	bool fw_ready;
	bool fw_error;
	int ref_count;
};

static struct camss_icp *g_icp;
static DEFINE_MUTEX(g_icp_mutex);

/* Register helpers */
static inline void icp_gp_write(struct camss_icp *icp, u32 reg, u32 val)
{
	writel(val, icp->csr_base + ICP_CSR_GP_REG_BASE + reg);
}

static inline u32 icp_gp_read(struct camss_icp *icp, u32 reg)
{
	return readl(icp->csr_base + ICP_CSR_GP_REG_BASE + reg);
}

/* Debug register dump on failure */
static void icp_dump_debug_regs(struct camss_icp *icp, const char *context)
{
	u32 hw_ver, tcm_size, dbg_status, dbg_ctrl;
	u32 gp[ICP_NUM_GP_REGS];
	int i;

	hw_ver = readl(icp->csr_base + ICP_CSR_HW_VERSION);
	tcm_size = readl(icp->csr_base + ICP_CSR_TCM_SIZE);
	dbg_status = readl(icp->csr_base + ICP_CSR_DBG_STATUS);
	dbg_ctrl = readl(icp->csr_base + ICP_CSR_DBG_CTRL);

	for (i = 0; i < ICP_NUM_GP_REGS; i++)
		gp[i] = icp_gp_read(icp, GP_REG(i));

	dev_err(icp->dev, "=== ICP Register Dump (%s) ===\n", context);
	dev_err(icp->dev, "HW_VERSION: 0x%08x  TCM_SIZE: 0x%08x\n", hw_ver, tcm_size);
	dev_err(icp->dev, "DBG_STATUS: 0x%08x  DBG_CTRL: 0x%08x\n", dbg_status, dbg_ctrl);

	/* Dump HFI interface registers - these are the important ones */
	dev_err(icp->dev, "HFI: FW_VER=0x%08x INIT_REQ=0x%08x INIT_RESP=0x%08x\n",
		readl(icp->csr_base + HFI_REG_FW_VERSION),
		readl(icp->csr_base + HFI_REG_HOST_ICP_INIT_REQUEST),
		readl(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESPONSE));
	dev_err(icp->dev, "HFI: SHMEM=0x%08x/0x%08x QTBL=0x%08x\n",
		readl(icp->csr_base + HFI_REG_SHARED_MEM_PTR),
		readl(icp->csr_base + HFI_REG_SHARED_MEM_SIZE),
		readl(icp->csr_base + HFI_REG_QTBL_PTR));
	dev_err(icp->dev, "HFI: SEC_HEAP=0x%08x/0x%08x SFR=0x%08x\n",
		readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR),
		readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE),
		readl(icp->csr_base + HFI_REG_SFR_PTR));
	dev_err(icp->dev, "HFI: QDSS=0x%08x/0x%08x IO1=0x%08x/0x%08x\n",
		readl(icp->csr_base + HFI_REG_QDSS_IOVA),
		readl(icp->csr_base + HFI_REG_QDSS_IOVA_SIZE),
		readl(icp->csr_base + HFI_REG_IO_REGION_IOVA),
		readl(icp->csr_base + HFI_REG_IO_REGION_SIZE));
	dev_err(icp->dev, "HFI: IO2=0x%08x/0x%08x FWUNC=0x%08x/0x%08x\n",
		readl(icp->csr_base + HFI_REG_IO2_REGION_IOVA),
		readl(icp->csr_base + HFI_REG_IO2_REGION_SIZE),
		readl(icp->csr_base + HFI_REG_FWUNCACHED_REGION_IOVA),
		readl(icp->csr_base + HFI_REG_FWUNCACHED_REGION_SIZE));

	/* GP registers for reference */
	dev_err(icp->dev, "GP[ 0- 3]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[0], gp[1], gp[2], gp[3]);
	dev_err(icp->dev, "GP[ 4- 7]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[4], gp[5], gp[6], gp[7]);
	dev_err(icp->dev, "GP[ 8-11]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[8], gp[9], gp[10], gp[11]);
	dev_err(icp->dev, "GP[12-15]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[12], gp[13], gp[14], gp[15]);
	dev_err(icp->dev, "GP[16-19]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[16], gp[17], gp[18], gp[19]);
	dev_err(icp->dev, "GP[20-23]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
		gp[20], gp[21], gp[22], gp[23]);

	/* Dump SFR (Subsystem Failure Reason) buffer if available */
	if (icp->hfi_mem.sfr.vaddr) {
		char *sfr = icp->hfi_mem.sfr.vaddr;
		u32 sfr_size = *(u32 *)sfr;
		/* Only print if firmware wrote an error (size < capacity) */
		if (sfr_size > 0 && sfr_size < HFI_SFR_SIZE - 8) {
			/* Safely null-terminate within buffer bounds */
			sfr[HFI_SFR_SIZE - 1] = '\0';
			dev_err(icp->dev, "SFR: %s\n", sfr + 4);
		} else {
			dev_err(icp->dev, "SFR: (empty or invalid, size=0x%x)\n", sfr_size);
		}
	}

	/* Dump queue table header */
	if (icp->qtbl) {
		struct hfi_queue_table_header *qtbl = icp->qtbl;
		dev_err(icp->dev, "QTBL: ver=0x%x size=0x%x num_q=%d active=%d\n",
			qtbl->version, qtbl->size, qtbl->num_queues, qtbl->num_active_queues);
		dev_err(icp->dev, "CMD_Q: status=%d ri=%d wi=%d\n",
			qtbl->queues[0].status, qtbl->queues[0].read_idx, qtbl->queues[0].write_idx);
		dev_err(icp->dev, "MSG_Q: status=%d ri=%d wi=%d\n",
			qtbl->queues[1].status, qtbl->queues[1].read_idx, qtbl->queues[1].write_idx);
	}

	dev_err(icp->dev, "=== End ICP Register Dump ===\n");
}

static int icp_alloc_region(struct camss_icp *icp, struct icp_mem_region *region,
			    size_t size, dma_addr_t iova)
{
	struct iommu_domain *domain;
	struct sg_table sgt;
	int ret;

	domain = iommu_get_domain_for_dev(icp->dev);
	if (!domain) {
		dev_err(icp->dev, "No IOMMU domain for device\n");
		return -ENODEV;
	}

	/* Allocate DMA coherent memory (zeroed) */
	region->vaddr = dma_alloc_coherent(icp->dev, size, &region->dma_addr,
					   GFP_KERNEL);
	if (!region->vaddr) {
		dev_err(icp->dev, "dma_alloc_coherent failed: size=0x%zx\n", size);
		return -ENOMEM;
	}

	/* Get scatter-gather table for the allocation */
	ret = dma_get_sgtable(icp->dev, &sgt, region->vaddr, region->dma_addr, size);
	if (ret < 0) {
		dev_err(icp->dev, "dma_get_sgtable failed: ret=%d\n", ret);
		goto err_free_dma;
	}

	/* Map to the specified IOVA */
	ret = iommu_map_sg(domain, iova, sgt.sgl, sgt.nents,
			   IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	sg_free_table(&sgt);

	if (ret < 0) {
		dev_err(icp->dev, "iommu_map_sg failed: iova=0x%llx size=0x%zx ret=%d\n",
			(u64)iova, size, ret);
		goto err_free_dma;
	}

	region->iova = iova;
	region->size = size;

	dev_dbg(icp->dev, "Allocated region: vaddr=%px dma=0x%llx iova=0x%llx size=0x%zx\n",
		region->vaddr, (u64)region->dma_addr, (u64)iova, size);

	return 0;

err_free_dma:
	dma_free_coherent(icp->dev, size, region->vaddr, region->dma_addr);
	region->vaddr = NULL;
	return ret;
}

static void icp_free_region(struct camss_icp *icp, struct icp_mem_region *region)
{
	struct iommu_domain *domain;

	if (!region->vaddr)
		return;

	domain = iommu_get_domain_for_dev(icp->dev);
	if (domain)
		iommu_unmap(domain, region->iova, region->size);

	dma_free_coherent(icp->dev, region->size, region->vaddr, region->dma_addr);
	region->vaddr = NULL;
}

/* HFI queue init - allocate each region separately */
static int icp_hfi_queue_init(struct camss_icp *icp)
{
	struct hfi_queue_table_header *qtbl;
	struct hfi_queue_header *qhdr;
	u32 *sfr;
	int ret;

	dev_info(icp->dev, "HFI struct sizes: q_hdr=%zu qtbl_hdr=%zu (expect 956, 24)\n",
		 sizeof(struct hfi_queue_header),
		 sizeof(struct hfi_queue_table_header));

	/* Allocate each region separately with its own IOMMU mapping */
	ret = icp_alloc_region(icp, &icp->hfi_mem.qtbl, HFI_QTBL_SIZE, ICP_IOVA_QTBL);
	if (ret)
		return ret;

	ret = icp_alloc_region(icp, &icp->hfi_mem.cmd_q, HFI_Q_SIZE, ICP_IOVA_CMD_Q);
	if (ret)
		goto free_qtbl;

	ret = icp_alloc_region(icp, &icp->hfi_mem.msg_q, HFI_Q_SIZE, ICP_IOVA_MSG_Q);
	if (ret)
		goto free_cmd_q;

	ret = icp_alloc_region(icp, &icp->hfi_mem.dbg_q, HFI_Q_SIZE, ICP_IOVA_DBG_Q);
	if (ret)
		goto free_msg_q;

	ret = icp_alloc_region(icp, &icp->hfi_mem.sfr, HFI_SFR_SIZE, ICP_IOVA_SFR);
	if (ret)
		goto free_dbg_q;

	ret = icp_alloc_region(icp, &icp->hfi_mem.shmem, HFI_SHMEM_SIZE, ICP_IOVA_SHARED);
	if (ret)
		goto free_sfr;

	ret = icp_alloc_region(icp, &icp->hfi_mem.secheap, HFI_SECHEAP_SIZE, ICP_IOVA_SECHEAP);
	if (ret)
		goto free_shmem;

	ret = icp_alloc_region(icp, &icp->hfi_mem.qdss, HFI_QDSS_SIZE, ICP_IOVA_QDSS);
	if (ret)
		goto free_secheap;

	dev_info(icp->dev, "HFI regions allocated:\n");
	dev_info(icp->dev, "  QTBL:    vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.qtbl.vaddr, (u64)icp->hfi_mem.qtbl.iova, icp->hfi_mem.qtbl.size);
	dev_info(icp->dev, "  CMD_Q:   vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.cmd_q.vaddr, (u64)icp->hfi_mem.cmd_q.iova, icp->hfi_mem.cmd_q.size);
	dev_info(icp->dev, "  MSG_Q:   vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.msg_q.vaddr, (u64)icp->hfi_mem.msg_q.iova, icp->hfi_mem.msg_q.size);
	dev_info(icp->dev, "  DBG_Q:   vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.dbg_q.vaddr, (u64)icp->hfi_mem.dbg_q.iova, icp->hfi_mem.dbg_q.size);
	dev_info(icp->dev, "  SFR:     vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.sfr.vaddr, (u64)icp->hfi_mem.sfr.iova, icp->hfi_mem.sfr.size);
	dev_info(icp->dev, "  SHMEM:   vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.shmem.vaddr, (u64)icp->hfi_mem.shmem.iova, icp->hfi_mem.shmem.size);
	dev_info(icp->dev, "  SECHEAP: vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.secheap.vaddr, (u64)icp->hfi_mem.secheap.iova, icp->hfi_mem.secheap.size);
	dev_info(icp->dev, "  QDSS:    vaddr=%px iova=0x%llx size=0x%zx\n",
		 icp->hfi_mem.qdss.vaddr, (u64)icp->hfi_mem.qdss.iova, icp->hfi_mem.qdss.size);

	/* Initialize SFR buffer - first u32 is the buffer capacity */
	sfr = icp->hfi_mem.sfr.vaddr;
	*sfr = HFI_SFR_SIZE - sizeof(u32);

	/* Initialize queue table header */
	qtbl = icp->hfi_mem.qtbl.vaddr;
	qtbl->version = HFI_QUEUE_TABLE_VERSION;
	qtbl->size = sizeof(struct hfi_queue_table_header) + 3 * sizeof(struct hfi_queue_header);
	qtbl->qhdr0_offset = sizeof(struct hfi_queue_table_header);
	qtbl->qhdr_size = sizeof(struct hfi_queue_header);
	qtbl->num_queues = 3;
	qtbl->num_active_queues = 3;

	/* CMD queue header */
	qhdr = &qtbl->queues[0];
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.cmd_q.iova;
	qhdr->type = HFI_QUEUE_CMD_TYPE;
	qhdr->q_size = HFI_CMD_Q_DATA_SIZE >> 2;  /* size in words */
	qhdr->pkt_size = 0;  /* variable packet size */
	qhdr->rx_wm = 1;
	qhdr->tx_wm = 1;
	qhdr->rx_req = 1;

	/* MSG queue header */
	qhdr = &qtbl->queues[1];
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.msg_q.iova;
	qhdr->type = HFI_QUEUE_MSG_TYPE;
	qhdr->q_size = HFI_MSG_Q_DATA_SIZE >> 2;
	qhdr->pkt_size = 0;
	qhdr->rx_wm = 1;
	qhdr->tx_wm = 1;
	qhdr->rx_req = 1;

	/* DBG queue header */
	qhdr = &qtbl->queues[2];
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.dbg_q.iova;
	qhdr->type = HFI_QUEUE_DBG_TYPE;
	qhdr->q_size = HFI_DBG_Q_DATA_SIZE >> 2;
	qhdr->pkt_size = 0;
	qhdr->rx_wm = 1;
	qhdr->tx_wm = 1024;

	/* Ensure all writes are visible before firmware accesses */
	wmb();

	/* Save pointers for later use */
	icp->qtbl = qtbl;
	icp->cmd_queue = icp->hfi_mem.cmd_q.vaddr;
	icp->msg_queue = icp->hfi_mem.msg_q.vaddr;

	dev_info(icp->dev, "QTBL initialized: ver=0x%x size=0x%x qhdr_size=0x%x\n",
		 qtbl->version, qtbl->size, qtbl->qhdr_size);

	return 0;

free_secheap:
	icp_free_region(icp, &icp->hfi_mem.secheap);
free_shmem:
	icp_free_region(icp, &icp->hfi_mem.shmem);
free_sfr:
	icp_free_region(icp, &icp->hfi_mem.sfr);
free_dbg_q:
	icp_free_region(icp, &icp->hfi_mem.dbg_q);
free_msg_q:
	icp_free_region(icp, &icp->hfi_mem.msg_q);
free_cmd_q:
	icp_free_region(icp, &icp->hfi_mem.cmd_q);
free_qtbl:
	icp_free_region(icp, &icp->hfi_mem.qtbl);
	return ret;
}

static void icp_hfi_queue_deinit(struct camss_icp *icp)
{
	icp_free_region(icp, &icp->hfi_mem.qdss);
	icp_free_region(icp, &icp->hfi_mem.secheap);
	icp_free_region(icp, &icp->hfi_mem.shmem);
	icp_free_region(icp, &icp->hfi_mem.sfr);
	icp_free_region(icp, &icp->hfi_mem.dbg_q);
	icp_free_region(icp, &icp->hfi_mem.msg_q);
	icp_free_region(icp, &icp->hfi_mem.cmd_q);
	icp_free_region(icp, &icp->hfi_mem.qtbl);
}

/* HFI message receive */
static int icp_hfi_recv_msg(struct camss_icp *icp, void *buf, u32 *size, u32 max)
{
	struct hfi_queue_header *qhdr = &icp->qtbl->queues[1];
	u32 *q = icp->msg_queue;
	u32 ri, wi, qs, ps, i;
	u32 *dst = buf;

	ri = qhdr->read_idx;
	wi = qhdr->write_idx;
	qs = qhdr->q_size;
	rmb();

	if (ri == wi)
		return -ENODATA;

	ps = q[ri];
	if (ps > max || ps < 4)
		return -EINVAL;

	*size = ps;
	for (i = 0; i < (ps >> 2); i++) {
		dst[i] = q[ri];
		ri = (ri + 1) % qs;
	}

	qhdr->read_idx = ri;
	wmb();
	return 0;
}

/* HFI command send */
static int icp_hfi_send_cmd(struct camss_icp *icp, void *pkt, u32 size)
{
	struct hfi_queue_header *qhdr = &icp->qtbl->queues[0];
	u32 *q = icp->cmd_queue;
	u32 *src = pkt;
	u32 ri, wi, qs, new_wi, i;

	if (size & 3) {
		dev_err(icp->dev, "HFI cmd size not aligned: %u\n", size);
		return -EINVAL;
	}

	ri = qhdr->read_idx;
	wi = qhdr->write_idx;
	qs = qhdr->q_size;

	/* Check space (simplified - assumes enough space) */
	new_wi = wi;
	for (i = 0; i < (size >> 2); i++) {
		q[new_wi] = src[i];
		new_wi = (new_wi + 1) % qs;
	}

	wmb();
	qhdr->write_idx = new_wi;
	wmb();

	/* Raise interrupt to ICP */
	writel(1, icp->cirq_base + CIRQ_HOST2ICPINT);

	return 0;
}

/* IRQ handler */
static irqreturn_t icp_irq_handler(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status = readl(icp->cirq_base + CIRQ_OB_STATUS);

	if (!status)
		return IRQ_NONE;

	writel(status, icp->cirq_base + CIRQ_OB_CLEAR);

	if (status & CIRQ_WDT_BITE_WS0) {
		dev_err(icp->dev, "Watchdog bite! CIRQ status=0x%08x\n", status);
		icp_dump_debug_regs(icp, "watchdog bite");
		icp->fw_error = true;
		complete(&icp->msg_complete);
	}

	if (status & CIRQ_ICP2HOSTINT) {
		u8 buf[256];
		u32 size;

		while (icp_hfi_recv_msg(icp, buf, &size, sizeof(buf)) == 0) {
			u32 pkt_type = ((u32 *)buf)[1];
			if (pkt_type == HFI_MSG_SYS_INIT_DONE) {
				dev_info(icp->dev, "SYS_INIT_DONE received\n");
				icp->fw_ready = true;
				complete(&icp->msg_complete);
			}
		}
	}

	return IRQ_HANDLED;
}

/* Firmware loading */
static int icp_load_firmware(struct camss_icp *icp)
{
	struct device_node *node;
	const struct firmware *fw;
	struct resource res;
	const char *fw_name;
	char fw_path[64];
	void *vaddr;
	ssize_t fw_size;
	int ret;

	ret = of_property_read_string(icp->dev->of_node, "firmware-name", &fw_name);
	if (ret)
		return ret;

	snprintf(fw_path, sizeof(fw_path), "%s.mdt", fw_name);

	node = of_parse_phandle(icp->dev->of_node, "memory-region", 0);
	if (!node)
		return -ENODEV;

	ret = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (ret)
		return ret;

	dev_info(icp->dev, "FW memory: phys=0x%pa size=%llu\n", &res.start, resource_size(&res));

	ret = firmware_request_nowarn(&fw, fw_path, icp->dev);
	if (ret)
		return ret;

	fw_size = qcom_mdt_get_size(fw);
	if (fw_size < 0 || (size_t)fw_size > resource_size(&res)) {
		release_firmware(fw);
		return -EINVAL;
	}

	vaddr = ioremap_wc(res.start, resource_size(&res));
	if (!vaddr) {
		release_firmware(fw);
		return -ENOMEM;
	}

	ret = qcom_mdt_load(icp->dev, fw, fw_path, ICP_PAS_ID, vaddr,
			    res.start, resource_size(&res), NULL);
	iounmap(vaddr);
	release_firmware(fw);

	if (ret == 0) {
		dev_info(icp->dev, "Firmware loaded: %s (%zd bytes)\n", fw_path, fw_size);
		icp->fw_phys = res.start;
		icp->fw_size = fw_size;
	}

	return ret;
}

/* Boot sequence */
static int icp_boot(struct camss_icp *icp)
{
	unsigned long timeout;
	u32 hw_version, status;
	int ret;

	/* Enable all clocks (ICP + BPS + IPE) */
	ret = clk_bulk_prepare_enable(icp->num_clocks, icp->clocks);
	if (ret) {
		dev_err(icp->dev, "Failed to enable clocks: %d\n", ret);
		return ret;
	}

	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 1000000000);

	/* Read HW version */
	hw_version = readl(icp->csr_base + ICP_CSR_HW_VERSION);
	dev_info(icp->dev, "HW version: 0x%08x\n", hw_version);

	if (hw_version == 0 || hw_version == 0xffffffff) {
		dev_err(icp->dev, "Invalid HW version - register access failed\n");
		ret = -EIO;
		goto err_clk;
	}

	/* Configure interrupts - unmask ICP2HOST */
	writel(0x7f, icp->cirq_base + CIRQ_OB_CLEAR);
	writel(CIRQ_ICP2HOSTINT | CIRQ_WDT_BITE_WS0, icp->cirq_base + CIRQ_OB_MASK);

	/* Load firmware into memory */
	ret = icp_load_firmware(icp);
	if (ret)
		goto err_clk;

	/* Allocate HFI queues BEFORE starting firmware */
	ret = icp_hfi_queue_init(icp);
	if (ret)
		goto err_clk;

	/* Start firmware via TrustZone FIRST - firmware must be running
	 * before we write the HFI config registers
	 */
	dev_info(icp->dev, "Starting ICP via TrustZone\n");
	ret = qcom_scm_pas_auth_and_reset(ICP_PAS_ID);
	if (ret) {
		dev_err(icp->dev, "TZ auth_and_reset failed: %d\n", ret);
		goto err_hfi;
	}
	icp->use_pil = true;

	/* Give firmware time to boot and reach its init wait state */
	msleep(100);

	/* NOW program HFI interface registers - firmware is running and waiting */
	dev_info(icp->dev, "Programming HFI interface registers:\n");
	dev_info(icp->dev, "  SHMEM=0x%llx/%zx QTBL=0x%llx\n",
		 (u64)icp->hfi_mem.shmem.iova, icp->hfi_mem.shmem.size,
		 (u64)icp->hfi_mem.qtbl.iova);
	dev_info(icp->dev, "  SECHEAP=0x%llx/%zx SFR=0x%llx\n",
		 (u64)icp->hfi_mem.secheap.iova, icp->hfi_mem.secheap.size,
		 (u64)icp->hfi_mem.sfr.iova);
	dev_info(icp->dev, "  QDSS=0x%llx/%zx\n",
		 (u64)icp->hfi_mem.qdss.iova, icp->hfi_mem.qdss.size);

	/* Write HFI configuration registers */
	writel(icp->hfi_mem.shmem.iova, icp->csr_base + HFI_REG_SHARED_MEM_PTR);
	writel(HFI_SHMEM_REGION_SIZE, icp->csr_base + HFI_REG_SHARED_MEM_SIZE);
	writel(icp->hfi_mem.qtbl.iova, icp->csr_base + HFI_REG_QTBL_PTR);
	writel(icp->hfi_mem.secheap.iova, icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR);
	writel(icp->hfi_mem.secheap.size, icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE);
	writel(icp->hfi_mem.sfr.iova, icp->csr_base + HFI_REG_SFR_PTR);
	writel(icp->hfi_mem.qdss.iova, icp->csr_base + HFI_REG_QDSS_IOVA);
	writel(icp->hfi_mem.qdss.size, icp->csr_base + HFI_REG_QDSS_IOVA_SIZE);

	/* IO regions - not allocated, just define the IOVA range for firmware */
	writel(ICP_IOVA_IO1, icp->csr_base + HFI_REG_IO_REGION_IOVA);
	writel(HFI_IO1_SIZE, icp->csr_base + HFI_REG_IO_REGION_SIZE);
	writel(ICP_IOVA_IO2, icp->csr_base + HFI_REG_IO2_REGION_IOVA);
	writel(HFI_IO2_SIZE, icp->csr_base + HFI_REG_IO2_REGION_SIZE);

	/* FW uncached region - same as secheap */
	writel(icp->hfi_mem.secheap.iova, icp->csr_base + HFI_REG_FWUNCACHED_REGION_IOVA);
	writel(HFI_FWUNCACHED_SIZE, icp->csr_base + HFI_REG_FWUNCACHED_REGION_SIZE);
	wmb();

	/* DIAGNOSTIC 1: Raw readback after writes, before INIT_REQUEST */
	dev_info(icp->dev, "=== DIAG1: After writes, before INIT_REQUEST ===\n");
	dev_info(icp->dev, "Raw CSR offsets 0x20-0x6C (GP registers):\n");
	dev_info(icp->dev, "  [0x20]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x20), readl(icp->csr_base + 0x24),
		readl(icp->csr_base + 0x28), readl(icp->csr_base + 0x2c));
	dev_info(icp->dev, "  [0x30]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x30), readl(icp->csr_base + 0x34),
		readl(icp->csr_base + 0x38), readl(icp->csr_base + 0x3c));
	dev_info(icp->dev, "  [0x40]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x40), readl(icp->csr_base + 0x44),
		readl(icp->csr_base + 0x48), readl(icp->csr_base + 0x4c));
	dev_info(icp->dev, "  [0x50]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x50), readl(icp->csr_base + 0x54),
		readl(icp->csr_base + 0x58), readl(icp->csr_base + 0x5c));
	dev_info(icp->dev, "  [0x60]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x60), readl(icp->csr_base + 0x64),
		readl(icp->csr_base + 0x68), readl(icp->csr_base + 0x6c));

	/* Signal host init request - firmware reads config registers NOW */
	dev_info(icp->dev, "Signaling host init request (offset 0x%02x = 1)\n",
		HFI_REG_HOST_ICP_INIT_REQUEST);
	writel(ICP_INIT_REQUEST_SET, icp->csr_base + HFI_REG_HOST_ICP_INIT_REQUEST);
	wmb();

	/* Raise interrupt to ICP */
	writel(1, icp->cirq_base + CIRQ_HOST2ICPINT);

	/* DIAGNOSTIC 2: Readback right after INIT_REQUEST */
	msleep(10);
	dev_info(icp->dev, "=== DIAG2: 10ms after INIT_REQUEST ===\n");
	dev_info(icp->dev, "Raw CSR offsets 0x20-0x6C (GP registers):\n");
	dev_info(icp->dev, "  [0x20]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x20), readl(icp->csr_base + 0x24),
		readl(icp->csr_base + 0x28), readl(icp->csr_base + 0x2c));
	dev_info(icp->dev, "  [0x30]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x30), readl(icp->csr_base + 0x34),
		readl(icp->csr_base + 0x38), readl(icp->csr_base + 0x3c));
	dev_info(icp->dev, "  [0x40]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x40), readl(icp->csr_base + 0x44),
		readl(icp->csr_base + 0x48), readl(icp->csr_base + 0x4c));
	dev_info(icp->dev, "  [0x50]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x50), readl(icp->csr_base + 0x54),
		readl(icp->csr_base + 0x58), readl(icp->csr_base + 0x5c));
	dev_info(icp->dev, "  [0x60]: %08x %08x %08x %08x\n",
		readl(icp->csr_base + 0x60), readl(icp->csr_base + 0x64),
		readl(icp->csr_base + 0x68), readl(icp->csr_base + 0x6c));

	/* Wait for response via HFI register */
	ret = readl_poll_timeout(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESPONSE,
				 status, status == ICP_INIT_RESP_SUCCESS,
				 HFI_POLL_DELAY_US, HFI_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(icp->dev, "Firmware init response timeout\n");
		/* DIAGNOSTIC 3: On timeout */
		dev_info(icp->dev, "=== DIAG3: TIMEOUT ===\n");
		dev_info(icp->dev, "Raw CSR offsets 0x20-0x6C (GP registers):\n");
		dev_info(icp->dev, "  [0x20]: %08x %08x %08x %08x\n",
			readl(icp->csr_base + 0x20), readl(icp->csr_base + 0x24),
			readl(icp->csr_base + 0x28), readl(icp->csr_base + 0x2c));
		dev_info(icp->dev, "  [0x30]: %08x %08x %08x %08x\n",
			readl(icp->csr_base + 0x30), readl(icp->csr_base + 0x34),
			readl(icp->csr_base + 0x38), readl(icp->csr_base + 0x3c));
		dev_info(icp->dev, "  [0x40]: %08x %08x %08x %08x\n",
			readl(icp->csr_base + 0x40), readl(icp->csr_base + 0x44),
			readl(icp->csr_base + 0x48), readl(icp->csr_base + 0x4c));
		dev_info(icp->dev, "  [0x50]: %08x %08x %08x %08x\n",
			readl(icp->csr_base + 0x50), readl(icp->csr_base + 0x54),
			readl(icp->csr_base + 0x58), readl(icp->csr_base + 0x5c));
		dev_info(icp->dev, "  [0x60]: %08x %08x %08x %08x\n",
			readl(icp->csr_base + 0x60), readl(icp->csr_base + 0x64),
			readl(icp->csr_base + 0x68), readl(icp->csr_base + 0x6c));
		icp_dump_debug_regs(icp, "init response timeout");
		goto err_shutdown;
	}

	icp->fw_version = readl(icp->csr_base + HFI_REG_FW_VERSION);
	dev_info(icp->dev, "Firmware version: 0x%08x\n", icp->fw_version);

	/* Send HFI_CMD_SYS_INIT to start firmware */
	{
		struct hfi_cmd_sys_init cmd = {
			.size = sizeof(cmd),
			.pkt_type = HFI_CMD_SYS_INIT,
		};
		dev_info(icp->dev, "Sending HFI_CMD_SYS_INIT\n");
		ret = icp_hfi_send_cmd(icp, &cmd, sizeof(cmd));
		if (ret) {
			dev_err(icp->dev, "Failed to send SYS_INIT: %d\n", ret);
			goto err_hfi;
		}
	}

	/* Wait for SYS_INIT_DONE */
	icp->fw_ready = false;
	icp->fw_error = false;
	reinit_completion(&icp->msg_complete);

	timeout = wait_for_completion_timeout(&icp->msg_complete,
					      msecs_to_jiffies(ICP_BOOT_TIMEOUT_MS));
	if (!timeout || icp->fw_error || !icp->fw_ready) {
		dev_err(icp->dev, "Boot failed: timeout=%lu error=%d ready=%d\n",
			timeout, icp->fw_error, icp->fw_ready);
		icp_dump_debug_regs(icp, "SYS_INIT_DONE timeout");
		ret = -ETIMEDOUT;
		goto err_shutdown;
	}

	icp->booted = true;
	dev_info(icp->dev, "ICP booted successfully\n");
	return 0;

err_shutdown:
	if (icp->use_pil)
		qcom_scm_pas_shutdown(ICP_PAS_ID);
	icp->use_pil = false;
err_hfi:
	icp_hfi_queue_deinit(icp);
err_clk:
	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);
	clk_bulk_disable_unprepare(icp->num_clocks, icp->clocks);
	return ret;
}

static void icp_shutdown(struct camss_icp *icp)
{
	if (!icp->booted)
		return;

	writel(0, icp->cirq_base + CIRQ_OB_MASK);
	if (icp->use_pil)
		qcom_scm_pas_shutdown(ICP_PAS_ID);
	icp_hfi_queue_deinit(icp);
	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);
	clk_bulk_disable_unprepare(icp->num_clocks, icp->clocks);
	icp->booted = false;
	icp->use_pil = false;
}

/* Public APIs */
struct camss_icp *camss_icp_get(struct device *dev)
{
	struct camss_icp *icp;

	mutex_lock(&g_icp_mutex);
	icp = g_icp;
	if (icp)
		icp->ref_count++;
	mutex_unlock(&g_icp_mutex);

	return icp ? icp : ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(camss_icp_get);

void camss_icp_put(struct camss_icp *icp)
{
	if (!icp)
		return;
	mutex_lock(&g_icp_mutex);
	if (icp->ref_count > 0)
		icp->ref_count--;
	mutex_unlock(&g_icp_mutex);
}
EXPORT_SYMBOL_GPL(camss_icp_put);

bool camss_icp_is_booted(struct camss_icp *icp)
{
	return icp && icp->booted;
}
EXPORT_SYMBOL_GPL(camss_icp_is_booted);

/* Platform driver */
static int camss_icp_probe(struct platform_device *pdev)
{
	struct dev_pm_domain_attach_data pd_data = {
		.pd_flags = PD_FLAG_DEV_LINK_ON,
	};
	struct camss_icp *icp;
	int ret, i;

	icp = devm_kzalloc(&pdev->dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;

	icp->dev = &pdev->dev;
	mutex_init(&icp->lock);
	init_completion(&icp->msg_complete);

	icp->csr_base = devm_platform_ioremap_resource_byname(pdev, "csr");
	if (IS_ERR(icp->csr_base))
		return PTR_ERR(icp->csr_base);

	icp->cirq_base = devm_platform_ioremap_resource_byname(pdev, "cirq");
	if (IS_ERR(icp->cirq_base))
		return PTR_ERR(icp->cirq_base);

	icp->irq = platform_get_irq(pdev, 0);
	if (icp->irq < 0)
		return icp->irq;

	/* Attach all power domains (TITAN_TOP, BPS, IPE) */
	ret = dev_pm_domain_attach_list(&pdev->dev, &pd_data, &icp->pd_list);
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "Failed to attach power domains: %d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "Attached %d power domains\n", ret > 0 ? ret : 1);

	/* Initialize reserved memory for ICP DMA allocations
	 * memory-region[0] = camera_fw_mem (for firmware loading)
	 * memory-region[1] = camera_icp_mem (for HFI queues/buffers)
	 */
	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 1);
	if (ret && ret != -ENODEV) {
		dev_warn(&pdev->dev, "Failed to init camera_icp_mem: %d (continuing anyway)\n", ret);
	} else if (ret == 0) {
		dev_info(&pdev->dev, "Using camera_icp_mem reserved memory pool\n");
	}

	/* Get all clocks (ICP + BPS + IPE) */
	for (i = 0; i < ICP_NUM_CLOCKS; i++)
		icp->clocks[i].id = icp_clock_names[i];

	ret = devm_clk_bulk_get_optional(&pdev->dev, ICP_NUM_CLOCKS, icp->clocks);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get clocks: %d\n", ret);
		goto err_pd;
	}

	/* Count how many clocks we actually got */
	icp->num_clocks = 0;
	for (i = 0; i < ICP_NUM_CLOCKS; i++) {
		if (icp->clocks[i].clk) {
			dev_info(&pdev->dev, "Got clock: %s\n", icp_clock_names[i]);
			icp->num_clocks = i + 1;
		}
	}
	dev_info(&pdev->dev, "Loaded %d clocks\n", icp->num_clocks);

	icp->icc_path = devm_of_icc_get(&pdev->dev, "mem");
	if (IS_ERR(icp->icc_path)) {
		if (PTR_ERR(icp->icc_path) != -ENODATA) {
			ret = PTR_ERR(icp->icc_path);
			goto err_pd;
		}
		icp->icc_path = NULL;
	}

	ret = devm_request_irq(&pdev->dev, icp->irq, icp_irq_handler,
			       IRQF_TRIGGER_RISING, "camss-icp", icp);
	if (ret)
		goto err_pd;

	ret = icp_boot(icp);
	if (ret)
		goto err_pd;

	platform_set_drvdata(pdev, icp);

	mutex_lock(&g_icp_mutex);
	g_icp = icp;
	mutex_unlock(&g_icp_mutex);

	dev_info(&pdev->dev, "CAMSS ICP driver probed\n");
	return 0;

err_pd:
	dev_pm_domain_detach_list(icp->pd_list);
	return ret;
}

static void camss_icp_remove(struct platform_device *pdev)
{
	struct camss_icp *icp = platform_get_drvdata(pdev);

	mutex_lock(&g_icp_mutex);
	g_icp = NULL;
	mutex_unlock(&g_icp_mutex);

	icp_shutdown(icp);
	of_reserved_mem_device_release(&pdev->dev);
	dev_pm_domain_detach_list(icp->pd_list);
}

static const struct of_device_id camss_icp_dt_match[] = {
	{ .compatible = "qcom,x1e80100-camss-icp" },
	{ }
};
MODULE_DEVICE_TABLE(of, camss_icp_dt_match);

static struct platform_driver camss_icp_driver = {
	.probe = camss_icp_probe,
	.remove = camss_icp_remove,
	.driver = {
		.name = "camss-icp",
		.of_match_table = camss_icp_dt_match,
	},
};

module_platform_driver(camss_icp_driver);

MODULE_DESCRIPTION("Qualcomm CAMSS ICP Driver");
MODULE_LICENSE("GPL");
