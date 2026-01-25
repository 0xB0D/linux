// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm ICP (Image Control Processor) driver for X1E80100
 * 
 * Simplified version: uses DMA addresses directly without remapping
 */

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

#define ICP_PAS_ID			33

/* CSR register offsets */
#define ICP_CSR_HW_VERSION		0x00
#define ICP_CSR_TCM_SIZE		0x08
#define ICP_CSR_DBG_STATUS		0x44
#define ICP_CSR_DBG_CTRL		0x48

/* General Purpose registers - offset 0x20 from CSR base */
#define HFI_REG_FW_VERSION		0x20  /* GP0 - firmware writes version here */
#define HFI_REG_HOST_ICP_MSG		0x24  /* GP1 - host to ICP message */
#define HFI_REG_ICP_HOST_MSG		0x28  /* GP2 - ICP to host init response */
#define HFI_REG_SHARED_MEM_PTR		0x30  /* GP4 - shared memory IOVA */
#define HFI_REG_SHARED_MEM_SIZE		0x34  /* GP5 - shared memory size */
#define HFI_REG_QTBL_PTR		0x38  /* GP6 - queue table IOVA */
#define HFI_REG_SECONDARY_HEAP_PTR	0x3C  /* GP7 - secondary heap IOVA */
#define HFI_REG_SECONDARY_HEAP_SIZE	0x40  /* GP8 - secondary heap size */
#define HFI_REG_RESERVED		0x44  /* GP9 - reserved/status */
#define HFI_REG_SFR_PTR			0x48  /* GP10 - SFR buffer IOVA */
#define HFI_REG_QDSS_IOVA		0x4C  /* GP11 - QDSS buffer IOVA */
#define HFI_REG_QDSS_IOVA_SIZE		0x50  /* GP12 - QDSS buffer size */
#define HFI_REG_IO_REGION_1_IOVA	0x54  /* GP13 - IO region 1 IOVA */
#define HFI_REG_IO_REGION_1_SIZE	0x58  /* GP14 - IO region 1 size */
#define HFI_REG_IO_REGION_2_IOVA	0x5C  /* GP15 - IO region 2 IOVA */
#define HFI_REG_IO_REGION_2_SIZE	0x60  /* GP16 - IO region 2 size */
#define HFI_REG_FWUNCACHED_IOVA		0x64  /* GP17 - FW uncached region IOVA */
#define HFI_REG_FWUNCACHED_SIZE		0x68  /* GP18 - FW uncached region size */

/* CIRQ register offsets */
#define CIRQ_OB_MASK			0x00
#define CIRQ_OB_STATUS			0x04
#define CIRQ_OB_CLEAR			0x08

/* CIRQ bits */
#define CIRQ_ICP2HOSTINT		BIT(0)
#define CIRQ_WDT_BITE_WS0		BIT(6)

/* HFI constants */
#define HFI_QUEUE_TABLE_VERSION		0xFFFFFFFF
#define HFI_Q_CMD_TYPE			0
#define HFI_Q_MSG_TYPE			1
#define HFI_Q_DBG_TYPE			2

/* Memory sizes - matching CAMX */
#define HFI_SHMEM_SIZE			SZ_1M
#define HFI_FWUNCACHED_SIZE		(7 * SZ_1M)
#define HFI_QDSS_SIZE			SZ_1M
#define HFI_QTBL_SIZE			SZ_1M
#define HFI_Q_SIZE			SZ_1M
#define HFI_SFR_SIZE			SZ_8K
#define HFI_SECHEAP_SIZE		SZ_1M

/*
 * HFI Queue Header - CAMX compatible
 * Each field is padded to 64 bytes (16 u32s) for cache line alignment.
 * Layout: 15 dummy u32s, then 1 real field, repeated for each field.
 * Final dummy14[15] after write_idx.
 * Total size: 956 bytes (0x3BC)
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
	u32 dummy14[15];
} __packed;

/* Queue table header - 24 bytes */
struct hfi_queue_table_header {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_queues;
	u32 num_active_queues;
} __packed;

/* Memory region tracking */
struct icp_mem_region {
	void *vaddr;
	dma_addr_t dma_addr;
	size_t size;
};

/* HFI memory layout */
struct icp_hfi_mem {
	struct icp_mem_region shmem;
	struct icp_mem_region fwuncached;  /* Contains QTBL, queues, SFR */
	struct icp_mem_region qdss;
	
	/* Pointers into fwuncached region */
	void *qtbl_vaddr;
	void *cmd_q_vaddr;
	void *msg_q_vaddr;
	void *dbg_q_vaddr;
	void *sfr_vaddr;
};

#define ICP_NUM_CLOCKS 13

struct camss_icp {
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *cirq_base;
	int irq;
	
	struct clk_bulk_data clocks[ICP_NUM_CLOCKS];
	int num_clocks;
	
	struct dev_pm_domain_list *pd_list;
	struct icc_path *icc_path;
	
	struct icp_hfi_mem hfi_mem;
	
	phys_addr_t fw_phys;
	size_t fw_size;
	bool use_pil;
};

static const char * const icp_clock_names[ICP_NUM_CLOCKS] = {
	"ahb", "core", "debug_xo",
	"bps_ahb", "bps_fast_ahb", "bps", "cpas_bps",
	"ipe_ahb", "ipe_nps_fast_ahb", "ipe_pps_fast_ahb",
	"ipe_nps", "ipe_pps", "cpas_ipe"
};

/* Simple DMA allocation - just use dma_alloc_coherent */
static int icp_alloc_dma(struct camss_icp *icp, struct icp_mem_region *region,
			 size_t size)
{
	region->vaddr = dma_alloc_coherent(icp->dev, size, &region->dma_addr,
					   GFP_KERNEL);
	if (!region->vaddr) {
		dev_err(icp->dev, "dma_alloc_coherent failed: size=0x%zx\n", size);
		return -ENOMEM;
	}

	region->size = size;

	dev_info(icp->dev, "Allocated: vaddr=%px dma_addr=0x%llx size=0x%zx\n",
		 region->vaddr, (u64)region->dma_addr, size);

	return 0;
}

static void icp_free_dma(struct camss_icp *icp, struct icp_mem_region *region)
{
	if (region->vaddr) {
		dma_free_coherent(icp->dev, region->size, region->vaddr,
				  region->dma_addr);
		region->vaddr = NULL;
	}
}

/* Initialize HFI queues */
static int icp_hfi_queue_init(struct camss_icp *icp)
{
	struct hfi_queue_table_header *qtbl;
	struct hfi_queue_header *qhdr;
	void *base;
	u32 *sfr;
	int ret;

	dev_info(icp->dev, "HFI struct sizes: q_hdr=%zu qtbl_hdr=%zu (expect 956, 24)\n",
		 sizeof(struct hfi_queue_header),
		 sizeof(struct hfi_queue_table_header));

	/* Allocate SHMEM */
	ret = icp_alloc_dma(icp, &icp->hfi_mem.shmem, HFI_SHMEM_SIZE);
	if (ret)
		return ret;

	/* Allocate FwUncached - single 7MB block for all HFI structures */
	ret = icp_alloc_dma(icp, &icp->hfi_mem.fwuncached, HFI_FWUNCACHED_SIZE);
	if (ret)
		goto free_shmem;

	/* Allocate QDSS */
	ret = icp_alloc_dma(icp, &icp->hfi_mem.qdss, HFI_QDSS_SIZE);
	if (ret)
		goto free_fwuncached;

	/*
	 * Carve sub-regions from FwUncached:
	 *   +0x000000: SecHeap (1MB)
	 *   +0x100000: QTBL (1MB)
	 *   +0x200000: CMD_Q (1MB)
	 *   +0x300000: MSG_Q (1MB)
	 *   +0x400000: DBG_Q (1MB)
	 *   +0x500000: SFR (8KB)
	 */
	base = icp->hfi_mem.fwuncached.vaddr;
	icp->hfi_mem.qtbl_vaddr = base + 0x100000;
	icp->hfi_mem.cmd_q_vaddr = base + 0x200000;
	icp->hfi_mem.msg_q_vaddr = base + 0x300000;
	icp->hfi_mem.dbg_q_vaddr = base + 0x400000;
	icp->hfi_mem.sfr_vaddr = base + 0x500000;

	dev_info(icp->dev, "HFI memory layout:\n");
	dev_info(icp->dev, "  SHMEM:      dma=0x%llx size=0x%zx\n",
		 (u64)icp->hfi_mem.shmem.dma_addr, icp->hfi_mem.shmem.size);
	dev_info(icp->dev, "  FwUncached: dma=0x%llx size=0x%zx\n",
		 (u64)icp->hfi_mem.fwuncached.dma_addr, icp->hfi_mem.fwuncached.size);
	dev_info(icp->dev, "  QDSS:       dma=0x%llx size=0x%zx\n",
		 (u64)icp->hfi_mem.qdss.dma_addr, icp->hfi_mem.qdss.size);

	/* Initialize SFR - first u32 is capacity */
	sfr = icp->hfi_mem.sfr_vaddr;
	*sfr = HFI_SFR_SIZE - sizeof(u32);

	/* Initialize queue table header */
	qtbl = icp->hfi_mem.qtbl_vaddr;
	memset(qtbl, 0, sizeof(*qtbl) + 4 * sizeof(struct hfi_queue_header));

	qtbl->version = HFI_QUEUE_TABLE_VERSION;
	qtbl->size = sizeof(struct hfi_queue_table_header) + 4 * sizeof(struct hfi_queue_header);
	qtbl->qhdr0_offset = sizeof(struct hfi_queue_table_header);
	qtbl->qhdr_size = sizeof(struct hfi_queue_header);
	qtbl->num_queues = 3;
	qtbl->num_active_queues = 3;

	/* CMD queue header */
	qhdr = (struct hfi_queue_header *)((u8 *)qtbl + qtbl->qhdr0_offset);
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.fwuncached.dma_addr + 0x200000;
	qhdr->type = HFI_Q_CMD_TYPE;
	qhdr->q_size = HFI_Q_SIZE / sizeof(u32);

	/* MSG queue header */
	qhdr = (struct hfi_queue_header *)((u8 *)qtbl + qtbl->qhdr0_offset + qtbl->qhdr_size);
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.fwuncached.dma_addr + 0x300000;
	qhdr->type = HFI_Q_MSG_TYPE;
	qhdr->q_size = HFI_Q_SIZE / sizeof(u32);

	/* DBG queue header */
	qhdr = (struct hfi_queue_header *)((u8 *)qtbl + qtbl->qhdr0_offset + 2 * qtbl->qhdr_size);
	qhdr->status = 1;
	qhdr->start_addr = icp->hfi_mem.fwuncached.dma_addr + 0x400000;
	qhdr->type = HFI_Q_DBG_TYPE;
	qhdr->q_size = HFI_Q_SIZE / sizeof(u32);

	dev_info(icp->dev, "QTBL initialized: ver=0x%x size=0x%x\n",
		 qtbl->version, qtbl->size);

	wmb();
	return 0;

free_fwuncached:
	icp_free_dma(icp, &icp->hfi_mem.fwuncached);
free_shmem:
	icp_free_dma(icp, &icp->hfi_mem.shmem);
	return ret;
}

static void icp_hfi_queue_deinit(struct camss_icp *icp)
{
	icp_free_dma(icp, &icp->hfi_mem.qdss);
	icp_free_dma(icp, &icp->hfi_mem.fwuncached);
	icp_free_dma(icp, &icp->hfi_mem.shmem);
}

/* Dump GP registers for debugging */
static void icp_dump_gp_registers(struct camss_icp *icp, const char *label)
{
	u32 regs[20];
	int i;

	dev_info(icp->dev, "=== %s ===\n", label);
	dev_info(icp->dev, "Raw CSR offsets 0x20-0x6C (GP registers):\n");

	for (i = 0; i < 20; i++)
		regs[i] = readl(icp->csr_base + 0x20 + i * 4);

	for (i = 0; i < 20; i += 4)
		dev_info(icp->dev, "  [0x%02x]: %08x %08x %08x %08x\n",
			 0x20 + i * 4, regs[i], regs[i+1], regs[i+2], regs[i+3]);
}

/* Load firmware */
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

static irqreturn_t icp_irq_handler(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status;

	status = readl(icp->cirq_base + CIRQ_OB_STATUS);
	writel(status, icp->cirq_base + CIRQ_OB_CLEAR);

	dev_info(icp->dev, "IRQ: status=0x%x\n", status);

	return IRQ_HANDLED;
}

static int icp_boot(struct camss_icp *icp)
{
	dma_addr_t shmem_iova, fwuncached_iova, qdss_iova;
	dma_addr_t qtbl_iova, sfr_iova;
	unsigned long timeout;
	u32 hw_version, status;
	int ret;

	/* Enable clocks */
	ret = clk_bulk_prepare_enable(icp->num_clocks, icp->clocks);
	if (ret) {
		dev_err(icp->dev, "Failed to enable clocks: %d\n", ret);
		return ret;
	}

	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 100000000, 1000000000);

	/* Verify HW version */
	hw_version = readl(icp->csr_base + ICP_CSR_HW_VERSION);
	dev_info(icp->dev, "HW version: 0x%08x\n", hw_version);

	if (hw_version == 0 || hw_version == 0xffffffff) {
		dev_err(icp->dev, "Invalid HW version\n");
		ret = -EIO;
		goto err_clk;
	}

	/* Configure interrupts */
	writel(0x7f, icp->cirq_base + CIRQ_OB_CLEAR);
	writel(CIRQ_ICP2HOSTINT | CIRQ_WDT_BITE_WS0, icp->cirq_base + CIRQ_OB_MASK);

	/* Load firmware */
	ret = icp_load_firmware(icp);
	if (ret)
		goto err_clk;

	/* Allocate HFI queues */
	ret = icp_hfi_queue_init(icp);
	if (ret)
		goto err_clk;

	/* Start firmware via TrustZone */
	dev_info(icp->dev, "Starting ICP via TrustZone\n");
	ret = qcom_scm_pas_auth_and_reset(ICP_PAS_ID);
	if (ret) {
		dev_err(icp->dev, "TZ auth_and_reset failed: %d\n", ret);
		goto err_hfi;
	}
	icp->use_pil = true;

	msleep(100);

	/* Get actual DMA addresses for GP registers */
	shmem_iova = icp->hfi_mem.shmem.dma_addr;
	fwuncached_iova = icp->hfi_mem.fwuncached.dma_addr;
	qdss_iova = icp->hfi_mem.qdss.dma_addr;
	qtbl_iova = fwuncached_iova + 0x100000;
	sfr_iova = fwuncached_iova + 0x500000;

	/* Program HFI registers with actual DMA addresses */
	dev_info(icp->dev, "Programming HFI registers with DMA addresses:\n");
	dev_info(icp->dev, "  SHMEM=0x%llx QTBL=0x%llx\n", (u64)shmem_iova, (u64)qtbl_iova);
	dev_info(icp->dev, "  SECHEAP=0x%llx SFR=0x%llx\n", (u64)fwuncached_iova, (u64)sfr_iova);
	dev_info(icp->dev, "  QDSS=0x%llx\n", (u64)qdss_iova);

	writel(shmem_iova, icp->csr_base + HFI_REG_SHARED_MEM_PTR);
	writel(0x0FC00000, icp->csr_base + HFI_REG_SHARED_MEM_SIZE);  /* 252MB range */
	writel(qtbl_iova, icp->csr_base + HFI_REG_QTBL_PTR);
	writel(fwuncached_iova, icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR);
	writel(HFI_SECHEAP_SIZE, icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE);
	writel(sfr_iova, icp->csr_base + HFI_REG_SFR_PTR);
	writel(qdss_iova, icp->csr_base + HFI_REG_QDSS_IOVA);
	writel(HFI_QDSS_SIZE, icp->csr_base + HFI_REG_QDSS_IOVA_SIZE);

	/* IO regions - use standard CAMX values */
	writel(0x10c00000, icp->csr_base + HFI_REG_IO_REGION_1_IOVA);
	writel(0xcf400000, icp->csr_base + HFI_REG_IO_REGION_1_SIZE);
	writel(0xe0800000, icp->csr_base + HFI_REG_IO_REGION_2_IOVA);
	writel(0x1e700000, icp->csr_base + HFI_REG_IO_REGION_2_SIZE);

	writel(fwuncached_iova, icp->csr_base + HFI_REG_FWUNCACHED_IOVA);
	writel(HFI_FWUNCACHED_SIZE, icp->csr_base + HFI_REG_FWUNCACHED_SIZE);

	wmb();

	icp_dump_gp_registers(icp, "DIAG1: Before INIT_REQUEST");

	/* Signal init request */
	dev_info(icp->dev, "Signaling host init request\n");
	writel(1, icp->csr_base + HFI_REG_ICP_HOST_MSG);
	wmb();

	msleep(10);
	icp_dump_gp_registers(icp, "DIAG2: 10ms after INIT_REQUEST");

	/* Wait for firmware response */
	timeout = jiffies + msecs_to_jiffies(2000);
	while (time_before(jiffies, timeout)) {
		status = readl(icp->csr_base + HFI_REG_ICP_HOST_MSG);
		if (status == 1) {
			dev_info(icp->dev, "Firmware initialized successfully!\n");
			return 0;
		}
		msleep(10);
	}

	dev_err(icp->dev, "Firmware init timeout\n");
	icp_dump_gp_registers(icp, "DIAG3: TIMEOUT");

	/* Check SFR for crash message */
	{
		u32 *sfr = icp->hfi_mem.sfr_vaddr;
		u32 sfr_size = *sfr;
		if (sfr_size > 0 && sfr_size < HFI_SFR_SIZE) {
			char *msg = (char *)(sfr + 1);
			dev_err(icp->dev, "SFR: %.*s\n", sfr_size, msg);
		}
	}

	ret = -ETIMEDOUT;

err_hfi:
	icp_hfi_queue_deinit(icp);
err_clk:
	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);
	clk_bulk_disable_unprepare(icp->num_clocks, icp->clocks);
	return ret;
}

static int camss_icp_probe(struct platform_device *pdev)
{
	struct dev_pm_domain_attach_data pd_data = { .pd_flags = PD_FLAG_DEV_LINK_ON };
	struct camss_icp *icp;
	int ret, i;

	icp = devm_kzalloc(&pdev->dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;

	icp->dev = &pdev->dev;
	platform_set_drvdata(pdev, icp);

	icp->csr_base = devm_platform_ioremap_resource_byname(pdev, "csr");
	if (IS_ERR(icp->csr_base))
		return PTR_ERR(icp->csr_base);

	icp->cirq_base = devm_platform_ioremap_resource_byname(pdev, "cirq");
	if (IS_ERR(icp->cirq_base))
		return PTR_ERR(icp->cirq_base);

	icp->irq = platform_get_irq(pdev, 0);
	if (icp->irq < 0)
		return icp->irq;

	/* Power domains */
	ret = dev_pm_domain_attach_list(&pdev->dev, &pd_data, &icp->pd_list);
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "Failed to attach power domains: %d\n", ret);
		return ret;
	}
	dev_info(&pdev->dev, "Attached %d power domains\n", ret > 0 ? ret : 1);

	/* Reserved memory for HFI */
	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 1);
	if (ret && ret != -ENODEV)
		dev_warn(&pdev->dev, "Failed to init reserved memory: %d\n", ret);
	else if (ret == 0)
		dev_info(&pdev->dev, "Using camera_icp_mem reserved memory\n");

	/* Clocks */
	for (i = 0; i < ICP_NUM_CLOCKS; i++)
		icp->clocks[i].id = icp_clock_names[i];

	ret = devm_clk_bulk_get_optional(&pdev->dev, ICP_NUM_CLOCKS, icp->clocks);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get clocks: %d\n", ret);
		goto err_pd;
	}

	icp->num_clocks = 0;
	for (i = 0; i < ICP_NUM_CLOCKS; i++) {
		if (icp->clocks[i].clk) {
			dev_info(&pdev->dev, "Got clock: %s\n", icp_clock_names[i]);
			icp->num_clocks = i + 1;
		}
	}
	dev_info(&pdev->dev, "Loaded %d clocks\n", icp->num_clocks);

	/* Interconnect */
	icp->icc_path = devm_of_icc_get(&pdev->dev, "mem");
	if (IS_ERR(icp->icc_path)) {
		if (PTR_ERR(icp->icc_path) != -ENODATA) {
			ret = PTR_ERR(icp->icc_path);
			goto err_pd;
		}
		icp->icc_path = NULL;
	}

	/* IRQ */
	ret = devm_request_irq(&pdev->dev, icp->irq, icp_irq_handler,
			       IRQF_TRIGGER_RISING, "camss-icp", icp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		goto err_pd;
	}

	/* Boot! */
	ret = icp_boot(icp);
	if (ret)
		goto err_pd;

	dev_info(&pdev->dev, "ICP probed successfully\n");
	return 0;

err_pd:
	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);
	return ret;
}

static void camss_icp_remove(struct platform_device *pdev)
{
	struct camss_icp *icp = platform_get_drvdata(pdev);

	if (icp->use_pil)
		qcom_scm_pas_shutdown(ICP_PAS_ID);

	icp_hfi_queue_deinit(icp);

	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);

	clk_bulk_disable_unprepare(icp->num_clocks, icp->clocks);

	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);
}

static const struct of_device_id camss_icp_of_match[] = {
	{ .compatible = "qcom,x1e80100-camss-icp" },
	{ }
};
MODULE_DEVICE_TABLE(of, camss_icp_of_match);

static struct platform_driver camss_icp_driver = {
	.probe = camss_icp_probe,
	.remove = camss_icp_remove,
	.driver = {
		.name = "camss-icp",
		.of_match_table = camss_icp_of_match,
	},
};

module_platform_driver(camss_icp_driver);

MODULE_DESCRIPTION("Qualcomm ICP driver");
MODULE_LICENSE("GPL");
