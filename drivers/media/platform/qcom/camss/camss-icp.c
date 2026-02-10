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

#include "camss-icp.h"
#include "camss-icp-hfi.h"

/* CSR register offsets */
#define ICP_CSR_HW_VERSION		0x00
#define ICP_CSR_TCM_SIZE		0x08
#define ICP_CSR_DBG_STATUS		0x44
#define ICP_CSR_DBG_CTRL		0x48

/* CIRQ register offsets */
#define CIRQ_OB_MASK			0x00
#define CIRQ_OB_STATUS			0x04
#define CIRQ_OB_CLEAR			0x08
#define CIRQ_HOST2ICPINT		0x124

/* CIRQ bits */
#define CIRQ_ICP2HOSTINT		BIT(0)
#define CIRQ_WDT_BITE_WS0		BIT(6)

#define ICP_CLK_MAX			4

#define POLLING_SLEEP_US		1000
#define POLLING_TIMEOUT_US		20000

/* HFI polling - match CAMX: 100us interval, 2 second timeout */
#define HFI_POLL_DELAY_US		100
#define HFI_POLL_TIMEOUT_US		2000000

struct camss_icp_resources {
	int pas_id;
	const char * const clk_names[ICP_CLK_MAX];
	int clk_num;
	struct hfi_resources hfi_res;
};

struct camss_icp {
	struct device *dev;
	void __iomem *csr_base;
	void __iomem *cirq_base;
	int irq;
	
	struct clk_bulk_data clks[ICP_CLK_MAX];

	struct dev_pm_domain_list *pd_list;
	struct icc_path *icc_path;

	struct icp_hfi hfi;

	phys_addr_t fw_phys;
	size_t fw_size;

	const struct camss_icp_resources *icp_res;
};

/*
 * Raise interrupt to firmware - called by HFI layer after CMD_Q write
 */
static void icp_raise_irq(void *priv)
{
	struct camss_icp *icp = priv;

	dev_info(icp->dev, "Raising HOST2ICPINT (CIRQ STATUS before: 0x%x)\n",
		 readl(icp->cirq_base + CIRQ_OB_STATUS));

	writel(1, icp->cirq_base + CIRQ_HOST2ICPINT);

	/* Small delay then check if anything changed */
	udelay(100);
	dev_info(icp->dev, "CIRQ STATUS after HOST2ICPINT: 0x%x\n",
		 readl(icp->cirq_base + CIRQ_OB_STATUS));
}

/* Load firmware */
static int icp_load_firmware(struct camss_icp *icp)
{
	const struct camss_icp_resources *icp_res = icp->icp_res;
	const struct firmware *fw;
	struct device_node *node;
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

	ret = qcom_mdt_load(icp->dev, fw, fw_path, icp_res->pas_id, vaddr,
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

static irqreturn_t camss_icp_isr_thread(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status;

	/* Re-read status for logging */
	status = readl(icp->cirq_base + CIRQ_OB_STATUS);

	dev_dbg(icp->dev, "IRQ thread: status=0x%x\n", status);

	if (status & CIRQ_WDT_BITE_WS0) {
		dev_err(icp->dev, "ICP watchdog bite!\n");
		icp_hfi_dump_sfr(&icp->hfi);
	}

	/* Process message queue - signals waiters if data present */
	icp_hfi_process_msg_queue(&icp->hfi);

	/* Flush debug queue - async debug messages */
	icp_hfi_flush_debug_queue(&icp->hfi);

	return IRQ_HANDLED;
}

static irqreturn_t camss_icp_isr(int irq, void *data)
{
	struct camss_icp *icp = data;
	u32 status;

	status = readl(icp->cirq_base + CIRQ_OB_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(status, icp->cirq_base + CIRQ_OB_CLEAR);

	return IRQ_WAKE_THREAD;
}

/* Dump GP registers for debugging */
void icp_dump_gp_regs(struct camss_icp *icp, const char *label)
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

void icp_hfi_latch_regs(struct camss_icp *icp)
{
	struct icp_hfi *hfi = &icp->hfi;
	struct device *dev = hfi->dev;

	/* Program HFI registers with actual DMA addresses */
	dev_info(dev, "Programming HFI registers with DMA addresses:\n");
	dev_info(dev, "  SHMEM=0x%llx\n", (u64)hfi->hfi_mem.shmem.dma_addr);
	dev_info(dev, "  QDSS=0x%llx FWUNCACHED=0x%llx\n", (u64)hfi->hfi_mem.qdss.dma_addr,
							   (u64)hfi->hfi_mem.fwuncached.dma_addr);
	dev_info(dev, "  SECHEAP=0x%llx QTBL=0x%llx\n", (u64)hfi->hfi_mem.secheap.dma_addr,
							(u64)hfi->hfi_mem.q_tbl.dma_addr);
	dev_info(dev, "  CMD_Q=0x%llx MSQ_Q=0x%llx\n", (u64)hfi->hfi_mem.q_data[HFI_Q_CMD_TYPE].dma_addr,
						       (u64)hfi->hfi_mem.q_data[HFI_Q_MSG_TYPE].dma_addr);
	dev_info(dev, "  DBG_Q=0x%llx SFR=0x%llx\n", (u64)hfi->hfi_mem.q_data[HFI_Q_DBG_TYPE].dma_addr,
						     (u64)hfi->hfi_mem.sfr.dma_addr);

	/* Shared memory */
	writel(hfi->hfi_mem.shmem.dma_addr, icp->csr_base + HFI_REG_SHARED_MEM_PTR);
	//writel(hfi->hfi_mem.shmem.size, icp->csr_base + HFI_REG_SHARED_MEM_SIZE);
	writel(0xfc000000, icp->csr_base + HFI_REG_SHARED_MEM_SIZE);

	/* QTBL - header describing HFI queues */
	writel(hfi->hfi_mem.q_tbl.dma_addr, icp->csr_base + HFI_REG_QTBL_PTR);

	/* QDSS */
	writel(hfi->hfi_mem.qdss.dma_addr, icp->csr_base + HFI_REG_QDSS_IOVA);
	writel(HFI_QDSS_SIZE, icp->csr_base + HFI_REG_QDSS_IOVA_SIZE);

	/* FW Uncached */
	writel(hfi->hfi_mem.fwuncached.dma_addr, icp->csr_base + HFI_REG_FWUNCACHED_IOVA);
	writel(HFI_FWUNCACHED_SIZE, icp->csr_base + HFI_REG_FWUNCACHED_SIZE);

	/* Sec heap */
	writel(hfi->hfi_mem.secheap.dma_addr, icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR);
	writel(HFI_SECHEAP_SIZE, icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE);

	/* SFR buffer */
	writel(hfi->hfi_mem.sfr.dma_addr, icp->csr_base + HFI_REG_SFR_PTR);

	/* IO regions - use standard CAMX values */
	writel(0x10c00000, icp->csr_base + HFI_REG_IO_REGION_1_IOVA);
	writel(0xcf400000, icp->csr_base + HFI_REG_IO_REGION_1_SIZE);
	writel(0xe0800000, icp->csr_base + HFI_REG_IO_REGION_2_IOVA);
	writel(0x1e700000, icp->csr_base + HFI_REG_IO_REGION_2_SIZE);
}

/* HFI operations */
static const struct icp_hfi_ops hfi_ops = {
	.raise_irq = icp_raise_irq,
};

static int icp_boot(struct camss_icp *icp)
{
	const struct camss_icp_resources *icp_res = icp->icp_res;
	u32 hw_version, data;
	int ret;

	/* Power on all domains via runtime PM */
	ret = pm_runtime_resume_and_get(icp->dev);
	if (ret) {
		dev_err(icp->dev, "Failed to power on: %d\n", ret);
		return ret;
	}

	/* Enable clocks */
	ret = clk_bulk_prepare_enable(icp_res->clk_num, icp->clks);
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

	/* Dump CIRQ state for debugging */
	dev_info(icp->dev, "CIRQ config: MASK=0x%x STATUS=0x%x\n",
		 readl(icp->cirq_base + CIRQ_OB_MASK),
		 readl(icp->cirq_base + CIRQ_OB_STATUS));

	/* Load firmware */
	ret = icp_load_firmware(icp);
	if (ret)
		goto err_clk;

	/* Allocate HFI queues */
	ret = icp_hfi_init_queues(&icp->hfi);
	if (ret)
		goto err_clk;

	/* Start firmware via TrustZone */
	dev_dbg(icp->dev, "Starting ICP via TrustZone\n");
	ret = qcom_scm_pas_auth_and_reset(icp_res->pas_id);
	if (ret) {
		dev_err(icp->dev, "TZ auth_and_reset failed: %d\n", ret);
		goto err_hfi;
	}

	/* Wait for firmware to boot */
	usleep_range(5000, 51000);

	/* Program HFI pointers after bootup */
	icp_hfi_latch_regs(icp);

	/*
	 * Boot handshake with ICP firmware.
	 *
	 * After TZ starts the ICP, firmware waits for host to program
	 * GP registers with memory addresses and signal readiness.
	 *
	 * CAMX hfi_reg.h register assignments:
	 *   GP1 (0x24) = FW_VERSION        - firmware writes its version
	 *   GP2 (0x28) = HOST_ICP_INIT_REQ - host writes 1 ("addresses ready")
	 *   GP3 (0x2C) = ICP_HOST_INIT_RESP - firmware writes 1 ("ready")
	 *   GP4-GP18   = memory addresses   - host programs with IOVAs
	 *
	 * Sequence (from CAMX cam_hfi_init):
	 *   1. Program GP4-GP18 with memory addresses  [done above]
	 *   2. Write GP2 = 1 (HOST_ICP_INIT_REQUEST)
	 *   3. Poll GP3 until ICP_INIT_RESP_SUCCESS (1)
	 *   4. Read GP1 for firmware version
	 *
	 * Previous bug: We wrote 1 to GP2 then polled GP2, which succeeded
	 * instantly since we read back our own write. Firmware never ran.
	 */

	/* Signal firmware that addresses are ready */
	dev_dbg(icp->dev, "Signaling host init request (GP2=1)\n");
	writel(ICP_INIT_REQUEST_SET, icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ);

	/* Wait for firmware to signal ready via GP3 */
	dev_dbg(icp->dev, "Waiting for firmware ready (polling GP3)...\n");

	ret = readl_poll_timeout(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP,
				 data, data == ICP_INIT_RESP_SUCCESS,
				 HFI_POLL_DELAY_US, HFI_POLL_TIMEOUT_US);

	if (ret < 0) {
		dev_err(icp->dev, "Firmware ready timeout (GP3=0x%08x)\n",
			readl(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP));
		dev_err(icp->dev, "  GP1 (FW_VERSION)=0x%08x GP2 (INIT_REQ)=0x%08x\n",
			readl(icp->csr_base + HFI_REG_FW_VERSION),
			readl(icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ));
		icp_dump_gp_regs(icp, "FW READY TIMEOUT");
		icp_hfi_dump_sfr(&icp->hfi);
		ret = -ETIMEDOUT;
		goto err_hfi;
	}

	/* Read firmware version from GP1 */
	icp->hfi.fw_version = readl(icp->csr_base + HFI_REG_FW_VERSION);
	dev_info(icp->dev, "Firmware ready! version=0x%08x\n", icp->hfi.fw_version);

	/* Dump CIRQ state after FW init */
	dev_info(icp->dev, "CIRQ after FW init: MASK=0x%x STATUS=0x%x\n",
		 readl(icp->cirq_base + CIRQ_OB_MASK),
		 readl(icp->cirq_base + CIRQ_OB_STATUS));

	/* Dump GP registers to verify what firmware sees */
	dev_info(icp->dev, "GP registers after FW init:\n");
	dev_info(icp->dev, "  GP0 (unused):        0x%08x\n", readl(icp->csr_base + 0x20));
	dev_info(icp->dev, "  GP1 (FW_VERSION):    0x%08x\n", readl(icp->csr_base + HFI_REG_FW_VERSION));
	dev_info(icp->dev, "  GP2 (INIT_REQ):      0x%08x\n", readl(icp->csr_base + HFI_REG_HOST_ICP_INIT_REQ));
	dev_info(icp->dev, "  GP3 (INIT_RESP):     0x%08x\n", readl(icp->csr_base + HFI_REG_ICP_HOST_INIT_RESP));
	dev_info(icp->dev, "  GP4 (SHMEM_PTR):     0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SHARED_MEM_PTR), (u32)icp->hfi.hfi_mem.shmem.dma_addr);
	dev_info(icp->dev, "  GP5 (SHMEM_SIZE):    0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SHARED_MEM_SIZE), (u32)icp->hfi.hfi_mem.shmem.size);
	dev_info(icp->dev, "  GP6 (QTBL_PTR):      0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_QTBL_PTR), (u32)icp->hfi.hfi_mem.q_tbl.dma_addr);
	dev_info(icp->dev, "  GP7 (SECHEAP_PTR):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_PTR), (u32)icp->hfi.hfi_mem.secheap.dma_addr);
	dev_info(icp->dev, "  GP8 (SECHEAP_SIZE):  0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SECONDARY_HEAP_SIZE), HFI_SECHEAP_SIZE);
	dev_info(icp->dev, "  GP9 (STATUS):        0x%08x\n", readl(icp->csr_base + HFI_REG_RESERVED));
	dev_info(icp->dev, "  GP10 (SFR_PTR):      0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_SFR_PTR), (u32)icp->hfi.hfi_mem.sfr.dma_addr);
	dev_info(icp->dev, "  GP11 (QDSS_IOVA):    0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_QDSS_IOVA), (u32)icp->hfi.hfi_mem.qdss.dma_addr);
	dev_info(icp->dev, "  GP12 (QDSS_SIZE):    0x%08x\n", readl(icp->csr_base + HFI_REG_QDSS_IOVA_SIZE));
	dev_info(icp->dev, "  GP17 (FWUNCACHED):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_FWUNCACHED_IOVA), (u32)icp->hfi.hfi_mem.fwuncached.dma_addr);
	dev_info(icp->dev, "  GP18 (FWUNC_SIZE):   0x%08x (we wrote: 0x%08x)\n",
		 readl(icp->csr_base + HFI_REG_FWUNCACHED_SIZE), HFI_FWUNCACHED_SIZE);

	return 0;
err_hfi:
	icp_hfi_deinit_queues(&icp->hfi);
err_clk:
	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);
	clk_bulk_disable_unprepare(icp_res->clk_num, icp->clks);
	return ret;
}

static int camss_icp_probe(struct platform_device *pdev)
{
	struct dev_pm_domain_attach_data pd_data = { .pd_flags = PD_FLAG_DEV_LINK_ON };
	const struct camss_icp_resources *icp_res;
	struct camss_icp *icp;
	int ret, i;

	icp_res = of_device_get_match_data(&pdev->dev);
	if (!icp_res)
		return -EINVAL;

	icp = devm_kzalloc(&pdev->dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;

	icp->dev = &pdev->dev;
	icp->icp_res = icp_res;
	icp->hfi.res = &icp_res->hfi_res;
	icp->hfi.ops = &hfi_ops;
	icp->hfi.dev = icp->dev;
	icp->hfi.priv = icp;

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
	dev_dbg(&pdev->dev, "Attached %d power domains\n", ret > 0 ? ret : 1);

	/* Reserved memory for HFI */
	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 1);
	if (ret && ret != -ENODEV) {
		dev_err(&pdev->dev, "Failed to init reserved memory: %d\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "Using camera_icp_mem reserved memory\n");

	/* Clocks */
	for (i = 0; i < icp_res->clk_num; i++)
		icp->clks[i].id = icp_res->clk_names[i];

	ret = devm_clk_bulk_get(&pdev->dev, icp_res->clk_num, icp->clks);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get clocks: %d\n", ret);
		goto err_pd;
	}

	icp->icc_path = devm_of_icc_get(&pdev->dev, "mem");
	if (IS_ERR(icp->icc_path)) {
		if (PTR_ERR(icp->icc_path) != -ENODATA) {
			ret = PTR_ERR(icp->icc_path);
			goto err_pd;
		}
		icp->icc_path = NULL;
	}
dev_info(&pdev->dev, "0xB0D requesting irq %d\n", icp->irq);
	ret = devm_request_threaded_irq(&pdev->dev, icp->irq, camss_icp_isr,
					camss_icp_isr_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"camss-icp", icp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		goto err_pd;
	}

	pm_runtime_enable(&pdev->dev);

	ret = icp_boot(icp);
	if (ret)
		goto err_pd;

	ret = icp_hfi_core_init(&icp->hfi);
	if (ret)
		goto err_pd;

	return 0;

err_pd:
	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);
	return ret;
}

static void camss_icp_remove(struct platform_device *pdev)
{
	struct camss_icp *icp = platform_get_drvdata(pdev);
	const struct camss_icp_resources *icp_res;

	icp_res = icp->icp_res;
	qcom_scm_pas_shutdown(icp_res->pas_id);

	icp_hfi_deinit_queues(&icp->hfi);

	if (icp->icc_path)
		icc_set_bw(icp->icc_path, 0, 0);

	clk_bulk_disable_unprepare(icp_res->clk_num, icp->clks);

	if (icp->pd_list)
		dev_pm_domain_detach_list(icp->pd_list);
}

struct camss_icp_resources x1e80100_icp_res = {
	.pas_id = 33,
	.clk_names = { "ahb", "core", "debug_xo" },
	.clk_num = 3,
	.hfi_res = {
		.shmem_size = SZ_1M,	 // change to 0x0FC00000 per downstream 
		.qdss_size = SZ_1M,
		.fwuncached_size = 7 * SZ_1M,
		/*
		 * Carve sub-regions from FwUncached:
		 * Over-allocate as CamX does for now.
		 *   +0x000000: SecHeap (1MB)
		 *   +0x100000: QTBL (1MB)
		 *   +0x200000: CMD_Q (1MB)
		 *   +0x300000: MSG_Q (1MB)
		 *   +0x400000: DBG_Q (1MB)
		 *   +0x500000: SFR (1MB alloc 4KB used)
		 */
		.secheap_size = SZ_1M,
		.q_tbl_size = SZ_1M,
		.qdata_size[HFI_Q_CMD_TYPE] = SZ_1M,
		.qdata_size[HFI_Q_MSG_TYPE] = SZ_1M,
		.qdata_size[HFI_Q_DBG_TYPE] = SZ_1M,
		.sfr_size = SZ_1M,
	},
};

static const struct of_device_id camss_icp_of_match[] = {
	{ .compatible = "qcom,x1e80100-camss-icp", .data = &x1e80100_icp_res},
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
