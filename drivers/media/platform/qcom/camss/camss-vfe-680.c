// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-680.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v680
 *
 * Copyright (C) 2025 Linaro Ltd.
 */
#define DEBUG
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"

#define VFE_TOP_IRQn_STATUS(vfe, n)		((vfe_is_lite(vfe) ? 0x1c : 0x44) + (n) * 4)
#define VFE_TOP_IRQn_MASK(vfe, n)		((vfe_is_lite(vfe) ? 0x24 : 0x34) + (n) * 4)
#define VFE_TOP_IRQn_CLEAR(vfe, n)		((vfe_is_lite(vfe) ? 0x2c : 0x3c) + (n) * 4)
#define		VFE_IRQ1_SOF(vfe, n)		((vfe_is_lite(vfe) ? BIT(2) : BIT(8)) << ((n) * 2))
#define		VFE_IRQ1_EOF(vfe, n)		((vfe_is_lite(vfe) ? BIT(3) : BIT(9)) << ((n) * 2))
#define VFE_TOP_IRQ_CMD(vfe)			(vfe_is_lite(vfe) ? 0x38 : 0x30)
#define		VFE_TOP_IRQ_CMD_GLOBAL_CLEAR	BIT(0)
#define VFE_TOP_DIAG_CONFIG			(vfe_is_lite(vfe) ? 0x40 : 0x50)

#define VFE_TOP_DEBUG_11(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xcc)
#define VFE_TOP_DEBUG_12(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xd0)
#define VFE_TOP_DEBUG_13(vfe)			(vfe_is_lite(vfe) ? 0x40 : 0xd4)

#define VFE_BUS_IRQn_MASK(vfe, n)		((vfe_is_lite(vfe) ? 0x218 : 0xc18) + (n) * 4)
#define VFE_BUS_IRQn_CLEAR(vfe, n)		((vfe_is_lite(vfe) ? 0x220 : 0xc20) + (n) * 4)
#define VFE_BUS_IRQn_STATUS(vfe, n)		((vfe_is_lite(vfe) ? 0x228 : 0xc28) + (n) * 4)
#define VFE_BUS_IRQ_GLOBAL_CLEAR(vfe)		(vfe_is_lite(vfe) ? 0x230 : 0xc30)
#define VFE_BUS_WR_VIOLATION_STATUS(vfe)	(vfe_is_lite(vfe) ? 0x264 : 0xc64)
#define VFE_BUS_WR_OVERFLOW_STATUS(vfe)		(vfe_is_lite(vfe) ? 0x268 : 0xc68)
#define VFE_BUS_WR_IMAGE_VIOLATION_STATUS(vfe)	(vfe_is_lite(vfe) ? 0x270 : 0xc70)

#define VFE_BUS_WRITE_CLIENT_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x400 : 0xe00) + (c) * 0x100)
#define		VFE_BUS_WRITE_CLIENT_CFG_EN	BIT(0)
#define VFE_BUS_IMAGE_ADDR(vfe, c)		((vfe_is_lite(vfe) ? 0x404 : 0xe04) + (c) * 0x100)
#define VFE_BUS_FRAME_INCR(vfe, c)		((vfe_is_lite(vfe) ? 0x408 : 0xe08) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG0(vfe, c)		((vfe_is_lite(vfe) ? 0x40c : 0xe0c) + (c) * 0x100)
#define		VFE_BUS_IMAGE_CFG0_DATA(h, s)	(((h) << 16) | ((s) >> 4))
#define WM_IMAGE_CFG_0_DEFAULT_WIDTH		(0xFFFF)

#define VFE_BUS_IMAGE_CFG1(vfe, c)		((vfe_is_lite(vfe) ? 0x410 : 0xe10) + (c) * 0x100)
#define VFE_BUS_IMAGE_CFG2(vfe, c)		((vfe_is_lite(vfe) ? 0x414 : 0xe14) + (c) * 0x100)
#define VFE_BUS_PACKER_CFG(vfe, c)		((vfe_is_lite(vfe) ? 0x418 : 0xe18) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x430 : 0xe30) + (c) * 0x100)
#define VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x434 : 0xe34) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PERIOD(vfe, c)	((vfe_is_lite(vfe) ? 0x438 : 0xe38) + (c) * 0x100)
#define VFE_BUS_FRAMEDROP_PATTERN(vfe, c)	((vfe_is_lite(vfe) ? 0x43c : 0xe3c) + (c) * 0x100)
#define VFE_BUS_MMU_PREFETCH_CFG(vfe, c)	((vfe_is_lite(vfe) ? 0x460 : 0xe60) + (c) * 0x100)
#define		VFE_BUS_MMU_PREFETCH_CFG_EN	BIT(0)
#define VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, c)	((vfe_is_lite(vfe) ? 0x464 : 0xe64) + (c) * 0x100)
#define VFE_BUS_ADDR_STATUS0(vfe, c)		((vfe_is_lite(vfe) ? 0x470 : 0xe70) + (c) * 0x100)

#define VFE_BUS_BW_LIMIT(vfe, c)           ((vfe_is_lite(vfe) ? 0x41c : 0xe1c) + (c) * 0x100)
#define VFE_BUS_DEBUG_STATUS_CFG(vfe, c)   ((vfe_is_lite(vfe) ? 0x480 : 0xe80) + (c) * 0x100)

#define IPP_LINE				3
#define IS_IPP(line)				(vfe_is_lite(vfe) ? 0 : line->id == IPP_LINE)

/* IFE 680 IQ Module MODULE_CFG registers */
#define IFE_DEMUX_MODULE_CFG            0x3960
#define IFE_CHANNEL_GAIN_MODULE_CFG     0x3B60
#define IFE_BPC_PDPC_MODULE_CFG         0x3D60
#define IFE_BINCORRECT_MODULE_CFG       0x3F60
#define IFE_COMPDECOMP_MODULE_CFG       0x4160
#define IFE_LSC_MODULE_CFG              0x4360
#define IFE_WB_GAIN_MODULE_CFG          0x4560
#define IFE_GIC_MODULE_CFG              0x4760
#define IFE_BPC_ABF_MODULE_CFG          0x4960
#define IFE_BLS_MODULE_CFG              0x4B60
#define IFE_BAYER_GTM_MODULE_CFG        0x4D60
#define IFE_BAYER_LTM_MODULE_CFG        0x5260
#define IFE_LCAC_MODULE_CFG             0x5460
#define IFE_DEMOSAIC_MODULE_CFG         0x5660
#define IFE_COLOR_CORRECT_MODULE_CFG    0x5860
#define IFE_GTM_MODULE_CFG              0x5A60
#define IFE_GLUT_MODULE_CFG             0x5F60
#define IFE_COLOR_XFORM_MODULE_CFG      0x6160
#define IFE_UVG_MODULE_CFG              0x6360
#define IFE_PREPROCESSOR_MODULE_CFG     0x6560   /* ← YOU ARE MISSING THIS */

/////////////////////////////////// test /////////////////////////////
#include "camss.h"
#include "camss-vfe.h"

/* ================================================================
 * BHIST register offsets (from titan680_rt.h, base 0xB400)
 * ================================================================ */

#define IFE_BHIST_HW_VERSION			0xB400
#define IFE_BHIST_MODULE_CFG			0xB460
#define IFE_BHIST_BLACK_LEVEL_SUB		0xB464
#define IFE_BHIST_RGN_OFFSET_CFG		0xB468
#define IFE_BHIST_RGN_NUM_CFG			0xB46C
#define IFE_BHIST_SEGMENT_BASE_CFG(n)		(0xB470 + (n) * 4)  /* n=0..7 */
#define IFE_BHIST_SEGMENT_SLOPE_CFG(n)		(0xB490 + (n) * 4)  /* n=0..1 */
#define IFE_BHIST_SEGMENT_START_BIN_CFG(n)	(0xB498 + (n) * 4)  /* n=0..2 */
#define IFE_BHIST_Y_CONV_COEFF_CFG		0xB4A4

/* MODULE_CFG fields */
#define BHIST_MODULE_CFG_EN			BIT(0)
#define BHIST_MODULE_CFG_CHAN_SEL_MASK		GENMASK(18, 16)
#define BHIST_MODULE_CFG_CHAN_SEL_SHIFT		16

/* CHAN_SEL values */
#define BHIST_CHAN_SEL_ALL			0  /* R + Gr + Gb + B */
#define BHIST_CHAN_SEL_R			1
#define BHIST_CHAN_SEL_GR			2
#define BHIST_CHAN_SEL_GB			4
#define BHIST_CHAN_SEL_B			3
#define BHIST_CHAN_SEL_Y			5  /* Y from Y_CONV_COEFF */

/* RGN_OFFSET_CFG fields */
#define BHIST_RGN_H_OFFSET_MASK			GENMASK(13, 0)
#define BHIST_RGN_V_OFFSET_MASK			GENMASK(29, 16)
#define BHIST_RGN_V_OFFSET_SHIFT		16

/* RGN_NUM_CFG fields */
#define BHIST_RGN_H_NUM_MASK			GENMASK(12, 0)
#define BHIST_RGN_V_NUM_MASK			GENMASK(28, 16)
#define BHIST_RGN_V_NUM_SHIFT			16

/* WM index for BHIST (from cam_vfe680.h, dmesg2: WM:12 STATS_BHIST) */
#define WM_BHIST				12
#define BHIST_BUF_SIZE				(32 * 1024)

/* BHIST output layout: 4 channels × 1024 bins × 4 bytes = 16384 bytes */
#define BHIST_NUM_BINS				1024
#define BHIST_NUM_CHANNELS			4
#define BHIST_DATA_SIZE				(BHIST_NUM_BINS * BHIST_NUM_CHANNELS * sizeof(u32))

/* ================================================================
 * DMA buffer management
 * ================================================================ */

static int vfe_stats_buf_alloc(struct device *dev,
			       struct vfe_stats_buf *buf, u32 size)
{
	buf->vaddr = dma_alloc_coherent(dev, size, &buf->dma_addr, GFP_KERNEL);
	if (!buf->vaddr)
		return -ENOMEM;

	buf->size = size;
	memset(buf->vaddr, 0, size);
	return 0;
}

static void vfe_stats_buf_free(struct device *dev,
			       struct vfe_stats_buf *buf)
{
	if (buf->vaddr) {
		dma_free_coherent(dev, buf->size, buf->vaddr, buf->dma_addr);
		buf->vaddr = NULL;
		buf->dma_addr = 0;
		buf->size = 0;
	}
}

/* ================================================================
 * Stats WM configuration
 *
 * Frame-based mode, verified from dmesg2 trace:
 *   "WM:12 STATS_BHIST width:0 height:0 stride:1 format:0x13
 *    en_ubwc:0 frame-based"
 *
 * Uses the same VFE_BUS_* macros as the proven RDI WM code.
 * ================================================================ */

static void vfe_stats_wm_configure(struct vfe_device *vfe, u8 wm,
				   dma_addr_t addr, u32 buf_size)
{
	/* Frame-based encoding: CFG0=0, CFG1=0, CFG2=1 (stride=1) */
	writel(0, vfe->base + VFE_BUS_IMAGE_CFG0(vfe, wm));
	writel(0, vfe->base + VFE_BUS_IMAGE_CFG1(vfe, wm));
	writel(1, vfe->base + VFE_BUS_IMAGE_CFG2(vfe, wm));
	writel(0x0A, vfe->base + VFE_BUS_PACKER_CFG(vfe, wm));  /* pk_fmt=10 decimal */

	/* Total DMA transfer size */
	writel(buf_size, vfe->base + VFE_BUS_FRAME_INCR(vfe, wm));

	/* DMA target address */
	writel(addr, vfe->base + VFE_BUS_IMAGE_ADDR(vfe, wm));

	/* SMMU prefetch */
	writel(VFE_BUS_MMU_PREFETCH_CFG_EN,
	       vfe->base + VFE_BUS_MMU_PREFETCH_CFG(vfe, wm));
	writel(~0u, vfe->base + VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm));

	/* Every frame, no drops */
	writel(1, vfe->base + VFE_BUS_FRAMEDROP_PATTERN(vfe, wm));
	writel(0, vfe->base + VFE_BUS_FRAMEDROP_PERIOD(vfe, wm));
	writel(1, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm));
	writel(0, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm));

	/* Enable */
	writel(VFE_BUS_WRITE_CLIENT_CFG_EN | BIT(16),
	       vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, wm));

	dev_dbg(vfe->camss->dev,
		"VFE%d: WM:%d stats addr=0x%pad size=%u frame-based\n",
		vfe->id, wm, &addr, buf_size);
}

static void vfe_stats_wm_disable(struct vfe_device *vfe, u8 wm)
{
	writel(0, vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, wm));
}

/* ================================================================
 * BHIST engine configuration
 *
 * Programs the BHIST stats hardware for full-frame, uniform
 * binning across all four Bayer channels.
 *
 * Uniform binning: all segment bases = 0, slopes = identity,
 * start bins evenly spaced. This maps the full sensor range
 * linearly into 1024 bins.
 * ================================================================ */

static void vfe_bhist_configure(struct vfe_device *vfe, struct vfe680_stats *stats,
				u32 width, u32 height)
{
	u32 h_num = (width / 2) - 1;
	u32 v_num = (height / 2) - 1;
	u32 h_offset = 0;
	u32 v_offset = 0;
	int i;

	/* Disable while configuring */
	writel(0, vfe->base + IFE_BHIST_MODULE_CFG);

	/* No black level subtraction */
	writel(0, vfe->base + IFE_BHIST_BLACK_LEVEL_SUB);

	/*
	 * Region count: for BHIST, h_num × v_num defines how many
	 * sub-regions the frame is divided into. The histogram is
	 * accumulated across all regions. Using 1×1 means the entire
	 * frame is one region (simplest configuration).
	 */
#if 0
	writel(((height - 1) << BHIST_RGN_V_NUM_SHIFT) | (width - 1),
	       vfe->base + IFE_BHIST_RGN_NUM_CFG);
#else

#endif
#if 0

	/* ROI: full frame, offset = 0,0 TODO: center this */
	writel(0, vfe->base + IFE_BHIST_RGN_OFFSET_CFG);

	/* Full frame in 2×2 region units TODO: constrain to smaller ROI */
	writel((v_num << 16) | h_num,
		vfe->base + IFE_BHIST_RGN_NUM_CFG);
#else
	#define BHIST_ROI_W   512
	#define BHIST_ROI_H   512

	h_num = (BHIST_ROI_W / 2) - 1;   /* 255 */
	v_num = (BHIST_ROI_H / 2) - 1;   /* 255 */
	h_offset = (width - BHIST_ROI_W) / 2;
	v_offset = (height - BHIST_ROI_H) / 2;

	writel((v_offset << 16) | h_offset, vfe->base + IFE_BHIST_RGN_OFFSET_CFG);
	writel((v_num << 16) | h_num, vfe->base + IFE_BHIST_RGN_NUM_CFG);

	stats->bhist_rgn_h_num = h_num;
	stats->bhist_rgn_v_num = v_num;
#endif
	/*
	 * Uniform binning: segments map input values linearly to bins.
	 * For a 14-bit pipeline (16384 values) into 1024 bins,
	 * each bin covers 16 values.
	 *
	 * Segment bases: all zero (linear from origin)
	 * Segment slopes: identity mapping
	 * Start bins: evenly distributed
	 *
	 * For initial bring-up, zero all segments for simplest mapping.
	 */
	for (i = 0; i < 8; i++)
		writel(0, vfe->base + IFE_BHIST_SEGMENT_BASE_CFG(i));
	for (i = 0; i < 2; i++)
		writel(0, vfe->base + IFE_BHIST_SEGMENT_SLOPE_CFG(i));
	for (i = 0; i < 3; i++)
		writel(0, vfe->base + IFE_BHIST_SEGMENT_START_BIN_CFG(i));

dev_info(vfe->camss->dev, "%s/%d ok\n", __func__, __LINE__);

	/*
	 * Y conversion coefficients (for CHAN_SEL=Y mode).
	 * Not used with CHAN_SEL=ALL but program sane defaults.
	 * a0=77 (R), a1=150 (G), a2=29 (B) ≈ BT.601 luma
	 */
	writel((77 << 0) | (150 << 9) | (29 << 18),
	       vfe->base + IFE_BHIST_Y_CONV_COEFF_CFG);
dev_info(vfe->camss->dev, "%s/%d ok\n", __func__, __LINE__);

	/* Enable: all four Bayer channels */

	writel(BHIST_MODULE_CFG_EN |
	       (BHIST_CHAN_SEL_ALL << BHIST_MODULE_CFG_CHAN_SEL_SHIFT),
	       vfe->base + IFE_BHIST_MODULE_CFG);

	dev_info(vfe->camss->dev,
		 "BHIST: full frame %ux%u, h_num=%u v_num=%u\n",
		 width, height, h_num, v_num);
}

static void vfe_bhist_disable(struct vfe_device *vfe)
{
	writel(0, vfe->base + IFE_BHIST_MODULE_CFG);
}

/* ================================================================
 * BHIST output validation
 *
 * Reads the DMA buffer and validates the histogram data.
 * For a correct capture:
 *   sum(R bins) + sum(Gr bins) + sum(Gb bins) + sum(B bins)
 *     == width × height (total pixel count)
 *
 * Each channel should have width/2 × height/2 pixels for RGGB.
 * ================================================================ */

void vfe_bhist_validate(struct vfe_device *vfe, struct vfe680_stats *stats, u32 width, u32 height)
{
	const u32 *data = stats->bhist.vaddr;
	u64 channel_sums[BHIST_NUM_CHANNELS] = {};
	u64 expected_per_channel;
	u32 peak_bin[BHIST_NUM_CHANNELS] = {};
	u32 peak_val[BHIST_NUM_CHANNELS] = {};
	const char *ch_names[] = { "R", "Gr", "Gb", "B" };
	int ch, bin;
	bool valid = true;

	if (!data)
		return;

	expected_per_channel = (u64)(width / 2) * (height / 2);

	for (ch = 0; ch < BHIST_NUM_CHANNELS; ch++) {
		const u32 *ch_data = &data[ch * BHIST_NUM_BINS];

		for (bin = 0; bin < BHIST_NUM_BINS; bin++) {
			u32 count = ch_data[bin];

			channel_sums[ch] += count;

			if (count > peak_val[ch]) {
				peak_val[ch] = count;
				peak_bin[ch] = bin;
			}
		}
	}

	dev_info(vfe->camss->dev, "=== BHIST frame %u ===\n", stats->frame_count);

	for (ch = 0; ch < BHIST_NUM_CHANNELS; ch++) {
		dev_info(vfe->camss->dev,
			 "  %2s: sum=%llu expected=%llu peak_bin=%u peak_count=%u\n",
			 ch_names[ch], channel_sums[ch], expected_per_channel,
			 peak_bin[ch], peak_val[ch]);

		if (channel_sums[ch] == 0)
			valid = false;
	}

	if (!valid) {
		dev_warn(vfe->camss->dev,
			 "BHIST: zero bin sums - stats engine not producing data\n");

		int i;
		/* Dump first 64 words (256 bytes) */
		for (i = 0; i < 64; i += 8) {
			dev_info(vfe->camss->dev,
				 "  raw[%3d..%3d]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				 i, i + 7,
				 data[i+0], data[i+1], data[i+2], data[i+3],
				 data[i+4], data[i+5], data[i+6], data[i+7]);
		}
		/* Also scan for any non-zero data in the full buffer */
		int nz = 0;
		for (i = 0; i < BHIST_BUF_SIZE / 4; i++)
			if (data[i]) nz++;
		dev_info(vfe->camss->dev, "  non-zero words: %d / %d\n",
			 nz, BHIST_BUF_SIZE / 4);

		/* Check BHIST MODULE_CFG readback */
		dev_info(vfe->camss->dev, "  BHIST MODULE_CFG: 0x%08x\n",
			 readl(vfe->base + IFE_BHIST_MODULE_CFG));
	} else {
		u64 total = channel_sums[0] + channel_sums[1] +
			    channel_sums[2] + channel_sums[3];
		u64 expected_total = (u64)width * height;

		dev_info(vfe->camss->dev,
			 "  total=%llu expected=%llu %s\n",
			 total, expected_total,
			 (total == expected_total) ? "MATCH" : "MISMATCH");
	}
}

/* ================================================================
 * Lifecycle: init / start / stop / cleanup
 * ================================================================ */

int vfe_stats_init(struct vfe_device *vfe)
{
	return 0;
}

int vfe_stats_start(struct vfe_device *vfe, u32 width, u32 height)
{
	int ret;

	/* Allocate BHIST DMA buffer */
	ret = vfe_stats_buf_alloc(vfe->camss->dev, &vfe->stats.bhist, BHIST_BUF_SIZE);
	if (ret) {
		dev_err(vfe->camss->dev, "Failed to allocate BHIST buffer\n");
		return ret;
	}

	/* === Open the VFE pixel pipeline === */

	/* 1. Override ALL clock gates — brute force for bring-up */
	#define CGC0_BUS_WR         BIT(0)
	#define CGC0_DEMUX          BIT(10)

	/* Override all clocks */
	writel(0, vfe->base + 0x18);
	writel(0, vfe->base + 0x1c);	/* CORE_CLK_CGC_CTRL_1 */
	writel(0, vfe->base + 0x20);		/* AHB_CLK_CGC_CTRL */
	writel(0, vfe->base + 0xc08);  /* BUS_WR INPUT_IF_CGC_OVERRIDE — all 28 WMs */

	/* Disable/bypass ALL IQ modules — prevent violations from unconfigured modules */
	writel(0, vfe->base + 0x3B60);  /* CHANNEL_GAIN */
	writel(0, vfe->base + 0x3D60);  /* BPC_PDPC */
	writel(0, vfe->base + 0x3F60);  /* BINCORRECT */
	writel(0, vfe->base + 0x4160);  /* COMPDECOMP */
	writel(0, vfe->base + 0x4360);  /* LSC */
	writel(0, vfe->base + 0x4560);  /* WB_GAIN */
	writel(0, vfe->base + 0x4760);  /* GIC */
	writel(0, vfe->base + 0x4960);  /* BPC_ABF */
	writel(0, vfe->base + 0x4B60);  /* BLS */
	writel(0, vfe->base + 0x4D60);  /* BAYER_GTM */
	writel(0, vfe->base + 0x5260);  /* BAYER_LTM */
	writel(0, vfe->base + 0x5460);  /* LCAC */
	writel(1, vfe->base + 0x5660);  /* DEMOSAIC */
	writel(0, vfe->base + 0x5860);  /* COLOR_CORRECT */
	writel(0, vfe->base + 0x5A60);  /* GTM */
	writel(0, vfe->base + 0x5F60);  /* GLUT */
	writel(0, vfe->base + 0x6160);  /* COLOR_XFORM */
	writel(0, vfe->base + 0x6360);  /* UVG */
	writel(0, vfe->base + IFE_PREPROCESSOR_MODULE_CFG);	/* Preprocessor disable */

	/* 2. CORE_CFG_0: PP_INPUT_FMT = BAYER (bits [3:2] = 0) */
	writel(0x0, vfe->base + 0x24);
	writel(0x10, vfe->base + 0x28);  /* CORE_CFG_1: PIXEL_RAW_FMT=PLAIN16 */

	/* 3. DEMUX — first module in pipeline, splits Bayer into channels
	 *
	 * MODULE_CFG: EN=1, PERIOD=1 (2-pixel Bayer repeat)
	 * EVEN_LINE_CFG: for SGRBG → Gr(ch1),R(ch0) repeated
	 * ODD_LINE_CFG:  for SGRBG → B(ch2),Gb(ch3) repeated
	 *
	 * Nibble encoding: each 4-bit nibble selects output channel
	 * for that pixel position within the period.
	 */
	writel(0, vfe->base + 0x3960);  /* EN + PERIOD=1 */

	dev_info(vfe->camss->dev, "VFE%d: CGC override + CORE_CFG + DEMUX enabled\n",
		 vfe->id);

	dev_info(vfe->camss->dev, "VFE%d DEBUG:\n"
		"  CGC_CTRL_0=0x%08x CGC_CTRL_1=0x%08x AHB_CGC=0x%08x\n"
		"  CORE_CFG_0=0x%08x\n"
		"  DEMUX_MODULE_CFG=0x%08x\n"
		"  DEMUX_EVEN=0x%08x DEMUX_ODD=0x%08x\n"
		"  TOP_IRQ_STATUS_0=0x%08x TOP_IRQ_STATUS_1=0x%08x\n",
		vfe->id,
		readl(vfe->base + 0x18),
		readl(vfe->base + 0x1c),
		readl(vfe->base + 0x20),
		readl(vfe->base + 0x24),
		readl(vfe->base + 0x3960),
		readl(vfe->base + 0x3968),
		readl(vfe->base + 0x396c),
		readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 0)),
		readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 1)));

	/* 0x3968: EVEN_LINE_CFG — Bayer pattern for even rows */
	/* 0x396C: ODD_LINE_CFG — Bayer pattern for odd rows */
	struct csid_device *csid = &vfe->camss->csid[0];

	dev_info(vfe->camss->dev, "CSID0 DEBUG:\n"
		"  en_port=0x%x\n"
		"  IPP_CFG0=0x%08x (expect bit31=EN, [15:12]=decode_fmt)\n"
		"  IPP_CTRL=0x%08x\n"
		"  IPP_CFG1=0x%08x\n"
		"  IPP_CAMIF_FRAME_CFG=0x%08x\n"
		"  TOP_IRQ_STATUS=0x%08x\n"
		"  RX_IRQ_STATUS=0x%08x\n",
		csid->phy.en_port,
		readl(csid->base + 0x300),
		readl(csid->base + 0x304),
		readl(csid->base + 0x310),
		readl(csid->base + 0x330),
		readl(csid->base + 0x07C),
		readl(csid->base + 0x09C));

	writel(0x0F, vfe->base + VFE_BUS_BW_LIMIT(vfe, 12));
	writel(0xB00, vfe->base + VFE_BUS_DEBUG_STATUS_CFG(vfe, 12));

	/* Configure BHIST stats engine */
	vfe_bhist_configure(vfe, &vfe->stats, width, height);

	/* Configure WM 12 to write BHIST output to our DMA buffer */
	vfe_stats_wm_configure(vfe, WM_BHIST, vfe->stats.bhist.dma_addr, BHIST_BUF_SIZE);

	/* Enable BUS WR error IRQs (downstream: 0xd0000000) */
	writel(0xd0000000, vfe->base + VFE_BUS_IRQn_MASK(vfe, 0));
	writel(0, vfe->base + VFE_BUS_IRQn_MASK(vfe, 1));

	/* Arm the bus write engine */
	writel(1, vfe->base + VFE_BUS_IRQ_GLOBAL_CLEAR(vfe));

/* CROP_RND_CLAMP_PIXEL_RAW_OUT — defines frame boundary for pipeline */
writel(BIT(0), vfe->base + 0x7660);              /* MODULE_CFG: EN */
writel((height - 1), vfe->base + 0x7668);  /* CROP_LINE_CFG: last line */
writel((width - 1), vfe->base + 0x766c);   /* CROP_PIXEL_CFG: last pixel */

	vfe->stats.frame_count = 0;

	writel(0xFFFFFFFF, vfe->base + VFE_TOP_IRQn_MASK(vfe, 0));
	writel(0xFFFFFFFF, vfe->base + VFE_TOP_IRQn_MASK(vfe, 1));

	dev_info(vfe->camss->dev,
		 "VFE%d: stats started (BHIST WM:%d addr=0x%08x)\n",
		 vfe->id, WM_BHIST, vfe->stats.bhist.dma_addr);
	return 0;
}

void vfe_stats_stop(struct vfe_device *vfe, u32 width, u32 height)
{
	struct csid_device *csid = &vfe->camss->csid[0];

	dev_info(vfe->camss->dev, "CSID0 DEBUG:\n"
		"  en_port=0x%x\n"
		"  IPP_CFG0=0x%08x (expect bit31=EN, [15:12]=decode_fmt)\n"
		"  IPP_CTRL=0x%08x\n"
		"  IPP_CFG1=0x%08x\n"
		"  IPP_CAMIF_FRAME_CFG=0x%08x\n"
		"  TOP_IRQ_STATUS=0x%08x\n"
		"  RX_IRQ_STATUS=0x%08x\n",
		csid->phy.en_port,
		readl(csid->base + 0x300),
		readl(csid->base + 0x304),
		readl(csid->base + 0x310),
		readl(csid->base + 0x330),
		readl(csid->base + 0x07C),
		readl(csid->base + 0x09C));

	/* In vfe_stats_stop, before disabling: */
	dev_info(vfe->camss->dev, "VFE%d STOP DEBUG:\n"
		"  TOP_IRQ_STATUS_0=0x%08x\n"
		"  TOP_IRQ_STATUS_1=0x%08x\n"
		"  DIAG_SENSOR_STATUS_0=0x%08x\n"
		"  DIAG_SENSOR_STATUS_1=0x%08x\n"
		"  VIOLATION_STATUS=0x%08x\n",
		vfe->id,
		readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 0)),
		readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 1)),
		readl(vfe->base + 0x54),  /* DIAG_SENSOR_STATUS_0 */
		readl(vfe->base + 0x58),  /* DIAG_SENSOR_STATUS_1 */
		readl(vfe->base + 0x64)); /* VIOLATION_STATUS */

	/* Debug: read WM status before stopping */
	dev_info(vfe->camss->dev,
		 "VFE%d: WM:%d ADDR_STATUS0=0x%08x CLIENT_CFG=0x%08x\n",
		 vfe->id, WM_BHIST,
		 readl(vfe->base + VFE_BUS_ADDR_STATUS0(vfe, WM_BHIST)),
		 readl(vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, WM_BHIST)));

	/* Validate what we captured */
	vfe_bhist_validate(vfe, &vfe->stats, width, height);

	/* Disable WM and engine */
	vfe_stats_wm_disable(vfe, WM_BHIST);
	vfe_bhist_disable(vfe);

	/* Free DMA buffer */
	vfe_stats_buf_free(vfe->camss->dev, &vfe->stats.bhist);

	dev_info(vfe->camss->dev, "VFE%d: stats stopped\n", vfe->id);
}

/////////////////////////////////// test /////////////////////////////
/*
 * TODO: differentiate the port id based on requested type of RDI, BHIST etc
 *
 * IFE write master IDs
 */

typedef enum {
	VFE_WM_VIDEO_FULL_Y = 0,
	VFE_WM_VIDEO_FULL_C,
	VFE_WM_VIDEO_DS_4,
	VFE_WM_VIDEO_DS_16,
	VFE_WM_DISPLAY_FULL_Y,
	VFE_WM_DISPLAY_FULL_C,
	VFE_WM_DISPLAY_DS_4,
	VFE_WM_DISPLAY_DS_16,
	VFE_WM_FD_Y,
	VFE_WM_FD_C,
	VFE_WM_PIXEL_RAW,
	VFE_WM_STATS_BE0,
	VFE_WM_STATS_BHIST0,
	VFE_WM_STATS_TINTLESS_BG,
	VFE_WM_STATS_AWB_BG,
	VFE_WM_STATS_AWB_BFW,
	VFE_WM_STATS_BAF,
	VFE_WM_STATS_BHIST,
	VFE_WM_STATS_RS,
	VFE_WM_STATS_IHIST,
	VFE_WM_SPARSE_PD,
	VFE_WM_PDAF_V2_0_PD_DATA,
	VFE_WM_PDAF_V2_0_SAD,
	VFE_WM_LCR,
	VFE_WM_RDI0,
	VFE_WM_RDI1,
	VFE_WM_RDI2,
	VFE_WM_LTM_STATS
}vfe_wm_t;

/* TODO:
 * IFE Lite write master IDs
 *
 * RDI0			0
 * RDI1			1
 * RDI2			2
 * RDI3			3
 * GAMMA		4
 * BE			5
 */

/* TODO: assign an ENUM in resources and use the provided master
 *       id directly for RDI, STATS, AWB_BG, BHIST.
 *       This macro only works because RDI is all we support right now.
 */
#define RDI_WM(n)			((vfe_is_lite(vfe) ? 0 : 24) + (n))

static void vfe_global_reset(struct vfe_device *vfe)
{
	/* VFE680 has no global reset, simply report a completion */
	complete(&vfe->reset_complete);
}

/*
 * vfe_isr - VFE module interrupt handler
 * @irq: Interrupt line
 * @dev: VFE device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 status0, status1, bus0, bus1;

	dev_info(vfe->camss->dev, "%s running !\n", __func__);

	/* Read and clear TOP IRQs */
	status0 = readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 0));
	status1 = readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 1));
	writel(status0, vfe->base + VFE_TOP_IRQn_CLEAR(vfe, 0));
	writel(status1, vfe->base + VFE_TOP_IRQn_CLEAR(vfe, 1));
	writel(VFE_TOP_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_TOP_IRQ_CMD(vfe));

	/* Read and clear BUS WR IRQs */
	bus0 = readl(vfe->base + VFE_BUS_IRQn_STATUS(vfe, 0));
	bus1 = readl(vfe->base + VFE_BUS_IRQn_STATUS(vfe, 1));
	writel(bus0, vfe->base + VFE_BUS_IRQn_CLEAR(vfe, 0));
	writel(bus1, vfe->base + VFE_BUS_IRQn_CLEAR(vfe, 1));
	writel(1, vfe->base + VFE_BUS_IRQ_GLOBAL_CLEAR(vfe));

	return IRQ_HANDLED;
}

/*
 * vfe_halt - Trigger halt on VFE module and wait to complete
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

static void vfe_disable_irq(struct vfe_device *vfe)
{
	writel(0u, vfe->base + VFE_TOP_IRQn_MASK(vfe, 0));
	writel(0u, vfe->base + VFE_TOP_IRQn_MASK(vfe, 1));
	writel(0u, vfe->base + VFE_BUS_IRQn_MASK(vfe, 0));
	writel(0u, vfe->base + VFE_BUS_IRQn_MASK(vfe, 1));
}

static void vfe_wm_update(struct vfe_device *vfe, u8 rdi, u32 addr,
			  struct vfe_line *line)
{
	u8 wm = RDI_WM(rdi);

	if (IS_IPP(line))
		wm = VFE_WM_PIXEL_RAW;

	dev_info(vfe->camss->dev, "VFE:%d updating WM with RDI id %d\n", line->id, wm);

	writel(addr, vfe->base + VFE_BUS_IMAGE_ADDR(vfe, wm));

	dev_info(vfe->camss->dev, "VFE:%d WM:%d addr=0x%08x readback=0x%08x\n",
		 line->id, wm, addr, readl(vfe->base + VFE_BUS_IMAGE_ADDR(vfe, wm)));

}

static void __vfe_wm_start_raw(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u32 cfg;

	cfg = VFE_BUS_IMAGE_CFG0_DATA(pix->height, stride);

	writel(cfg, vfe->base + VFE_BUS_IMAGE_CFG0(vfe, wm));
	writel(0, vfe->base + VFE_BUS_IMAGE_CFG1(vfe, wm));
	writel(stride, vfe->base + VFE_BUS_IMAGE_CFG2(vfe, wm));
	writel(0, vfe->base + VFE_BUS_PACKER_CFG(vfe, wm));

	/* Set total frame increment value */
	writel(pix->plane_fmt[0].bytesperline * pix->height,
	       vfe->base + VFE_BUS_FRAME_INCR(vfe, wm));

	/* MMU */
	writel(VFE_BUS_MMU_PREFETCH_CFG_EN, vfe->base + VFE_BUS_MMU_PREFETCH_CFG(vfe, wm));
	writel(~0u, vfe->base + VFE_BUS_MMU_PREFETCH_MAX_OFFSET(vfe, wm));

	/* no dropped frames, one irq per frame */
	writel(1, vfe->base + VFE_BUS_FRAMEDROP_PATTERN(vfe, wm));
	writel(0, vfe->base + VFE_BUS_FRAMEDROP_PERIOD(vfe, wm));
	writel(1, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PATTERN(vfe, wm));
	writel(0, vfe->base + VFE_BUS_IRQ_SUBSAMPLE_PERIOD(vfe, wm));

	/* We don't process IRQs for VFE in RDI mode at the moment */
	vfe_disable_irq(vfe);

	/* Enable WM */
	writel(VFE_BUS_WRITE_CLIENT_CFG_EN,
	       vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, wm));

	dev_dbg(vfe->camss->dev, "RAW%d WM:%d width %d height %d stride %d\n",
		line->id, wm, pix->width, pix->height, stride);
}

static void __vfe_wm_start_pix(struct vfe_device *vfe, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	int ret;

    /* === Pixel pipeline init === */
    writel(0, vfe->base + 0x18);   /* CGC_CTRL_0 */
    writel(0, vfe->base + 0x1c);   /* CGC_CTRL_1 */
    writel(0, vfe->base + 0x20);          /* AHB_CGC */
    writel(0, vfe->base + 0xc08);  /* BUS_WR CGC */

    writel(0x0, vfe->base + 0x24);          /* CORE_CFG_0: BAYER */

    /* Disable all IQ modules */
    writel(0, vfe->base + 0x3960);
    writel(0, vfe->base + 0x3B60);
    writel(0, vfe->base + 0x3D60);
    writel(0, vfe->base + 0x3F60);
    writel(0, vfe->base + 0x4160);
    writel(0, vfe->base + 0x4360);
    writel(0, vfe->base + 0x4560);
    writel(0, vfe->base + 0x4760);
    writel(0, vfe->base + 0x4960);
    writel(0, vfe->base + 0x4B60);
    writel(0, vfe->base + 0x4D60);
    writel(0, vfe->base + 0x5260);
    writel(0, vfe->base + 0x5460);
    writel(0, vfe->base + 0x5660);
    writel(0, vfe->base + 0x5860);
    writel(0, vfe->base + 0x5A60);
    writel(0, vfe->base + 0x5F60);
    writel(0, vfe->base + 0x6160);
    writel(0, vfe->base + 0x6360);
    writel(0, vfe->base + 0x6560);

    /* CROP_RND_CLAMP_PIXEL_RAW_OUT — frame boundary */
    writel(BIT(0), vfe->base + 0x7660);
    writel(pix->height - 1, vfe->base + 0x7668);
    writel(pix->width - 1, vfe->base + 0x766c);

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_SGRBG10P:
	case V4L2_PIX_FMT_SRGGB10P:
	case V4L2_PIX_FMT_SBGGR10P:
	case V4L2_PIX_FMT_SGBRG10P:
		__vfe_wm_start_raw(vfe, VFE_WM_PIXEL_RAW, line);
		break;
	case V4L2_PIX_FMT_NV12:
		//ret = __vfe_wm_start_pix_nv12(vfe, line);
		break;
	}
#if 0
	ret = vfe_stats_start(vfe, pix->width, pix->height);

	dev_dbg(vfe->camss->dev, "PIX WM:%d width %d height %d stride %d\n",
		12, pix->width, pix->height, stride);
#endif
}

static void vfe_wm_start(struct vfe_device *vfe, u8 rdi, struct vfe_line *line)
{
	if (IS_IPP(line))
		__vfe_wm_start_pix(vfe, line);
	else
		__vfe_wm_start_raw(vfe, RDI_WM(rdi), line);
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 rdi, struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;

	u8 wm = RDI_WM(rdi);
	if (IS_IPP(line))
		wm = VFE_WM_PIXEL_RAW;

dev_info(vfe->camss->dev, "VFE0 PIX_RAW DEBUG:\n"
    "  WM10 ADDR_STATUS0=0x%08x CLIENT_CFG=0x%08x\n"
    "  BUS_WR VIOLATION=0x%08x OVERFLOW=0x%08x\n"
    "  TOP_IRQ_STATUS_1=0x%08x\n"
    "  CROP_RAW CFG=0x%08x LINE=0x%08x PIXEL=0x%08x\n",
    readl(vfe->base + VFE_BUS_ADDR_STATUS0(vfe, 10)),
    readl(vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, 10)),
    readl(vfe->base + 0xc64),
    readl(vfe->base + 0xc68),
    readl(vfe->base + VFE_TOP_IRQn_STATUS(vfe, 1)),
    readl(vfe->base + 0x7660),
    readl(vfe->base + 0x7668),
    readl(vfe->base + 0x766c));

	vfe_stats_stop(vfe, pix->width, pix->height);
	writel(0, vfe->base + VFE_BUS_WRITE_CLIENT_CFG(vfe, wm));

}

static const struct camss_video_ops vfe_video_ops_680 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_680;
	vfe_stats_init(vfe);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, false);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	int port_id = line_id;

	camss_reg_update(vfe->camss, vfe->id, port_id, true);
}

const struct vfe_hw_ops vfe_ops_680 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_halt = vfe_halt,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done,
	.vfe_wm_update = vfe_wm_update,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
};
