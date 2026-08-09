// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-480.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v480 (SM8250)
 *
 * Copyright (C) 2020-2021 Linaro Ltd.
 * Copyright (C) 2021 Jonathan Marek
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-480.h"

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
	VFW_WM_CAMIF_PD,
	VFE_WM_STATS_HDR_BE,
	VFE_WM_STATS_BHIST0,
	VFE_WM_STATS_TINTLESS_BG,
	VFE_WM_STATS_AWB_BG,
	VFE_WM_STATS_BHIST,
	VFE_WM_STATS_RS,
	VFE_WM_STATS_CS,
	VFE_WM_STATS_IHIST,
	VFE_WM_STATS_AWB_BF,
	VFE_WM_STATS_PDAF_V2,
	VFE_WM_LCR,
	VFE_WM_RDI0,
	VFE_WM_RDI1,
	VFE_WM_RDI2,
}vfe_wm_t;

static u32 pix_packer_fmt(u32 v4l2_fmt)
{
return 0x03;
	switch (v4l2_fmt) {
	case V4L2_PIX_FMT_SRGGB10P:
	case V4L2_PIX_FMT_SGRBG10P:
	case V4L2_PIX_FMT_SGBRG10P:
	case V4L2_PIX_FMT_SBGGR10P:
		return VFE_BUS_WM_PACKER_FMT_MIPI10;
	case V4L2_PIX_FMT_SRGGB12P:
	case V4L2_PIX_FMT_SGRBG12P:
	case V4L2_PIX_FMT_SGBRG12P:
	case V4L2_PIX_FMT_SBGGR12P:
		return VFE_BUS_WM_PACKER_FMT_MIPI12;
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
		return VFE_BUS_WM_PACKER_FMT_PLAIN_16_10BPP;
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
		return VFE_BUS_WM_PACKER_FMT_PLAIN_16_12BPP;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		return VFE_BUS_WM_PACKER_FMT_PLAIN_8;
	default:
		return 0x0;
	}
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	writel_relaxed(IRQ_MASK_0_RESET_ACK, vfe->base + VFE_IRQ_MASK(0));
	writel_relaxed(GLOBAL_RESET_HW_AND_REG, vfe->base + VFE_GLOBAL_RESET_CMD);
}

static void __vfe_wm_start(struct vfe_device *vfe, struct vfe_output *output,
			   u8 bus_client, unsigned int plane)
{
	struct v4l2_pix_format_mplane *pix =
		&output->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[plane].bytesperline;
	u32 height = pix->height;
	u32 mode;

	/* Chroma plane of a 4:2:0 semiplanar buffer is half height */
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		stride = pix->plane_fmt[0].bytesperline;
		if (plane > 0)
			height /= 2;
		break;
	default:
		break;
	}

	/* no clock gating at bus input */
	writel_relaxed(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel_relaxed(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

	if (output->line->is_pix) { /* PIX - line based */
		writel_relaxed(height << 16 | pix->width,
			       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(bus_client));
		dev_info(vfe->camss->dev, "VFE_BUS_WM_IMAGE_CFG_0(%d) @ VFE_BUS_WM_IMAGE_CFG_0(bus_client)/0x%08x == 0x%08x\n", bus_client, VFE_BUS_WM_IMAGE_CFG_0(bus_client),  height << 16 | pix->width);

		writel_relaxed(0, vfe->base + VFE_BUS_WM_IMAGE_CFG_1(bus_client));
		dev_info(vfe->camss->dev, "VFE_BUS_WM_IMAGE_CFG_1(%d) @ VFE_BUS_WM_IMAGE_CFG_1(bus_client)/0x%08x == 0x%08x\n", bus_client, VFE_BUS_WM_IMAGE_CFG_1(bus_client),  0);

		writel_relaxed(stride,
			       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(bus_client));
		dev_info(vfe->camss->dev, "VFE_BUS_WM_IMAGE_CFG_2(%d) @ VFE_BUS_WM_IMAGE_CFG_2(bus_client)/0x%08x == 0x%08x\n", bus_client, VFE_BUS_WM_IMAGE_CFG_2(bus_client),  stride);
		
		writel_relaxed(stride * height,
			       vfe->base + VFE_BUS_WM_FRAME_INCR(bus_client));
		dev_info(vfe->camss->dev, "VFE_BUS_WM_IMAGE_INCR(%d) @ VFE_BUS_WM_IMAGE_INCR(bus_client)/0x%08x == 0x%08x\n", bus_client, VFE_BUS_WM_FRAME_INCR(bus_client), stride * height );
		
		writel_relaxed(pix_packer_fmt(pix->pixelformat),
			       vfe->base + VFE_BUS_WM_PACKER_CFG(bus_client));
		dev_info(vfe->camss->dev, "VFE_BUS_WM_PACKER_CFG(%d) @ VFE_BUS_WM_PACKER_CFG(bus_client)/0x%08x == 0x%08x\n", bus_client, VFE_BUS_WM_PACKER_CFG(bus_client),  pix_packer_fmt(pix->pixelformat));

		mode = 1 << WM_CFG_EN;

	} else { /* RDI - frame based */
		writel_relaxed(WM_IMAGE_CFG_0_DEFAULT_WIDTH,
			       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(bus_client));
		writel_relaxed(stride,
			       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(bus_client));
		writel_relaxed(stride * height,
			       vfe->base + VFE_BUS_WM_FRAME_INCR(bus_client));
		writel_relaxed(0,
			       vfe->base + VFE_BUS_WM_PACKER_CFG(bus_client));

		mode = 1 << WM_CFG_EN | MODE_MIPI_RAW << WM_CFG_MODE;
	}

	writel_relaxed(0, vfe->base + VFE_BUS_WM_PACKER_CFG(bus_client));

	/* no dropped frames, one irq per frame */
	writel_relaxed(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(bus_client));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(bus_client));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(bus_client));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(bus_client));

	writel_relaxed(mode,
		       vfe->base + VFE_BUS_WM_CFG(bus_client));

	dev_info(vfe->camss->dev, "%s wm %d output->line->is_pix %s\n",
		 __func__, bus_client, output->line->is_pix ? "true" : "false");
}

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	__vfe_wm_start(vfe, &line->output[0], wm, 0);
}

static void __vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	__vfe_wm_stop(vfe, line->output[0].wm[0].bus_client);
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, u32 addr,
			  struct vfe_line *line)
{
	wm = line->output[0].wm[0].bus_client;
	writel_relaxed(addr, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));
}

static inline u32 reg_update_src(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	if (!vfe_is_lite(vfe) && vfe->line[line_id].is_pix)
		return REG_UPDATE_IPP;

	return REG_UPDATE_RDI(vfe, line_id);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= reg_update_src(vfe, line_id);
	writel_relaxed(vfe->reg_update, vfe->base + VFE_REG_UPDATE_CMD);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~reg_update_src(vfe, line_id);
}

static void vfe_enable_irq(struct vfe_device *vfe)
{
	int i, j;
	u32 bus_irq_mask = 0;

	if (!vfe->stream_count)
		/* enable reset ack IRQ and top BUS status IRQ */
		writel(IRQ_MASK_0_RESET_ACK | IRQ_MASK_0_BUS_TOP_IRQ,
		       vfe->base + VFE_IRQ_MASK(0));

	for (i = 0; i < VFE_LINE_NUM_MAX; i++) {
		struct vfe_line *line = &vfe->line[i];

		for (j = 0; j < line->num_outputs; j++) {
			struct vfe_output *output = &line->output[j];

			/* Enable IRQ for newly added lines, but also keep already running lines's IRQ */
			dev_info(vfe->camss->dev, "output %d is reserved %d on %d\n",
				 j, output->state == VFE_OUTPUT_RESERVED, output->state == VFE_OUTPUT_ON);
			if (output->state == VFE_OUTPUT_RESERVED ||
			    output->state == VFE_OUTPUT_ON) {
				bus_irq_mask |= bus_irq_mask_comp_done(vfe, output->comp_group);
			}
		}
	}
bus_irq_mask = ~0u;
dev_info(vfe->camss->dev, "vfe_bus_mask_irq(0) @ 0x%08x = 0x%08x\n", VFE_BUS_IRQ_MASK(0), bus_irq_mask);
	writel(bus_irq_mask, vfe->base + VFE_BUS_IRQ_MASK(0));
}

static void vfe_isr_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id);
static void vfe_output_buf_done(struct vfe_device *vfe, struct vfe_output *output);

#define D(dev, fmt, ...) dev_info(dev, fmt, ##__VA_ARGS__)

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
	u32 status[3];
	int i, j;

	status[0] = readl_relaxed(vfe->base + VFE_IRQ_STATUS(0));
	status[1] = readl_relaxed(vfe->base + VFE_IRQ_STATUS(1));
	status[2] = readl_relaxed(vfe->base + VFE_IRQ_STATUS(2));
	writel_relaxed(status[0], vfe->base + VFE_IRQ_CLEAR(0));
	writel_relaxed(IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_IRQ_CMD);

/* in the ISR, when BIT(30)|BIT(31) set: */
dev_err(vfe->camss->dev, "VIOLATION_STATUS = 0x%08x\n",
	readl_relaxed(vfe->base + BUS_REG_BASE + 0x64));
dev_err(vfe->camss->dev, "IMAGE_SIZE_VIOLATION_STATUS = 0x%08x\n",
	readl_relaxed(vfe->base + BUS_REG_BASE + 0x70));
dev_err(vfe->camss->dev, "OVERFLOW_STATUS = 0x%08x\n",
	readl_relaxed(vfe->base + BUS_REG_BASE + 0x68));
dev_err(vfe->camss->dev, "PP Violation status = 0x%08x\n",
	readl_relaxed(vfe->base + BUS_REG_BASE + 0x74));


	D(vfe->camss->dev, "VFE IRQ0 0x%08x%s%s%s%s\n", status[0],
	  status[0] & BIT(31) ? " RESET_ACK" : "",
	  status[0] & BIT(1)  ? " BUS_TOP" : "",
	  status[0] & BIT(0)  ? " FRAME_HDR?" : "",
	  status[0] & ~(BIT(31) | BIT(1) | BIT(0)) ? " +unknown" : "");

	D(vfe->camss->dev, "VFE IRQ1 0x%08x%s%s%s%s%s\n", status[1],
	  status[1] & BIT(0) ? " CAMIF_SOF" : "",
	  status[1] & BIT(1) ? " CAMIF_EOF" : "",
	  status[1] & BIT(2) ? " EPOCH0" : "",
	  status[1] & BIT(3) ? " EPOCH1" : "",
	  status[1] & ~GENMASK(3, 0) ? " +more" : "");

	D(vfe->camss->dev, "VFE IRQ2 0x%08x%s%s%s%s%s%s%s%s\n", status[2],
	  status[2] & BIT(11) ? " PP_CAMIF_VIOL" : "",
	  status[2] & BIT(12) ? " PP_VIOL" : "",
	  status[2] & BIT(15) ? " LCR_CAMIF_VIOL" : "",
	  status[2] & BIT(16) ? " LCR_VIOL" : "",
	  status[2] & BIT(17) ? " RDI0_CAMIF_VIOL" : "",
	  status[2] & BIT(18) ? " RDI1_CAMIF_VIOL" : "",
	  status[2] & BIT(19) ? " RDI2_CAMIF_VIOL" : "",
	  status[2] & BIT(7)  ? " DSP_PROTO" : "");

	if (status[0] & IRQ_MASK_0_RESET_ACK)
		vfe_isr_reset_ack(vfe);

u32 bus0, bus1;

bus0 = readl_relaxed(vfe->base + VFE_BUS_IRQ_STATUS(0));
bus1 = readl_relaxed(vfe->base + VFE_BUS_IRQ_STATUS(1));

/* ================= BUS_IRQ_STATUS 0 ================= */
	D(vfe->camss->dev, "B0 0x%08x rup[ipp:%d pd:%d lcr:%d rdi:%d%d%d] comp[grp0:%d grp3:%d grp7:%d grp11-13:%d%d%d]%s%s\n",
	  bus0,
	  !!(bus0 & BIT(0)), !!(bus0 & BIT(1)), !!(bus0 & BIT(2)),
	  !!(bus0 & BIT(3)), !!(bus0 & BIT(4)), !!(bus0 & BIT(5)),
	  !!(bus0 & BIT(6)),		/* comp grp0 = FULL   */
	  !!(bus0 & BIT(9)),		/* comp grp3 = PIXEL_RAW */
	  !!(bus0 & BIT(13)),		/* comp grp7 = stats  */
	  !!(bus0 & BIT(17)), !!(bus0 & BIT(18)), !!(bus0 & BIT(19)),
	  bus0 & BIT(30) ? " CCIF_VIOL" : "",
	  bus0 & BIT(31) ? " IMG_SZ_VIOL" : "");

/* ================= BUS_IRQ_STATUS 1: per-client BUF_DONE ================= */
	D(vfe->camss->dev, "B1 0x%08x done[Y:%d C:%d ds4:%d ds16:%d bhist:%d ihist:%d lcr:%d rdi:%d%d%d]\n",
	  bus1,
	  !!(bus1 & BIT(0)),  !!(bus1 & BIT(1)),	/* FULL_Y / FULL_C */
	  !!(bus1 & BIT(2)),  !!(bus1 & BIT(3)),	/* DS4 / DS16      */
	  !!(bus1 & BIT(16)), !!(bus1 & BIT(19)),	/* BHIST / IHIST   */
	  !!(bus1 & BIT(22)),				/* LCR             */
	  !!(bus1 & BIT(23)), !!(bus1 & BIT(24)), !!(bus1 & BIT(25)));	/* RDI0-2 */


	if (status[0] & IRQ_MASK_0_BUS_TOP_IRQ) {
		u32 bus_status = readl_relaxed(vfe->base + VFE_BUS_IRQ_STATUS(0));

		writel_relaxed(bus_status, vfe->base + VFE_BUS_IRQ_CLEAR(0));
		writel_relaxed(1, vfe->base + VFE_BUS_IRQ_CLEAR_GLOBAL);

		for (i = 0; i < MAX_VFE_OUTPUT_LINES; i++) {
			if (bus_status & bus_irq_mask_rup(vfe, i))
				vfe_isr_reg_update(vfe, i);
		}

		/* Loop through all WMs IRQs */
		for (i = 0; i < VFE_LINE_NUM_MAX; i++) {
			struct vfe_line *line = &vfe->line[i];

			for (j = 0; j < line->num_outputs; j++) {
				struct vfe_output *output = &line->output[j];

				dev_info(vfe->camss->dev, "%s is pix %d bus_status 0x%08x required comp_group %x\n",
					 __func__, output->line->is_pix,bus_status, output->comp_group);

				if (bus_status & bus_irq_mask_comp_done(vfe, output->comp_group))
					vfe_output_buf_done(vfe, output);
			}
		}
	}

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

/*
 * vfe_isr_reg_update - Process reg update interrupt
 * @vfe: VFE Device
 * @line_id: VFE line
 */
static void vfe_isr_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	struct vfe_output *output;
	unsigned long flags;

	spin_lock_irqsave(&vfe->output_lock, flags);
	vfe_reg_update_clear(vfe, line_id);

	output = &vfe->line[line_id].output[0];

	if (output->wait_reg_update) {
		output->wait_reg_update = 0;
		complete(&output->reg_update);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static const struct camss_video_ops vfe_video_ops_480 = {
	.queue_buffer = vfe_queue_buffer_v2,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->video_ops = vfe_video_ops_480;

	/* RDI0 */
	vfe->line[VFE_LINE_RDI0].output[0].wm_num = 1;
	vfe->line[VFE_LINE_RDI0].output[0].wm[0].bus_client = VFE_WM_RDI0;
	vfe->line[VFE_LINE_RDI0].output[0].wm[0].plane = 0;
	vfe->line[VFE_LINE_RDI0].output[0].comp_group = VFE_V3_COMP_GRP_11;
	vfe->line[VFE_LINE_RDI0].output[0].type = VFE_OUTPUT_TYPE_PIXEL_RAW;

	/* RDI1 */
	vfe->line[VFE_LINE_RDI1].output[0].wm_num = 1;
	vfe->line[VFE_LINE_RDI1].output[0].wm[0].bus_client = VFE_WM_RDI1;
	vfe->line[VFE_LINE_RDI1].output[0].wm[0].plane = 0;
	vfe->line[VFE_LINE_RDI1].output[0].comp_group = VFE_V3_COMP_GRP_12;
	vfe->line[VFE_LINE_RDI1].output[0].type = VFE_OUTPUT_TYPE_PIXEL_RAW;

	/* RDI2 */
	vfe->line[VFE_LINE_RDI2].output[0].wm_num = 1;
	vfe->line[VFE_LINE_RDI2].output[0].wm[0].bus_client = VFE_WM_RDI2;
	vfe->line[VFE_LINE_RDI2].output[0].wm[0].plane = 0;
	vfe->line[VFE_LINE_RDI2].output[0].comp_group = VFE_V3_COMP_GRP_13;
	vfe->line[VFE_LINE_RDI2].output[0].type = VFE_OUTPUT_TYPE_PIXEL_RAW;

	/* PIX YUV - consisting of two write masters one plane each in one comp_group */
	vfe->line[VFE_LINE_PIX].output[0].wm_num = 2;
	vfe->line[VFE_LINE_PIX].output[0].wm[0].bus_client = VFE_WM_VIDEO_FULL_Y;
	vfe->line[VFE_LINE_PIX].output[0].wm[0].plane = 0;
	vfe->line[VFE_LINE_PIX].output[0].wm[1].bus_client = VFE_WM_VIDEO_FULL_C;
	vfe->line[VFE_LINE_PIX].output[0].wm[1].plane = 1;
	vfe->line[VFE_LINE_PIX].output[0].comp_group = VFE_V3_COMP_GRP_0;
	vfe->line[VFE_LINE_PIX].output[0].type = VFE_OUTPUT_TYPE_PIXEL_YUV;

	vfe->line[VFE_LINE_PIX].is_pix = true;
}

static void vfe_isr_read(struct vfe_device *vfe, u32 *value0, u32 *value1)
{
	/* nop */
}

static void vfe_violation_read(struct vfe_device *vfe)
{
	/* nop */
}

static void vfe_buf_done_480(struct vfe_device *vfe, int port_id)
{
	/* nop */
}
/* camss-vfe-480.h */
/* JSON: "MODULE_LITE_CFG" in PP_CLC_CAMIF; Valve VFEWrapper line 721 */
#define VFE_PP_CAMIF_MODULE_CFG		0x2660
#define		CAMIF_MODULE_EN		BIT(0)
#define		CAMIF_PIXEL_PATTERN	24	/* [26:24] */

/* camss-vfe-480.c */
static u32 vfe_bayer_pattern(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 0;	/* RGRGRG */
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return 1;	/* GRGRGR */
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return 2;	/* BGBGBG */
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		return 3;	/* GBGBGB */
	default:
		return 0;
	}
}

static void vfe_pix_camif_enable(struct vfe_device *vfe, struct vfe_line *line)
{
	u32 pattern = vfe_bayer_pattern(line->fmt[MSM_VFE_PAD_SINK].code);

	writel_relaxed(CAMIF_MODULE_EN | pattern << CAMIF_PIXEL_PATTERN,
		       vfe->base + VFE_PP_CAMIF_MODULE_CFG);

dev_info(vfe->camss->dev, "CAMIF @ 0x%08x = 0x%08x\n", VFE_PP_CAMIF_MODULE_CFG, CAMIF_MODULE_EN | pattern << CAMIF_PIXEL_PATTERN);
}

/* ---------- camss-vfe-480.h : ADD bit-field helpers under the blocks ---------- */

/* CROP_RND_CLAMP_*: Kona RTL — CROP: FIRST[29:16]|LAST[13:0] inclusive;
 * CLAMP: MAX[25:16]|MIN[9:0]; ROUNDING: OFF_BITS[5:3], PATTERN[2:1],
 * INTERLEAVED[0]. All i_rup_stb shadowed. */
#define		PP_CROP_FIRST(v)		((v) << 16)
#define		PP_CROP_LAST(v)			(v)
#define		PP_CLAMP(min, max)		((max) << 16 | (min))
#define		PP_RND_OFF_BITS(n)		((n) << 3)
#define		PP_RND_PATTERN_REGULAR		(0 << 1)
#define		PP_RND_CH_INTERLEAVED		BIT(0)
/* DOWNSCALE_MN_C: CFG 0x600 = h+v scale en; H/V_CFG ratio-2 encoding */
#define		PP_MN_C_CFG_HV_EN		0x600
#define		PP_MN_C_SCALE_DIV2		(2 << 21 | 3 << 30)
/* DEMOSAIC defaults (Valve) */
#define		PP_DEMOSAIC_INTERP_COEFF_DEFAULT	0x00000080
#define		PP_DEMOSAIC_INTERP_CLASSIFIER_DEFAULT	0x00200066
#define		PP_DEMOSAIC_WB_GAIN_UNITY		0x400	/* 1.0 Q3.10 */

/* BT.601 full-range RGB->YCbCr, Q3.10 signed, 14-bit fields */
#define CST_Q(x)	((x) & 0x3fff)
static const u16 cst_601[3][3] = {
	{ CST_Q(306),  CST_Q(601),  CST_Q(117)  },	/*  .299  .587  .114 */
	{ CST_Q(-173), CST_Q(-339), CST_Q(512)  },	/* -.169 -.331  .5   */
	{ CST_Q(512),  CST_Q(-429), CST_Q(-83)  },	/*  .5   -.419 -.081 */
};
static const u16 cst_post_off[3] = { 0, 512, 512 };

/* ---------- camss-vfe-480.c ---------- */

static void __vfe_configure_white_balance(struct vfe_device *vfe,
			const struct camss_params_wb_gain *wb)
{
	writel_relaxed(wb->b_gain << 16 | wb->g_gain,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_0);
dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_0 @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_0, wb->b_gain << 16 | wb->g_gain);

	writel_relaxed(wb->r_gain,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_1);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_1 @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_WB_GAIN_CFG_1, wb->r_gain);

	writel_relaxed(wb->b_offset << 16 | wb->g_offset,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_0);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_0 @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_0, wb->b_offset << 16 | wb->g_offset);

	writel_relaxed(wb->r_offset,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_1);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_0 @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_WB_OFFSET_CFG_1, wb->r_offset);

}

static void __vfe_configure_demosaic(struct vfe_device *vfe,
			      const struct camss_params_demosaic *dm)
{
	/* TODO: pack dm->wk/ak/lambda_* once bit layout verified;
	 * defaults produce the known-good constants */
	writel_relaxed(PP_DEMOSAIC_INTERP_COEFF_DEFAULT,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_INTERP_COEFF_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_INTERP_COEFF_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_INTERP_COEFF_CFG, PP_DEMOSAIC_INTERP_COEFF_DEFAULT);

	writel_relaxed(PP_DEMOSAIC_INTERP_CLASSIFIER_DEFAULT,
		       vfe->base + VFE_PP_CLC_DEMOSAIC_INTERP_CLASSIFIER_CFG);

dev_info(vfe->camss->dev, "%s PP_DEMOSAIC_INTERP_CLASSIFIER_DEFAULT @ 0x%08x = 0x%08x\n",
	 __func__, PP_DEMOSAIC_INTERP_CLASSIFIER_DEFAULT, VFE_PP_CLC_DEMOSAIC_INTERP_CLASSIFIER_CFG);

	writel_relaxed(1, vfe->base + VFE_PP_CLC_DEMOSAIC_MODULE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DEMOSAIC_MODULE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DEMOSAIC_MODULE_CFG, 1);

}

static void __vfe_configure_color_xform(struct vfe_device *vfe,
			 const struct camss_params_color_xform *cx)
{
	/* CH register groups stride 0x10 from CH0 */
	u32 base0 = VFE_PP_CLC_COLOR_XFORM_COLOR_XFORM_CH0_COEFF_CFG_0;
	unsigned int ch;

	for (ch = 0; ch < 3; ch++) {
		u32 b = base0 + ch * 0x10;

		writel_relaxed((u16)cx->m[ch][1] << 16 | (u16)cx->m[ch][0],
			       vfe->base + b);			/* COEFF_CFG_0 */
		writel_relaxed((u16)cx->m[ch][2],
			       vfe->base + b + 0x4);		/* COEFF_CFG_1 */
		writel_relaxed((cx->s[ch] & 0x7ff) << 16 |
			       (cx->o[ch] & 0x7ff),
			       vfe->base + b + 0x8);		/* OFFSET_CFG */
		writel_relaxed(PP_CLAMP(0, 1023),
			       vfe->base + b + 0xc);		/* CLAMP_CFG */
	}
	writel_relaxed(1, vfe->base + VFE_PP_CLC_COLOR_XFORM_MODULE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_COLOR_XFORM_MODULE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_COLOR_XFORM_MODULE_CFG, 1);

}

static void __vfe_configure_chroma_down_sampling(struct vfe_device *vfe, u32 w, u32 h)
{
	writel_relaxed(PP_MN_C_CFG_HV_EN,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_CFG);


dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_CFG, PP_MN_C_CFG_HV_EN);

	writel_relaxed((h - 1) | (w - 1) << 16,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_IMAGE_SIZE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_IMAGE_SIZE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_IMAGE_SIZE_CFG, (h - 1) | (w - 1) << 16);

	writel_relaxed(PP_MN_C_SCALE_DIV2,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_CFG, PP_MN_C_SCALE_DIV2);

	writel_relaxed(0,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_PHASE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_PHASE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_PHASE_CFG, 0);

	writel_relaxed(PP_MN_C_SCALE_DIV2,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_CFG);

dev_info(vfe->camss->dev, "%sVFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_CFG, PP_MN_C_SCALE_DIV2);

	writel_relaxed(0,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_PHASE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_PHASE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_PHASE_CFG, 0);

	/* MN_C's own CROP_LINE/PIXEL (0x667c/0x6680): untouched, as Valve —
	 * reset state passes through */
	writel_relaxed(1,
		       vfe->base + VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG, 1);

}

static void __vfe_configure_round_clamp(struct vfe_device *vfe, u32 w, u32 h)
{
	/* Y */
	writel_relaxed(PP_CROP_FIRST(0) | PP_CROP_LAST(h - 1),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_LINE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_LINE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_LINE_CFG, PP_CROP_FIRST(0) | PP_CROP_LAST(h - 1));

	writel_relaxed(PP_CROP_FIRST(0) | PP_CROP_LAST(w - 1),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_PIXEL_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_PIXEL_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CROP_PIXEL_CFG, PP_CROP_FIRST(0) | PP_CROP_LAST(w - 1));

	writel_relaxed(PP_CLAMP(0, 255),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_CFG, PP_CLAMP(0, 255));

	writel_relaxed(PP_RND_OFF_BITS(2) | PP_RND_PATTERN_REGULAR,
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_ROUNDING_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_ROUNDING_CFG @ 0x%08x = 0x%08x\n",
	__func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_ROUNDING_CFG, PP_RND_OFF_BITS(2) | PP_RND_PATTERN_REGULAR);

	writel_relaxed(1,
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_MODULE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_MODULE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_MODULE_CFG, 1);

	/* C: post-MN interleaved CbCr, h/2 lines x w samples.
	 * If chroma shows halved horizontally: PP_CROP_LAST(w / 2 - 1). */
	writel_relaxed(PP_CROP_FIRST(0) | PP_CROP_LAST(h / 2 - 1),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_LINE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_LINE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_LINE_CFG, PP_CROP_FIRST(0) | PP_CROP_LAST(h / 2 - 1));

	writel_relaxed(PP_CROP_FIRST(0) | PP_CROP_LAST(w - 1),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_PIXEL_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_PIXEL_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CROP_PIXEL_CFG, PP_CROP_FIRST(0) | PP_CROP_LAST(w - 1));

	writel_relaxed(PP_CLAMP(0, 255),
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_CFG, PP_CLAMP(0, 255));

	writel_relaxed(PP_RND_OFF_BITS(2) | PP_RND_PATTERN_REGULAR |
		       PP_RND_CH_INTERLEAVED,
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_ROUNDING_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_ROUNDING_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_ROUNDING_CFG, PP_RND_OFF_BITS(2) | PP_RND_PATTERN_REGULAR |
                       PP_RND_CH_INTERLEAVED);

	writel_relaxed(1,
		       vfe->base + VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG);

dev_info(vfe->camss->dev, "%s VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG @ 0x%08x = 0x%08x\n",
	 __func__, VFE_PP_CLC_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG, 1);

}

static void vfe_pix_pp_full_config(struct vfe_device *vfe, struct vfe_line *line)
{
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;

	/* Unity white balance: 1.0 Q3.10 gains, zero offsets. AWB updates via
	 * params node later. Channel order g/b/r per the UAPI (matches WB_GAIN
	 * register pairing: CFG_0 = b << 16 | g, CFG_1 = r). */
	static const struct camss_params_wb_gain vfe_pp_wb_default = {
		.g_gain		= PP_DEMOSAIC_WB_GAIN_UNITY,	/* 0x400 */
		.b_gain		= PP_DEMOSAIC_WB_GAIN_UNITY,
		.r_gain		= PP_DEMOSAIC_WB_GAIN_UNITY,
		.g_offset	= 0,
		.b_offset	= 0,
		.r_offset	= 0,
	};

	/* Demosaic interpolation defaults. Zero-initialized: the applier maps a
	 * default struct to the known-good register constants
	 * (INTERP_COEFF 0x00000080, INTERP_CLASSIFIER 0x00200066, per Valve)
	 * until the wk/ak/lambda field packing is verified. */
	static const struct camss_params_demosaic vfe_pp_demosaic_default = {
		.wk		= 0,
		.ak		= 0,
		.lambda_g	= 0,
		.lambda_rb	= 0,
		/* dir_/clamp_ disable flags: 0 = enabled */
	};

	/* RGB -> YCbCr, BT.601 full range, coefficients Q3.10 signed.
	 *   Y  =  .299 R + .587 G + .114 B
	 *   Cb = -.169 R - .331 G + .500 B + 512
	 *   Cr =  .500 R - .419 G - .081 B + 512
	 * o[] = pre-matrix offsets (none), s[] = post-matrix offsets recentering
	 * chroma at 10-bit mid-scale; clamp 0..1023 applied by the applier.
	 * 10 -> 8 bit conversion happens later at CROP_RND_CLAMP (round-off 2). */
	static const struct camss_params_color_xform vfe_pp_cst_601_default = {
		.m = {
			{  306,  601,  117 },
			{ -173, -339,  512 },
			{  512, -429,  -83 },
		},
		.o = { 0, 0, 0 },
		.s = { 0, 512, 512 },
		/* c0/c01, c1/c11, c2/c21 clamp fields: left zero; applier writes
		 * PP_CLAMP(0, 1023) until the UAPI clamp-field pairing is
		 * confirmed against the register layout */
	};

	u32 w = line->fmt[MSM_VFE_PAD_SINK].width;
	u32 h = line->fmt[MSM_VFE_PAD_SINK].height;

//	__vfe_configure_white_balance(vfe, &vfe_pp_wb_default);
	__vfe_configure_demosaic(vfe, &vfe_pp_demosaic_default);
	__vfe_configure_color_xform(vfe, &vfe_pp_cst_601_default);
	__vfe_configure_chroma_down_sampling(vfe, w, h);
//	__vfe_configure_round_clamp(vfe, w, h);

	ops->reg_update(vfe, line->id);
}

/* Output based API - new and shiny */
static void vfe_output_start(struct vfe_device *vfe, struct vfe_output *output)
{
	struct v4l2_pix_format_mplane *pix =
		&output->video_out.active_fmt.fmt.pix_mp;
	int i;

	writel_relaxed(CORE_CFG_0_VID_DS16_R2PD_DISABLE |
		       CORE_CFG_0_VID_DS4_R2PD_DISABLE |
		       CORE_CFG_0_DISP_DS16_R2PD_DISABLE |
		       CORE_CFG_0_DISP_DS4_R2PD_DISABLE,
		       vfe->base + VFE_TOP_CORE_CFG_0);	/* = 0x78000000 */
	writel_relaxed(0, vfe->base + VFE_TOP_CORE_CFG_1);

dev_info(vfe->camss->dev, "%s VFE_TOP_CORE_CFG_0 = 0x%08x\n",
	 __func__, readl(vfe->base + VFE_TOP_CORE_CFG_0));
dev_info(vfe->camss->dev, "%s VFE_TOP_CORE_CFG_1 = 0x%08x\n",
	 __func__, readl(vfe->base + VFE_TOP_CORE_CFG_1));


dev_info(vfe->camss->dev, "%s is pix %d\n", __func__, output->line->is_pix);
	if (output->line->is_pix) {
		vfe_pix_camif_enable(vfe, output->line);
		if (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
		    pix->pixelformat == V4L2_PIX_FMT_NV21)
			vfe_pix_pp_full_config(vfe, output->line);
	}

	for (i = 0; i < output->wm_num; i++)
		__vfe_wm_start(vfe, output, output->wm[i].bus_client, output->wm[i].plane);
}

static void vfe_output_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	int i;

dev_info(vfe->camss->dev, "%s is pix %d\n", __func__, output->line->is_pix);
	for (i = output->wm_num - 1; i >= 0; i--)
		__vfe_wm_stop(vfe, output->wm[i].bus_client);
}

static void vfe_output_buf_done(struct vfe_device *vfe, struct vfe_output *output)
{
	const struct vfe_hw_ops *ops = vfe->res->hw_ops;
	struct vfe_line *line = output->line;
	struct camss_buffer *ready_buf;
	unsigned long flags;
	u32 index;
	u64 ts = ktime_get_ns();

	spin_lock_irqsave(&vfe->output_lock, flags);

	ready_buf = output->buf[0];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	ready_buf->vb.vb2_buf.timestamp = ts;
	ready_buf->vb.sequence = output->sequence++;

	index = 0;
	output->buf[0] = output->buf[1];
	if (output->buf[0])
		index = 1;

	output->buf[index] = vfe_buf_get_pending(output);
	if (output->buf[index]) {
		ops->vfe_output_update(vfe, output, output->buf[index]);
		ops->reg_update(vfe, line->id);
	} else {
		output->gen2.active_num--;
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	vb2_buffer_done(&ready_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static void vfe_output_update(struct vfe_device *vfe, struct vfe_output *output,
			      struct camss_buffer *buf)
{
	unsigned int i;

	for (i = 0; i < output->wm_num; i++)
		writel(buf->addr[i], vfe->base + VFE_BUS_WM_IMAGE_ADDR(output->wm[i].bus_client));
}

const struct vfe_hw_ops vfe_ops_480 = {
	.enable_irq = vfe_enable_irq,
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr = vfe_isr,
	.isr_read = vfe_isr_read,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe_disable,
	.vfe_enable = vfe_enable_v2,
	.vfe_halt = vfe_halt,
	.violation_read = vfe_violation_read,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_buf_done = vfe_buf_done_480,
	.vfe_wm_update = vfe_wm_update,

	.vfe_output_start = vfe_output_start,
	.vfe_output_stop = vfe_output_stop,
	.vfe_output_buf_done = vfe_output_buf_done,
	.vfe_output_update = vfe_output_update,
};
