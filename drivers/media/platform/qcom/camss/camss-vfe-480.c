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

static u32 pix_raw_packer_fmt(u32 v4l2_fmt)
{
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

	/* Chroma plane of a 4:2:0 semiplanar buffer is half height */
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		if (plane > 0)
			height /= 2;
		break;
	default:
		break;
	}

	/* no clock gating at bus input */
	writel_relaxed(WM_CGC_OVERRIDE_ALL, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel_relaxed(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

	writel_relaxed(stride * height,
		       vfe->base + VFE_BUS_WM_FRAME_INCR(bus_client));
	writel_relaxed(0xf, vfe->base + VFE_BUS_WM_BURST_LIMIT(bus_client));
	writel_relaxed(WM_IMAGE_CFG_0_DEFAULT_WIDTH,
		       vfe->base + VFE_BUS_WM_IMAGE_CFG_0(bus_client));
	writel_relaxed(stride,
		       vfe->base + VFE_BUS_WM_IMAGE_CFG_2(bus_client));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_PACKER_CFG(bus_client));

	/* no dropped frames, one irq per frame */
	writel_relaxed(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(bus_client));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(bus_client));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(bus_client));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(bus_client));

	writel_relaxed(1 << WM_CFG_EN | MODE_MIPI_RAW << WM_CFG_MODE,
		       vfe->base + VFE_BUS_WM_CFG(bus_client));
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

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= REG_UPDATE_RDI(vfe, line_id);
	writel_relaxed(vfe->reg_update, vfe->base + VFE_REG_UPDATE_CMD);
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~REG_UPDATE_RDI(vfe, line_id);
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
			if (output->state == VFE_OUTPUT_RESERVED ||
			    output->state == VFE_OUTPUT_ON) {
				bus_irq_mask |= bus_irq_mask_comp_done(vfe, output->comp_group);
			}
		}
	}

	writel(bus_irq_mask, vfe->base + VFE_BUS_IRQ_MASK(0));
}

static void vfe_isr_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id);
static void vfe_output_buf_done(struct vfe_device *vfe, struct vfe_output *output);

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
	u32 status;
	int i, j;

	status = readl_relaxed(vfe->base + VFE_IRQ_STATUS(0));
	writel_relaxed(status, vfe->base + VFE_IRQ_CLEAR(0));
	writel_relaxed(IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_IRQ_CMD);

	if (status & IRQ_MASK_0_RESET_ACK)
		vfe_isr_reset_ack(vfe);

	if (status & IRQ_MASK_0_BUS_TOP_IRQ) {
		u32 status = readl_relaxed(vfe->base + VFE_BUS_IRQ_STATUS(0));

		writel_relaxed(status, vfe->base + VFE_BUS_IRQ_CLEAR(0));
		writel_relaxed(1, vfe->base + VFE_BUS_IRQ_CLEAR_GLOBAL);

		for (i = 0; i < MAX_VFE_OUTPUT_LINES; i++) {
			if (status & bus_irq_mask_rup(vfe, i))
				vfe_isr_reg_update(vfe, i);
		}

		/* Loop through all WMs IRQs */
		for (i = 0; i < VFE_LINE_NUM_MAX; i++) {
			struct vfe_line *line = &vfe->line[i];

			for (j = 0; j < line->num_outputs; j++) {
				struct vfe_output *output = &line->output[j];

				if (status & bus_irq_mask_comp_done(vfe, output->comp_group))
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

/* Output based API - new and shiny */
static void vfe_output_start(struct vfe_device *vfe, struct vfe_output *output)
{
	int i;

	for (i = 0; i < output->wm_num; i++)
		__vfe_wm_start(vfe, output, output->wm[i].bus_client, output->wm[i].plane);
}

static void vfe_output_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	int i;

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
