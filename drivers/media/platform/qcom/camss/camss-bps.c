// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm CAMSS BPS (Bayer Processing Segment) V4L2 M2M Driver
 *
 * Responsibilities:
 * - Manage BPS power and clocks (independent of ICP)
 * - Implement V4L2 mem2mem interface for demosaicing
 * - Use ICP for HFI communication with firmware
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#include <linux/clk.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "camss-icp.h"
#include "camss-icp-hfi-frame.h"
#include "camss-bps-cmd.h"
#include "camss-bps-iq.h"

#define BPS_NAME		"qcom-camss-bps"
#define BPS_CLK_MAX		4

struct bps_device {
	struct device *dev;
	void __iomem *base;

	/* BPS's own clocks */
	struct clk_bulk_data clocks[BPS_CLK_MAX];
	int num_clocks;

	/* BPS's own interconnect */
	struct icc_path *icc_mem;

	/* BPS's own power state */
	bool powered;

	/* UBWC config */
	struct icp_ubwc_cfg ubwc;

	/* ICP reference */
	struct icp_device *icp;

	/* V4L2 */
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock;
};

struct bps_ctx {
	struct v4l2_fh fh;
	struct bps_device *bps;
	struct icp_context *icp_ctx;

	/* Format info */
	u32 width;
	u32 height;
	u32 src_fmt;
	u32 dst_fmt;
	u32 bayer_pattern;

	/* Command buffers (DMA memory for HFI) */
	struct bps_cmd_bufs *cmd_bufs;

	/* IQ configuration */
	struct bps_iq_config iq;

	/* Frame counter */
	u32 frame_id;
};

/* ============================================================
 * BPS Power Management (independent of ICP)
 * ============================================================ */

static int bps_power_on(struct bps_device *bps)
{
	int ret;

	if (bps->powered)
		return 0;

	ret = pm_runtime_resume_and_get(bps->dev);
	if (ret)
		return ret;

	ret = icc_set_bw(bps->icc_mem, 0, MBps_to_icc(8000));
	if (ret)
		goto err_rpm;

	/* Set clock rate */
	clk_set_rate(bps->clocks[2].clk, 600000000);  /* core */

	ret = clk_bulk_prepare_enable(bps->num_clocks, bps->clocks);
	if (ret)
		goto err_icc;

	bps->powered = true;
	dev_dbg(bps->dev, "BPS powered on\n");
	return 0;

err_icc:
	icc_set_bw(bps->icc_mem, 0, 0);
err_rpm:
	pm_runtime_put(bps->dev);
	return ret;
}

static void bps_power_off(struct bps_device *bps)
{
	if (!bps->powered)
		return;

	clk_bulk_disable_unprepare(bps->num_clocks, bps->clocks);
	icc_set_bw(bps->icc_mem, 0, 0);
	pm_runtime_put(bps->dev);

	bps->powered = false;
	dev_dbg(bps->dev, "BPS powered off\n");
}

/* ============================================================
 * V4L2 Operations
 * ============================================================ */

static int bps_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, BPS_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm BPS", sizeof(cap->card));

	return 0;
}

static int bps_enum_fmt_src(struct file *file, void *priv,
			    struct v4l2_fmtdesc *f)
{
	/* Input: Raw Bayer formats */
	static const u32 fmts[] = {
		V4L2_PIX_FMT_SRGGB10,
		V4L2_PIX_FMT_SGRBG10,
		V4L2_PIX_FMT_SGBRG10,
		V4L2_PIX_FMT_SBGGR10,
	};

	if (f->index >= ARRAY_SIZE(fmts))
		return -EINVAL;

	f->pixelformat = fmts[f->index];
	return 0;
}

static int bps_enum_fmt_dst(struct file *file, void *priv,
			    struct v4l2_fmtdesc *f)
{
	/* Output: NV12 (demosaiced) */
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int bps_g_fmt_src(struct file *file, void *priv, struct v4l2_format *f)
{
	struct bps_ctx *ctx = container_of(priv, struct bps_ctx, fh);

	f->fmt.pix_mp.width = ctx->width ?: 4096;
	f->fmt.pix_mp.height = ctx->height ?: 3072;
	f->fmt.pix_mp.pixelformat = ctx->src_fmt ?: V4L2_PIX_FMT_SRGGB10;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width * 2;
	f->fmt.pix_mp.plane_fmt[0].sizeimage =
		f->fmt.pix_mp.plane_fmt[0].bytesperline * f->fmt.pix_mp.height;

	return 0;
}

static int bps_g_fmt_dst(struct file *file, void *priv, struct v4l2_format *f)
{
	struct bps_ctx *ctx = container_of(priv, struct bps_ctx, fh);

	f->fmt.pix_mp.width = ctx->width ?: 4096;
	f->fmt.pix_mp.height = ctx->height ?: 3072;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	f->fmt.pix_mp.num_planes = 2;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;
	f->fmt.pix_mp.plane_fmt[0].sizeimage =
		f->fmt.pix_mp.width * f->fmt.pix_mp.height;
	f->fmt.pix_mp.plane_fmt[1].bytesperline = f->fmt.pix_mp.width;
	f->fmt.pix_mp.plane_fmt[1].sizeimage =
		f->fmt.pix_mp.width * f->fmt.pix_mp.height / 2;

	return 0;
}

static int bps_s_fmt_src(struct file *file, void *priv, struct v4l2_format *f)
{
	struct bps_ctx *ctx = container_of(priv, struct bps_ctx, fh);

	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;
	ctx->src_fmt = f->fmt.pix_mp.pixelformat;

	return bps_g_fmt_src(file, priv, f);
}

static int bps_s_fmt_dst(struct file *file, void *priv, struct v4l2_format *f)
{
	struct bps_ctx *ctx = container_of(priv, struct bps_ctx, fh);

	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;

	return bps_g_fmt_dst(file, priv, f);
}

static const struct v4l2_ioctl_ops bps_ioctl_ops = {
	.vidioc_querycap		= bps_querycap,
	.vidioc_enum_fmt_vid_out	= bps_enum_fmt_src,
	.vidioc_enum_fmt_vid_cap	= bps_enum_fmt_dst,
	.vidioc_g_fmt_vid_out_mplane	= bps_g_fmt_src,
	.vidioc_g_fmt_vid_cap_mplane	= bps_g_fmt_dst,
	.vidioc_s_fmt_vid_out_mplane	= bps_s_fmt_src,
	.vidioc_s_fmt_vid_cap_mplane	= bps_s_fmt_dst,
	.vidioc_try_fmt_vid_out_mplane	= bps_g_fmt_src,
	.vidioc_try_fmt_vid_cap_mplane	= bps_g_fmt_dst,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
};

/* ============================================================
 * V4L2 M2M Callbacks
 * ============================================================ */

static void bps_job_abort(void *priv)
{
	struct bps_ctx *ctx = priv;

	if (ctx->icp_ctx)
		icp_ctx_abort(ctx->icp_ctx);
}

static void bps_frame_done(struct icp_context *icp_ctx, int status)
{
	struct bps_ctx *ctx = icp_ctx->priv;
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (status) {
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
	} else {
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	}

	v4l2_m2m_job_finish(ctx->bps->m2m_dev, ctx->fh.m2m_ctx);
}

static void bps_device_run(void *priv)
{
	struct bps_ctx *ctx = priv;
	struct vb2_v4l2_buffer *src, *dst;
	struct icp_frame_request req;
	dma_addr_t input_iova, output_iova;
	u32 input_size, output_size;
	u32 input_stride, output_stride;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	input_iova = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	input_size = vb2_plane_size(&src->vb2_buf, 0);
	input_stride = ctx->width * 2;	/* 10-bit packed = ~2 bytes/pixel */

	output_iova = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	output_size = vb2_plane_size(&dst->vb2_buf, 0);
	output_stride = ctx->width;	/* NV12 Y plane stride */

	/* Build frame command with IQ settings */
	ret = bps_build_frame_cmd(ctx->cmd_bufs,
				  ctx->frame_id++,
				  ctx->width, ctx->height,
				  ctx->bayer_pattern,
				  input_iova, input_stride, input_size,
				  output_iova, output_stride, output_size,
				  &ctx->iq);
	if (ret) {
		bps_frame_done(ctx->icp_ctx, -EIO);
		return;
	}

	/* Submit to ICP */
	req.input_iova = input_iova;
	req.input_size = input_size;
	req.output_iova = output_iova;
	req.output_size = output_size;
	req.cmdbufs_iova = bps_get_cmd_iova(ctx->cmd_bufs);
	req.cmdbufs_size = bps_get_cmd_size(ctx->cmd_bufs);
	req.priv = ctx;

	if (icp_ctx_submit_frame(ctx->icp_ctx, &req))
		bps_frame_done(ctx->icp_ctx, -EIO);
}

static const struct v4l2_m2m_ops bps_m2m_ops = {
	.device_run = bps_device_run,
	.job_abort = bps_job_abort,
};

/* ============================================================
 * V4L2 Queue Operations
 * ============================================================ */

static int bps_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct bps_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		/* Raw input */
		if (*nplanes) {
			if (*nplanes != 1)
				return -EINVAL;
			return 0;
		}
		*nplanes = 1;
		sizes[0] = ctx->width * ctx->height * 2;
	} else {
		/* NV12 output */
		if (*nplanes) {
			if (*nplanes != 2)
				return -EINVAL;
			return 0;
		}
		*nplanes = 2;
		sizes[0] = ctx->width * ctx->height;
		sizes[1] = ctx->width * ctx->height / 2;
	}

	return 0;
}

static int bps_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

static void bps_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct bps_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int bps_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct bps_ctx *ctx = vb2_get_drv_priv(vq);
	struct bps_device *bps = ctx->bps;
	int ret;

	/* Allocate command buffers */
	ctx->cmd_bufs = bps_cmd_bufs_alloc(bps->dev);
	if (!ctx->cmd_bufs)
		return -ENOMEM;

	/* Initialize default IQ (passthrough demosaic) */
	bps_iq_init_passthrough(&ctx->iq);
	ctx->frame_id = 0;

	/* Power on BPS (BPS manages its own power) */
	ret = bps_power_on(bps);
	if (ret)
		goto err_bufs;

	/*
	 * Create ICP context - ICP boots itself automatically
	 * on first context creation
	 */
	ctx->icp_ctx = icp_ctx_create(bps->icp, HFI_DEV_TYPE_BPS,
				      bps_frame_done, ctx);
	if (IS_ERR(ctx->icp_ctx)) {
		ret = PTR_ERR(ctx->icp_ctx);
		ctx->icp_ctx = NULL;
		goto err_power;
	}

	return 0;

err_power:
	bps_power_off(bps);
err_bufs:
	bps_cmd_bufs_free(bps->dev, ctx->cmd_bufs);
	ctx->cmd_bufs = NULL;
	return ret;
}

static void bps_stop_streaming(struct vb2_queue *vq)
{
	struct bps_ctx *ctx = vb2_get_drv_priv(vq);
	struct bps_device *bps = ctx->bps;
	struct vb2_v4l2_buffer *vbuf;

	/* Return all buffers */
	while ((vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	while ((vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

	/*
	 * Destroy ICP context - ICP shuts itself down automatically
	 * when last context is destroyed
	 */
	if (ctx->icp_ctx) {
		icp_ctx_destroy(ctx->icp_ctx);
		ctx->icp_ctx = NULL;
	}

	bps_power_off(bps);

	/* Free command buffers */
	if (ctx->cmd_bufs) {
		bps_cmd_bufs_free(bps->dev, ctx->cmd_bufs);
		ctx->cmd_bufs = NULL;
	}
}

static const struct vb2_ops bps_vb2_ops = {
	.queue_setup = bps_queue_setup,
	.buf_prepare = bps_buf_prepare,
	.buf_queue = bps_buf_queue,
	.start_streaming = bps_start_streaming,
	.stop_streaming = bps_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int bps_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct bps_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &bps_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->bps->lock;
	src_vq->dev = ctx->bps->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &bps_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->bps->lock;
	dst_vq->dev = ctx->bps->dev;

	return vb2_queue_init(dst_vq);
}

/* ============================================================
 * File Operations
 * ============================================================ */

static int bps_open(struct file *file)
{
	struct bps_device *bps = video_drvdata(file);
	struct bps_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->bps = bps;
	ctx->width = 4096;
	ctx->height = 3072;
	ctx->src_fmt = V4L2_PIX_FMT_SRGGB10;
	ctx->bayer_pattern = HFI_BAYER_RGGB;  /* Default, updated by s_fmt */

	v4l2_fh_init(&ctx->fh, &bps->vdev);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(bps->m2m_dev, ctx, bps_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		int ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}

	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh, file);

	return 0;
}

static int bps_release(struct file *file)
{
	struct bps_ctx *ctx = container_of(file->private_data, struct bps_ctx, fh);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations bps_fops = {
	.owner = THIS_MODULE,
	.open = bps_open,
	.release = bps_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ============================================================
 * Platform Driver
 * ============================================================ */

static int camss_bps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bps_device *bps;
	int ret;

	bps = devm_kzalloc(dev, sizeof(*bps), GFP_KERNEL);
	if (!bps)
		return -ENOMEM;

	bps->dev = dev;
	platform_set_drvdata(pdev, bps);
	mutex_init(&bps->lock);

	/* Map registers */
	bps->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bps->base))
		return PTR_ERR(bps->base);

	/* Get clocks */
	bps->clocks[0].id = "ahb";
	bps->clocks[1].id = "fast_ahb";
	bps->clocks[2].id = "core";
	bps->clocks[3].id = "cpas";
	bps->num_clocks = BPS_CLK_MAX;

	ret = devm_clk_bulk_get(dev, bps->num_clocks, bps->clocks);
	if (ret)
		return ret;

	/* Get interconnect */
	bps->icc_mem = devm_of_icc_get(dev, "mem");
	if (IS_ERR(bps->icc_mem))
		return PTR_ERR(bps->icc_mem);

	/* Get UBWC config */
	of_property_read_u32(dev->of_node, "ubwc-fetch-cfg", &bps->ubwc.fetch_cfg);
	of_property_read_u32(dev->of_node, "ubwc-write-cfg", &bps->ubwc.write_cfg);

	/* Get ICP reference via phandle */
	{
		struct device_node *icp_np;
		struct platform_device *icp_pdev;

		icp_np = of_parse_phandle(dev->of_node, "qcom,icp", 0);
		if (!icp_np) {
			dev_err(dev, "missing qcom,icp phandle\n");
			return -ENODEV;
		}

		icp_pdev = of_find_device_by_node(icp_np);
		of_node_put(icp_np);

		if (!icp_pdev)
			return -EPROBE_DEFER;

		bps->icp = platform_get_drvdata(icp_pdev);
		if (!bps->icp) {
			put_device(&icp_pdev->dev);
			return -EPROBE_DEFER;
		}

		/* Keep reference to ICP device */
		get_device(bps->icp->dev);
		put_device(&icp_pdev->dev);
	}

	/* Set UBWC config in ICP */
	icp_set_ubwc_config(bps->icp, HFI_DEV_TYPE_BPS, &bps->ubwc);

	/* Register V4L2 device */
	ret = v4l2_device_register(dev, &bps->v4l2_dev);
	if (ret)
		goto err_icp;

	/* Create M2M device */
	bps->m2m_dev = v4l2_m2m_init(&bps_m2m_ops);
	if (IS_ERR(bps->m2m_dev)) {
		ret = PTR_ERR(bps->m2m_dev);
		goto err_v4l2;
	}

	/* Register video device */
	bps->vdev.fops = &bps_fops;
	bps->vdev.ioctl_ops = &bps_ioctl_ops;
	bps->vdev.release = video_device_release_empty;
	bps->vdev.v4l2_dev = &bps->v4l2_dev;
	bps->vdev.vfl_dir = VFL_DIR_M2M;
	bps->vdev.lock = &bps->lock;
	bps->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(bps->vdev.name, "qcom-bps", sizeof(bps->vdev.name));
	video_set_drvdata(&bps->vdev, bps);

	ret = video_register_device(&bps->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	pm_runtime_enable(dev);

	dev_info(dev, "BPS registered as /dev/video%d\n", bps->vdev.num);

	return 0;

err_m2m:
	v4l2_m2m_release(bps->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&bps->v4l2_dev);
err_icp:
	put_device(bps->icp->dev);
	return ret;
}

static void camss_bps_remove(struct platform_device *pdev)
{
	struct bps_device *bps = platform_get_drvdata(pdev);

	pm_runtime_disable(bps->dev);
	video_unregister_device(&bps->vdev);
	v4l2_m2m_release(bps->m2m_dev);
	v4l2_device_unregister(&bps->v4l2_dev);
	put_device(bps->icp->dev);
}

static const struct of_device_id camss_bps_dt_match[] = {
	{ .compatible = "qcom,x1e80100-camss-bps" },
	{ }
};
MODULE_DEVICE_TABLE(of, camss_bps_dt_match);

static struct platform_driver camss_bps_driver = {
	.probe = camss_bps_probe,
	.remove = camss_bps_remove,
	.driver = {
		.name = "camss-bps",
		.of_match_table = camss_bps_dt_match,
	},
};

module_platform_driver(camss_bps_driver);

MODULE_DESCRIPTION("Qualcomm CAMSS BPS V4L2 M2M driver");
MODULE_LICENSE("GPL");
