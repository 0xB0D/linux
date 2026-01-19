// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm CAMSS IPE (Image Processing Engine) V4L2 M2M Driver
 *
 * Responsibilities:
 * - Manage IPE power and clocks (independent of ICP)
 * - Implement V4L2 mem2mem interface
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

#define IPE_NAME		"qcom-camss-ipe"
#define IPE_CLK_MAX		6

struct ipe_device {
	struct device *dev;
	void __iomem *base;

	/* IPE's own clocks */
	struct clk_bulk_data clocks[IPE_CLK_MAX];
	int num_clocks;

	/* IPE's own interconnect */
	struct icc_path *icc_mem;

	/* IPE's own power state */
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

struct ipe_ctx {
	struct v4l2_fh fh;
	struct ipe_device *ipe;
	struct icp_context *icp_ctx;

	/* Format info */
	u32 width;
	u32 height;
	u32 src_fmt;
	u32 dst_fmt;
};

/* ============================================================
 * IPE Power Management (independent of ICP)
 * ============================================================ */

static int ipe_power_on(struct ipe_device *ipe)
{
	int ret;

	if (ipe->powered)
		return 0;

	ret = pm_runtime_resume_and_get(ipe->dev);
	if (ret)
		return ret;

	ret = icc_set_bw(ipe->icc_mem, 0, MBps_to_icc(8000));
	if (ret)
		goto err_rpm;

	/* Set clock rates */
	clk_set_rate(ipe->clocks[3].clk, 700000000);  /* nps */
	clk_set_rate(ipe->clocks[4].clk, 700000000);  /* pps */

	ret = clk_bulk_prepare_enable(ipe->num_clocks, ipe->clocks);
	if (ret)
		goto err_icc;

	ipe->powered = true;
	dev_dbg(ipe->dev, "IPE powered on\n");
	return 0;

err_icc:
	icc_set_bw(ipe->icc_mem, 0, 0);
err_rpm:
	pm_runtime_put(ipe->dev);
	return ret;
}

static void ipe_power_off(struct ipe_device *ipe)
{
	if (!ipe->powered)
		return;

	clk_bulk_disable_unprepare(ipe->num_clocks, ipe->clocks);
	icc_set_bw(ipe->icc_mem, 0, 0);
	pm_runtime_put(ipe->dev);

	ipe->powered = false;
	dev_dbg(ipe->dev, "IPE powered off\n");
}

/* ============================================================
 * V4L2 Operations
 * ============================================================ */

static int ipe_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strscpy(cap->driver, IPE_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm IPE", sizeof(cap->card));

	return 0;
}

static int ipe_enum_fmt(struct file *file, void *priv,
			struct v4l2_fmtdesc *f)
{
	/* Support NV12 for now */
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int ipe_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ipe_ctx *ctx = container_of(priv, struct ipe_ctx, fh);

	f->fmt.pix_mp.width = ctx->width ?: 1920;
	f->fmt.pix_mp.height = ctx->height ?: 1080;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	f->fmt.pix_mp.num_planes = 2;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;
	f->fmt.pix_mp.plane_fmt[0].sizeimage = f->fmt.pix_mp.width * f->fmt.pix_mp.height;
	f->fmt.pix_mp.plane_fmt[1].bytesperline = f->fmt.pix_mp.width;
	f->fmt.pix_mp.plane_fmt[1].sizeimage = f->fmt.pix_mp.width * f->fmt.pix_mp.height / 2;

	return 0;
}

static int ipe_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ipe_ctx *ctx = container_of(priv, struct ipe_ctx, fh);

	ctx->width = f->fmt.pix_mp.width;
	ctx->height = f->fmt.pix_mp.height;

	return ipe_g_fmt(file, priv, f);
}

static const struct v4l2_ioctl_ops ipe_ioctl_ops = {
	.vidioc_querycap		= ipe_querycap,
	.vidioc_enum_fmt_vid_cap	= ipe_enum_fmt,
	.vidioc_enum_fmt_vid_out	= ipe_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane	= ipe_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= ipe_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= ipe_s_fmt,
	.vidioc_s_fmt_vid_out_mplane	= ipe_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= ipe_g_fmt,
	.vidioc_try_fmt_vid_out_mplane	= ipe_g_fmt,

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

static void ipe_job_abort(void *priv)
{
	struct ipe_ctx *ctx = priv;

	if (ctx->icp_ctx)
		icp_ctx_abort(ctx->icp_ctx);
}

static void ipe_frame_done(struct icp_context *icp_ctx, int status)
{
	struct ipe_ctx *ctx = icp_ctx->priv;
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

	v4l2_m2m_job_finish(ctx->ipe->m2m_dev, ctx->fh.m2m_ctx);
}

static void ipe_device_run(void *priv)
{
	struct ipe_ctx *ctx = priv;
	struct vb2_v4l2_buffer *src, *dst;
	struct icp_frame_request req;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	req.input_iova = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	req.input_size = vb2_plane_size(&src->vb2_buf, 0);
	req.output_iova = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	req.output_size = vb2_plane_size(&dst->vb2_buf, 0);
	req.cmdbufs_iova = 0;  /* Would be set up properly */
	req.cmdbufs_size = 0;
	req.priv = ctx;

	if (icp_ctx_submit_frame(ctx->icp_ctx, &req))
		ipe_frame_done(ctx->icp_ctx, -EIO);
}

static const struct v4l2_m2m_ops ipe_m2m_ops = {
	.device_run = ipe_device_run,
	.job_abort = ipe_job_abort,
};

/* ============================================================
 * V4L2 Queue Operations
 * ============================================================ */

static int ipe_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct ipe_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int size = ctx->width * ctx->height * 3 / 2;

	if (*nplanes) {
		if (*nplanes != 2)
			return -EINVAL;
		return 0;
	}

	*nplanes = 2;
	sizes[0] = ctx->width * ctx->height;
	sizes[1] = size - sizes[0];

	return 0;
}

static int ipe_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

static void ipe_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ipe_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int ipe_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ipe_ctx *ctx = vb2_get_drv_priv(vq);
	struct ipe_device *ipe = ctx->ipe;
	int ret;

	/* Power on IPE (IPE manages its own power) */
	ret = ipe_power_on(ipe);
	if (ret)
		return ret;

	/*
	 * Create ICP context - ICP boots itself automatically
	 * on first context creation
	 */
	ctx->icp_ctx = icp_ctx_create(ipe->icp, HFI_DEV_TYPE_IPE,
				      ipe_frame_done, ctx);
	if (IS_ERR(ctx->icp_ctx)) {
		ret = PTR_ERR(ctx->icp_ctx);
		ctx->icp_ctx = NULL;
		ipe_power_off(ipe);
		return ret;
	}

	return 0;
}

static void ipe_stop_streaming(struct vb2_queue *vq)
{
	struct ipe_ctx *ctx = vb2_get_drv_priv(vq);
	struct ipe_device *ipe = ctx->ipe;
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

	ipe_power_off(ipe);
}

static const struct vb2_ops ipe_vb2_ops = {
	.queue_setup = ipe_queue_setup,
	.buf_prepare = ipe_buf_prepare,
	.buf_queue = ipe_buf_queue,
	.start_streaming = ipe_start_streaming,
	.stop_streaming = ipe_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int ipe_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct ipe_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	src_vq->ops = &ipe_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->ipe->lock;
	src_vq->dev = ctx->ipe->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vb2_v4l2_buffer);
	dst_vq->ops = &ipe_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->ipe->lock;
	dst_vq->dev = ctx->ipe->dev;

	return vb2_queue_init(dst_vq);
}

/* ============================================================
 * File Operations
 * ============================================================ */

static int ipe_open(struct file *file)
{
	struct ipe_device *ipe = video_drvdata(file);
	struct ipe_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->ipe = ipe;
	ctx->width = 1920;
	ctx->height = 1080;

	v4l2_fh_init(&ctx->fh, &ipe->vdev);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(ipe->m2m_dev, ctx, ipe_queue_init);
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

static int ipe_release(struct file *file)
{
	struct ipe_ctx *ctx = container_of(file->private_data, struct ipe_ctx, fh);

	v4l2_fh_del(&ctx->fh, file);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations ipe_fops = {
	.owner = THIS_MODULE,
	.open = ipe_open,
	.release = ipe_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ============================================================
 * Platform Driver
 * ============================================================ */

static int camss_ipe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipe_device *ipe;
	int ret;

	ipe = devm_kzalloc(dev, sizeof(*ipe), GFP_KERNEL);
	if (!ipe)
		return -ENOMEM;

	ipe->dev = dev;
	platform_set_drvdata(pdev, ipe);
	mutex_init(&ipe->lock);

	/* Map registers */
	ipe->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ipe->base))
		return PTR_ERR(ipe->base);

	/* Get clocks */
	ipe->clocks[0].id = "ahb";
	ipe->clocks[1].id = "nps_fast_ahb";
	ipe->clocks[2].id = "pps_fast_ahb";
	ipe->clocks[3].id = "nps";
	ipe->clocks[4].id = "pps";
	ipe->clocks[5].id = "cpas";
	ipe->num_clocks = IPE_CLK_MAX;

	ret = devm_clk_bulk_get(dev, ipe->num_clocks, ipe->clocks);
	if (ret)
		return ret;

	/* Get interconnect */
	ipe->icc_mem = devm_of_icc_get(dev, "mem");
	if (IS_ERR(ipe->icc_mem))
		return PTR_ERR(ipe->icc_mem);

	/* Get UBWC config */
	of_property_read_u32(dev->of_node, "ubwc-fetch-cfg", &ipe->ubwc.fetch_cfg);
	of_property_read_u32(dev->of_node, "ubwc-write-cfg", &ipe->ubwc.write_cfg);

	/* Get ICP reference */
	ipe->icp = icp_get(dev);
	if (IS_ERR(ipe->icp))
		return PTR_ERR(ipe->icp);

	/* Set UBWC config in ICP */
	icp_set_ubwc_config(ipe->icp, HFI_DEV_TYPE_IPE, &ipe->ubwc);

	/* Register V4L2 device */
	ret = v4l2_device_register(dev, &ipe->v4l2_dev);
	if (ret)
		goto err_icp;

	/* Create M2M device */
	ipe->m2m_dev = v4l2_m2m_init(&ipe_m2m_ops);
	if (IS_ERR(ipe->m2m_dev)) {
		ret = PTR_ERR(ipe->m2m_dev);
		goto err_v4l2;
	}

	/* Register video device */
	ipe->vdev.fops = &ipe_fops;
	ipe->vdev.ioctl_ops = &ipe_ioctl_ops;
	ipe->vdev.release = video_device_release_empty;
	ipe->vdev.v4l2_dev = &ipe->v4l2_dev;
	ipe->vdev.vfl_dir = VFL_DIR_M2M;
	ipe->vdev.lock = &ipe->lock;
	ipe->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	strscpy(ipe->vdev.name, "qcom-ipe", sizeof(ipe->vdev.name));
	video_set_drvdata(&ipe->vdev, ipe);

	ret = video_register_device(&ipe->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	pm_runtime_enable(dev);

	dev_info(dev, "IPE registered as /dev/video%d\n", ipe->vdev.num);

	return 0;

err_m2m:
	v4l2_m2m_release(ipe->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&ipe->v4l2_dev);
err_icp:
	icp_put(ipe->icp);
	return ret;
}

static void camss_ipe_remove(struct platform_device *pdev)
{
	struct ipe_device *ipe = platform_get_drvdata(pdev);

	pm_runtime_disable(ipe->dev);
	video_unregister_device(&ipe->vdev);
	v4l2_m2m_release(ipe->m2m_dev);
	v4l2_device_unregister(&ipe->v4l2_dev);
	icp_put(ipe->icp);
}

static const struct of_device_id camss_ipe_dt_match[] = {
	{ .compatible = "qcom,x1e80100-camss-ipe" },
	{ }
};
MODULE_DEVICE_TABLE(of, camss_ipe_dt_match);

static struct platform_driver camss_ipe_driver = {
	.probe = camss_ipe_probe,
	.remove = camss_ipe_remove,
	.driver = {
		.name = "camss-ipe",
		.of_match_table = camss_ipe_dt_match,
	},
};

module_platform_driver(camss_ipe_driver);

MODULE_DESCRIPTION("Qualcomm CAMSS IPE V4L2 M2M driver");
MODULE_LICENSE("GPL");
