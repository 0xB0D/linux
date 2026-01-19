/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm CAMSS IPE (Image Processing Engine) Driver
 *
 * The IPE performs image post-processing controlled by ICP firmware.
 * It consists of two sub-blocks:
 * - NPS (Noise Processing Segment): Noise reduction, sharpening
 * - PPS (Post Processing Segment): Color correction, scaling
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#ifndef __CAMSS_IPE_H__
#define __CAMSS_IPE_H__

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect.h>
#include <linux/types.h>

/* Forward declaration */
struct camss_icp;

/*
 * IPE Clock Indices
 */
enum ipe_clk_id {
	IPE_CLK_NPS_AHB,
	IPE_CLK_NPS_FAST_AHB,
	IPE_CLK_PPS_FAST_AHB,
	IPE_CLK_NPS,
	IPE_CLK_PPS,
	IPE_CLK_CPAS_IPE_NPS,
	IPE_CLK_MAX,
};

/*
 * IPE Device
 */
struct camss_ipe {
	struct device *dev;
	struct camss_icp *icp;

	/* Register base (informational - ICP firmware programs registers) */
	void __iomem *base;

	/* Clocks */
	struct clk *clocks[IPE_CLK_MAX];

	/* Power domain (IPE_0_GDSC) */
	struct device *pd;
	struct device_link *pd_link;

	/* Interconnect */
	struct icc_path *icc_mem;

	/* State */
	bool powered;
	u32 clock_rate;
};

/*
 * API Functions
 */
int camss_ipe_power_on(struct camss_ipe *ipe);
void camss_ipe_power_off(struct camss_ipe *ipe);
int camss_ipe_set_clock(struct camss_ipe *ipe, u32 rate);

#endif /* __CAMSS_IPE_H__ */
