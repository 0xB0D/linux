/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm CAMSS BPS (Bayer Processing Segment) Driver
 *
 * The BPS performs Bayer demosaicing and initial image processing,
 * controlled by ICP firmware.
 *
 * Processing pipeline:
 * - Pedestal correction
 * - Linearization
 * - Black level correction
 * - Bad pixel correction
 * - Demosaicing (Bayer to RGB/YUV)
 * - Color correction
 * - Gamma correction
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#ifndef __CAMSS_BPS_H__
#define __CAMSS_BPS_H__

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect.h>
#include <linux/types.h>

/* Forward declaration */
struct camss_icp;

/*
 * BPS Clock Indices
 */
enum bps_clk_id {
	BPS_CLK_AHB,
	BPS_CLK_FAST_AHB,
	BPS_CLK_CORE,
	BPS_CLK_CPAS,
	BPS_CLK_MAX,
};

/*
 * BPS Device
 */
struct camss_bps {
	struct device *dev;
	struct camss_icp *icp;

	/* Register base (informational - ICP firmware programs registers) */
	void __iomem *base;

	/* Clocks */
	struct clk *clocks[BPS_CLK_MAX];

	/* Power domain (BPS_GDSC) */
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
int camss_bps_power_on(struct camss_bps *bps);
void camss_bps_power_off(struct camss_bps *bps);
int camss_bps_set_clock(struct camss_bps *bps, u32 rate);

#endif /* __CAMSS_BPS_H__ */
