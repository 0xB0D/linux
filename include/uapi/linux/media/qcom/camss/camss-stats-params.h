/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Qualcomm CAMSS ISP statistics and parameters UAPI.
 *
 * Uses the generic V4L2 extensible ISP parameters buffer format
 * defined in <uapi/linux/media/v4l2-isp.h>.
 *
 * This header defines the userspace ABI for the Qualcomm CAMSS
 * subsystem in two directions:
 *
 *   - Parameters (userspace -> kernel): IQ tuning blocks and
 *     stats-engine configuration, carried as extensible per-block
 *     entries in a V4L2 params buffer. See
 *     &enum camss_params_block_type below.
 *
 *   - Statistics output (kernel -> userspace): hardware-written
 *     memory layouts for each stats engine, surfaced via V4L2
 *     meta-capture nodes. See the "Stats outputs" section.
 *
 * CAMSS has two distinct conceptual modes of operation Inline and Offline.
 *
 * Inline:
 *
 *	Inline processing mode operates on data that has been or is in the
 *	process of being delivered by a sensor into DRAM.
 *	This is the fastest and most efficient method of delivering YUV
 *	data to DRAM. The Inline Front Ends (IFEs) contain Raw Data
 *	Interfaces (RDIs) and Pixel Paths (PIX). The PIX path is capable of
 *	generating bayer statistics, applying given parameters, generating
 *	YUV output, as well as various different types of downscaling.
 *
 * Offline:
 *
 *	Offline processing engines do not operate on sensor data they are
 *	memory-to-memory devices. A firmware running on a co-processor
 *	called the Image Control Processor (ICP) implements a message
 *	passing system called the Host Firmware Interface (HFI). This HFI
 *	provides an abstraction over two further blocks the Bayer
 *	Processing Segment (BPS) and Image Processing Engine (IPE).
 *	The ICP/BPS/IPE nexus provides a session context which in turn
 *	allows the ICP firmware to implement things like temporal
 *	denoising.
 *
 *	A second type of offline processing is the Thin Front End (TFE)
 *	paired with the Offline Processing Engine (OPE). In this case the
 *	TFE can produce minimal bayer stats with raw Bayer data into
 *	memory. The OPE operates as a memory-to-memory device without an
 *	ICP. The OPE can be viewed as a reduced amalgam of the BPS and IPE
 *	above.
 *
 * Design principles:
 *
 *   - Fields carry their Q-format and bit-width inline in the kdoc,
 *     matching what the hardware consumes natively, so userspace
 *     can produce values at the right precision.
 *
 *   - The struct layout is not a register-level layout. Register
 *     packing, Camera Data Mover (CDM) program composition can be
 *     specified in any order and is up to the kernel to decide.
 *
 * Future work:
 *
 *   IPE advanced-processing blocks:
 *
 *     - ASF (Adaptive Sharpening Filter) — expected to split by
 *       sub-function (config / gain LUTs / soft-threshold LUTs /
 *       skin detection).
 *     - ANR (Advanced Noise Reduction) — multi-pass pyramid
 *       denoiser, IPE-only.
 *     - TF (Temporal Filter) — needs reference-frame management
 *       across frames; depends on ICP firmware for state.
 *     - ICA (Image Correction Algorithm) — grid-based geometric
 *       warp, two instances on IPE (pre and post).
 *     - CAC (Chromatic Aberration Correction) — IPE-only.
 *     - HNR / LENR — noise reduction variants. HNR placement
 *       varies across SoC generations (BPS on older, IPE on
 *       newer).
 *     - Skin-colour enhancement, grain adder, HDR10 display block,
 *       DSX / VSE / upscale chains — all IPE-only.
 *     - PDPC (PDAF + bad-pixel correction)
 *     - GIC (green-imbalance correction)
 *     - LCAC (local CAC)
 *     - HDR reconstruction variants (v22, v23, v30)
 *     - Bayer-domain GTM
 *
 *   Further work is required to tie-down the formats for IFE/BPS
 *   components:
 *
 *     - STATS_AWB_BFW — Bayer Focus Window stats for AWB
 *     - STATS_SPARSE_PD — sparse PDAF pixel extraction config
 *     - STATS_PDAF_V2 — PDAF v2.0 data and SAD
 *
 * Copyright (c) 2026 Linaro Ltd.
 * Author: Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 */

#ifndef _UAPI_LINUX_CAMSS_STATS_PARAMS_H
#define _UAPI_LINUX_CAMSS_STATS_PARAMS_H

#include <linux/types.h>
#include <linux/media/v4l2-isp.h>
#include <linux/videodev2.h>

/**
 * enum camss_params_block_type - CAMSS ISP parameter block identifiers
 *
 * Each value identifies one ISP processing block. The value is
 * placed in the @type field of &struct v4l2_isp_params_block_header.
 *
 * Reserved fields are added to allow for future additions say in the pre
 * demosiac phase.
 */
enum camss_params_block_type {
	/* Raw Bayer / pre-demosaic (IFE + BPS) */
	CAMSS_PARAMS_BLS               = 1,
	CAMSS_PARAMS_LINEARISATION     = 2,
	CAMSS_PARAMS_DEMUX             = 3,
	CAMSS_PARAMS_BINCORRECT        = 4,
	CAMSS_PARAMS_ABF_FILTER        = 5,
	CAMSS_PARAMS_ABF_NOISE_LUT     = 6,
	CAMSS_PARAMS_ABF_ACTIVITY_LUT  = 7,
	CAMSS_PARAMS_ABF_DARK_LUT      = 8,
	CAMSS_PARAMS_ABF_RNR           = 9,

	/* Demosaic / post-demosaic RGB (IFE + BPS) */
	CAMSS_PARAMS_WB_GAIN           = 20,
	CAMSS_PARAMS_LSC               = 21,
	CAMSS_PARAMS_DEMOSAIC          = 22,

	/*
	 * Present on IFE, BPS and IPE.
	 */
	CAMSS_PARAMS_COLOR_CORRECT     = 40, /* CCM matrix */
	CAMSS_PARAMS_COLOR_XFORM       = 41,
	CAMSS_PARAMS_GLUT              = 42, /* gamma LUT */
	CAMSS_PARAMS_GTM               = 43,
	CAMSS_PARAMS_UVG               = 44, /* UV gain */

	/* LTM (Local Tone Mapping).
	 *
	 * On newer SoCs there is a Bayer-domain LTM instance on IFE
	 * and BPS, and a YUV-domain LTM on IPE. The tuning-parameter
	 * shape is the same in both cases (the algorithm is shared;
	 * only the pipeline stage and pixel-domain differ).
	 *
	 * Split by update cadence so userspace re-uploads only the
	 * parts that changed.
	 */
	CAMSS_PARAMS_LTM_CONFIG        = 60,
	CAMSS_PARAMS_LTM_CURVES        = 61,
	CAMSS_PARAMS_LTM_GAMMA         = 62,
	CAMSS_PARAMS_LTM_MASK_FILTER   = 63,

	/* 64–99 reserved for IPE advanced processing (future work):
	 * ASF sub-blocks, ANR, TF, ICA, CAC, HNR, LENR, SCE, UPSCALE,
	 * GRA, HDR10, DSX, VSE, 2DLUT, ...
	 */

	CAMSS_PARAMS_MAX,
};

/**
 * enum camss_stats_block_type - CAMSS ISP statistics block identifiers
 *
 * Each value identifies one ISP processing block. The value is
 * placed in the @type field of &struct v4l2_isp_params_block_header.
 *
 */
enum camss_stats_block_type {
	/* Stats blocks */
	CAMSS_STATS_BG          = 1, /* AWB / HDR-BE / Tintless via @usage */
	CAMSS_STATS_BHIST       = 2, /* AEC / GTM via @usage */
	CAMSS_STATS_HDR_BHIST   = 3,
	CAMSS_STATS_IHIST       = 4, /* IFE only */
	CAMSS_STATS_RS          = 5, /* IFE only — flicker detect */
	CAMSS_STATS_BAF         = 6, /* IFE only — focus stats */
	CAMSS_STATS_AWB_BFW     = 7, /* Bayer Focus Window for AWB */
	CAMSS_STATS_SPARSE_PD   = 8, /* Sparse PDAF pixel extract */
	CAMSS_STATS_PDAF_V2     = 9, /* PDAF v2.0 data + SAD */
	CAMSS_STATS_MAX,
};

/* Maximum number of regions the gridded BG/BE engines can produce
 * per frame (horizontal_num_max * vertical_num_max).
 */
#define CAMSS_STATS_BG_MAX_REGIONS   (64 * 48)

/**
 * struct camss_stats_bg_region - Per-region BG output, regular mode
 *
 * Hardware layout: 4 x 32-bit words per region = 16 bytes.
 *   Word 0: RSum (bits 29:0), reserved (31:30)
 *   Word 1: BSum (bits 29:0), reserved (31:30)
 *   Word 2: GrSum (bits 29:0), reserved (31:30)
 *   Word 3: GbSum (bits 29:0), reserved (31:30)
 *   Word 4: RCnt (bits 15:0), BCnt (bits 31:16)
 *   Word 5: GrCnt (bits 15:0), GbCnt (bits 31:16)
 *
 * Note: the layout actually uses 6 32-bit words (24 bytes). Some
 * variants pack differently; see the *_sat and *_ystats variants.
 *
 * Sums accumulate per-channel pixel intensities; counts track how
 * many pixels contributed to each sum (pixels excluded by channel
 * gain thresholds are not counted).
 *
 * @r_sum:      R channel sum (30u)
 * @_reserved0: reserved, hardware-zero
 * @b_sum:      B channel sum (30u)
 * @_reserved1: reserved, hardware-zero
 * @gr_sum:     Gr channel sum (30u)
 * @_reserved2: reserved, hardware-zero
 * @gb_sum:     Gb channel sum (30u)
 * @_reserved3: reserved, hardware-zero
 * @r_count:    count of R pixels contributing to @r_sum (16u)
 * @b_count:    count of B pixels (16u)
 * @gr_count:   count of Gr pixels (16u)
 * @gb_count:   count of Gb pixels (16u)
 */
struct camss_stats_bg_region {
	__u32 r_sum       : 30;
	__u32 _reserved0  :  2;
	__u32 b_sum       : 30;
	__u32 _reserved1  :  2;
	__u32 gr_sum      : 30;
	__u32 _reserved2  :  2;
	__u32 gb_sum      : 30;
	__u32 _reserved3  :  2;
	__u32 r_count     : 16;
	__u32 b_count     : 16;
	__u32 gr_count    : 16;
	__u32 gb_count    : 16;
} __attribute__((packed));

/**
 * struct camss_stats_bg_region_sat - Per-region BG output, saturation mode
 *
 * Extended layout used when the matching params block specifies
 * @output_mode = CAMSS_STATS_BG_MODE_SATURATION. Carries a second
 * set of sums and counts, this time for pixels that saturated
 * (hit the gain threshold). Useful for AWB to avoid bias from
 * blown highlights.
 *
 * @r_sum..@gb_count:       regular-pixel accumulators (as in
 *                          &struct camss_stats_bg_region)
 * @sat_r_sum:              saturated R pixels sum (30u)
 * @_reserved4:             reserved
 * @sat_b_sum:              saturated B pixels sum (30u)
 * @_reserved5:             reserved
 * @sat_gr_sum:             saturated Gr pixels sum (30u)
 * @_reserved6:             reserved
 * @sat_gb_sum:             saturated Gb pixels sum (30u)
 * @_reserved7:             reserved
 * @sat_r_count:            saturated R pixel count (16u)
 * @sat_b_count:            saturated B pixel count (16u)
 * @sat_gr_count:           saturated Gr pixel count (16u)
 * @sat_gb_count:           saturated Gb pixel count (16u)
 */
struct camss_stats_bg_region_sat {
	__u32 r_sum       : 30;
	__u32 _reserved0  :  2;
	__u32 b_sum       : 30;
	__u32 _reserved1  :  2;
	__u32 gr_sum      : 30;
	__u32 _reserved2  :  2;
	__u32 gb_sum      : 30;
	__u32 _reserved3  :  2;
	__u32 r_count     : 16;
	__u32 b_count     : 16;
	__u32 gr_count    : 16;
	__u32 gb_count    : 16;
	__u32 sat_r_sum   : 30;
	__u32 _reserved4  :  2;
	__u32 sat_b_sum   : 30;
	__u32 _reserved5  :  2;
	__u32 sat_gr_sum  : 30;
	__u32 _reserved6  :  2;
	__u32 sat_gb_sum  : 30;
	__u32 _reserved7  :  2;
	__u32 sat_r_count : 16;
	__u32 sat_b_count : 16;
	__u32 sat_gr_count: 16;
	__u32 sat_gb_count: 16;
} __attribute__((packed));

/**
 * struct camss_stats_bg_region_ystats - Per-region BG output, Y-stats mode
 *
 * Layout used when @output_mode = CAMSS_STATS_BG_MODE_Y_STATS in
 * the matching params block. Instead of per-channel Gr/Gb
 * separation, the hardware accumulates a unified G channel and
 * computes a Y (luma) value using the R/G/B weights from the
 * params block.
 *
 * @r_sum:      R channel sum (30u)
 * @_reserved0: reserved
 * @b_sum:      B channel sum (30u)
 * @_reserved1: reserved
 * @g_sum:      G channel sum (30u; unified Gr + Gb)
 * @_reserved2: reserved
 * @y_sum:      Y (luma) sum (31u)
 * @_reserved3: reserved
 * @r_count:    R pixel count (16u)
 * @b_count:    B pixel count (16u)
 * @g_count:    G pixel count (16u)
 * @y_count:    Y pixel count (16u)
 */
struct camss_stats_bg_region_ystats {
	__u32 r_sum       : 30;
	__u32 _reserved0  :  2;
	__u32 b_sum       : 30;
	__u32 _reserved1  :  2;
	__u32 g_sum       : 30;
	__u32 _reserved2  :  2;
	__u32 y_sum       : 31;
	__u32 _reserved3  :  1;
	__u32 r_count     : 16;
	__u32 b_count     : 16;
	__u32 g_count     : 16;
	__u32 y_count     : 16;
} __attribute__((packed));

#define CAMSS_STATS_BHIST_BINS  1024

/**
 * struct camss_stats_bhist_bin - One histogram bin
 *
 * @count:     per-bin pixel count (25u)
 * @_reserved: reserved, hardware-zero (7u)
 */
struct camss_stats_bhist_bin {
	__u32 count     : 25;
	__u32 _reserved :  7;
} __attribute__((packed));

/**
 * struct camss_stats_bhist_output - BHist stats buffer
 *
 * Hardware availability: IFE, BPS.
 *
 * Single-channel Bayer histogram. The channel is selected by the
 * matching &struct camss_stats_bhist (@channel field);
 * see &enum camss_stats_color_channel.
 *
 * Bit-width: 25-bit per-bin counts, leaving headroom for the
 * maximum possible pixel count at full-frame BG region coverage.
 *
 * @bins: 1024 per-bin entries, each with a 25-bit count
 */
struct camss_stats_bhist_output {
	struct camss_stats_bhist_bin bins[CAMSS_STATS_BHIST_BINS];
} __attribute__((packed));

#define CAMSS_STATS_HDR_BHIST_BINS  256

/**
 * struct camss_stats_hdr_bhist_bins - R/G/B triple for one HDR-BHist bin
 *
 * The HDR Bayer histogram accumulates all three colour channels
 * in parallel, one triple per bin. 12 bytes per bin (3 x 32-bit
 * words, 25 valid bits each).
 *
 * @red_bin:    R channel count (25u)
 * @_reserved0: reserved (7u)
 * @green_bin:  G channel count (25u)
 * @_reserved1: reserved (7u)
 * @blue_bin:   B channel count (25u)
 * @_reserved2: reserved (7u)
 */
struct camss_stats_hdr_bhist_bins {
	__u32 red_bin    : 25;
	__u32 _reserved0 :  7;
	__u32 green_bin  : 25;
	__u32 _reserved1 :  7;
	__u32 blue_bin   : 25;
	__u32 _reserved2 :  7;
} __attribute__((packed));

/**
 * struct camss_stats_hdr_bhist_output - HDR BHist stats buffer
 *
 * Hardware availability: IFE, BPS.
 *
 * Three-channel Bayer histogram for HDR exposure analysis. The
 * exposure field (long / short / all) is selected by the matching
 * &struct camss_stats_hdr_bhist (@field_select field).
 *
 * @bins: 256 per-bin R/G/B triples
 */
struct camss_stats_hdr_bhist_output {
	struct camss_stats_hdr_bhist_bins bins[CAMSS_STATS_HDR_BHIST_BINS];
} __attribute__((packed));

#define CAMSS_STATS_IHIST_BINS  256

/**
 * struct camss_stats_ihist_output - Image histogram stats buffer
 *
 * Hardware availability: IFE only.
 *
 * Image histogram for post-demosaic luminance / chroma analysis.
 * The active channel arrays depend on @channel in the matching
 * &struct camss_stats_ihist:
 *
 *   - CAMSS_STATS_IHIST_Y:  @ycc_histogram holds Y
 *   - CAMSS_STATS_IHIST_CB: @ycc_histogram holds Cb
 *   - CAMSS_STATS_IHIST_CR: @ycc_histogram holds Cr
 *
 * The per-channel R/G/B histograms are always populated; the YCC
 * slot is multiplexed depending on selection.
 *
 * @ycc_histogram:   Y or Cb or Cr histogram (per @channel)
 * @green_histogram: G histogram
 * @blue_histogram:  B histogram
 * @red_histogram:   R histogram
 */
struct camss_stats_ihist_output {
	__u16 ycc_histogram[CAMSS_STATS_IHIST_BINS];
	__u16 green_histogram[CAMSS_STATS_IHIST_BINS];
	__u16 blue_histogram[CAMSS_STATS_IHIST_BINS];
	__u16 red_histogram[CAMSS_STATS_IHIST_BINS];
} __attribute__((packed));

#define CAMSS_STATS_RS_MAX_REGIONS  (16 * 16)

/**
 * struct camss_stats_rs_region - One Row-Sum region
 *
 * @luma_sum: per-region luma sum (32u; field width is sufficient
 *            for the maximum full-frame region and full bit depth)
 */
struct camss_stats_rs_region {
	__u32 luma_sum;
} __attribute__((packed));

/**
 * struct camss_stats_rs_output - Row Sum stats buffer
 *
 * Hardware availability: IFE only.
 *
 * Row sums over configurable horizontal regions, used primarily
 * for 50/60 Hz flicker detection. The region count is set by the
 * matching &struct camss_stats_rs (@stats_h_num,
 * @stats_v_num); only (stats_h_num * stats_v_num) entries of
 * @regions carry valid data.
 *
 * Regions are stored in raster order: row 0 left-to-right, then
 * row 1, and so on.
 *
 * @regions: up to 16 x 16 region sums
 */
struct camss_stats_rs_output {
	struct camss_stats_rs_region regions[CAMSS_STATS_RS_MAX_REGIONS];
} __attribute__((packed));

#define CAMSS_STATS_BAF_MAX_REGIONS  180

/**
 * struct camss_stats_baf_region - One BAF per-region focus measurement
 *
 * Hardware layout: 32 bytes per region.
 *
 * Each region carries sums, counts, and sharpness measures from
 * both the horizontal and vertical filter chains. The sharpness
 * value is a 40-bit accumulator of squared filter-response
 * values — larger magnitudes indicate stronger edges in the
 * region, which is what the focus algorithm maximises.
 *
 * The @sel bit selects between two horizontal filter chains
 * (H1 and H2); the vertical chain is always active in the
 * vertical fields.
 *
 * Region layout in the buffer is not raster-ordered; instead
 * each region carries explicit @region_id and @output_id fields
 * identifying which ROI it corresponds to. The last valid entry
 * sets @end_of_buffer = 1; userspace must stop parsing there.
 *
 * @h1_sum:          horizontal filter sum (37u)
 * @_reserved0:      reserved (2u)
 * @h1_count:        horizontal pixels contributing (23u)
 * @_reserved1:      reserved (1u)
 * @sel:             H-filter selector: 0 = H1, 1 = H2 (1u)
 * @h1_sharpness:    horizontal sharpness accumulator (40u)
 * @_reserved2:      reserved (14u)
 * @region_id:       ROI index from the params block (8u)
 * @_reserved3:      reserved (2u)
 * @v_sum:           vertical filter sum (37u)
 * @_reserved4:      reserved (2u)
 * @v_count:         vertical pixels contributing (23u)
 * @_reserved5:      reserved (2u)
 * @v_sharpness:     vertical sharpness accumulator (40u)
 * @_reserved6:      reserved (14u)
 * @output_id:       output-sequence ID from the params block (8u)
 * @merge:           1 = this region spans a dual-IFE split and must
 *                   be merged with its peer region
 * @end_of_buffer:   1 = last valid entry in the buffer (1u)
 */
struct camss_stats_baf_region {
	__u64 h1_sum       : 37;
	__u64 _reserved0   :  2;
	__u64 h1_count     : 23;
	__u64 _reserved1   :  1;
	__u64 sel          :  1;

	__u64 h1_sharpness : 40;
	__u64 _reserved2   : 14;
	__u64 region_id    :  8;
	__u64 _reserved3   :  2;

	__u64 v_sum        : 37;
	__u64 _reserved4   :  2;
	__u64 v_count      : 23;
	__u64 _reserved5   :  2;

	__u64 v_sharpness  : 40;
	__u64 _reserved6   : 14;
	__u64 output_id    :  8;
	__u64 merge        :  1;
	__u64 end_of_buffer:  1;
} __attribute__((packed));

/**
 * struct camss_stats_baf_output - BAF (focus) stats buffer
 *
 * Hardware availability: IFE only.
 *
 * Focus statistics per ROI. Up to 180 ROIs can be configured via
 * the matching &struct camss_stats_baf. The output buffer
 * is sized for the worst case; the hardware terminates the valid
 * region list by setting @end_of_buffer = 1 on the last entry.
 *
 * Userspace scans @regions in order, parsing each entry until
 * @end_of_buffer is found. Values past that entry are undefined.
 *
 * @regions: up to 180 ROI measurements
 */
struct camss_stats_baf_output {
	struct camss_stats_baf_region regions[CAMSS_STATS_BAF_MAX_REGIONS];
} __attribute__((packed));

/* ===================================================================
 * Raw Bayer / pre-demosaic blocks
 *
 * Hardware availability: IFE and BPS.
 *
 * These blocks are instantiated as the same silicon IP in both
 * engines. The UAPI struct describes the tuning; the kernel
 * translator selected by the V4L2 device node programs the
 * corresponding engine.
 * =================================================================== */

/**
 * struct camss_params_bls - Black level subtraction
 *
 * Hardware availability: IFE, BPS.
 *
 * BLS subtracts a constant offset from each Bayer channel and then
 * scales to restore the dynamic range. Per-channel thresholds gate
 * the subtraction to avoid clipping dark noise.
 *
 * The scale field compensates for the offset removal:
 *     scale = (1 << PIPELINE_BITWIDTH) /
 *             ((1 << PIPELINE_BITWIDTH) - offset)
 *
 * @header:       block header; @header.type = CAMSS_PARAMS_BLS
 * @offset:       subtractive offset, identical across 4 channels (14u)
 * @scale:        gain scaling after subtraction (17u Q11)
 * @threshold_r:  threshold for R  channel
 * @threshold_gr: threshold for Gr channel
 * @threshold_gb: threshold for Gb channel
 * @threshold_b:  threshold for B  channel
 * @_pad:         reserved, must be zero
 *
 */
struct camss_params_bls {
	struct v4l2_isp_params_block_header header;
	__u16 offset;
	__u32 scale;
	__u16 threshold_r;
	__u16 threshold_gr;
	__u16 threshold_gb;
	__u16 threshold_b;
	__u16 _pad[1];
} __attribute__((aligned(8)));

#define CAMSS_LINEARISATION_KNEE_POINTS  8
#define CAMSS_LINEARISATION_SEGMENTS     9

/**
 * struct camss_params_linearisation_channel - one Bayer channel's curve
 *
 * @knee_points: x-axis breakpoints of the piecewise-linear curve
 * @base:        y-value at each segment start
 * @delta:       slope of each segment (output per input unit)
 */
struct camss_params_linearisation_channel {
	__u16 knee_points[CAMSS_LINEARISATION_KNEE_POINTS];
	__u16 base[CAMSS_LINEARISATION_SEGMENTS];
	__u32 delta[CAMSS_LINEARISATION_SEGMENTS];
} __attribute__((packed));

/**
 * struct camss_params_linearisation - Sensor response linearisation
 *
 * Hardware availability: IFE, BPS.
 *
 * Four independent piecewise-linear curves (R/Gr/Gb/B), each with
 * 8 knee points and 9 segments. Used to undo sensor companding or
 * non-linear response before downstream blocks (which assume
 * linear-light data).
 *
 * On newer SoCs this block is named CLC_COMPDECOMP (compand /
 * de-compand) in the hardware; the operation and register layout
 * are equivalent.
 *
 * The hardware uses double-buffered LUTs internally; bank
 * selection is kernel-managed and not exposed.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_LINEARISATION
 * @r:      R  channel curve
 * @gr:     Gr channel curve
 * @gb:     Gb channel curve
 * @b:      B  channel curve
 */
struct camss_params_linearisation {
	struct v4l2_isp_params_block_header header;
	struct camss_params_linearisation_channel r;
	struct camss_params_linearisation_channel gr;
	struct camss_params_linearisation_channel gb;
	struct camss_params_linearisation_channel b;
} __attribute__((aligned(8)));

/**
 * struct camss_params_demux - Channel demux and per-channel gain
 *
 * Hardware availability: IFE, BPS.
 *
 * Demultiplexes the Bayer stream into four logical channels,
 * applying a per-channel gain and a per-channel black-level offset.
 * Used to compensate for per-channel sensor-gain imbalance early
 * in the pipeline.
 *
 * Gain range approximately 0.0 .. 4.0 in 16uQ10; offsets are
 * signed and channel-specific.
 *
 * @header:    block header; @header.type = CAMSS_PARAMS_DEMUX
 * @gain_even: gain for even-column channel (16uQ10)
 * @gain_odd:  gain for odd-column channel (16uQ10)
 * @gain_gr:   Gr channel gain (16uQ10)
 * @gain_gb:   Gb channel gain (16uQ10)
 * @gain_r:    R  channel gain (16uQ10)
 * @gain_b:    B  channel gain (16uQ10)
 * @period:    blanking period for odd/even alternation
 * @blk_in:    input black level
 * @blk_out:   output black level after demux
 * @_pad:      reserved, must be zero
 */
struct camss_params_demux {
	struct v4l2_isp_params_block_header header;
	__u16 gain_even;
	__u16 gain_odd;
	__u16 gain_gr;
	__u16 gain_gb;
	__u16 gain_r;
	__u16 gain_b;
	__u16 period;
	__u16 blk_in;
	__u16 blk_out;
	__u16 _pad[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_bincorrect - Binning correction
 *
 * Hardware availability: IFE, BPS.
 *
 * Corrects pixel positions when the sensor has been configured for
 * binning (combining adjacent photosites to trade resolution for
 * sensitivity). Sensors in 2x2 binning shift the effective pixel
 * centres by half a sample; bincorrect re-aligns the Bayer grid to
 * integer positions so downstream demosaic sees correctly
 * positioned samples.
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_BINCORRECT
 * @ver_w1:  vertical width 1
 * @ver_w2:  vertical width 2
 * @hor_w1:  horizontal width 1
 * @hor_w2:  horizontal width 2
 */
struct camss_params_bincorrect {
	struct v4l2_isp_params_block_header header;
	__u16 ver_w1;
	__u16 ver_w2;
	__u16 hor_w1;
	__u16 hor_w2;
} __attribute__((aligned(8)));

/*
 * ABF (Adaptive Bilateral Filter) — split into five sub-blocks
 *
 * Hardware availability: IFE, BPS.
 *
 * ABF is the primary Bayer-stage noise reduction block. It
 * combines a bilateral filter, min/max processing, radial noise
 * reduction (RNR), activity-based modulation, dark-region
 * adjustment, and edge handling. The full configuration is
 * substantial (~80 scalar fields plus three DMI LUTs and an RNR
 * curve) and different parts update at different rates.
 *
 * Sub-block decomposition by update lifecycle:
 *
 *   FILTER       — scalar control, bilateral filter coefficients,
 *                  block-matching levels, per-channel filter
 *                  strength, min/max filter, edge handling. Can
 *                  change per frame for adaptive behaviour or with
 *                  scene/illuminant changes.
 *   NOISE_LUT    — 64-entry shot-noise LUT. Updates when analog
 *                  gain changes meaningfully (typically banded:
 *                  one LUT per analog-gain band, swapped on band
 *                  crossings).
 *   ACTIVITY_LUT — 32-entry activity-modulation LUT. Mostly static;
 *                  may update on tone-preset switch.
 *   DARK_LUT     — 42-entry dark-region adjustment LUT. Mostly
 *                  static.
 *   RNR          — radial NR curve (4 anchor points). Sensor-
 *                  geometry-derived; effectively static once
 *                  calibrated.
 *
 * Splitting lets userspace re-upload only the pieces that
 * actually changed; the noise LUT is the most frequent updater
 * (per-gain-band) while the others typically load once per
 * session.
 *
 */

#define CAMSS_ABF_NUM_CHANNELS         4
#define CAMSS_ABF_NUM_ANCHORS          4
#define CAMSS_ABF_NUM_BLOCK_PIX_LEVELS 2
#define CAMSS_ABF_NOISE_LUT_SIZE       64
#define CAMSS_ABF_ACTIVITY_LUT_SIZE    32
#define CAMSS_ABF_DARK_LUT_SIZE        42

/**
 * struct camss_params_abf_filter - ABF scalar filter configuration
 *
 * Hardware availability: IFE, BPS.
 *
 * Scalar filter fields of the ABF block. Excludes the three DMI
 * LUTs (noise, activity, dark) and the RNR curve, which are
 * separate sub-blocks.
 *
 * @header:               block header; @header.type = CAMSS_PARAMS_ABF_FILTER
 *
 * Sub-function enables:
 * @bilateral_enable:     enable bilateral filter (FILTER_EN in HW)
 * @cross_process_enable: enable cross-channel processing
 * @direct_smooth_enable: enable directional smoothing
 * @dark_desat_enable:    enable dark-region desaturation
 * @dark_smooth_enable:   enable dark-region smoothing
 * @act_adj_enable:       enable activity adjustment
 * @minmax_enable:        enable built-in min/max pixel filter
 * @block_opt:            block matching pattern selector, 0..2
 *
 * Block matching:
 * @block_pix_level:      pixel-match levels [0]=R/B, [1]=G
 * @distance_level:       distance levels [3][6] — 3 levels x
 *                        6 channels; [][0..2] for R/B, [][3..5]
 *                        for Gr/Gb
 *
 * Per-channel filter strength (channel order R, Gr, Gb, B):
 * @curve_offset:         curve offset (7u per channel)
 * @edge_softness:        edge softness (9u per channel)
 * @filter_strength:      filter strength (9u per channel)
 *
 * Min/max filter:
 * @minmax_max_shift:     MINMAX_MAXSHFT
 * @minmax_min_shift:     MINMAX_MINSHFT
 * @minmax_offset:        (bs-2)u — bs is pipeline bit-depth
 * @minmax_bls:           (bs-2)u
 *
 * Activity normalisation:
 * @activity_factor0:     normalisation factor 0
 * @activity_factor1:     normalisation factor 1
 * @activity_threshold0:  normalisation threshold 0
 * @activity_threshold1:  normalisation threshold 1
 * @activity_smooth_threshold0: smoothing-kernel threshold 0
 * @activity_smooth_threshold1: smoothing-kernel threshold 1
 *
 * Dark-region:
 * @dark_threshold:       dark normalisation threshold
 *
 * Cross-channel ratios (12u each, AWB-derived):
 * @gr_ratio:             Green-Red ratio
 * @rg_ratio:             Red-Green ratio
 * @gb_ratio:             Green-Blue ratio
 * @bg_ratio:             Blue-Green ratio
 * @rb_ratio:             Red-Blue ratio
 * @br_ratio:             Blue-Red ratio
 *
 * Edge handling:
 * @edge_detect_threshold:    edge detection threshold (4uQ2)
 * @edge_count_low:           edge-count low parameter
 * @edge_detect_noise_scalar: edge-detect noise scalar (12uQ6)
 * @edge_smooth_strength:     edge-smoothing strength (Q6)
 * @edge_smooth_noise_scalar: per-channel edge-smooth noise scalar
 *                            (12uQ8, channel order R, Gr, Gb, B)
 */
struct camss_params_abf_filter {
	struct v4l2_isp_params_block_header header;

	__u8  bilateral_enable;
	__u8  cross_process_enable;
	__u8  direct_smooth_enable;
	__u8  dark_desat_enable;
	__u8  dark_smooth_enable;
	__u8  act_adj_enable;
	__u8  minmax_enable;
	__u8  block_opt;

	__u16 block_pix_level[CAMSS_ABF_NUM_BLOCK_PIX_LEVELS];
	__u16 distance_level[3][6];

	__u16 curve_offset[CAMSS_ABF_NUM_CHANNELS];
	__u16 edge_softness[CAMSS_ABF_NUM_CHANNELS];
	__u16 filter_strength[CAMSS_ABF_NUM_CHANNELS];

	__u16 minmax_max_shift;
	__u16 minmax_min_shift;
	__u16 minmax_offset;
	__u16 minmax_bls;

	__u16 activity_factor0;
	__u16 activity_factor1;
	__u16 activity_threshold0;
	__u16 activity_threshold1;
	__u16 activity_smooth_threshold0;
	__u16 activity_smooth_threshold1;

	__u16 dark_threshold;

	__u16 gr_ratio;
	__u16 rg_ratio;
	__u16 gb_ratio;
	__u16 bg_ratio;
	__u16 rb_ratio;
	__u16 br_ratio;

	__u16 edge_detect_threshold;
	__u16 edge_count_low;
	__u16 edge_detect_noise_scalar;
	__u16 edge_smooth_strength;
	__u16 edge_smooth_noise_scalar[CAMSS_ABF_NUM_CHANNELS];
} __attribute__((aligned(8)));

/**
 * struct camss_params_abf_noise_lut - ABF shot-noise LUT
 *
 * Hardware availability: IFE, BPS.
 *
 * 64-entry 32-bit LUT representing per-luma shot-noise standard
 * deviation. Updates when analog gain changes meaningfully —
 * typically banded so a small number of LUTs are pre-computed and
 * one is uploaded on each gain-band crossing.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_ABF_NOISE_LUT
 * @lut:    shot-noise LUT
 */
struct camss_params_abf_noise_lut {
	struct v4l2_isp_params_block_header header;
	__u32 lut[CAMSS_ABF_NOISE_LUT_SIZE];
} __attribute__((aligned(8)));

/**
 * struct camss_params_abf_activity_lut - ABF activity-adjust LUT
 *
 * Hardware availability: IFE, BPS.
 *
 * 32-entry adjustment for activity-based filter modulation.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_ABF_ACTIVITY_LUT
 * @lut:    activity-adjustment LUT
 */
struct camss_params_abf_activity_lut {
	struct v4l2_isp_params_block_header header;
	__u16 lut[CAMSS_ABF_ACTIVITY_LUT_SIZE];
	__u16 _pad[2];
} __attribute__((aligned(8)));

/**
 * struct camss_params_abf_dark_lut - ABF dark-factor LUT
 *
 * Hardware availability: IFE, BPS.
 *
 * 42-entry LUT for dark-region adjustment.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_ABF_DARK_LUT
 * @lut:    dark-factor LUT
 */
struct camss_params_abf_dark_lut {
	struct v4l2_isp_params_block_header header;
	__u16 lut[CAMSS_ABF_DARK_LUT_SIZE];
	__u16 _pad[2];
} __attribute__((aligned(8)));

/**
 * struct camss_params_abf_rnr - ABF radial noise reduction curve
 *
 * Hardware availability: IFE, BPS.
 *
 * Defines a piecewise curve in r-squared (radius squared) space
 * with 4 anchor points; outside the lens optical centre the noise
 * reduction strength is modulated based on radial distance.
 *
 * The geometry fields (@bx, @by, @r_square_init) are sensor-derived
 * and could move to a kernel-only domain if a sensor-driver hook
 * supplied them; for now exposed in UAPI for direct control.
 *
 * @header:           block header; @header.type = CAMSS_PARAMS_ABF_RNR
 *
 * Curve geometry:
 * @bx:               14s — init_h_offset - h_centre
 * @by:               14s — init_v_offset - v_centre
 * @r_square_scale:   7u  — r^2 scale factor
 * @r_square_shift:   4u  — r^2 right-shift
 * @r_square_init:    28u — initial r^2 for top-left of stripe
 *
 * Per-anchor RNR curve (4 anchors):
 * @rnr_anchor:       12u anchor x-positions
 * @rnr_base0:        8u, RNR_NOISE_BASE
 * @rnr_slope0:       12u, RNR_NOISE_SLOPE
 * @rnr_shift0:       4u, RNR_NOISE_SHIFT
 * @rnr_base1:        8s, RNR_THRESH_BASE
 * @rnr_slope1:       10s, RNR_THRESH_SLOPE
 * @rnr_shift1:       4s, RNR_THRESH_SHIFT
 *
 * Noise-preservation curve (2 tables x 4 anchors):
 * @nprsv_anchor:     12u, NP_ANCHOR
 * @nprsv_base:       8u, NP_BASE [0]=R/B, [1]=Gr/Gb
 * @nprsv_shift:      4u, NP_SHIFT [0]=R/B, [1]=Gr/Gb
 * @nprsv_slope:      9s, NP_SLOPE
 */
struct camss_params_abf_rnr {
	struct v4l2_isp_params_block_header header;

	__s16 bx;
	__s16 by;
	__u16 r_square_scale;
	__u16 r_square_shift;
	__u32 r_square_init;

	__u16 rnr_anchor[CAMSS_ABF_NUM_ANCHORS];
	__u16 rnr_base0[CAMSS_ABF_NUM_ANCHORS];
	__u16 rnr_slope0[CAMSS_ABF_NUM_ANCHORS];
	__u16 rnr_shift0[CAMSS_ABF_NUM_ANCHORS];
	__s16 rnr_base1[CAMSS_ABF_NUM_ANCHORS];
	__s16 rnr_slope1[CAMSS_ABF_NUM_ANCHORS];
	__s16 rnr_shift1[CAMSS_ABF_NUM_ANCHORS];

	__u16 nprsv_anchor[CAMSS_ABF_NUM_ANCHORS];
	__u16 nprsv_base[2][CAMSS_ABF_NUM_ANCHORS];
	__u16 nprsv_shift[2][CAMSS_ABF_NUM_ANCHORS];
	__s16 nprsv_slope[2][CAMSS_ABF_NUM_ANCHORS];
} __attribute__((aligned(8)));

/* ===================================================================
 * Demosaic / post-demosaic RGB blocks
 *
 * Hardware availability: IFE and BPS.
 * =================================================================== */

/**
 * struct camss_params_wb_gain - White balance gains and offsets
 *
 * Hardware availability: IFE, BPS.
 *
 * Channel ordering follows the hardware output-channel convention:
 * channel 0 = G, 1 = B, 2 = R.
 *
 * Gains are 12uQ10 (12-bit unsigned, 10 fractional bits, range
 * approximately 0.0 .. 4.0). Offsets are signed and clamped to 13
 * or 15 bits depending on hardware variant.
 *
 * @header:    block header; @header.type = CAMSS_PARAMS_WB_GAIN
 * @g_gain:    green channel gain  (12uQ10)
 * @b_gain:    blue  channel gain  (12uQ10)
 * @r_gain:    red   channel gain  (12uQ10)
 * @g_offset:  green channel offset
 * @b_offset:  blue  channel offset
 * @r_offset:  red   channel offset
 * @_pad:      reserved, must be zero
 */
struct camss_params_wb_gain {
	struct v4l2_isp_params_block_header header;
	__u16 g_gain;
	__u16 b_gain;
	__u16 r_gain;
	__u16 g_offset;
	__u16 b_offset;
	__u16 r_offset;
	__u16 _pad[2];
} __attribute__((aligned(8)));

#define CAMSS_LSC_MESH_H  17
#define CAMSS_LSC_MESH_V  13

/**
 * struct camss_params_lsc - Lens shading correction (rolloff)
 *
 * Hardware availability: IFE, BPS.
 *
 * A 13x17 mesh of per-channel gain values (13uQ10, range
 * ~0.0 .. 8.0) for each of R/Gr/Gb/B. The hardware bicubically
 * interpolates between mesh points. Grid geometry (subgrid width
 * and height, start offsets, x/y delta reciprocals) is computed
 * by the kernel from the sensor output dimensions; userspace
 * supplies only the 4-channel mesh.
 *
 * The mesh values encode the full CCT-interpolated table for the
 * current illuminant. Userspace computes the interpolation between
 * calibrated illuminant tables (A, D50, D65, etc.) and sends the
 * result. The kernel does not maintain per-illuminant calibration
 * tables.
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_LSC
 * @mesh_r:  R  channel mesh (13uQ10)
 * @mesh_gr: Gr channel mesh
 * @mesh_gb: Gb channel mesh
 * @mesh_b:  B  channel mesh
 */
struct camss_params_lsc {
	struct v4l2_isp_params_block_header header;
	__u16 mesh_r[CAMSS_LSC_MESH_V][CAMSS_LSC_MESH_H];
	__u16 mesh_gr[CAMSS_LSC_MESH_V][CAMSS_LSC_MESH_H];
	__u16 mesh_gb[CAMSS_LSC_MESH_V][CAMSS_LSC_MESH_H];
	__u16 mesh_b[CAMSS_LSC_MESH_V][CAMSS_LSC_MESH_H];
} __attribute__((aligned(8)));

/**
 * struct camss_params_demosaic - Bayer demosaicing tuning
 *
 * Hardware availability: IFE, BPS.
 *
 * The demosaic algorithm itself is fixed in hardware; these
 * parameters tune the directional interpolation heuristics that
 * decide how to reconstruct missing colour samples at each pixel.
 *
 * @header:           block header; @header.type = CAMSS_PARAMS_DEMOSAIC
 * @cosited_rgb:      1 = cosited RGB output sampling
 * @demosaic_v4:      1 = use v4 interpolation filters, 0 = v3
 * @disable_dir_g:    disable directional G interpolation
 * @disable_dir_rb:   disable directional R/B interpolation
 * @dynamic_clamp_g:  enable G dynamic clamping
 * @dynamic_clamp_rb: enable R/B dynamic clamping
 * @_pad:             reserved, must be zero
 * @wk:               directional interpolation weight factor (signed)
 * @ak:               directional interpolation activity factor
 * @lambda_g:         lambda for G channel
 * @lambda_rb:        lambda for R/B channels
 */
struct camss_params_demosaic {
	struct v4l2_isp_params_block_header header;
	__u8  cosited_rgb;
	__u8  demosaic_v4;
	__u8  disable_dir_g;
	__u8  disable_dir_rb;
	__u8  dynamic_clamp_g;
	__u8  dynamic_clamp_rb;
	__u16 _pad;
	__s16 wk;
	__u16 ak;
	__u16 lambda_g;
	__u16 lambda_rb;
} __attribute__((aligned(8)));

/* ===================================================================
 * Tier 1 blocks — also present on IPE in future work
 *
 * These blocks are instantiated on IFE, BPS, and IPE. The UAPI
 * struct applies regardless of the engine.
 * =================================================================== */

/**
 * struct camss_params_color_correct - 3x3 CCM with per-channel offsets
 *
 * Hardware availability: IFE, BPS. (IPE: future work.)
 *
 * Matrix computation, output channel order G, B, R:
 *   Out_ch0 (G) = a[0]*G + b[0]*B + c[0]*R + k[0]
 *   Out_ch1 (B) = a[1]*G + b[1]*B + c[1]*R + k[1]
 *   Out_ch2 (R) = a[2]*G + b[2]*B + c[2]*R + k[2]
 *
 * Input channel order is G, B, R (not R, G, B) — matches the
 * hardware register layout.
 *
 * All 9 coefficients share a single Q-factor selector @qfactor,
 * which applies a right-shift to the multiplier output.
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_COLOR_CORRECT
 * @a:       G-input coefficients per output channel (12s;
 *           a[0]=Out_G, a[1]=Out_B, a[2]=Out_R)
 * @b:       B-input coefficients (12s)
 * @c:       R-input coefficients (12s)
 * @k:       per-output-channel offsets (typically 9s effective)
 * @qfactor: Q-format selector (2u):
 *               0 = Q7, 1 = Q8, 2 = Q9, 3 = Q10
 *           Applies uniformly to all 9 matrix coefficients.
 * @_pad:    reserved, must be zero
 */
struct camss_params_color_correct {
	struct v4l2_isp_params_block_header header;
	__u16 a[3];
	__u16 b[3];
	__u16 c[3];
	__u16 k[3];
	__u16 qfactor;
	__u16 _pad[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_color_xform - Colour-space transform
 *
 * Hardware availability: IFE, BPS. (IPE: future work — IPE carries
 * three instances of this block for pre/mid/post stages.)
 *
 * Applies the transform:
 *     output = M * clamp(input - C) + O
 * where C is a per-channel input clamp point, M is a 3x3 matrix,
 * and O is a per-channel output offset. S0..S2 are output
 * saturation clamps.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_COLOR_XFORM
 * @c0:     input channel-0 clamp
 * @c1:     input channel-1 clamp
 * @c2:     input channel-2 clamp
 * @c01:    input channel-0 secondary clamp
 * @c11:    input channel-1 secondary clamp
 * @c21:    input channel-2 secondary clamp
 * @m:      3x3 matrix, row-major [row][col] (12s)
 * @o:      output offsets [0..2] (signed)
 * @s:      output saturation clamps [0..2]
 * @_pad:   reserved, must be zero
 */
struct camss_params_color_xform {
	struct v4l2_isp_params_block_header header;
	__u16 c0, c1, c2;
	__u16 c01, c11, c21;
	__s16 m[3][3];
	__s16 o[3];
	__s16 s[3];
	__u16 _pad[2];
} __attribute__((aligned(8)));

#define CAMSS_GLUT_LUT_SIZE  64

/**
 * struct camss_params_glut - Per-channel gamma LUT
 *
 * Hardware availability: IFE, BPS. (IPE: future work. IPE has
 * multiple gamma instances on modern SoCs; @instance will select
 * which when IPE support lands. For IFE and BPS there is a single
 * instance per engine and @instance is reserved.)
 *
 * Three independent 64-entry LUTs for R, G, B, flattened inline.
 *
 * Entries are packed as pairs in the hardware registers but
 * exposed here as a flat array; the kernel repacks as needed.
 * Hardware double-buffer bank selectors are kernel-managed.
 *
 * @header:   block header; @header.type = CAMSS_PARAMS_GLUT
 * @instance: reserved (0 for IFE/BPS); used by IPE extension
 * @_pad:     reserved, must be zero
 * @r:        R channel gamma LUT
 * @g:        G channel gamma LUT
 * @b:        B channel gamma LUT
 */
struct camss_params_glut {
	struct v4l2_isp_params_block_header header;
	__u8  instance;
	__u8  _pad[7];
	__u16 r[CAMSS_GLUT_LUT_SIZE];
	__u16 g[CAMSS_GLUT_LUT_SIZE];
	__u16 b[CAMSS_GLUT_LUT_SIZE];
} __attribute__((aligned(8)));

#define CAMSS_GTM_LUT_SIZE  64

/**
 * struct camss_params_gtm - Global tone mapping
 *
 * Hardware availability: IFE, BPS. (IPE: future work.)
 *
 * Piecewise-linear LUT on luminance ratios, 64 segments. Each
 * segment is described by a base value and a slope.
 *
 * The y_ratio_base values are packed as sign-extended 18-bit in
 * the hardware register. 32 bits here preserves the full dynamic
 * range; the kernel packs.
 *
 * @header:        block header; @header.type = CAMSS_PARAMS_GTM
 * @y_ratio_base:  base value at start of each segment (unsigned)
 * @y_ratio_slope: slope within each segment (signed)
 */
struct camss_params_gtm {
	struct v4l2_isp_params_block_header header;
	__u32 y_ratio_base[CAMSS_GTM_LUT_SIZE];
	__s32 y_ratio_slope[CAMSS_GTM_LUT_SIZE];
} __attribute__((aligned(8)));

/**
 * struct camss_params_uvg - UV gain
 *
 * Hardware availability: IFE, BPS. (IPE: future work.)
 *
 * Simple per-channel gain on the U and V chroma components after
 * colour conversion. Used for chroma saturation control.
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_UVG
 * @u_gain:  U channel gain (10uQ8; 1.0 = no change)
 * @v_gain:  V channel gain (10uQ8)
 * @_pad:    reserved, must be zero
 */
struct camss_params_uvg {
	struct v4l2_isp_params_block_header header;
	__u16 u_gain;
	__u16 v_gain;
	__u16 _pad[2];
} __attribute__((aligned(8)));

/* ===================================================================
 * Local Tone Mapping (LTM) — split into four sub-blocks
 *
 * Hardware availability: IFE, BPS. (IPE: future work — IPE has a
 * YUV-domain LTM instance using the same tuning parameters; the
 * structs defined here apply unchanged when IPE support lands.)
 *
 * LTM is a heavyweight block combining seven 1D LUTs (weight,
 * la_curve, mask_rect_curve, ltm_curve, ltm_scale, lce_scale_pos,
 * lce_scale_neg, igamma64), a 5x5 mask filter kernel, a scalar
 * control section (data-collection counters, downscaling
 * parameters, image-processing offsets), and two large across-
 * frame state buffers (LUT25b, avg3d).
 *
 * The cross-frame state buffers (LUT25b, avg3d) are hardware ping-
 * pong buffers managed by the kernel between frames and are NOT
 * exposed to userspace.
 *
 * The remainder is split into four sub-blocks by expected update
 * cadence:
 *
 *   CONFIG       — scalar control, data collection, image processing
 *                  (may change per frame for adaptive behaviour)
 *   CURVES       — six tone-mapping LUTs (la, mask_rect, ltm_curve,
 *                  ltm_scale, lce_scale_pos, lce_scale_neg); change
 *                  on tone-preset switch
 *   GAMMA        — inverse-gamma 64-LUT; typically loaded once per
 *                  session, occasionally on tone-preset change
 *   MASK_FILTER  — 5x5 mask kernel; rarely touched after tuning
 *
 * Splitting lets userspace re-upload only the pieces that actually
 * changed; a per-frame adaptive update typically sends only the
 * CONFIG block while the LUT blocks stay resident in the
 * hardware.
 *
 * Cross-sub-block consistency is the caller's responsibility:
 * blocks apply on top of previous state, so an update to CURVES
 * without a corresponding CONFIG update will use the existing
 * scalar control values.
 * =================================================================== */

#define CAMSS_LTM_WEIGHT_LUT_SIZE    12
#define CAMSS_LTM_CURVE_LUT_SIZE     65
#define CAMSS_LTM_SCALE_LUT_SIZE     65
#define CAMSS_LTM_LCE_SCALE_LUT_SIZE 17
#define CAMSS_LTM_GAMMA_LUT_SIZE     64
#define CAMSS_LTM_MASK_FILTER_SIZE    6

/**
 * struct camss_params_ltm_config - LTM control, downscale, IP setup
 *
 * Hardware availability: IFE, BPS.
 *
 * Scalar control fields for LTM. Combines three logical groups
 * (data collection, downscaling, image processing) that share an
 * update lifetime — they typically all change together when LTM
 * is reconfigured for a new pipeline geometry or tone preset.
 *
 * Fields prefixed @dc_ configure the data-collection stage (3D
 * bilateral table accumulation), @ds_ configures the downscaler
 * that feeds it, and @ip_ configures the image-processing stage
 * that consumes the resulting 3D LUT.
 *
 * @header:                  block header; @header.type = CAMSS_PARAMS_LTM_CONFIG
 *
 * Top-level enables:
 * @data_collect_en:         enable data-collection stage
 * @img_process_en:          enable image-processing stage
 * @igamma_en:               enable inverse-gamma stage
 * @la_en:                   enable la_curve application
 *
 * Data-collection stage (post-cropping cell layout):
 * @dc_3dtable_avg_pong_sel: ping-pong selector for 3D avg table
 * @dc_init_cellnum_x:       starting cell x index (4u)
 * @dc_init_dx:              initial dx (5u)
 * @dc_3d_sum_clear:         clear 3D sum accumulator
 * @bin_init_cnt:            initial bin count (4u)
 * @dc_conv_start_cell_x:    convolution start cell x (4u)
 * @dc_conv_end_cell_x:      convolution end cell x (4u)
 * @_pad0:                   reserved, must be zero
 * @dc_xstart:               post-cropping x start (8u)
 * @dc_xend:                 post-cropping x end (8u)
 *
 * Downscaler:
 * @ds_horizontal_skip_cnt:  horizontal skip count, pre-cropping (10u)
 * @ds_vertical_skip_cnt:    vertical skip count (14u)
 * @ds_fac:                  downscale factor (2u):
 *                             0 = 288x216 ds size, cell w/h 24
 *                             1 = 144x108 ds size, cell w/h 12
 *                             2 =  72x54  ds size, cell w/h  6
 * @_pad1:                   reserved, must be zero
 *
 * Image-processing stage:
 * @ip_init_cellnum_x:       starting cell x index (4u)
 * @ip_init_cellnum_y:       starting cell y index (4u)
 * @ip_3dtable_avg_pong_sel: ping-pong selector for IP-side 3D avg
 * @_pad2:                   reserved, must be zero
 * @ip_init_dx:              initial dx (11u)
 * @ip_init_dy:              initial dy (11u)
 * @ip_init_px:              initial px (18u)
 * @ip_init_py:              initial py (18u)
 * @ip_inv_cellwidth:        inverse cell width (14u)
 * @ip_inv_cellheight:       inverse cell height (14u)
 * @ip_cellwidth:            cell width (11u)
 * @ip_cellheight:           cell height (11u)
 *
 * Mixing coefficients (10/11-bit each):
 * @c1: coefficient 1 (10u)
 * @c2: coefficient 2 (10u)
 * @c3: coefficient 3 (10u)
 * @c4: coefficient 4 (11u)
 *
 * Weight LUT (bundled with config since it is tightly coupled):
 * @weight_lut: 12-entry weight LUT
 *
 * Misc:
 * @lce_thd:       LCE (local contrast enhancement) threshold
 * @y_ratio_max:   maximum Y ratio (10u)
 * @debug_out_sel: hardware debug-out selector (2u)
 * @_pad3:         reserved, must be zero
 */
struct camss_params_ltm_config {
	struct v4l2_isp_params_block_header header;

	__u8  data_collect_en;
	__u8  img_process_en;
	__u8  igamma_en;
	__u8  la_en;

	__u8  dc_3dtable_avg_pong_sel;
	__u8  dc_init_cellnum_x;
	__u8  dc_init_dx;
	__u8  dc_3d_sum_clear;
	__u8  bin_init_cnt;
	__u8  dc_conv_start_cell_x;
	__u8  dc_conv_end_cell_x;
	__u8  _pad0;
	__u16 dc_xstart;
	__u16 dc_xend;

	__u16 ds_horizontal_skip_cnt;
	__u16 ds_vertical_skip_cnt;
	__u8  ds_fac;
	__u8  _pad1[3];

	__u8  ip_init_cellnum_x;
	__u8  ip_init_cellnum_y;
	__u8  ip_3dtable_avg_pong_sel;
	__u8  _pad2;
	__u16 ip_init_dx;
	__u16 ip_init_dy;
	__u32 ip_init_px;
	__u32 ip_init_py;
	__u16 ip_inv_cellwidth;
	__u16 ip_inv_cellheight;
	__u16 ip_cellwidth;
	__u16 ip_cellheight;

	__u16 c1;
	__u16 c2;
	__u16 c3;
	__u16 c4;

	__u16 weight_lut[CAMSS_LTM_WEIGHT_LUT_SIZE];

	__u16 lce_thd;
	__u16 y_ratio_max;
	__u8  debug_out_sel;
	__u8  _pad3[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_ltm_curves - LTM tone-mapping curve LUTs
 *
 * Hardware availability: IFE, BPS.
 *
 * Six 1D tone / scale LUTs:
 *   - la_curve:        local-adaptation curve (post-igamma)
 *   - mask_rect_curve: mask-shaping curve
 *   - ltm_curve:       core LTM transfer curve
 *   - ltm_scale:       scale modulation
 *   - lce_scale_pos:   LCE positive-direction scale
 *   - lce_scale_neg:   LCE negative-direction scale
 *
 * The la_curve and mask_rect_curve are 65-entry, the LCE scales
 * are 17-entry, matching the hardware LUT sizes.
 *
 * Per hardware spec: la_curve and mask_rect_curve are unsigned
 * (10+e)u; ltm_curve, ltm_scale, lce_scale_pos, lce_scale_neg are
 * signed (10+e)s.
 *
 * @header:        block header; @header.type = CAMSS_PARAMS_LTM_CURVES
 * @la_curve:      local-adaptation curve (10+e)u
 * @mask_rect_curve: mask-shaping curve (10+e)u
 * @ltm_curve:     LTM transfer curve (10+e)s
 * @ltm_scale:     scale modulation (10+e)s
 * @lce_scale_pos: LCE positive scale (10+e)s
 * @lce_scale_neg: LCE negative scale (10+e)s
 */
struct camss_params_ltm_curves {
	struct v4l2_isp_params_block_header header;
	__u16 la_curve[CAMSS_LTM_CURVE_LUT_SIZE];
	__u16 mask_rect_curve[CAMSS_LTM_CURVE_LUT_SIZE];
	__s16 ltm_curve[CAMSS_LTM_CURVE_LUT_SIZE];
	__s16 ltm_scale[CAMSS_LTM_SCALE_LUT_SIZE];
	__s16 lce_scale_pos[CAMSS_LTM_LCE_SCALE_LUT_SIZE];
	__s16 lce_scale_neg[CAMSS_LTM_LCE_SCALE_LUT_SIZE];
} __attribute__((aligned(8)));

/**
 * struct camss_params_ltm_gamma - LTM inverse-gamma LUT
 *
 * Hardware availability: IFE, BPS.
 *
 * 64-entry inverse gamma applied before LTM processing. Typically
 * loaded once per session or on tone-preset change. Separate from
 * the main gamma block (CAMSS_PARAMS_GLUT) because that drives the
 * global post-LTM gamma stage.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_LTM_GAMMA
 * @lut:    inverse gamma LUT, 64 entries (10+e)s
 */
struct camss_params_ltm_gamma {
	struct v4l2_isp_params_block_header header;
	__s16 lut[CAMSS_LTM_GAMMA_LUT_SIZE];
} __attribute__((aligned(8)));

/**
 * struct camss_params_ltm_mask_filter - LTM 5x5 mask filter kernel
 *
 * Hardware availability: IFE, BPS.
 *
 * The 5x5 mask filter kernel is symmetric and stored as 6 unique
 * coefficients.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_LTM_MASK_FILTER
 * @kernel: 6 unique 5x5 symmetric kernel coefficients (4u)
 * @scale:  output scale factor (12u)
 * @shift:  output shift (5u)
 * @enable: 1 = apply mask filter
 * @_pad:   reserved, must be zero
 */
struct camss_params_ltm_mask_filter {
	struct v4l2_isp_params_block_header header;
	__u16 kernel[CAMSS_LTM_MASK_FILTER_SIZE];
	__u16 scale;
	__u16 shift;
	__u8  enable;
	__u8  _pad[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_roi - Rectangle-of-interest coordinates
 *
 * @left:   x coordinate of the ROI
 * @top:    y coordinate of the ROI
 * @width:  width of the ROI
 * @height: height of the ROI
 */
struct camss_params_roi {
	__u32 left;
	__u32 top;
	__u32 width;
	__u32 height;
} __attribute__((packed));

/**
 * enum camss_stats_bg_usage - Which BG/BE consumer this block targets
 */
enum camss_stats_bg_usage {
	CAMSS_STATS_BG_AWB      = 0, /* AWB Bayer Grid */
	CAMSS_STATS_BG_HDR_BE   = 1, /* HDR Bayer Exposure */
	CAMSS_STATS_BG_TINTLESS = 2, /* Tintless Bayer Grid */
};

/**
 * enum camss_stats_bg_output_mode - BG/BE stats output mode
 */
enum camss_stats_bg_output_mode {
	CAMSS_STATS_BG_MODE_REGULAR    = 0, /* regular */
	CAMSS_STATS_BG_MODE_SATURATION = 1, /* saturation enabled */
	CAMSS_STATS_BG_MODE_Y_STATS    = 2, /* Y-stats enabled */
};

/**
 * enum camss_stats_bg_green_type - Which green pixels to use for Y-stats
 */
enum camss_stats_bg_green_type {
	CAMSS_STATS_BG_GREEN_GR      = 0, /* use Gr */
	CAMSS_STATS_BG_GREEN_GB      = 1, /* use Gb */
	CAMSS_STATS_BG_GREEN_AVERAGE = 2, /* use (Gr + Gb) / 2 */
};

/**
 * struct camss_stats_bg - Bayer Grid / Bayer Exposure stats
 *
 * Hardware availability: IFE (AWB-BG, HDR-BE, Tintless-BG),
 * BPS (BG).
 *
 * Drives the gridded Bayer accumulator hardware primitive, which
 * is instantiated up to four times across IFE and BPS for
 * different consumers. The @usage field selects which.
 *
 * The hardware divides the @roi rectangle into @horizontal_num by
 * @vertical_num regions and accumulates per-channel statistics
 * over each region. @channel_gain_threshold caps per-pixel
 * contribution before accumulation. Channel indexing is Bayer
 * order: 0 = R, 1 = B, 2 = Gr, 3 = Gb.
 *
 * @header:                 block header; @header.type = CAMSS_PARAMS_STATS_BG
 * @usage:                  see &enum camss_stats_bg_usage
 * @output_mode:            see &enum camss_stats_bg_output_mode
 * @green_type:             see &enum camss_stats_bg_green_type
 * @_pad0:                  reserved, must be zero
 * @horizontal_num:         number of horizontal regions
 * @vertical_num:           number of vertical regions
 * @roi:                    active region on the sensor
 * @channel_gain_threshold: max gain threshold for [R, B, Gr, Gb]
 * @output_bit_depth:       requested stats bit depth (0 = hw default)
 * @region_width:           width of each subgrid region in pixels
 * @region_height:          height of each subgrid region in pixels
 * @y_stats_weights_q8:     R/G/B weights for Y-stats mode (Q8)
 */
struct camss_stats_bg {
	struct v4l2_isp_params_block_header header;
	__u8  usage;
	__u8  output_mode;
	__u8  green_type;
	__u8  _pad0;
	__s32 horizontal_num;
	__s32 vertical_num;
	struct camss_params_roi roi;
	__u32 channel_gain_threshold[4];
	__u32 output_bit_depth;
	__u32 region_width;
	__u32 region_height;
	__u32 y_stats_weights_q8[3];
} __attribute__((aligned(8)));

/**
 * enum camss_stats_color_channel - Colour channel selector
 */
enum camss_stats_color_channel {
	CAMSS_STATS_CHANNEL_INVALID = 0,
	CAMSS_STATS_CHANNEL_R       = 1,
	CAMSS_STATS_CHANNEL_GR      = 2,
	CAMSS_STATS_CHANNEL_GB      = 3,
	CAMSS_STATS_CHANNEL_B       = 4,
	CAMSS_STATS_CHANNEL_G       = 5,
	CAMSS_STATS_CHANNEL_Y       = 6,
};

/**
 * enum camss_stats_bhist_usage - Which consumer this BHist instance feeds
 *
 * The same hardware histogram primitive is instantiated multiple
 * times on IFE for different consumers (one dedicated to AEC, one
 * driving GTM tone-mapping decisions). The @usage field in &struct
 * camss_stats_bhist routes to the correct hardware instance.
 */
enum camss_stats_bhist_usage {
	CAMSS_STATS_BHIST_AEC = 0, /* AEC-dedicated histogram */
	CAMSS_STATS_BHIST_GTM = 1, /* GTM-dedicated histogram */
};

/**
 * struct camss_stats_bhist - Bayer histogram
 *
 * Hardware availability: IFE, BPS.
 *
 * On IFE the hardware instantiates this block multiple times for
 * different consumers (AEC, GTM). The @usage field selects which.
 * On BPS there is a single instance; @usage is reserved (0).
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_STATS_BHIST
 * @roi:     ROI rectangle to collect statistics from
 * @channel: channel selector (see &enum camss_stats_color_channel)
 * @uniform: 1 = uniform histogram binning
 * @usage:   consumer selector on IFE; see &enum camss_stats_bhist_usage
 * @_pad:    reserved, must be zero
 */
struct camss_stats_bhist {
	struct v4l2_isp_params_block_header header;
	struct camss_params_roi roi;
	__u8  channel;
	__u8  uniform;
	__u8  usage;
	__u8  _pad[5];
} __attribute__((aligned(8)));

/**
 * enum camss_stats_hdr_bhist_green - HDR-BHist green channel selection
 */
enum camss_stats_hdr_bhist_green {
	CAMSS_STATS_HDR_BHIST_GR = 0,
	CAMSS_STATS_HDR_BHIST_GB = 1,
};

/**
 * enum camss_stats_hdr_bhist_field - HDR-BHist input field selection
 *
 * Selects which exposure the stats are collected from in HDR
 * staggered modes.
 */
enum camss_stats_hdr_bhist_field {
	CAMSS_STATS_HDR_BHIST_ALL   = 0, /* all lines, non-HDR */
	CAMSS_STATS_HDR_BHIST_LONG  = 1, /* HDR T1 (long exposure) */
	CAMSS_STATS_HDR_BHIST_SHORT = 2, /* HDR T2 (short exposure) */
};

/**
 * struct camss_stats_hdr_bhist - HDR Bayer histogram
 *
 * Hardware availability: IFE (AEC variant), BPS.
 *
 * @header:        block header; @header.type = CAMSS_PARAMS_STATS_HDR_BHIST
 * @roi:           ROI rectangle
 * @green_channel: green channel selector
 *                 (see &enum camss_stats_hdr_bhist_green)
 * @field_select:  exposure field selector
 *                 (see &enum camss_stats_hdr_bhist_field)
 * @_pad:          reserved, must be zero
 */
struct camss_stats_hdr_bhist {
	struct v4l2_isp_params_block_header header;
	struct camss_params_roi roi;
	__u8  green_channel;
	__u8  field_select;
	__u8  _pad[6];
} __attribute__((aligned(8)));

/**
 * enum camss_stats_ihist_channel - Image histogram channel selection
 *
 * IHist operates after the YCbCr colour conversion, so the
 * channels are YCC not Bayer.
 */
enum camss_stats_ihist_channel {
	CAMSS_STATS_IHIST_Y  = 0,
	CAMSS_STATS_IHIST_CB = 1,
	CAMSS_STATS_IHIST_CR = 2,
};

/**
 * struct camss_stats_ihist - Image histogram
 *
 * Hardware availability: IFE only.
 *
 * @header:                block header; @header.type = CAMSS_PARAMS_STATS_IHIST
 * @roi:                   ROI rectangle
 * @channel:               YCC channel selector
 * @_pad:                  reserved, must be zero
 * @max_pixel_sum_per_bin: histogram output bit-shift control.
 *                         When image data concentrates in few bins
 *                         the output may saturate; this field
 *                         tells the hardware how many bits to
 *                         shift so pixel sums fit in the output
 *                         bin width. 0 = use hardware default.
 */
struct camss_stats_ihist {
	struct v4l2_isp_params_block_header header;
	struct camss_params_roi roi;
	__u8  channel;
	__u8  _pad[3];
	__u32 max_pixel_sum_per_bin;
} __attribute__((aligned(8)));

/**
 * struct camss_stats_rs - Row Sum stats
 *
 * Hardware availability: IFE only.
 *
 * @header:          block header; @header.type = CAMSS_PARAMS_STATS_RS
 * @stats_h_num:     horizontal region count
 * @stats_v_num:     vertical region count
 * @rscs_color_conv: 1 = enable RGB->Y conversion on input pixels
 * @_pad:            reserved, must be zero
 */
struct camss_stats_rs {
	struct v4l2_isp_params_block_header header;
	__u32 stats_h_num;
	__u32 stats_v_num;
	__u8  rscs_color_conv;
	__u8  _pad[7];
} __attribute__((aligned(8)));

/* ------------------------------------------------------------------
 * Bayer Auto-Focus (BAF) stats — the large one
 *
 * BAF configuration splits into sub-sections (input channel
 * select, gamma LUT, scaler, FIR filter, IIR filter, ROI layout,
 * coring, filter-chain composition). For UAPI we flatten this
 * into a single block with the sub-structures inline and caps on
 * variable-length arrays.
 * ------------------------------------------------------------------ */

#define CAMSS_BAF_MAX_GAMMA_ENTRIES      32
#define CAMSS_BAF_MAX_FIR_COEFFICIENTS   13
#define CAMSS_BAF_MAX_CORING_ENTRIES     17
#define CAMSS_BAF_FILTER_TYPE_COUNT       3
#define CAMSS_BAF_MAX_ROI_REGIONS       180
#define CAMSS_BAF_Y_CONFIG_COUNT          3

/**
 * enum camss_baf_channel_select - BAF primary channel selection
 *
 * Picks whether BAF operates on green or luma.
 */
enum camss_baf_channel_select {
	CAMSS_BAF_CHANNEL_G = 0,
	CAMSS_BAF_CHANNEL_Y = 1,
};

/**
 * enum camss_baf_input_g_select - BAF Green input channel
 *
 * When channel select is G, picks which Bayer green to use.
 */
enum camss_baf_input_g_select {
	CAMSS_BAF_INPUT_GR = 0,
	CAMSS_BAF_INPUT_GB = 1,
};

/**
 * enum camss_baf_stats_region_type - BAF ROI region classification
 */
enum camss_baf_stats_region_type {
	CAMSS_BAF_REGION_PRIMARY   = 0,
	CAMSS_BAF_REGION_SECONDARY = 1,
};

/**
 * enum camss_baf_stats_roi_type - BAF ROI mode
 */
enum camss_baf_stats_roi_type {
	CAMSS_BAF_ROI_DEFAULT = 0, /* hardware default grid */
	CAMSS_BAF_ROI_CUSTOM  = 1, /* user-provided ROI list */
};

/**
 * struct camss_baf_roi - One BAF statistics ROI
 *
 * @region:      see &enum camss_baf_stats_region_type
 * @_pad:        reserved, must be zero
 * @roi:         ROI coordinates on the sensor
 * @region_num:  index within the region's group
 * @output_id:   output ordering ID
 * @need_merge:  1 if region spans a dual-IFE split and must be merged
 * @_pad2:       reserved, must be zero
 */
struct camss_baf_roi {
	__u8  region;
	__u8  _pad[3];
	struct camss_params_roi roi;
	__u32 region_num;
	__u32 output_id;
	__u8  need_merge;
	__u8  _pad2[7];
} __attribute__((packed));

/**
 * struct camss_baf_fir_filter - BAF FIR filter configuration
 *
 * @enable:       1 = FIR filter active
 * @_pad:         reserved, must be zero
 * @num_coeffs:   how many leading entries of @coefficients are valid
 * @coefficients: FIR filter taps
 */
struct camss_baf_fir_filter {
	__u8  enable;
	__u8  _pad[3];
	__u32 num_coeffs;
	__s32 coefficients[CAMSS_BAF_MAX_FIR_COEFFICIENTS];
} __attribute__((packed));

/**
 * struct camss_baf_iir_filter - BAF IIR filter configuration
 *
 * Second-order sections with floating-point coefficients. The
 * hardware's IIR stage natively consumes float, so we keep float
 * in UAPI.
 *
 * @enable: 1 = IIR filter active
 * @_pad:   reserved, must be zero
 * @b10..a22: biquad coefficients
 */
struct camss_baf_iir_filter {
	__u8  enable;
	__u8  _pad[7];
	float b10, b11, b12, a11, a12;
	float b20, b21, b22, a21, a22;
} __attribute__((packed));

/**
 * struct camss_baf_filter_coring - BAF coring configuration
 *
 * @threshold:   filter threshold
 * @gain:        filter gain
 * @num_entries: number of valid entries in @core
 * @_pad:        reserved, must be zero
 * @core:        coring LUT
 */
struct camss_baf_filter_coring {
	__s32 threshold;
	__u32 gain;
	__u32 num_entries;
	__u32 _pad;
	__u32 core[CAMSS_BAF_MAX_CORING_ENTRIES];
} __attribute__((packed));

/**
 * struct camss_baf_filter - One BAF filter
 *
 * Indexed in the outer block by filter type:
 *   0 = horizontal 1
 *   1 = horizontal 2
 *   2 = vertical
 *
 * @is_valid:                1 = this filter slot carries valid config
 * @horizontal_scale_enable: 1 = enable horizontal scaling
 * @_pad:                    reserved, must be zero
 * @shift_bits:              post-filter bit-shift
 * @fir:                     FIR stage
 * @iir:                     IIR stage
 * @coring:                  coring stage
 */
struct camss_baf_filter {
	__u8  is_valid;
	__u8  horizontal_scale_enable;
	__u8  _pad[2];
	__s32 shift_bits;
	struct camss_baf_fir_filter fir;
	struct camss_baf_iir_filter iir;
	struct camss_baf_filter_coring coring;
} __attribute__((packed));

/**
 * struct camss_stats_baf - Bayer Auto-Focus stats
 *
 * Hardware availability: IFE only.
 *
 * BAF is the most complex stats block. The hardware can be
 * programmed with up to 180 ROIs, three filter chains (2
 * horizontal + 1 vertical) each with independent FIR/IIR/coring
 * config, a 32-entry gamma LUT, and a scaling stage.
 *
 * Most fields carry an @is_valid / @enable companion to allow
 * partial configuration: userspace can leave sections disabled
 * and the kernel will skip them when composing the CDM program.
 *
 * @header:                   block header; @header.type = CAMSS_PARAMS_STATS_BAF
 *
 * Input channel configuration:
 * @input_is_valid:           1 = input config section valid
 * @input_channel_select:     G vs Y (see &enum camss_baf_channel_select)
 * @input_g_select:           Gr vs Gb (see &enum camss_baf_input_g_select)
 * @_pad0:                    reserved, must be zero
 * @y_config_q8:              Y-channel mix weights (Q8)
 *
 * Gamma LUT:
 * @gamma_is_valid:           1 = gamma LUT section valid
 * @_pad1:                    reserved, must be zero
 * @num_gamma_entries:        valid leading entries of @gamma_lut
 * @gamma_lut:                pre-filter gamma correction LUT
 *
 * Scaler:
 * @scale_is_valid:           1 = scaler section valid
 * @scale_enable:             1 = enable scaler
 * @_pad2:                    reserved, must be zero
 * @scale_m:                  M scalar
 * @scale_n:                  N scalar
 * @pixel_offset:             pixel offset
 * @mn_init:                  MN init value
 * @interpolation_resolution: scaler interpolation resolution
 * @phase_init:               phase init
 * @phase_step:               phase step
 * @input_image_width:        input image width for scaler
 *
 * Filters:
 * @filters:                  one entry per filter type (H1, H2, V)
 *
 * ROIs:
 * @roi_type:                 default grid vs custom list
 *                            (see &enum camss_baf_stats_roi_type)
 * @_pad3:                    reserved, must be zero
 * @num_rois:                 valid leading entries of @rois
 * @last_primary_region:      index of last primary-type ROI
 * @rois:                     ROI list
 */
struct camss_stats_baf {
	struct v4l2_isp_params_block_header header;

	/* Input channel config */
	__u8  input_is_valid;
	__u8  input_channel_select;
	__u8  input_g_select;
	__u8  _pad0;
	__u32 y_config_q8[CAMSS_BAF_Y_CONFIG_COUNT];

	/* Gamma LUT */
	__u8  gamma_is_valid;
	__u8  _pad1[3];
	__u32 num_gamma_entries;
	__u32 gamma_lut[CAMSS_BAF_MAX_GAMMA_ENTRIES];

	/* Scaler */
	__u8  scale_is_valid;
	__u8  scale_enable;
	__u8  _pad2[2];
	__s32 scale_m;
	__s32 scale_n;
	__s32 pixel_offset;
	__s32 mn_init;
	__s32 interpolation_resolution;
	__s32 phase_init;
	__s32 phase_step;
	__s32 input_image_width;

	/* Three filter chains (H1, H2, V) */
	struct camss_baf_filter filters[CAMSS_BAF_FILTER_TYPE_COUNT];

	/* ROI list */
	__u8  roi_type;
	__u8  _pad3[3];
	__u32 num_rois;
	__u32 last_primary_region;
	struct camss_baf_roi rois[CAMSS_BAF_MAX_ROI_REGIONS];
} __attribute__((aligned(8)));

/* ===================================================================
 * Buffer sizing helpers
 * =================================================================== */

/**
 * CAMSS_PARAMS_MAX_SIZE - Maximum total size of CAMSS IQ parameters
 *
 * Sum of every IQ parameter block's sizeof. The driver uses this
 * to size the params buffer so userspace can set all IQ parameter
 * blocks in a single frame if needed.
 *
 * Stats blocks are excluded from this sum:
 *   - Stats engine configuration (&struct camss_stats_*) is sized
 *     separately by &CAMSS_STATS_CONFIG_MAX_SIZE.
 *   - Stats output buffers (&struct camss_stats_*_output) are
 *     sized separately per stats engine; they are kernel ->
 *     userspace buffers and do not travel in the params buffer.
 *
 * Where the same struct is used for multiple block types in
 * &enum camss_params_block_type, it is counted once per usage so
 * userspace can set every distinct block in a single frame.
 */
#define CAMSS_PARAMS_MAX_SIZE                                      \
	(sizeof(struct camss_params_bls)                          + \
	 sizeof(struct camss_params_linearisation)                + \
	 sizeof(struct camss_params_demux)                        + \
	 sizeof(struct camss_params_bincorrect)                   + \
	 sizeof(struct camss_params_abf_filter)                   + \
	 sizeof(struct camss_params_abf_noise_lut)                + \
	 sizeof(struct camss_params_abf_activity_lut)             + \
	 sizeof(struct camss_params_abf_dark_lut)                 + \
	 sizeof(struct camss_params_abf_rnr)                      + \
	 sizeof(struct camss_params_wb_gain)                      + \
	 sizeof(struct camss_params_lsc)                          + \
	 sizeof(struct camss_params_demosaic)                     + \
	 sizeof(struct camss_params_color_correct)                + \
	 sizeof(struct camss_params_color_xform)                  + \
	 sizeof(struct camss_params_glut)                         + \
	 sizeof(struct camss_params_gtm)                          + \
	 sizeof(struct camss_params_uvg)                          + \
	 sizeof(struct camss_params_ltm_config)                   + \
	 sizeof(struct camss_params_ltm_curves)                   + \
	 sizeof(struct camss_params_ltm_gamma)                    + \
	 sizeof(struct camss_params_ltm_mask_filter))

/**
 * CAMSS_STATS_CONFIG_MAX_SIZE - Maximum total size of CAMSS stats config
 *
 * Sum of every stats engine configuration block's sizeof. The
 * driver uses this to size the stats configuration buffer so
 * userspace can configure all stats engines in a single frame if
 * needed.
 *
 * Stats output buffers are NOT included; those are sized
 * separately per stats engine based on the relevant
 * &struct camss_stats_*_output.
 */
#define CAMSS_STATS_CONFIG_MAX_SIZE                                \
	(sizeof(struct camss_stats_bg)                            + \
	 sizeof(struct camss_stats_bhist)                         + \
	 sizeof(struct camss_stats_hdr_bhist)                     + \
	 sizeof(struct camss_stats_ihist)                         + \
	 sizeof(struct camss_stats_rs)                            + \
	 sizeof(struct camss_stats_baf))

#endif /* _UAPI_LINUX_CAMSS_STATS_PARAMS_H */
