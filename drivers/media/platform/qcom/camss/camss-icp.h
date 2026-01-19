/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm CAMSS ICP Header
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#ifndef __CAMSS_ICP_H__
#define __CAMSS_ICP_H__

struct camss_icp;
struct device;

/*
 * camss_icp_get - Get reference to ICP
 * @dev: Caller's device (for error messages)
 *
 * Returns ICP pointer or ERR_PTR on failure.
 * Call camss_icp_put() when done.
 */
struct camss_icp *camss_icp_get(struct device *dev);

/*
 * camss_icp_put - Release reference to ICP
 * @icp: ICP pointer from camss_icp_get()
 */
void camss_icp_put(struct camss_icp *icp);

/*
 * camss_icp_is_booted - Check if ICP is booted
 * @icp: ICP pointer
 *
 * Returns true if ICP firmware is running.
 */
bool camss_icp_is_booted(struct camss_icp *icp);

#endif /* __CAMSS_ICP_H__ */
