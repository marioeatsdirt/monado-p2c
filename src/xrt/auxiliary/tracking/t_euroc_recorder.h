// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  EuRoC dataset recorder utility.
 * @author Mateo de Mayo <mateo.demayo@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frame.h"

#define CSV_EOL "\r\n"
#define CSV_PRECISION 10

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create SLAM sinks to record samples in EuRoC format.
 *
 * @param xfctx Frame context for the sinks.
 * @param record_path Directory name to save the dataset or NULL for a default based on the current datetime.
 * @return struct xrt_slam_sinks* Sinks to push samples to for recording.
 *
 * @ingroup aux_tracking
 */
struct xrt_slam_sinks *
euroc_recorder_create(struct xrt_frame_context *xfctx, const char *record_path);

#ifdef __cplusplus
}
#endif
