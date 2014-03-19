/*
 * matcher for computer-vision based SW testing
 * Copyright (c) 2012-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU Lesser General Public License,
 * version 2.1, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef MatchEntryGPU_HPP
#define MatchEntryGPU_HPP

#include <opencv2/gpu/gpu.hpp>

/**
 * Class to store match entry in GPU format
 */
class MatchEntryGPU {
 public:
    MatchEntryGPU();
    ~MatchEntryGPU();

    /**
     * Image matrix
     */
    cv::gpu::GpuMat image;

    /**
     * Computed keypoints
     */
    cv::gpu::GpuMat keypoints;

    /**
     * Mask specifying where to look for keypoints (optional). It must be a
     * 8-bit integer matrix with non-zero values in the region of interest.
     * If no mask is specified then keypoints are calculated in a whole image.
     */
    cv::gpu::GpuMat mask;

    /**
     * Computed descriptors
     */
    cv::gpu::GpuMat descriptors;
};

#endif  // MatchEntryGPU_HPP
