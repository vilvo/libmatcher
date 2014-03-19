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

#ifndef MatchEntry_HPP
#define MatchEntry_HPP

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Class to store match entry in CPU
 */
class MatchEntry {
 public:
    /**
     * A constructor that takes image as an argument
     * @param img Image to be stored in MatchEntry
     */
    explicit MatchEntry(const cv::Mat img = cv::Mat());

    ~MatchEntry();

    /**
     * Checks whether match entry is valid, i.e. contains keypoints, descriptors
     * and image.
     * @return True if match entry is valid. Otherwsie returns false.
     */
    bool isValid();

    /**
     * Image matrix
     */
    cv::Mat image;

    /**
     * Vector of computed keypoints
     */
    std::vector<cv::KeyPoint> keypoints;

    /**
     * Mask specifying where to look for keypoints (optional). It must be a
     * 8-bit integer matrix with non-zero values in the region of interest.
     * If no mask is specified then keypoints are calculated in a whole image.
     */
    cv::Mat mask;

    /**
     * Computed descriptors
     */
    cv::Mat descriptors;
};

#endif  // MatchEntry_HPP
