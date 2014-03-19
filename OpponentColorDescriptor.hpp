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

#ifndef OpponentColorDescriptor_HPP
#define OpponentColorDescriptor_HPP

#include <vector>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "matcher_consts.hpp"

using std::vector;

/**
 * Adapts a descriptor extractor to compute descripors in Opponent Color Space
 * (refer to van de Sande et al."CGIV 2008 "Color Descriptors for Object
 * Category Recognition").
 * Input RGB image is transformed to Opponent Color Space. Then unadapted
 * descriptor extractor (set in constructor) computes descriptors on each of
 * the three channel and concatenate them into a single color descriptor.
 */
class OpponentColorDescriptor {
 public:
    /**
     * A constructor taking opencv descriptor-extractor as a parameter
     */
    OpponentColorDescriptor(
        const cv::Ptr<cv::DescriptorExtractor>& descriptorExtractor);

    /**
     * Computes descriptors on each of the three channel and concatenate
     * them into a single color descriptor.
     * @param image       Input RGB image transformed to Opponent Color Space.
     * @param keypoints   output keypoints
     * @param descriptors concatenated output descriptors
     */
    void compute(const cv::Mat& image,
                 vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors) const;

 private:
    /**
     * Merges keypoints and descriptors from matcher::NOPP channels to look like
     * as in one single channel.
     * @param dSize               descriptor size
     * @param outKeypoints        merged keypoints vector
     * @param mergedDescriptors   merged descriptors matrix
     * @param idxs                idxs
     * @param channelDescriptors  matrix of descriptors from each channel
     * @param channelKeypoints    matrix of keypoints from each channel
     * @param maxKeypointsCount   maximum keypoints size
     * @return                    merged descriptors count
     */
    int merge(const int &dSize,
              vector<cv::KeyPoint> &outKeypoints,
              cv::Mat &mergedDescriptors,
              std::vector<int> (&idxs)[matcher::NOPP],
              cv::Mat (&channelDescriptors)[matcher::NOPP],
              std::vector<cv::KeyPoint> (&channelKeypoints)[matcher::NOPP],
              const int &maxKeypointsCount) const;

    /**
     * Merges descriptors from matcher::NOPP channels into single matrix.
     * @param dSize               descriptor size
     * @param keypoints           input keypoints vector
     * @param outKeypoints        merged keypoints vector
     * @param mergedDescriptors   merged descriptors matrix
     * @param idxs                idxs
     * @param channelDescriptors  matrix of descriptors from each channel
     * @param channelKeypoints    matrix of keypoints from each channel
     * @param mergedCount         current merged descriptors count
     * @param maxInitIdx          maxInitIdx
     * @param cp                  current channel position
     * @param idx1                index of the second opponent channel
     * @param idx2                index of the third opponent channel
     */
    void mergeDescriptors(const int &dSize,
                  vector<cv::KeyPoint> &keypoints,
                  vector<cv::KeyPoint> &outKeypoints,
                  cv::Mat &mergedDescriptors,
                  std::vector<int> (&idxs)[matcher::NOPP],
                  cv::Mat (&channelDescriptors)[matcher::NOPP],
                  std::vector<cv::KeyPoint> (&channelKeypoints)[matcher::NOPP],
                  int *mergedCount,
                  const int &maxInitIdx,
                  size_t (&cp)[matcher::NOPP],
                  const int& idx1,
                  const int& idx2) const;

    /**
     * Unadapted opencv descriptor extractor.
     */
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
};

#endif  // OpponentColorDescriptor_HPP
