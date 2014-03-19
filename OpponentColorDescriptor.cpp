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

#include <vector>
#include <limits>
#include <algorithm>  // for swap
#include "OpponentColorDescriptor.hpp"
#include "MatcherUtils.hpp"

// Public methods

OpponentColorDescriptor::OpponentColorDescriptor(
    const cv::Ptr<cv::DescriptorExtractor>& descriptorExtractor) :
        descriptor_(descriptorExtractor) {}

// helper function to compare keypoints
struct KP_LessThan {
    explicit KP_LessThan(const std::vector<cv::KeyPoint>& _kp) : kp(&_kp) {}
    bool operator()(int i, int j) const {
        return (*kp)[i].class_id < (*kp)[j].class_id;
    }
    const std::vector<cv::KeyPoint>* kp;
};

void OpponentColorDescriptor::compute(const cv::Mat& bgrImage,
                                               vector<cv::KeyPoint>& keypoints,
                                               cv::Mat& descriptors) const {
    if (bgrImage.empty() || keypoints.empty()) {
        descriptors.release();
        return;
    }

    cv::KeyPointsFilter::runByImageBorder(keypoints, bgrImage.size(), 0);
    cv::KeyPointsFilter::runByKeypointSize(keypoints,
                                        std::numeric_limits<float>::epsilon());

    std::vector<cv::Mat> opponentChannels =
        MatcherUtils::convertBGRImageToOpponentColorSpace(bgrImage);

    std::vector<cv::KeyPoint> channelKeypoints[matcher::NOPP];
    cv::Mat channelDescriptors[matcher::NOPP];
    std::vector<int> idxs[matcher::NOPP];

    // Compute descriptors three times, once for each Opponent channel to
    // concatenate into a single color descriptor
    int maxKeypointsCount = 0;
    for (int ci = 0; ci < matcher::NOPP; ci++) {
        channelKeypoints[ci].insert(channelKeypoints[ci].begin(),
                                    keypoints.begin(),
                                    keypoints.end());
        // Use class_id member to get indices into initial keypoints vector
        for (size_t ki = 0; ki < channelKeypoints[ci].size(); ki++)
            channelKeypoints[ci][ki].class_id = static_cast<int>(ki);

        descriptor_->compute(opponentChannels[ci],
                             channelKeypoints[ci],
                             channelDescriptors[ci]);
        idxs[ci].resize(channelKeypoints[ci].size());
        for (size_t ki = 0; ki < channelKeypoints[ci].size(); ki++) {
            idxs[ci][ki] = static_cast<int>(ki);
        }
        std::sort(idxs[ci].begin(),
                  idxs[ci].end(),
                  KP_LessThan(channelKeypoints[ci]));
        maxKeypointsCount =
                        std::max(maxKeypointsCount,
                                 static_cast<int>(channelKeypoints[ci].size()));
    }

    int dSize = descriptor_->descriptorSize();
    cv::Mat mergedDescriptors(maxKeypointsCount,
                              matcher::NOPP*dSize,
                              descriptor_->descriptorType());
    int mergedCount = merge(dSize, keypoints, mergedDescriptors, idxs,
                            channelDescriptors, channelKeypoints,
                            maxKeypointsCount);

    mergedDescriptors.rowRange(0, mergedCount).copyTo(descriptors);
}

int OpponentColorDescriptor::merge(const int &dSize,
                vector<cv::KeyPoint> &keypoints,
                cv::Mat &mergedDescriptors,
                std::vector<int> (&idxs)[matcher::NOPP],
                cv::Mat (&channelDescriptors)[matcher::NOPP],
                std::vector<cv::KeyPoint> (&channelKeypoints)[matcher::NOPP],
                const int &maxKeypointsCount) const {
    std::vector<cv::KeyPoint> outKeypoints;
    outKeypoints.reserve(keypoints.size());

    int mergedCount = 0;
    // cp - current channel position
    size_t cp[] = {0, 0, 0};
    const int idx1 = MIN(1, matcher::NOPP-1);
    const int idx2 = MIN(2, matcher::NOPP-1);
    while (cp[0] < channelKeypoints[0].size() &&
           cp[1] < channelKeypoints[idx1].size() &&
           cp[2] < channelKeypoints[idx2].size()) {
        const int maxInitIdx =
                std::max(0,
                  std::max(channelKeypoints[0][idxs[0][cp[0]]].class_id,
                    std::max(channelKeypoints[idx1][idxs[1][cp[1]]].class_id,
                             channelKeypoints[idx2][idxs[2][cp[2]]].class_id)));

        for (int i = 0; i < matcher::NOPP; i++) {
            while (channelKeypoints[i][idxs[i][cp[i]]].class_id < maxInitIdx &&
                   cp[i] < channelKeypoints[i].size()) {
                cp[i]++;
            }
        }

        if (cp[0] >= channelKeypoints[0].size() ||
            cp[1] >= channelKeypoints[idx1].size() ||
            cp[2] >= channelKeypoints[idx2].size() )
            break;

        mergeDescriptors(dSize, keypoints, outKeypoints, mergedDescriptors,
                         idxs, channelDescriptors, channelKeypoints,
                         &mergedCount, maxInitIdx, cp, idx1, idx2);
    }
    std::swap(outKeypoints, keypoints);
    return mergedCount;
}

void OpponentColorDescriptor::mergeDescriptors(
                const int &dSize,
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
                const int& idx2) const {
    if (channelKeypoints[0][idxs[0][cp[0]]].class_id == maxInitIdx &&
        channelKeypoints[idx1][idxs[1][cp[1]]].class_id == maxInitIdx &&
        channelKeypoints[idx2][idxs[2][cp[2]]].class_id == maxInitIdx) {
        outKeypoints.push_back(keypoints[maxInitIdx]);
        // merge descriptors
        for (int ci = 0; ci < matcher::NOPP; ci++) {
            cv::Mat dst = mergedDescriptors(cv::Range(*mergedCount,
                                                      (*mergedCount)+1),
                                            cv::Range(ci*dSize,
                                                      (ci+1)*dSize));
            channelDescriptors[ci].row(idxs[ci][cp[ci]]).copyTo(dst);
            cp[ci]++;
        }
        (*mergedCount)++;
    }
}
