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
#include "MatchEntry.hpp"


MatchEntry::MatchEntry(const cv::Mat img) : image(img) {}

MatchEntry::~MatchEntry() {}

bool MatchEntry::isValid() {
    if (this->image.empty() || this->keypoints.size() == 0 ||
        this->descriptors.empty())
        return false;
    return true;
}
