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

#include <map>
#include <string>
#include "matcher_types.hpp"
#include "matcher_consts.hpp"

namespace matcher {
    // Point

    Point::Point() : x(0), y(0) {}

    Point::Point(int32_t _x, int32_t _y) :  x(_x), y(_y) {}

    std::ostream& operator <<(std::ostream& os, const Point& p) {
         os << "[" << p.x << ", " << p.y << "]";
         return os;
    }

    // Rect

    Rect::Rect() : x(0), y(0), width(0), height(0) {}

    Rect::Rect(const cv::Rect &rect)
        : x(rect.x), y(rect.y), width(rect.width), height(rect.height) {}

    Rect::Rect(const Rect &r)
        : x(r.x), y(r.y), width(r.width), height(r.height) {}

    Rect::Rect(int32_t _x, int32_t _y, int32_t _w, int32_t _h)
        : x(_x), y(_y), width(_w), height(_h) {}

    bool Rect::empty() const {
        if (!x && !y && !width && !height)
            return true;
        return false;
    }

    std::ostream& operator <<(std::ostream& os, const Rect& r ) {
         os << "[" << r.x << ", " << r.y << "] (" << r.width << ", "
            << r.height << ")";
         return os;
    }

    // MatchCQuery

    MatchCQuery::MatchCQuery()
        : screenshot(0), icon(0), threshold(0), method(0), roi(0),
          scale_factor(0), resultimage(0), maxlocations(1) {}

    MatchCQuery::MatchCQuery(const MatchCQuery &m)
        : screenshot(0),
          icon(0),
          threshold(m.threshold),
          method(m.method),
          roi(0),
          searcharea(m.searcharea),
          scale_factor(m.scale_factor),
          resultimage(0),
          maxlocations(m.maxlocations) {}

    // MatchQuery

    MatchQuery::MatchQuery()
        : MatchCQuery(), resultimage(matcher::RESULTPATH) {}

    MatchQuery::MatchQuery(const MatchCQuery &m)
        : MatchCQuery(m),
          screenshot(m.screenshot ? string(m.screenshot) : string()),
          icon(m.icon ? string(m.icon) : string()),
          roi(m.roi ? string(m.roi) : string()),
          resultimage(m.resultimage ? string(m.resultimage)
                                    : matcher::RESULTPATH) {}

    MatchQuery& MatchQuery::operator =(const MatchQuery& m) {
        screenshot = m.screenshot;
        icon = m.icon;
        threshold = m.threshold;
        method = m.method;
        roi = m.roi;
        searcharea = m.searcharea;
        scale_factor = m.scale_factor;
        resultimage = m.resultimage;
        maxlocations = m.maxlocations;
        return *this;
    }

    // MatchResult

    MatchCResult::MatchCResult()
        : result(0), bbox(0), center(0), resultimage(0), nInliers(0),
          nMatches(0), message(0) {}

    MatchResult::MatchResult(int16_t maxlocations) : MatchCResult() {
          if (maxlocations < 1)
              maxlocations = 1;
          result.resize(maxlocations);
          bbox.resize(maxlocations);
          center.resize(maxlocations);
    }

    void MatchResult::operator()(const MatchResult &m, const int leftidx,
                                 const int rightidx) {
        result[leftidx] = m.result[rightidx];
        bbox[leftidx] = m.bbox[rightidx];
        center[leftidx] = m.center[rightidx];
        message = m.message;
    }

    bool MatchResult::operator() (MatchResult i, MatchResult j) {
        return (i.result[0] > j.result[0]);
    }

    void MatchResult::clear() {
        result.clear();
        bbox.clear();
        center.clear();
    }

    void MatchResult::resize(const unsigned int size) {
        result.resize(size);
        bbox.resize(size);
        center.resize(size);
    }

    // LocateMethod

    map<int16_t, string> create_locate_map() {
        map<int16_t, string> m;
        m[LocateMethod::FEATURE] = "FEATURE";
        m[LocateMethod::MATCHTEMPLATE] = "MATCHTEMPLATE";
        m[LocateMethod::SOBEL] = "SOBEL";
        m[LocateMethod::OCR] = "OCR";
        return m;
    }

    map<int16_t, string> LocateMethod::VALUES_TO_NAMES = create_locate_map();


    // FeaturePhases

    map<Feature::phase, string> create_feature_map() {
        map<Feature::phase, string> m;
        m[Feature::DEFAULT] = "Configured ORB's bbox.";
        m[Feature::RAISESCORE] = "Configured ORB's bbox & Score raised.";
        m[Feature::SOBEL] = "Sobel's bbox & Configured ORB.";
        m[Feature::OPPONENT] = "Sobel's bbox & Opponent Colour Space.";
        return m;
    }

    map<Feature::phase, string> Feature::VALUES_TO_DESC = create_feature_map();
}
