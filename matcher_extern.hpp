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

#ifndef matcher_extern_HPP
#define matcher_extern_HPP

#include "Matcher.hpp"

using matcher::MatchCQuery;
using matcher::MatchCResult;

extern "C" {
    Matcher* Matcher_new(const MatcherConfig &mconfig) {
        return new Matcher(mconfig);
    }

    bool Matcher_loadImage(Matcher* matcher, const char* screenshot) {
        return matcher->loadImage(screenshot);
    }

    bool Matcher_unloadImage(Matcher* matcher, const char* screenshot) {
        return matcher->unloadImage(screenshot);
    }

    void Matcher_match(Matcher* matcher,
                       const MatchCQuery &mquery,
                       MatchCResult *mcresult) {
        MatchResult *mresult = new MatchResult(mquery.maxlocations);
        matcher->match(MatchQuery(mquery), mresult);
        for (size_t i = 0; i < mresult->result.size(); i++) {
            mcresult->result[i] = mresult->result.at(i);
            mcresult->bbox[i] = matcher::Rect(mresult->bbox.at(i));
            mcresult->center[i] = mresult->center.at(i);
        }
        mcresult->resultimage = strdup(mresult->resultimage.c_str());
        mcresult->message = strdup(mresult->message.c_str());
        delete mresult;
    }

    void Matcher_locateCharacters(Matcher* matcher,
                                  const MatchCQuery &mquery,
                                  MatchCResult *mcresult) {
        MatchResult *mresult = new MatchResult(mquery.maxlocations);
        matcher->locateCharacters(MatchQuery(mquery), mresult);
        mcresult->result[0] = mresult->result[0];
        for (size_t i = 0; i < mresult->bbox.size(); i++) {
            mcresult->bbox[i] = matcher::Rect(mresult->bbox.at(i));
            mcresult->center[i] = mresult->center.at(i);
        }
        mcresult->resultimage = strdup(mresult->resultimage.c_str());
        mcresult->message = strdup(mresult->message.c_str());
        delete mresult;
    }
}

#endif  // matcher_extern_HPP
