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

#include <resultiterator.h>  // for tesseract OCR
#include <string>
#include <vector>
#include <map>
#include <algorithm>  // for sort
#include <iomanip>  // for setprecision
#include <climits>  // for INT_MAX

#include "Matcher.hpp"
#include "MatchEntry.hpp"
#include "MatchEntryGPU.hpp"
#include "OpponentColorDescriptor.hpp"
#include "matcher_consts.hpp"
#include "matcher_extern.hpp"
#include "MatcherUtils.hpp"

using matcher::LocateMethod;

Matcher::Matcher(const MatcherConfig &mconfig)
    :   entry_(new MatchEntry()),
        qentry_(new MatchEntry()) {
    configure(mconfig);
}

Matcher::~Matcher() {
    entry_.release();
    qentry_.release();
    orb_.release();
    matcher_.release();
    if (conf_.gpu_enabled) {
        entryGPU_.release();
        qentryGPU_.release();
        orbGPU_.release();
        matcherGPU_.release();
    }
}

// Private

void Matcher::configure(const MatcherConfig &mconfig) {
    conf_ = mconfig;

    // Initialize OCR
    if (conf_.useOCR) {
        tess_ = new tesseract::TessBaseAPI();
        tess_->Init(NULL, matcher::LANGUAGE, tesseract::OEM_DEFAULT);
        tess_->SetPageSegMode(tesseract::PSM_AUTO);
    }

    // Quit configure if not using anything else but OCR
    if (!conf_.useOIR)
        return;

    // Get GPU device
    if (conf_.gpu_enabled && cv::gpu::getCudaEnabledDeviceCount() != 0) {
        entryGPU_ = new MatchEntryGPU();
        qentryGPU_ = new MatchEntryGPU();
    } else {
        conf_.gpu_enabled = false;
    }

    // Set GPU parameters
    if (conf_.gpu_enabled) {
        orbGPU_.release();
        matcherGPU_.release();
        // GPU constructors
        orbGPU_ = new cv::gpu::ORB_GPU(conf_.nfeatures,
                                       conf_.scaleFactor,
                                       conf_.nlevels,
                                       conf_.edgeThreshold,
                                       conf_.firstLevel,
                                       conf_.WTA_K,
                                       conf_.scoreType,
                                       conf_.patchSize);
        matcherGPU_ = new cv::gpu::BruteForceMatcher_GPU_base(
                            cv::gpu::BruteForceMatcher_GPU_base::HammingDist);
    } else {
        orb_.release();
        matcher_.release();
        // CPU constructors
        orb_ = new cv::ORB();
        orbcustom_ = new cv::ORB(conf_.nfeatures,
                                 conf_.scaleFactor,
                                 conf_.nlevels,
                                 conf_.edgeThreshold,
                                 conf_.firstLevel,
                                 conf_.WTA_K,
                                 conf_.scoreType,
                                 conf_.patchSize);
        opponent_ = new OpponentColorDescriptor(orb_);
        matcher_ = new cv::BFMatcher(cv::NORM_HAMMING);
    }
    // Set order of feature matching phases.
    static const Feature::phase arr[] = { Feature::DEFAULT,
                                          Feature::RAISESCORE,
                                          Feature::SOBEL,
                                          Feature::OPPONENT };
    featurephases_ = std::vector<Feature::phase>(arr, arr + sizeof(arr)/
                                                            sizeof(arr[0]));
}

// Public methods

cv::Ptr<cv::Mat> Matcher::match(const cv::Mat &image,
                                const MatchQuery &mquery,
                                MatchResult *mresult,
                                cv::Ptr<MatchEntry> entry,
                                cv::Ptr<MatchEntry> roientry) {
    if (!mresult)
        mresult = new MatchResult(mquery.maxlocations);
    if ((!entry || !conf_.useOIR) && mquery.method != LocateMethod::OCR) {
        mresult->result[0] = matcher::input_entry_err;
        return NULL;
    }
    qentry_ = entry;

    switch (mquery.method) {
        case LocateMethod::MATCHTEMPLATE:
        case LocateMethod::SOBEL:
            action = &Matcher::runTemplateMatching;
            break;
        case LocateMethod::OCR:
            action = &Matcher::runOCR;
            break;
        case LocateMethod::FEATURE:
        default:
            action = &Matcher::runFeatureMatching;
    }

    try {
        if (!validateImages(image, mquery, mresult))
            return NULL;
        if (conf_.locate && ((mquery.roi.length() > 0) ||
            !mquery.searcharea.empty())) {
            if (!locateROI(mquery, mresult, roientry))
                return NULL;
        }
        return (this->*action)(mquery, mresult);
    } catch(cv::Exception const& e) {
        mresult->result[0] = matcher::verification_failed;
        mresult->message = e.what();
    } catch(...) {
        mresult->result[0] = matcher::verification_failed;
        mresult->message = "Unknown exception in match.";
    }
    return NULL;
}

void Matcher::match(const MatchQuery &mquery, MatchResult *mresult) {
    try {
        loadImages(mquery, mresult);
        cv::Ptr<cv::Mat> resultimage = match(image_, mquery, mresult, qentry_,
                                             roientry_);
        if (resultimage == NULL || mquery.resultimage.empty())
            return;
        drawMatchDetails(mquery, mresult, resultimage);
        MatcherUtils::saveResultImage(resultimage, mquery, mresult,
                                      screenshot_);
    } catch(...) {
        mresult->result[0] = matcher::verification_failed;
    }
}

cv::Ptr<cv::Mat> Matcher::locateCharacters(const cv::Mat &frame,
                                           const MatchQuery &mquery,
                                           MatchResult *mresult) {
    if (!conf_.useOCR)
        return NULL;
    try {
        if (!validateImages(frame, mquery, mresult))
            return NULL;
        // Set ROI
        cv::Mat image = MatcherUtils::convertToGray(frame);
        cv::Rect roi = cv::Rect(0, 0, image.cols, image.rows);
        if (!mquery.searcharea.empty()) {
            roi = cv::Rect(mquery.searcharea.x, mquery.searcharea.y,
                           mquery.searcharea.width, mquery.searcharea.height);
        }

        // Remove duplicate letters
        std::string noduplicate(mquery.icon);
        MatcherUtils::removeDuplicateLetters(&noduplicate);
        mresult->clear();
        mresult->resize(noduplicate.length());

        // Create map character <-> bounding box
        boost::unordered_map<char, cv::Rect> lettermap;
        boost::unordered_map<char, float> confmap;
        mresult->result[0] = mapCharToBBox(image, roi, mresult, lettermap,
                                           confmap, mquery.icon, noduplicate,
                                           mquery.threshold);

        // Create result vector
        int idx = 0;
        for (size_t i = 0; i < mquery.icon.length(); i++) {
            if (lettermap.find(mquery.icon[i]) != lettermap.end()) {
                mresult->bbox[idx] = lettermap[mquery.icon[i]];
                idx++;
            }
        }
        MatcherUtils::calcCenter(mresult);
        return drawchars(mquery, mresult, roi, &lettermap, &confmap);
    } catch(cv::Exception const& e) {
        mresult->result[0] = matcher::verification_failed;
        mresult->message = e.what();
    } catch(...) {
        mresult->result[0] = matcher::verification_failed;
        mresult->message = "Unknown exception in locateCharacters.";
    }
    return NULL;
}

void Matcher::locateCharacters(const MatchQuery &mquery, MatchResult *mresult) {
    try {
        loadImages(mquery, mresult);
        cv::Ptr<cv::Mat> resultimage = locateCharacters(image_, mquery,
                                                        mresult);
        if (resultimage == NULL || mquery.resultimage.empty())
            return;
        drawMatchDetails(mquery, mresult, resultimage);
        MatcherUtils::saveResultImage(resultimage, mquery, mresult,
                                      screenshot_);
    } catch(...) {
        mresult->result[0] = matcher::verification_failed;
    }
}

bool Matcher::loadImage(const char* &screenshot) {
    if (screenshot && strlen(screenshot) > 0) {
        screenshot_ = std::string(screenshot);
        if (images_.find(screenshot_) == images_.end())
            images_[screenshot_] = cv::imread(screenshot);
        image_.release();
        image_ = images_[screenshot_];
        return true;
    }
    return false;
}

bool Matcher::unloadImage(const char* &screenshot) {
    boost::unordered_map<std::string, cv::Mat>::iterator it =
        images_.find(std::string(screenshot));
    if (it != images_.end()) {
        images_.erase(it);
        return true;
    }
    return false;
}

// Private methods

bool Matcher::validateImages(const cv::Mat &image,
                             const MatchQuery &mquery,
                             MatchResult* mresult) {
    entry_.release();
    if (image.empty() ||
        (mquery.method != LocateMethod::OCR && qentry_->image.empty())) {
        mresult->result[0] = matcher::empty_image_err;
        return false;
    }
    entry_ = new MatchEntry(image);
    if (MatcherUtils::isBlack(entry_->image)) {
        mresult->result[0] = matcher::screen_off_identified;
        return false;
    }
    return true;
}

void Matcher::loadImages(const MatchQuery &mquery, MatchResult *mresult) {
    if (!mquery.screenshot.empty()) {
        image_.release();
        if (images_.find(mquery.screenshot) == images_.end())
            image_ = cv::imread(mquery.screenshot);
        else
            image_ = images_[mquery.screenshot];
    }
    qentry_.release();
    if (mquery.method != LocateMethod::OCR)
        qentry_ = new MatchEntry(cv::imread(mquery.icon));
    roientry_.release();
    if (!mquery.roi.empty())
        roientry_ = new MatchEntry(cv::imread(mquery.roi));
}

bool Matcher::locateROI(const MatchQuery &mquery,
                        MatchResult *mresult,
                        cv::Ptr<MatchEntry> roientry) {
    // Backup and create new query
    MatchQuery queryroi = mquery;
    queryroi.icon = mquery.roi;
    cv::Ptr<MatchEntry> refentry = qentry_;

    if (!mquery.searcharea.empty()) {
        entry_->image = MatcherUtils::setROI(entry_->image, mquery.searcharea);
    } else {
        qentry_ = roientry;
        cv::Ptr<cv::Mat> resultimage = (this->*action)(queryroi, mresult);
        if (resultimage == NULL || mresult->result[0] <= mquery.threshold) {
            mresult->result[0] = matcher::verification_failed;
            return false;
        }
        entry_->image = MatcherUtils::setROI(entry_->image, mresult->bbox[0]);
    }

    // Restore
    qentry_ = refentry;
    return true;
}

cv::Ptr<cv::Mat> Matcher::draw(const MatchQuery &mquery,
                               MatchResult* mresult,
                               const Draw::flag &flag) {
    cv::Mat img, rightimage;
    std::vector<cv::DMatch> matches = filtered_matches_;
    cv::Scalar color;

    if (entry_->image.empty() ||
        (qentry_ && qentry_->image.empty() && flag != Draw::NO_REF_IMAGE
         && flag != Draw::NO_BBOX))
        return NULL;

    // Draw keypoints, matches and boundning box
    switch (flag) {
    case Draw::ONLY_KEYPOINTS:
        matches.clear();
        MatcherUtils::drawMatches(entry_->image,
                                  entry_->keypoints,
                                  qentry_->image,
                                  qentry_->keypoints,
                                  matches, img);
        break;
    case Draw::NO_REF_IMAGE:
    case Draw::NO_BBOX:
        rightimage = cv::Mat(entry_->image.size(), entry_->image.type());
        rightimage = cv::Scalar(0);
        img = MatcherUtils::combine(entry_->image, rightimage);
        break;
    case Draw::DRAW_MATCHES:
        MatcherUtils::drawMatches(entry_->image,
                                  entry_->keypoints,
                                  qentry_->image,
                                  qentry_->keypoints,
                                  matches, img,
                                  cv::Scalar::all(-1), CV_RGB(0, 0, 255), \
                                  matches_mask_,
                                  cv::DrawMatchesFlags::DEFAULT);
        break;
    case Draw::ONLY_BBOX:
    default:
        img = MatcherUtils::combine(entry_->image, qentry_->image);
    }

    for (int i = 0; i < mquery.maxlocations; i++) {
        if (mresult->result[i] < 0 || !conf_.locate || flag == Draw::NO_BBOX)
            continue;
        // Draw bounding box
        color = MatcherUtils::getColor(mresult->result[i],
                                       mquery.threshold);
        cv::Point tl(mresult->bbox[i].x, mresult->bbox[i].y);
        cv::Point br(mresult->bbox[i].x + mresult->bbox[i].width,
                     mresult->bbox[i].y + mresult->bbox[i].height);
        cv::rectangle(img, tl, br, color, 4, 4, 0);
        // Draw cross in the middle of bounding box
        cv::Point cross[4];
        int d = MIN(MIN(mresult->bbox[i].width, mresult->bbox[i].height)/5,
                    matcher::MAXCROSSSIZE);
        cross[0] = cv::Point(mresult->center[i].x-d, mresult->center[i].y);
        cross[1] = cv::Point(mresult->center[i].x+d, mresult->center[i].y);
        cross[2] = cv::Point(mresult->center[i].x, mresult->center[i].y-d);
        cross[3] = cv::Point(mresult->center[i].x, mresult->center[i].y+d);
        cv::line(img, cross[0], cross[1], color, 4, 4, 0);
        cv::line(img, cross[2], cross[3], color, 4, 4, 0);
        // Put index
        std::ostringstream ss;
        ss << mresult->result[i];
        cv::putText(img, ss.str(), cv::Point(tl.x, tl.y-10), matcher::FONT,
                    matcher::FONTSCALE, color, matcher::FONTTHICKNESS);
    }
    return cv::Ptr<cv::Mat>(new cv::Mat(img));
}

cv::Ptr<cv::Mat> Matcher::drawchars(
                      const MatchQuery &mquery,
                      MatchResult* mresult,
                      const cv::Rect roi,
                      const boost::unordered_map<char, cv::Rect> *lettermap,
                      const boost::unordered_map<char, float> *confmap) {
    std::ostringstream stext;
    cv::Ptr<cv::Mat> resultimage = draw(mquery, mresult, Draw::NO_BBOX);
    if (resultimage == NULL)
        return NULL;

    cv::Scalar color = MatcherUtils::getColor(mresult->result[0],
                                              mquery.threshold);
    if (roi != cv::Rect() \
        && roi != cv::Rect(0, 0, entry_->image.cols, entry_->image.rows))
        cv::rectangle(*resultimage, roi.tl(), roi.br(), color, 2, 2, 0);
    if (!lettermap || !confmap)
        return resultimage;

    boost::unordered_map<char, cv::Rect>::const_iterator it;
    for (it = lettermap->begin();
        it != lettermap->end(); ++it) {
        cv::rectangle(*resultimage, it->second.tl(), it->second.br(), color,
                      4, 4, 0);
        const char* letter = &it->first;
        cv::putText(*resultimage, letter, it->second.br()+cv::Point(4, 0),
                    matcher::FONT, matcher::FONTSCALE, color,
                    matcher::FONTTHICKNESS);
        cv::Point orig = it->second.tl() + cv::Point(0, it->second.height+20);
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << confmap->at(it->first);
        cv::putText(*resultimage, ss.str(), orig, matcher::FONT, 1, color,
                    matcher::FONTTHICKNESS-1);
        stext << it->first;
    }
    ocrtext_ = stext.str();
    return resultimage;
}

void Matcher::drawMatchDetails(const MatchQuery &mquery,
                               MatchResult* mresult,
                               cv::Ptr<cv::Mat> img) {
    if (!img)
        return;

    // Edit text that will be placed onto image
    std::stringstream text[4];
    text[1] << "inliers/outliers: " << mresult->nInliers << "/"
            << mresult->nMatches-mresult->nInliers;
    text[2] << "result/threshold: " << mresult->result[0] << "/"
            << mquery.threshold;
    if (conf_.locate) {
        text[0] << "locate method: "
                << LocateMethod::VALUES_TO_NAMES.at(mquery.method);
        switch (mquery.method) {
        case LocateMethod::MATCHTEMPLATE:
        case LocateMethod::SOBEL:
            text[1].str(std::string(matcher::MAXLINELENGTH/2, '-'));
            break;
        case LocateMethod::OCR:
            if (ocrtext_.length() > matcher::MAXLINELENGTH)
                ocrtext_ = ocrtext_.substr(0, matcher::MAXLINELENGTH) + "(...)";
            text[1].str(std::string("Found text: " + ocrtext_));
            break;
        case LocateMethod::FEATURE:
        default:
            break;
        }
    }

    // Put text onto image
    std::string additionalinfo = mresult->message;
    cv::Scalar color = MatcherUtils::getColor(mresult->result[0],
                                              mquery.threshold,
                                              &additionalinfo);
    text[3] << additionalinfo;
    cv::Mat overlay = img->clone();
    double scale = (img->cols/2)/1080.0;
    cv::Rect rect(img->cols/2, 0, img->cols/2, scale*(matcher::RECTHEIGHT));
    cv::rectangle(*img, rect.tl(), rect.br(), CV_RGB(0, 0, 0), CV_FILLED);
    cv::Point origin(rect.x + scale*matcher::MARGIN, 0);
    cv::addWeighted(overlay, matcher::OPACITY, *img, 1 - matcher::OPACITY, 0,
                    *img);
    for (int i = 0; i < 4; i++) {
        origin.y += scale*matcher::TEXTHEIGHT;
        cv::putText(*img, text[i].str(), origin, matcher::FONT,
                    scale*matcher::FONTSCALE, color,
                    scale*matcher::FONTTHICKNESS);
    }
}

cv::Ptr<cv::Mat> Matcher::runTemplateMatching(const MatchQuery &mquery,
                                              MatchResult* mresult) {
    if (!qentry_)
        return NULL;

    // Backup match entry for reference image
    cv::Ptr<MatchEntry> refentry = new MatchEntry();
    qentry_->image.copyTo(refentry->image);

    if (conf_.scale_invariant && mquery.scale_factor != 0) {
        cv::resize(refentry->image, qentry_->image, cv::Size(),
                   mquery.scale_factor, mquery.scale_factor, cv::INTER_LINEAR);
        matchTemplate(mquery, mresult);
        qentry_ = refentry;
        return draw(mquery, mresult);
    }

    // Match template in a loop and take the highest score
    std::vector<MatchResult> mresults;
    for (double s = 1.1; s <= matcher::MAXSCALE; s += 0.1) {
        MatchResult results(mquery.maxlocations);
        matchTemplate(mquery, &results);
        // Serialize results
        for (int16_t i = 0; i < mquery.maxlocations; i++) {
            MatchResult temp;
            temp(results, i);
            temp.message = MatcherUtils::getmessage("Scaling factor: ", s);
            mresults.push_back(temp);
        }
        if (!conf_.scale_invariant)
            break;
        cv::resize(refentry->image, qentry_->image, cv::Size(), s, s,
                   cv::INTER_LINEAR);
        if (qentry_->image.cols > entry_->image.cols ||
            qentry_->image.rows > entry_->image.rows)
            break;
    }

    // Sort results and save only those with best score (but not overlapping)
    std::sort(mresults.begin(), mresults.end(), *mresult);
    (*mresult)(mresults[0], 0);
    int idx = 1;
    for (uint i = 0; i < mresults.size() && idx < mquery.maxlocations; i++) {
        cv::Rect prev(mresults[i].bbox[0].x, mresults[i].bbox[0].y,
                      mresults[i].bbox[0].width, mresults[i].bbox[0].height);
        for (uint j = 0; j < mresults.size() && idx <mquery.maxlocations; j++) {
            if (i == j)
                continue;
            cv::Rect next(mresults[j].bbox[0].x, mresults[j].bbox[0].y,
                         mresults[j].bbox[0].width, mresults[j].bbox[0].height);
            cv::Rect intersection = prev & next;
            if (intersection == cv::Rect()) {
                mresult->result[idx] = mresults[j].result[0];
                mresult->bbox[idx] = mresults[j].bbox[0];
                mresult->center[idx] = mresults[j].center[0];
                idx++;
            }
        }
    }

    // Prepare for drawing
    qentry_ = refentry;
    return draw(mquery, mresult);
}

cv::Ptr<cv::Mat> Matcher::runOCR(const MatchQuery &mquery,
                                 MatchResult* mresult) {
    if (mquery.icon.length() == 0 || !conf_.useOCR) {
        mresult->result[0] = 0;
        return NULL;
    }

    cv::Mat image, sharpened, thresholded;

    // Upscale image if too small. Tesseract doesn't like low resolution.
    if (entry_->image.cols < matcher::MINOCRIMAGE.width ||
        entry_->image.rows < matcher::MINOCRIMAGE.height) {
        int s = std::floor(matcher::MINOCRIMAGE.height/entry_->image.rows+0.5);
        cv::resize(entry_->image, entry_->image, cv::Size(), s, s,
                   cv::INTER_CUBIC);
    }
    image = MatcherUtils::convertToGray(entry_->image);
    cv::Rect roi = getRoi4Text(image, mquery.icon, mquery.threshold);

    if (mquery.icon.length() == 1) {
        // If requested finding single character then use different approach.
        boost::unordered_map<char, cv::Rect> lettermap;
        boost::unordered_map<char, float> confmap;
        mresult->result[0] = mapCharToBBox(image, roi, mresult, lettermap,
                                           confmap, mquery.icon, mquery.icon,
                                           mquery.threshold);
        mresult->bbox[0] = lettermap[mquery.icon[0]];
        MatcherUtils::calcCenter(mresult);
        return draw(mquery, mresult, Draw::NO_REF_IMAGE);
    }

    // First search in image as it is
    bool found = findText(image, mquery.icon, roi, mquery, mresult);

    // If still not found then perform thresholding
    for (int i = matcher::MINTHRESH; i < matcher::MAXTHRESH && !found;
         i+= matcher::STEPTHRESH) {
        MatcherUtils::thresholdImage(image, &thresholded, i);
        found = findText(thresholded, mquery.icon, roi, mquery, mresult);
        mresult->message = MatcherUtils::getmessage("Thresholding parameter: ",
                                                    i, 0);
    }

    // If it is not found then sharpen and search again
    for (double i = matcher::MINSHARP; i < matcher::MAXSHARP && !found;
         i += matcher::STEPSHARP) {
        MatcherUtils::sharpenImage(image, &sharpened, i);
        found = findText(sharpened, mquery.icon, roi, mquery, mresult);
        mresult->message = MatcherUtils::getmessage("Sharpening parameter: ",
                                                    i, 1);
    }

    MatcherUtils::calcCenter(mresult);
    tess_->ClearAdaptiveClassifier();
    return draw(mquery, mresult, Draw::NO_REF_IMAGE);
}

cv::Ptr<cv::Mat> Matcher::runFeatureMatching(const MatchQuery &mquery,
                                             MatchResult* mresult) {
    if (!computeKeyPoints(mquery.icon)) {
        mresult->result[0] = matcher::input_entry_err;
        return NULL;
    }
    if (!entry_->isValid()) {
        mresult->result[0] = 0;
        mresult->message = "No keypoints found!";
        return draw(mquery, mresult, Draw::ONLY_BBOX);
    }
    matchDescriptors();
    genResults(mquery, mresult);
    for (size_t p = 1; p < featurephases_.size() && conf_.locate; p++) {
        mresult->message = Feature::VALUES_TO_DESC[featurephases_.at(p)];
        verifyLocation(mquery, mresult, featurephases_.at(p));
        if (mresult->result[0] > 0)
            break;
        // Set bounding box (mresult->bbox[0]) using Sobel for all phases once.
        if (p == 1)
            matchTemplate(mquery, mresult);
    }
    return draw(mquery, mresult, Draw::DRAW_MATCHES);
}

bool Matcher::computeKeyPoints(const std::string &name) {
    if (!qentry_->isValid())
        computeMatchEntry(qentry_);
    if (!conf_.gpu_enabled) {
        computeMatchEntry(entry_);
    } else {
        computeMatchEntryGPU(entryGPU_, entry_->image);
        qentryGPU_->descriptors.upload(qentry_->descriptors);
    }
    return true;
}

void Matcher::matchDescriptors() {
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    // Unused variables but required by OpenCV functions.
    cv::gpu::GpuMat trainIdx, distance, allDist;
    filtered_matches_.clear();

    if (conf_.gpu_enabled) {
        matcherGPU_->knnMatchSingle(entryGPU_->descriptors,
                                    qentryGPU_->descriptors,
                                    trainIdx, distance, allDist, matcher::KNN);
        cv::gpu::BruteForceMatcher_GPU_base::knnMatchDownload(trainIdx,
                                                              distance,
                                                              matches12);
        matcherGPU_->knnMatchSingle(qentryGPU_->descriptors,
                                    entryGPU_->descriptors,
                                    trainIdx, distance, allDist, matcher::KNN);
        cv::gpu::BruteForceMatcher_GPU_base::knnMatchDownload(trainIdx,
                                                              distance,
                                                              matches21);
    } else {
        matcher_->knnMatch(entry_->descriptors, qentry_->descriptors,
                           matches12, matcher::KNN);
        matcher_->knnMatch(qentry_->descriptors, entry_->descriptors,
                           matches21, matcher::KNN);
    }

    // Filter matches using cross check, thesholding and ratio test
    for (size_t m = 0; m < matches12.size(); m++) {
        if (matches12[m].size() == 2 && cv::norm(matches12[m].at(0).distance - \
            matches12[m].at(1).distance) < conf_.min_dist_thresh)
            continue;
        if (matches12[m].size() == 2 && matches12[m].at(0).distance > \
            conf_.max_nndr_ratio * matches12[m].at(1).distance)
            continue;
        bool findCrossCheck = false;
        for (size_t fk = 0; fk < matches12[m].size(); fk++) {
            cv::DMatch forward = matches12[m][fk];
            for (size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++) {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if (backward.trainIdx == forward.queryIdx) {
                    filtered_matches_.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if (findCrossCheck)
                break;
        }
    }
    // Cleanup
    trainIdx.release();
    distance.release();
    allDist.release();
}

void Matcher::genResults(const MatchQuery &mquery,
                         MatchResult* mresult,
                         const Feature::phase &phase) {
    // Vectors used for homography
    std::vector<cv::Point2f> mpts, ref_mpts;
    std::vector<int> indexes, ref_indexes;
    std::vector<uchar> outlier_mask;
    matches_mask_.resize(filtered_matches_.size());

    // Download keypoints from GPU
    if (conf_.gpu_enabled) {
        orbGPU_->downloadKeyPoints(entryGPU_->keypoints, entry_->keypoints);
    }

    // Find correspondences
    for (unsigned int i = 0; i < filtered_matches_.size(); ++i) {
        mpts.push_back(entry_->keypoints.at(filtered_matches_[i].queryIdx).pt);
        indexes.push_back(filtered_matches_[i].queryIdx);

        ref_mpts.push_back(qentry_->keypoints.at(
                           filtered_matches_[i].trainIdx).pt);
        ref_indexes.push_back(filtered_matches_[i].trainIdx);
    }

    // Find homography
    if (ref_mpts.size() < static_cast<size_t>(conf_.min_inliers)) {
        mresult->message = MatcherUtils::getmessage("Not enough matches for "
                                               "homography: ", ref_mpts.size());
        mresult->result[0] = 0;
        return;
    }

    cv::Mat H = cv::findHomography(ref_mpts,
                                   mpts,
                                   cv::RANSAC,
                                   conf_.ransac_reprojection_thresh,
                                   outlier_mask);
    if (H.empty()) {
        mresult->message = "Homography matrix is empty.";
        mresult->result[0] = 0;
        return;
    }

    mresult->nInliers = 0;
    for (unsigned int k = 0; k < ref_mpts.size(); ++k) {
        if (outlier_mask.at(k))
            ++mresult->nInliers;
    }
    mresult->nMatches = ref_mpts.size();
    mresult->result[0] = static_cast<float>(100*mresult->nInliers)/ \
                     static_cast<float>(mresult->nMatches);

    matches_mask_.resize(outlier_mask.size());
    for (size_t i = 0; i < outlier_mask.size(); i++)
        matches_mask_[i] = outlier_mask[i];

    if (mresult->nInliers < conf_.min_inliers) {
        mresult->message = MatcherUtils::getmessage("Too little inliers: ",
                                                    mresult->nInliers, 0);
        mresult->result[0] = 0;
        return;
    }

    if (conf_.locate && phase != Feature::RAISESCORE) {
        // Get the corners of the located object
        std::vector<cv::Point2f> corners(4);
        cv::Rect rect(0, 0, qentry_->image.cols, qentry_->image.rows);
        cv::perspectiveTransform(MatcherUtils::rectToVector(rect), corners, H);
        rect = MatcherUtils::boundingBox(corners, entry_->image.size());
        // Check whether returned bounding box is valid
        bool sizevalid = MatcherUtils::checkSize(rect, qentry_->image.size(),
                                                 conf_.scale_invariant);
        if (!MatcherUtils::checkAspectRatio(rect, qentry_->image.size()) ||
            !sizevalid) {
            mresult->message = sizevalid
                               ? "Located object size is invalid."
                               : "Located object aspect ratio is invalid";
            mresult->result[0] = 0;
        } else {
            mresult->bbox[0] = rect;
            MatcherUtils::calcCenter(mresult);
        }
    }
}

void Matcher::verifyLocation(const MatchQuery &mquery,
                             MatchResult *mresult,
                             const Feature::phase &phase) {
    if (phase == Feature::RAISESCORE && mresult->result[0] <= 0)
        return;

    // Set a mask, i.e. region of interest where keypoints will be detected
    entry_->mask = MatcherUtils::getMask(entry_->image.size(),
                                         mresult->bbox[0]);

    if (phase == Feature::OPPONENT) {
        cv::Ptr<MatchEntry> temp = new MatchEntry(qentry_->image);
        computeMatchEntry(temp, Feature::OPPONENT);
        if (!temp->isValid()) {
            mresult->result[0] = 0;
            return;
        }
        qentry_ = temp;
        if (conf_.gpu_enabled) {
            qentryGPU_->descriptors.upload(qentry_->descriptors);
        }
    }

    // Match ROI to the template image
    if (conf_.gpu_enabled) {
        computeMatchEntryGPU(entryGPU_, entry_->image);
        entryGPU_->mask.upload(entry_->mask);
    } else {
        computeMatchEntry(entry_, phase);
        if (!entry_->isValid()) {
            mresult->result[0] = 0;
            return;
        }
    }
    matchDescriptors();
    genResults(mquery, mresult, phase);
}

void Matcher::matchTemplate(const MatchQuery &mquery,
                            MatchResult *mresult) {
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::Mat image;

    // Assumption made here is that both entry_->image and qentry_->image
    // have the same colour scale. If they are not, then exception will be
    // raised here.
    cv::Mat result(entry_->image.cols - qentry_->image.cols + 1,
                   entry_->image.rows - qentry_->image.rows + 1, CV_32FC1);
    if (mquery.method == LocateMethod::MATCHTEMPLATE) {
        cv::matchTemplate(entry_->image, qentry_->image, result,
                          CV_TM_SQDIFF_NORMED);
    } else {
        // Calculates the first image derivatives using an extended Sobel
        // operator.
        cv::Mat refimage;
        cv::Sobel(entry_->image, image, -1, 1, 1);
        cv::Sobel(qentry_->image, refimage, -1, 1, 1);
        cv::matchTemplate(image, refimage, result, CV_TM_CCOEFF_NORMED);
    }

    for (int i = 0; i < mquery.maxlocations; i++) {
        cv::Mat mask(result.size(), CV_8UC1, cv::Scalar(255));
        for (int j = 0; j < i; j++) {
            cv::Rect bbox(mresult->bbox[j].x, mresult->bbox[j].y, 1, 1);
            mask(bbox).setTo(cv::Scalar::all(0));
        }
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, mask);
        mresult->bbox[i] = mquery.method == LocateMethod::MATCHTEMPLATE
                         ? cv::Rect(minLoc.x, minLoc.y, qentry_->image.cols,
                                    qentry_->image.rows)
                         : cv::Rect(maxLoc.x, maxLoc.y, qentry_->image.cols,
                                    qentry_->image.rows);
        mresult->result[i] = mquery.method == LocateMethod::MATCHTEMPLATE
                           ? matcher::MAXCONFIDENCE*(1-minVal)
                           : matcher::MAXCONFIDENCE*maxVal;
    }
    MatcherUtils::calcCenter(mresult);
}

bool Matcher::findText(const cv::Mat &image, const std::string &textinput,
                       const cv::Rect &roi, const MatchQuery &mquery,
                       MatchResult *mresult) {
    tess_->TesseractRect(image.data, 1, image.step1(),
                         roi.x, roi.y, roi.width, roi.height);

    int textinputlength = textinput.length();
    tesseract::PageIteratorLevel level;
    level = MatcherUtils::getPageIterationLevel(textinput);
    tesseract::ResultIterator* it = tess_->GetIterator();
    int idx = 0;
    int dist = INT_MAX;
    do {
        char* text = it->GetUTF8Text(level);
        if (!text)
            continue;
        cv::Rect bbox;
        it->BoundingBox(level, &bbox.x, &bbox.y,
                        &bbox.width, &bbox.height);
        bbox.width-=bbox.x;
        bbox.height-=bbox.y;

        // Calculate Levenshtein edit distance
        int d = MatcherUtils::distance(textinput.c_str(), textinputlength,
                                       text, strlen(text));
        if (d < dist) {
            dist = d;
            ocrtext_ = std::string(text);
        }

        if (level == tesseract::RIL_TEXTLINE) {
            size_t pos = std::string(text).find(textinput.c_str());
            if (pos == std::string::npos) {
                delete[] text;
                continue;
            }
            dist = 0;
            ocrtext_ = std::string(text);
            // Strip 'new line' character
            ocrtext_.erase(std::remove(ocrtext_.begin(), ocrtext_.end(), '\n'),
                           ocrtext_.end());
            // Correct bounding box
            int linelenght = ocrtext_.length();
            double letterwidth = static_cast<double>(bbox.width)/
                                 static_cast<double>(linelenght);
            bbox.x+=letterwidth*(pos);
            bbox.width-=letterwidth*(linelenght-textinputlength);
            // Correct text
            ocrtext_ = ocrtext_.substr(pos, textinputlength);
        }
        delete[] text;

        // For multiple-OCR
        int result = matcher::MAXCONFIDENCE*(1-static_cast<double>(dist)/
                                  static_cast<double>(textinputlength));
        if (result > mresult->result[idx]) {
            mresult->result[idx] = result;
            mresult->bbox[idx] = bbox;
        }
        if (result >= mquery.threshold) {
            mresult->result[idx] = result;
            mresult->bbox[idx] = bbox;
            dist = INT_MAX;
            idx++;
        }
    } while (it->Next(level) && idx < mquery.maxlocations);
    return idx == mquery.maxlocations;
}

int Matcher::mapCharToBBox(const cv::Mat &image, const cv::Rect &roi,
                           MatchResult *mresult,
                           boost::unordered_map<char, cv::Rect> &lettermap,
                           boost::unordered_map<char, float> &confmap,
                           const std::string &inputtext,
                           const std::string &noduplicate,
                           const int &threshold) {
    cv::Mat thresholded;
    boost::unordered_map<char, cv::Rect> bboxes;
    boost::unordered_map<char, float> confs;

    for (size_t i = 0; i < noduplicate.length(); i++) {
        confs[noduplicate[i]] = 0.0f;
        tess_->TesseractRect(image.data, 1, image.step1(),
                             roi.x, roi.y, roi.width, roi.height);
        findLetter(noduplicate[i], bboxes, &confs[noduplicate[i]]);
    }
    lettermap = bboxes;
    confmap = confs;
    float maxaverageconf = MatcherUtils::calcAverageConf(confs);

    // Perform thresholding to achieve higher contrast images
    for (int i = matcher::MINTHRESH; i < matcher::MAXTHRESH;
         i+= matcher::STEPTHRESH) {
        bboxes.clear();
        confs.clear();
        MatcherUtils::thresholdImage(image, &thresholded, i);
        tess_->TesseractRect(thresholded.data, 1, thresholded.step1(),
                             roi.x, roi.y, roi.width, roi.height);
        for (size_t k = 0; k < noduplicate.length(); k++) {
            confs[noduplicate[k]] = 0.0f;
            findLetter(noduplicate[k], bboxes, &confs[noduplicate[k]]);
        }
        float averageconf = MatcherUtils::calcAverageConf(confs);
        if (averageconf > maxaverageconf) {
            maxaverageconf = averageconf;
            lettermap.swap(bboxes);
            confmap.swap(confs);
            mresult->message = MatcherUtils::getmessage("Thresholding "
                                                        "parameter: ", i, 0);
        }
    }
    return MatcherUtils::calcTypeMessageResult(inputtext, lettermap);
}

void Matcher::findLetter(const char &character,
                         boost::unordered_map<char, cv::Rect> &map,
                         float *conf) {
    std::string letter(1, character);
    tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
    tesseract::ResultIterator* it = tess_->GetIterator();
    it->Begin();
    do {
        char* text = it->GetUTF8Text(level);
        if (!text)
            continue;
        cv::Rect bbox;
        it->BoundingBox(level, &bbox.x, &bbox.y,
                        &bbox.width, &bbox.height);
        bbox.width-=bbox.x;
        bbox.height-=bbox.y;

        if (strlen(text) == 1 && strcmp(text, letter.c_str()) == 0) {
            float textconf = it->Confidence(level);
            if (textconf > *conf) {
                *conf = textconf;
                ocrtext_ = std::string(text);
                map[character] = bbox;
            }
            continue;
        }
        delete[] text;
    } while (it->Next(level));
}

cv::Rect Matcher::getRoi4Text(const cv::Mat &image, const std::string &message,
                              const int &threshold) {
    cv::Rect roi(0, 0, image.cols, image.rows);

    // If input text doesn't contain new line then return full frame ROI
    size_t pos = message.find('\n');
    if (pos == std::string::npos)
        return roi;

    // Get first line from message
    std::string firstline = message.substr(0, pos);

    // Get last line
    pos = message.rfind('\n', message.length());
    std::string lastline = std::string(message.substr(pos+1));

    MatchResult mresult(2);
    cv::Mat sharpened, thresholded;
    MatchQuery mquery;
    mquery.threshold = threshold;
    mquery.maxlocations = 1;

    // Find first line
    MatchResult res;  // temporary result
    bool found = findText(image, firstline, roi, mquery, &res);
    for (int i = matcher::MINTHRESH; i < matcher::MAXTHRESH && !found;
         i+= matcher::STEPTHRESH) {
        MatcherUtils::thresholdImage(image, &thresholded, i);
        found = findText(thresholded, firstline, roi, mquery, &res);
    }
    mresult(res);

    if (!found)
        return roi;

    // Set region of interest for the last line search
    double letw = static_cast<double>(mresult.bbox[0].width) /
                  static_cast<double>(firstline.length());  // letter width
    cv::Rect roilastline(cv::Point(MAX(mresult.bbox[0].x - \
                            matcher::PARAINDENT*letw, 0), mresult.bbox[0].y),
                         cv::Point(image.cols, image.rows));

    // Find last line
    found = findText(image, lastline, roilastline, mquery, &res);
    for (int i = matcher::MINTHRESH; i < matcher::MAXTHRESH && !found;
         i+= matcher::STEPTHRESH) {
        MatcherUtils::thresholdImage(image, &thresholded, i);
        found = findText(thresholded, lastline, roilastline, mquery, &res);
    }
    mresult(res, 1);

    return (mresult.result[0] > 0 && mresult.result[1] > 0) ?
            mresult.bbox[0] | mresult.bbox[1] : roi;
}

void Matcher::computeMatchEntry(cv::Ptr<MatchEntry> entry,
                                const Feature::phase &phase) {
    if (entry->image.empty())
        return;

    if (conf_.gpu_enabled) {
        cv::Ptr<MatchEntryGPU> entryGPU = new MatchEntryGPU();
        entryGPU->mask.upload(entry->mask);
        computeMatchEntryGPU(entryGPU, entry->image);
        orbGPU_->downloadKeyPoints(entryGPU->keypoints, entry->keypoints);
        entry->descriptors = cv::Mat(entryGPU->descriptors);
        return;
    }
    try {
        switch (phase) {
        case Feature::OPPONENT:
            orb_->detect(entry->image, entry->keypoints, entry->mask);
            opponent_->compute(entry->image, entry->keypoints,
                               entry->descriptors);
            break;
        case Feature::RAISESCORE:
        case Feature::SOBEL:
        case Feature::DEFAULT:
        default:
            (*orbcustom_)(MatcherUtils::convertToGray(entry->image),
                          entry->mask, entry->keypoints, entry->descriptors);
        }
    } catch(...) {
        // Due to bug OpenCV bug #3026 (http://code.opencv.org/issues/3026#) ORB
        // operator() throws exception when calculating the pyramid
        // levels in the orb descriptors
        // NOTE: use lower MatcherConfig nScale with lower res images
        // Issue affects OpenCV versions 2.4.0 - 2.4.4.
    }
}

void Matcher::computeMatchEntryGPU(cv::Ptr<MatchEntryGPU> entryGPU,
                                   const cv::Mat &image) {
    if (image.empty() || !conf_.gpu_enabled)
        return;

    try {
        // Convert image to grayscale and upload it to the GPU
        entryGPU->image = MatcherUtils::convertToGPUGray(image);
        // Compute keypoints and descriptors
        (*orbGPU_)(entryGPU->image, entryGPU->mask, entryGPU->keypoints,
                   entryGPU->descriptors);
    } catch(...) {
         // When no keypoints are found then instance of cv::Exception is
         // thrown from operator() of ORB_GPU.
         // Additionally there is produced output by OpenCV:
         // OpenCV Error: Assertion failed (0 <= roi.x && 0 <= roi.width &&
         // roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height &&
         // roi.y + roi.height <= m.rows) in GpuMat, file
         // ../OpenCV-2.4.3/modules/core/src/gpumat.cpp, line 570
    }
}
