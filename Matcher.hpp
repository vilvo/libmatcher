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

#ifndef Matcher_HPP
#define Matcher_HPP

#include <sys/types.h>
#include <baseapi.h>  // for tesseract OCR
#include <boost/unordered_map.hpp>
#include <vector>
#include <string>
#include <map>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/gpu/gpu.hpp"

#include "matcher_types.hpp"

class MatchEntry;
class MatchEntryGPU;
class OpponentColorDescriptor;

using matcher::MatcherConfig;
using matcher::MatchQuery;
using matcher::MatchResult;
using matcher::Feature;
using matcher::Draw;

class Matcher {
 public:
    /**
     * A default constructor.
     * @param mconfig Matcher configuration containing ORB constructor
     * parameters and other thresholds used by the class.
     */
    explicit Matcher(const MatcherConfig &mconfig);

    ~Matcher();

    /**
     * Function that processes input image according to a given request.
     * According to the MatcherConfig it can perform two tasks: locate
     * qentry_ in entry_ or verify qentry_ against entry_ (qentry_ and
     * entry_ are class members). entry_ contains currently processed image
     * and it keypoints and descriptors. ref_entry is loaded from verificaiton
     * cache based on the thrift request message.
     * @param  image Image to be processed.
     * @param  mquery currently processed query
     * @param  mresult pointer to the result struct
     * @param  entry MatchEntry containing template/reference image
     * @param  roientry MatchEntry containing Region of Interest image
     * processing image (inliers, outliers, result ratio).
     */
    cv::Ptr<cv::Mat> match(const cv::Mat &image,
                           const MatchQuery &mquery,
                           MatchResult *mresult,
                           cv::Ptr<MatchEntry> entry = NULL,
                           cv::Ptr<MatchEntry> roientry = NULL);

    /**
     * Function that processes input image according to a given query.
     * According to the MatcherConfig it can perform two tasks: locate
     * qentry_ in entry_ or verify qentry_ against entry_ (qentry_ and
     * entry_ are class members). entry_ contains currently processed image
     * and its keypoints and descriptors.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     */
    void match(const MatchQuery &mquery, MatchResult *mresult);

    /**
     * Locates characters in the given text message (mquery.message) using
     * OCR. Coordinates of each letter are put to the result struct.
     * @param  image Image to be processed.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     * @return Pointer to the result image.
     */
    cv::Ptr<cv::Mat> locateCharacters(const cv::Mat &image,
                                      const MatchQuery &mquery,
                                      MatchResult *mresult);

    /**
     * Locates characters in the given text message (mquery.icon) using
     * OCR. Coordinates of each letter are put to coords vector.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     */
    void locateCharacters(const MatchQuery &mquery, MatchResult *mresult);

    /**
     * Computes missing values (i.e. keypoints and descriptors) of the given
     * cache entry.
     * @param entry Input entry, which must at least contain image matrix.
     * @param phase Currently used feature matching phase.
     */
    void computeMatchEntry(cv::Ptr<MatchEntry> entry,
                           const Feature::phase &phase = Feature::DEFAULT);

    /**
     * Draws matching information (locate method, inliers, outliers, result,
     * threshold and error message) to the image.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     * @param image   Image onto which the additional information will be drawn
     */
    void drawMatchDetails(const MatchQuery &mquery,
                          MatchResult *mresult,
                          cv::Ptr<cv::Mat> image);

    /**
     * Loads image and stores in image_ and images_ map.
     * @param  screenshot Path to the image (absolute or relative).
     * @return            True if loading succeeded. False otherwise.
     */
    bool loadImage(const char* &screenshot);

    /**
     * Unloads image, i.e. removes it from images_ map and releases the memory.
     * @param  screenshot Path to the image (absolute or relative).
     * @return            True if image was in the images_ map. False otherwise.
     */
    bool unloadImage(const char* &screenshot);

 private:
    /**
     * Configures class with the given configuration. Resets all the variables,
     * including cache.
     * @param mconfig Matcher configuration defined in thrift file.
     */
    void configure(const MatcherConfig &mconfig);

    /**
     * Draws result image according to the query and flags.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     * @param flag @see Draw::flags.
     * @return image which consists of left and right image. Currently processed
     * image is on the left, and reference/template on the right. Therefore
     * size of result image is defined as follows:
     * processed_image_height x 2*processed_image_width
     */
    cv::Ptr<cv::Mat> draw(const MatchQuery &mquery,
                          MatchResult *mresult,
                          const Draw::flag &flag = Draw::ONLY_BBOX);

    /**
     * Draws results of locatecharacters()
     * @param  mquery      currently processed query
     * @param  mresult     pointer to the result struct
     * @param  roi         Region of interest.
     * @param  lettermap   letter<->bounding box map
     * @param  confmap     letter<->confidence map
     * @return result image (@see draw()).
     */
    cv::Ptr<cv::Mat> drawchars(
                      const MatchQuery &mquery,
                      MatchResult *mresult,
                      const cv::Rect roi,
                      const boost::unordered_map<char, cv::Rect> *lettermap = 0,
                      const boost::unordered_map<char, float> *confmap = 0);

    /**
     * Locates roi template image of name given in mquery.roi in input image
     * and set its boundning box to class member bbox_.
     * @param mquery currently processed query
     * @param mresult pointer to the result struct
     * @param roientry entry containing Region of Interest image.
     * @return True if both roi and template have been found. False otherwise.
     */
    bool locateROI(const MatchQuery &mquery, MatchResult *mresult,
                   cv::Ptr<MatchEntry> roientry);

    /**
     * Performs location/verificaiton using one of the template matching
     * methods (MATCHTEMPLATE or SOBEL).
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @return pointer to the image with drawn bounding box
     */
    cv::Ptr<cv::Mat> runTemplateMatching(const MatchQuery &mquery,
                                         MatchResult *mresult);

    /**
     * Performs location/verificaiton using Optical Character Recognition.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @return pointer to the image with drawn bounding box
     */
    cv::Ptr<cv::Mat> runOCR(const MatchQuery &mquery,
                            MatchResult *mresult);

    /**
     * Performs location/verificaiton using ORB feature matching.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @return pointer to the image with drawn bounding box
     */
    cv::Ptr<cv::Mat> runFeatureMatching(const MatchQuery &mquery,
                                        MatchResult *mresult);

    /**
     * Validates input image whether it is empty or is black. Other validation
     * checks may be added here, like MatcherUtils::isBlurred().
     * @param image Currently processed frame to be validated.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @return True if image is valid.
     */
    bool validateImages(const cv::Mat &image,
                        const MatchQuery &mquery,
                        MatchResult *mresult);

    /**
     * Loads images (screenshot and icon) and stores them in entry_ and
     * qentry_ respectively.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     */
    void loadImages(const MatchQuery &mquery, MatchResult *mresult);

    /**
     * Computes keypoints and descriptors of a current frame and loads
     * reference/template image parameters from the cache and stores them in
     * entry_ and qentry_ respectively.
     * @param name Name of the cache entry with reference/template image.
     * @return True if keypoints were computed without any problems. If the
     * cache entry does not exist, then returned value is False.
     */
    bool computeKeyPoints(const std::string &name);

    /**
     * Performs matching of the descriptors from the current frame and the
     * reference/template image using one of the matching method.
     * Matching method is defined by match_method field of conf_ member.
     * @exception Possible cv::Exception in case of empty descriptors
     */
    void matchDescriptors();

    /**
     * Generate results such as inliers and outliers count, finds homography
     * and location of the template image in the current frame.
     * All the results are kept in req_ member variable.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @param phase Currently used feature matching phase.
     */
    void genResults(const MatchQuery &mquery,
                    MatchResult *mresult,
                    const Feature::phase &phase = Feature::DEFAULT);

    /**
     * Function that performs the second verification, i.e. another way of
     * suppressing outliers and therefore yielding higher results
     * (inliers-to-all-ratio) is double verification. After locating an
     * object (template image), we set a region of interest (ROI) on the
     * currently processed image and verify that ROI against the template image
     * again. Therefore we match features that we are only interested in.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     * @param phase Currently used feature matching phase.
     */
    void verifyLocation(const MatchQuery &mquery,
                        MatchResult *mresult,
                        const Feature::phase &phase = Feature::DEFAULT);

    /**
     * Function that performs template matching accroding to the method set in
     * the input query. Default bahaviour is using correlation-based
     * template matching with Sobel operator preprocessing (edge filtering).
     * Other option is simple template matching using squared-differences.
     * The location result is stored in bbox_ and mresult fields.
     * @param mquery  currently processed query
     * @param mresult pointer to the result struct
     */
    void matchTemplate(const MatchQuery &mquery, MatchResult *mresult);

    /**
     * Locates input text in the given image.
     * @param  image        input image
     * @param  text         text to be located.
     * @param  roi          Region of Interest within which letter is searched.
     * @return              True if text objects were found.
     */
    bool findText(const cv::Mat &image, const std::string &text,
                  const cv::Rect &roi, const MatchQuery &mquery,
                  MatchResult *mresult);

    /**
     * Maps characters to the bounding boxes stored in lettermap.
     * @param  image       Processed image.
     * @param  roi         Region of interest within which letters are searched.
     * @param  mresult     Pointer to the result struct
     * @param  lettermap   letter<->bounding box map
     * @param  confmap     letter<->confidence map
     * @param  inputtext   Processed text.
     * @param  noduplicate Processed text without duplications.
     * @param  threshold   Processing threshold. Letter is considered as found
     * when confidence >= threshold.
     * @return             Confidence as a ratio of the number of letters that
     * were successfully found to the total length of the message
     */
    int mapCharToBBox(const cv::Mat &image, const cv::Rect &roi,
                      MatchResult *mresult,
                      boost::unordered_map<char, cv::Rect> &lettermap,
                      boost::unordered_map<char, float> &confmap,
                      const std::string &inputtext,
                      const std::string &noduplicate,
                      const int &threshold);

    /**
     * Locates letter and puts its bounding box into char<->rect map.
     * @param  character letter to be found
     * @param  map       character <-> bounding box map.
     * @param  conf      found letter confidence
     */
    void findLetter(const char &character,
                    boost::unordered_map<char, cv::Rect> &map,
                    float *conf);

    /**
     * Returns region of interest (ROI) for OCR according to the message.
     * If message is multiline then ROI spans from first to the last line.
     * Otherwise ROI embraces whole image.
     * @param  image     Processed image
     * @param  message   Input text message.
     * @param  threshold Processing threshold used when message is multiline.
     * Line is considered as found when confidence >= threshold.
     * @return           region of interest
     */
    cv::Rect getRoi4Text(const cv::Mat &image, const std::string &message,
                         const int &threshold);

    /**
     * Computes GPU cache entry based on the image given as a second parameter.
     * @param entry GPU cache entry to be filled with image, keypoints and
     *  descriptors in GPU format (cv::gpu::GpuMat).
     * @param image Input image matrix from which an entry will be computed.
     */
    void computeMatchEntryGPU(cv::Ptr<MatchEntryGPU> entry,
                              const cv::Mat &image);

     /**
      * Currently used matcher's configuration
      */
    MatcherConfig conf_;

    /**
     * Smart pointer to default ORB feature detector in CPU.
     */
    cv::Ptr<cv::ORB> orb_;

    /**
     * Smart pointer to customized ORB feature detector in CPU.
     */
    cv::Ptr<cv::ORB> orbcustom_;

    /**
     * Smart pointer to the Opponent Color Space ORB descriptor extractor.
     * An unadapted descriptor extractor (set in the constructor) computes
     * descriptors on each of three channels and concatenates them into
     * a single color descriptor. Works only with RGB images.
     */
    cv::Ptr<OpponentColorDescriptor> opponent_;

    /**
     * Smart pointer to dynamically allocate CPU feauture matcher.
     */
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    /**
     * Cache entry to store image, keypoints and descriptors of the currently
     * processed frame. CPU format. Pointer released and created anew every
     * call to match.
     */
    cv::Ptr<MatchEntry> entry_;

    /**
     * Cache entry to store image, keypoints and descriptors of the queried
     * reference or template image that is currently in use. CPU format.
     */
    cv::Ptr<MatchEntry> qentry_;

    /**
     * Match entry to store image, keypoints and descriptors of the template
     * image that acts as region of interest (search area). CPU format.
     */
    cv::Ptr<MatchEntry> roientry_;

    /**
     * Screenshot image loaded by loadImage function.
     */
    cv::Mat image_;

    /**
     * Currently processed screenshot name
     */
    std::string screenshot_;

    /**
     * Cache entry to store image, keypoints and descriptors of the currently
     * processed frame. GPU format.
     */
    cv::Ptr<MatchEntryGPU> entryGPU_;

    /**
     * Cache entry to store image, keypoints and descriptors of the queried
     * reference or template image that is currently in use. GPU format.
     */
    cv::Ptr<MatchEntryGPU> qentryGPU_;

    /**
     * Vector to store filtered matches between current frame and reference/
     * template image.
     */
    std::vector<cv::DMatch> filtered_matches_;

    /**
     * ORB pointer for feature detection in GPU
     */
    cv::Ptr<cv::gpu::ORB_GPU> orbGPU_;

    /**
     * GPU brute force matcher
     */
    cv::Ptr<cv::gpu::BruteForceMatcher_GPU_base> matcherGPU_;

    /**
     * Mask that represents inliers. Used for drawing
     */
    std::vector<char> matches_mask_;

    /**
     * Text detected by OCR saved together with result image
     */
    std::string ocrtext_;

    /**
     * Pointer to tesseract base api object.
     */
    tesseract::TessBaseAPI *tess_;

    /**
     * Pointer to the method that run requested action: locate (according to
     * the requested location method) or verify.
     */
    cv::Ptr<cv::Mat> (Matcher::*action)(const MatchQuery &mquery,
                                        MatchResult *mresult);

    /**
     * Vector of feature matching phases. @see Feature::phase enum.
     */
    std::vector<Feature::phase> featurephases_;

    /**
     * Map screenshotFilename <-> image matrix, for storing screenshots.
     */
    boost::unordered_map<std::string, cv::Mat> images_;
};

#endif  // Matcher_HPP
