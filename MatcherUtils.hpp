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

#ifndef MatcherUtils_HPP
#define MatcherUtils_HPP

#include <boost/unordered_map.hpp>
#include <pageiterator.h>  // for tesseract OCR
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "matcher_types.hpp"

using matcher::MatchQuery;
using matcher::MatchResult;
using matcher::Rect;
using std::vector;

/**
 * A static class acting as a library containing useful functions for computer
 * vision tasks.
 */
class MatcherUtils {
    /**
     * Private constructor to prevent creating an object from class.
     */
    MatcherUtils();

  public:
    /**
     * Function that returns bounding box around given vector of points.
     * If bounding box exceeds img_size boundaries then it is cropped to
     * img_size
     * @param  points  Input vector of points
     * @param  img_size Size of the image to which returned bounding box is
     * cropped.
     * @return          bounding box around given vector of points.
     */
    static cv::Rect boundingBox(const std::vector<cv::Point2f> &points,
                                const cv::Size &img_size);

    /**
     * Checks whether given bounding box aspect ratio is correct, i.e. within
     * certain offset (matcher::AROFFSET).
     * @param  bbox     Input bounding box to be validated
     * @param  img_size Original template image size
     * @return          True if aspect ratio is correct. Otherwise false.
     */
    static bool checkAspectRatio(const cv::Rect &bbox,
                                 const cv::Size &img_size);

    /**
     * Checks whether given bounding box size is not smaller than some certain
     * offset (matcher::SIZEOFFSET) from the input image (template/icon). If
     * scale_invariant flag is set to false then size must be also not bigger
     * than matcher::SIZEOFFSET.
     * @param  bbox     Input bounding box to be validated
     * @param  img_size Original template/icon image size
     * @param  scale_invariant  Flag indicating whether matching is scale
     * variant (false) or invariant (true).
     * @return          Returns true if size is correct. Otherwise false.
     */
    static bool checkSize(const cv::Rect &bbox, const cv::Size &img_size,
                          const bool &scale_invariant);

    /**
     * Converts input rectangle to the vector of points.
     * @param  rect Input rectangle
     * @return      Vector of points, i.e. corners coordinates of the rect
     */
    static std::vector<cv::Point2f> rectToVector(const cv::Rect &rect);

    /**
     * Aligns given input image to the center of black frame. Alignment is done
     * if width or height of input image is smaller than size.
     * @param  image Input image to be aligned. If size of the image is biger
     * than size then this image is returned.
     * @param  size  Minimum size of the image.
     * @return       Aligned image to center of the black image.
     */
    static cv::Mat alignToCenter(const cv::Mat &image, const cv::Size &size);

    /**
     * Function that returns mask specifying where to look for keypoints.
     * Returned matrix is 8-bit size with non-zero values in the region of
     * interest. Region of interes is given as second parameter (bbox).
     * If no mask is specified then keypoints are calculated in a whole image.
     * @param  img_size Image size. The same rules apply as for alignToCenter
     * function. @see alignToCenter
     * @param  bbox     Region of interest (optional).
     * @exception Possible exception when bbox exceeds with its size img_size.
     * @return          mask specifying where to look for keypoints.
     */
    static cv::Mat getMask(const cv::Size &img_size, const cv::Rect &bbox);

    /**
     * Combines two given frames into one: frame on the left and ref_image
     * on the right side.
     * @param  frame     Input image to be placed on the left side.
     * @param  ref_image Input image to be placed on the right side.
     * @return           Combined image containing frame on the left and
     * ref_image on the right.
     */
    static cv::Mat combine(const cv::Mat &frame, const cv::Mat &ref_image);

    /**
     * Sets region of interest (rect) to the input image.
     * @param  image Input image
     * @param  rect  Region of interest
     * @return       Image with only region of interest set visible. Other
     * part of the image is black.
     */
    static cv::Mat setROI(const cv::Mat &image, const Rect &rect);

    /**
     * Sets region of interest (rect) to the input image.
     * @param  image Input image
     * @param  rect  Region of interest
     * @return       Image with only region of interest set visible. Other
     * part of the image is black.
     */
    static cv::Mat setROI(const cv::Mat &image, const cv::Rect &rect);

    /**
     * Calculates the central point from bounding box and updates center
     * coordinates field in result structure.
     * @param mresult Input/output result structure.
     */
    static void calcCenter(MatchResult *mresult);

    /**
     * Returns colour based on result and threshold.
     * @param  result    input result [0-100]
     * @param  threshold input threshold [0-100]
     * @param  text      input/output text with message according to the
     *                    result and threshold.
     * @return           Red if result is zero.
     * Orange when result is greater than zero but less or equal threshold.
     * Green when result is greater than threshold.
     */
    static cv::Scalar getColor(const int &result, const int &threshold,
                               std::string *text = 0);

    /**
     * Calculates Levenshtein edit distance between two strings. The Levenshtein
     * distance between two strings is the number of changes to transform one
     * string into another.
     * @param  word1 first c-string
     * @param  len1  length of the first c-string
     * @param  word2 second c-string
     * @param  len2  length of the second c-string
     * @return       Levenshtein edit distance between word1 and word2
     */
    static int distance(const char *word1, int len1,
                        const char *word2, int len2);

    /**
     * Removes duplicate letters from string and places them in the same string.
     * @param text input/output text string.
     */
    static void removeDuplicateLetters(std::string *text);

    /**
     * Calculates result for typing the message. Result is defined as ratio of
     * number of letters that were successfully mapped to theirs bounding boxes
     * and the length of the message.
     * @param  message Processed message to by typed.
     * @param  map     letter <-> bounding box map
     * @return         Ratio of the number of letters that were successfully
     * found to the total length of the message.
     */
    static int calcTypeMessageResult(const std::string &message,
                            const boost::unordered_map<char, cv::Rect> &map);

    /**
     * Calculates average confidence of the elements in the input map
     * @param  confmap letter <-> confidence map
     * @return         Average confidence of elements in confmap.
     */
    static float calcAverageConf(
                            const boost::unordered_map<char, float> &confmap);

    /**
     * Returns tesseract page iteration level.
     * @param  text input text
     * @return      page iteration level. Possible values:
     * * RIL_PARA - for paragraph
     * * RIL_TEXTLINE - for single text line
     * * RIL_WORD - for single word
     * * RIL_SYMBOL - for single character
     */
    static tesseract::PageIteratorLevel getPageIterationLevel(
                                                    const std::string &text);

    /**
     * Sharpen an image by certain parameter and places is to sharpened matrix.
     * @param image     Input image to be sharpened.
     * @param sharpened Output sharpened image.
     * @param i         Sharpening parameter [0.0 - infinity]. In practise only
     * values between (0.0 - 10.0) make sense. The bigger the value the more
     * sharper is the output image. 0.0 means that image is not sharpened at
     * all.
     */
    static void sharpenImage(const cv::Mat &image, cv::Mat *sharpened,
                             const double &i);

    /**
     * The function applies fixed-level thresholding to a single-channel image.
     * It gets a bi-level (binary) image out of a grayscale image and therefore
     * removes the noise, that is, filtering out pixels with values smaller
     * than 'threshold'.
     * @param image       Input image to be sharpened.
     * @param thresholded Output thresholded image.
     * @param threshold   Thresholding parameter [0 - 255]. Pixels below that
     * value are set to maximum (255).
     */
    static void thresholdImage(const cv::Mat &image, cv::Mat *thresholded,
                               const int &threshold);

    /**
     * Checks whether frame is black. It slides through the whole frame with
     *  a window of a size [block_size x block_size] and calculates mean pixel
     *  value of each window. If all of them are below the threshold
     *  (global constant mean_pix_thresh) then we can cosider the frame as black
     * @param  image Input image (can be both grayscale or colorful).
     * @param block_size Size of the block within which average pixel value
     * is calculated. If that average is lower than mean_pix_thresh then frame
     * is considered as black.
     * @param mean_pix_thresh Minimum acceptable value of the average pixel
     * value within one block (of size block_size x block_size).
     * @return       True if the frame is black (see description above).
     */
    static bool isBlack(const cv::Mat& image, const int block_size = 10,
                        const float mean_pix_thresh = 0.1);

    /**
     * Uploads input image to the GPU and there converts it to the grayscale.
     * If the image is already gray then it is just uploaded to the GPU and
     * returned.
     * @param  image Input image. Can be both grayscale and color.
     * @return       Returns grayscale image in GPU format.
     */
    static cv::gpu::GpuMat convertToGPUGray(const cv::Mat &image);

    /**
     * Converts input image to the grayscale. If the image is already gray then
     * function does nothing but returns it back.
     * @param  image Input image. Can be both grayscale and color.
     * @return       Returns grayscale image in CPU format.
     */
    static cv::Mat convertToGray(const cv::Mat &image);

    /**
     * Converts RGB image to Opponent Color Space
     * @param  image RGB image
     * @return vector containing 3 opponent channel matrices
     */
    static std::vector<cv::Mat> convertBGRImageToOpponentColorSpace(
                                                    const cv::Mat& image);

    /**
     * Returns current GPU memory usage.
     * @return current GPU memory usage as a fraction from 0.0 to 1.0.
     */
    static float getMemoryUsage();

    /**
     * Checks whether frame is blurred. Only entirely blurred frame are
     * considered as blurred.
     * @param  image Input image to be processed.
     * @param kernel_size Box filter kernel size. The higher the more image
     * is smudged.
     * @param blur_thresh Minimum allowed blurness threshold. It can be a value
     * between 0 and 255. The lower the value the more blurred images are
     * allowed.
     * @return       True if the image is blurred. Otherwise false.
     */
    static bool isBlurred(const cv::Mat &image, const int kernel_size = 3,
                          const int blur_thresh = 60);

    /**
     * Removes illegal characters from the string
     * @param s String to be processed.
     */
    static void removeillegalchars(std::string* s);

    /**
     * Removes path and suffix from the string
     * @param s String to be processed.
     */
    static void prettyImageName(std::string *s);

    /**
     * Creats message by combining input text with digit with certain precision.
     * @param  text      Input text to be placed in the result message
     * @param  digit     Digit to be placed just after text
     * @param  precision Precision of the digit.
     * @return           Pointer to the combined message;
     */
    static std::string getmessage(const std::string &text, const double &digit,
                                  const int &precision = 2);

    /**
     * Saves result image to the disk
     * @param image      Image to be saved.
     * @param mquery     Input query containing screenshot and icon name.
     * @param mresult    Input struct containing match result.
     * @param screenshot Screenshot name if mquery.screenshot is not set.
     */
    static void saveResultImage(const cv::Ptr<cv::Mat> image,
                                const MatchQuery &mquery,
                                MatchResult *mresult,
                                std::string &screenshot);

    /**
     * Rotates image by given angle.
     * @param src   Source image
     * @param dst   Destination image (rotated by angle).
     * @param angle Angle (0, 90, 180, 270)
     */
    static void rotate(const cv::Mat& src, cv::Mat* dst, const int angle);

    /**
     * Draws keypoints on the outImage.
     * @param image     Input raw image
     * @param keypoints keypoints to be drawn.
     * @param outImage  Output image with keypoints. Its content depends on the
     * flags value defining what is drawn in the output image. See possible
     * flags bit values below.
     * @param _color    Colour of keypoints (random by default)
     * @param flags     Flags setting drawing features. Possible flags bit
     * values are defined by cv::DrawMatchesFlags. Possible values are:
     * * DEFAULT = 0, // Output image matrix will be created (Mat::create),
     *   i.e. existing memory of output image may be reused.
     *   Two source images, matches, and single keypoints will be drawn.
     *   For each keypoint, only the center point will be drawn (without a
     *   circle around the keypoint with the keypoint size and orientation).
     * * DRAW_OVER_OUTIMG = 1, // Output image matrix will not be created
     *   (using Mat::create). Matches will be drawn on existing content of output
     *   image.
     * * NOT_DRAW_SINGLE_POINTS = 2, // Single keypoints will not be drawn.
     * * DRAW_RICH_KEYPOINTS = 4 // For each keypoint, the circle around
     *   keypoint with keypoint size and orientation will be drawn.
     * @param translate [description]
     */
    static void drawKeypoints(const cv::Mat& image,
                              const vector<cv::KeyPoint>& keypoints,
                              cv::Mat& outImage,
                              const cv::Scalar& _color = cv::Scalar::all(-1),
                              int flags = cv::DrawMatchesFlags::DEFAULT,
                              const cv::Point *translate = 0);

    /**
     * Draws the found matches of keypoints from two images.
     * @param img1        First image (left or top in the result image).
     * @param keypoints1  Keypoints from the first source image
     * @param img2        Second image (right or bottom in the result image).
     * @param keypoints2  Keypoints from the second source image.
     * @param matches1to2 Matches from the first image to the second one, which
     * means that keypoints1[i] has a corresponding point in keypoints2[matches[i]]
     * @param outImg      Output image. Its content depends on the flags value
     * defining what is drawn in the output image. See possible flags bit
     * values below.
     * @param matchColor  Color of matches (lines and connected keypoints).
     * If matchColor==Scalar::all(-1) , the color is generated randomly.
     * @param singleColor Color of single keypoints (circles), which means that
     * keypoints do not have the matches. If singleColor==Scalar::all(-1), the
     * color is generated randomly.
     * @param matchesMask Mask determining which matches are drawn. If the mask
     * is empty, all matches are drawn.
     * @param flags       @see flags in drawKeypoints
     */
    static void drawMatches(const cv::Mat& img1,
                            const vector<cv::KeyPoint>& keypoints1,
                            const cv::Mat& img2,
                            const vector<cv::KeyPoint>& keypoints2,
                            const vector<cv::DMatch>& matches1to2,
                            cv::Mat& outImg,
                            const cv::Scalar& matchColor = cv::Scalar::all(-1),
                            const cv::Scalar& singleColor = cv::Scalar::all(-1),
                            const vector<char>& matchesMask = vector<char>(),
                            int flags = cv::DrawMatchesFlags::DEFAULT);
};

#endif  // MatcherUtils_HPP
