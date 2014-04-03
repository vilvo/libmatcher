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

#include <string>
#include <vector>
#include <iomanip>  // for setprecision
#include <algorithm>
#include "MatcherUtils.hpp"
#include "matcher_consts.hpp"

cv::Rect MatcherUtils::boundingBox(const std::vector<cv::Point2f> &points,
                                   const cv::Size &img_size) {
    cv::Rect bbox = cv::boundingRect(points);
    if (bbox.x < 0)
        bbox.x = 0;
    if (bbox.y < 0)
        bbox.y = 0;
    if (bbox.x + bbox.width > img_size.width)
        bbox.width = img_size.width - bbox.x - 1;
    if (bbox.y + bbox.height > img_size.height)
        bbox.height = img_size.height - bbox.y - 1;
    return bbox;
}

bool MatcherUtils::checkAspectRatio(const cv::Rect &bbox,
                                    const cv::Size &img_size) {
    double bbox_ar = static_cast<double>(bbox.width)/bbox.height;
    double img_ar = static_cast<double>(img_size.width)/img_size.height;
    if (std::abs(bbox_ar - img_ar) > matcher::AROFFSET)
        return false;
    return true;
}

bool MatcherUtils::checkSize(const cv::Rect &bbox,
                             const cv::Size &img_size,
                             const bool &scale_invariant) {
    // Check minimum size of bounding box. If smaller than offset then reject
    if ((img_size.height-bbox.height) > matcher::SIZEOFFSET*img_size.height ||
        (img_size.width-bbox.width) > matcher::SIZEOFFSET*img_size.width)
        return false;
    // Check maximum size of bounding box in case of scale variance.
    if (!scale_invariant &&
        ((bbox.height-img_size.height) > matcher::SIZEOFFSET*img_size.height ||
        (bbox.width-img_size.width) > matcher::SIZEOFFSET*img_size.width))
        return false;
    return true;
}

std::vector<cv::Point2f> MatcherUtils::rectToVector(const cv::Rect &rect) {
    std::vector<cv::Point2f> corners(4);
    corners[0] = rect.tl();
    corners[1] = cvPoint(rect.x + rect.width, rect.y);
    corners[2] = rect.br();
    corners[3] = cvPoint(rect.x, rect.y + rect.height);
    return corners;
}

cv::Mat MatcherUtils::alignToCenter(const cv::Mat &image,
                                    const cv::Size &size) {
    cv::Mat processed_image;
    cv::Size framesize(MAX(image.cols, size.width),
                       MAX(image.rows, size.height));
    processed_image = cv::Mat(framesize, image.type());
    processed_image = cv::Scalar(0);
    cv::Rect templ_rect((framesize.width-image.cols)/2,
                        (framesize.height-image.rows)/2,
                        image.cols, image.rows);
    cv::Mat roi(processed_image, templ_rect);
    image.copyTo(roi);
    return processed_image;
}

cv::Mat MatcherUtils::getMask(const cv::Size &size, const cv::Rect &rect) {
    cv::Mat mask = cv::Mat(size, CV_8UC1, cv::Scalar(0));
    mask(rect).setTo(cv::Scalar::all(255));
    return mask;
}

cv::Mat MatcherUtils::combine(const cv::Mat &frame, const cv::Mat &ref_image) {
    cv::Mat right_image = MatcherUtils::alignToCenter(ref_image, frame.size());
    cv::Size size = (frame.rows >= frame.cols)
                    ? cv::Size(2*frame.cols, frame.rows)
                    : cv::Size(frame.cols, 2*frame.rows);
    cv::Mat img_combined = cv::Mat(size, frame.type());
    img_combined = cv::Scalar(0);
    cv::Rect roi_rect = cv::Rect(0, 0, frame.cols, frame.rows);
    cv::Mat roi(img_combined, roi_rect);
    frame.copyTo(roi);
    if (frame.rows >= frame.cols)
        roi_rect = cv::Rect(frame.cols, 0, frame.cols, frame.rows);
    else
        roi_rect = cv::Rect(0, frame.rows, frame.cols, frame.rows);
    roi = cv::Mat(img_combined, roi_rect);
    right_image.copyTo(roi);
    return img_combined;
}

cv::Mat MatcherUtils::setROI(const cv::Mat &image, const Rect &rect) {
    cv::Mat mask = cv::Mat(image.size(), image.type(), cv::Scalar(0));
    cv::Rect roi(rect.x, rect.y, rect.width, rect.height);
    image(roi).copyTo(mask(roi));
    return mask;
}

cv::Mat MatcherUtils::setROI(const cv::Mat &image, const cv::Rect &rect) {
    return MatcherUtils::setROI(image, Rect(rect));
}

void MatcherUtils::calcCenter(MatchResult *mresult) {
    for (size_t i = 0; i < mresult->center.size(); i++) {
        mresult->center[i] = matcher::Point(mresult->bbox[i].x +
                                            mresult->bbox[i].width/2,
                                            mresult->bbox[i].y +
                                            mresult->bbox[i].height/2);
    }
}

cv::Scalar MatcherUtils::getColor(const int &result, const int &threshold,
                                  std::string *textinput) {
    cv::Scalar color;
    std::string text, drawmsg_;
    if (textinput)
        drawmsg_ = *textinput;

    if (result == 0) {
        text = std::string("MATCH FAILED! " + drawmsg_);
        color = CV_RGB(255, 0, 0);  // red
    } else if (result > 0 && result < threshold) {
        text = std::string("MATCH FAILED! Confidence below threshold.");
        color = CV_RGB(255, 165, 0);  // orange
    } else if (result >= threshold) {
        text = "SUCCESS! " + drawmsg_;
        color = CV_RGB(0, 255, 0);  // green
    } else {
        CV_RGB(255, 255, 255);  // white
    }

    if (textinput)
        *textinput = text;
    return color;
}

int MatcherUtils::distance(const char *word1, int len1,
                           const char *word2, int len2) {
    int matrix[len1 + 1][len2 + 1];
    int i, j, temp, insert, substitute, minimum;
    for (i = 0; i <= len1; i++) {
        matrix[i][0] = i;
    }
    for (i = 0; i <= len2; i++) {
        matrix[0][i] = i;
    }
    for (i = 1; i <= len1; i++) {
        char c1 = word1[i-1];
        for (j = 1; j <= len2; j++) {
            char c2 = word2[j-1];
            if (c1 == c2) {
                matrix[i][j] = matrix[i-1][j-1];
                continue;
            } else {
                temp = matrix[i-1][j] + 1;
                insert = matrix[i][j-1] + 1;
                substitute = matrix[i-1][j-1] + 1;
                minimum = temp;
                if (insert < minimum) {
                    minimum = insert;
                }
                if (substitute < minimum) {
                    minimum = substitute;
                }
                matrix[i][j] = minimum;
            }
        }
    }
    return matrix[len1][len2];
}

void MatcherUtils::removeDuplicateLetters(std::string *text) {
    if (text == NULL)
        return;
    int pos;
    std::string inputtext = *text;
    for (size_t i = 0; i < inputtext.length(); i++) {
        if ((pos = (*text).find(inputtext[i])) < 0)
            (*text) += inputtext[i];
    }
}

int MatcherUtils::calcTypeMessageResult(const std::string &message,
                          const boost::unordered_map<char, cv::Rect> &map) {
    int counter = 0;
    for (size_t i = 0; i < message.length(); i++) {
        if (map.find(message[i]) != map.end())
            counter++;
    }
    return matcher::MAXCONFIDENCE*(static_cast<double>(counter)/
                                static_cast<double>(message.length()));
}

float MatcherUtils::calcAverageConf(
                            const boost::unordered_map<char, float> &confmap) {
    boost::unordered_map<char, float>::const_iterator it;
    float averageconf = 0.0f;
    for (it = confmap.begin(); it != confmap.end(); it++) {
        averageconf += it->second;
    }
    averageconf /= confmap.size();
    return averageconf;
}

tesseract::PageIteratorLevel MatcherUtils::getPageIterationLevel(
                                                    const std::string &text) {
    tesseract::PageIteratorLevel level;
    if (text.find('\n') != std::string::npos)
        level = tesseract::RIL_PARA;
    else if (std::string(text).find(' ') != std::string::npos)
        level = tesseract::RIL_TEXTLINE;
    else if (text.length() == 1)
        level = tesseract::RIL_SYMBOL;
    else
        level = tesseract::RIL_WORD;
    return level;
}

void MatcherUtils::sharpenImage(const cv::Mat &image, cv::Mat *sharpened,
                                const double &i) {
    cv::GaussianBlur(image, *sharpened, cv::Size(0, 0), matcher::BLURNESS);
    cv::addWeighted(image, 1.0 + i, *sharpened, -i, 0, *sharpened);
}

void MatcherUtils::thresholdImage(const cv::Mat &image, cv::Mat *thresholded,
                                  const int &threshold) {
    cv::threshold(image, *thresholded, threshold, 255, CV_THRESH_BINARY_INV);
}

bool MatcherUtils::isBlack(const cv::Mat& image, const int block_size,
                           const float mean_pix_thresh) {
    for (int i = 0; i < image.cols - block_size - 1; i += block_size) {
        for (int j = 0; j < image.rows - block_size - 1; j += block_size) {
            // sum up pixels values in the square of size
            // (block_size x block_size)
            cv::Rect roi_rect(i, j, block_size, block_size);
            cv::Mat roi_image(image, roi_rect);
            cv::Scalar result = cv::mean(roi_image);
            double mean = result[0]/255.0;
            if (mean > mean_pix_thresh) {
                return false;
            }
        }
    }
    return true;
}

cv::Mat MatcherUtils::convertToGray(const cv::Mat &image) {
    if (image.type() == CV_LOAD_IMAGE_GRAYSCALE)
        return image;

    cv::Mat dst;
    cv::cvtColor(image, dst, CV_BGR2GRAY);
    return dst;
}

cv::gpu::GpuMat MatcherUtils::convertToGPUGray(const cv::Mat &image) {
    cv::gpu::GpuMat src;
    src.upload(image);
    if (src.channels() == 1)
        return src;

    cv::gpu::GpuMat dst;
    cv::gpu::cvtColor(src, dst, CV_BGR2GRAY);
    src.release();
    return dst;
}

std::vector<cv::Mat> MatcherUtils::convertBGRImageToOpponentColorSpace(
                                                    const cv::Mat& img) {
    std::vector<cv::Mat> opponentChannels;
    if (img.type() != CV_8UC3)
        return opponentChannels;

    // Prepare opponent color space storage matrices.
    opponentChannels.resize(matcher::NOPP);
    for (int i = 0; i < matcher::NOPP; i++)
        opponentChannels[i] = cv::Mat(img.size(), CV_8UC1);

    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            cv::Vec3b v = img.at<cv::Vec3b>(y, x);
            uchar& b = v[0];
            uchar& g = v[1];
            uchar& r = v[2];

            // According to van de Sande (@see OpponentColorDescriptorExtractor
            // header file) the set of coefficients for each opponent channel
            // should be sqrt(2), sqrt(6) and sqrt(3) respectively. However
            // based on our data set from releasetests/locate*.feature lettuce
            // tests the best results yield the following set of coefficients:
            // 1/sqrt(2), 1/4, and 1/3.

            // (R - G)/sqrt(2)
            opponentChannels[0].at<uchar>(y, x) =
                                cv::saturate_cast<uchar>(0.707f*(255+g-r));
            // (R + G - 2B)/sqrt(6)
            opponentChannels[MIN(1, matcher::NOPP-1)].at<uchar>(y, x) =
                                cv::saturate_cast<uchar>(0.25f*(510+r+g-2*b));
            // (R + G + B)/sqrt(3)
            opponentChannels[MIN(2, matcher::NOPP-1)].at<uchar>(y, x) =
                                cv::saturate_cast<uchar>(1.f/3.f*(r+g+b));
        }
    }
    return opponentChannels;
}

float MatcherUtils::getMemoryUsage() {
    float memory_usage = 0.0f;
    if (cv::gpu::getCudaEnabledDeviceCount() != 0) {
        cv::gpu::DeviceInfo devInfo;
        float fm = devInfo.freeMemory();
        float tm = devInfo.totalMemory();
        memory_usage = 100*(tm-fm)/tm;
    }
    return memory_usage;
}

bool MatcherUtils::isBlurred(const cv::Mat &image, const int kernel_size,
                             const int blur_thresh) {
    cv::Mat abs_dst, dst;

    cv::blur(image, dst, cv::Size(kernel_size, kernel_size));
    cv::Laplacian(dst, dst, CV_16S);
    cv::convertScaleAbs(dst, abs_dst);

    double minVal, maxVal;
    cv::minMaxIdx(abs_dst, &minVal, &maxVal);

    if (maxVal <= blur_thresh) {
        return true;
    }
    return false;
}

void MatcherUtils::removeillegalchars(std::string* s) {
    std::string::iterator it;
    for (it = s->begin(); it < s->end(); ++it) {
        if (matcher::ILLEGALCHARS.find(*it) != std::string::npos) {
            *it = ' ';
        }
    }
}

void MatcherUtils::prettyImageName(std::string *s) {
    // Remove suffix
    int slash = s->find_last_of("/\\");
    int dot = s->find_last_of(".");
    if (dot != -1) {
        *s = s->substr(slash+1, dot-slash-1);
    }
    // If multi-line then crop to contain only first line
    size_t pos = s->find('\n');
    if (pos != std::string::npos) {
        *s = s->substr(0, pos);
    }
}

std::string MatcherUtils::getmessage(const std::string &text,
                                     const double &digit,
                                     const int &precision) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision);
    ss << text << digit;
    return ss.str();
}

void MatcherUtils::saveResultImage(const cv::Ptr<cv::Mat> image,
                                   const MatchQuery &mquery,
                                   MatchResult *mresult,
                                   std::string &screenshot) {
    if (mresult->result[0] < 0 || !image)
        return;
    std::stringstream out_name;
    if (mquery.resultimage.compare(matcher::RESULTPATH) != 0) {
        out_name << mquery.resultimage;
        mresult->resultimage = mquery.resultimage;
    } else {
        time_t seconds = time(NULL);
        std::string icon(mquery.icon);
        std::stringstream filename;
        if (!mquery.screenshot.empty())
            screenshot = mquery.screenshot;
        MatcherUtils::prettyImageName(&screenshot);
        MatcherUtils::prettyImageName(&icon);
        MatcherUtils::removeillegalchars(&icon);
        filename << mresult->result[0] << "_" << screenshot << "__" << icon
                 << "_" << seconds << matcher::IMAGETYPE;
        mresult->resultimage = filename.str();
        out_name << mquery.resultimage << filename.str();
    }
    cv::imwrite((out_name.str()).c_str(), *image);
}

void MatcherUtils::rotate(const cv::Mat& src, cv::Mat* dst, const int angle) {
    if (angle == 0 || angle == 180)
        *dst = src.clone();
    else
        cv::transpose(src, *dst);

    if (angle == 90)
        cv::flip(*dst, *dst, 0);
    else if (angle == 180)
        cv::flip(*dst, *dst, -1);
    else if (angle == 270)
        cv::flip(*dst, *dst, 1);
}

/*
 * Helper function to draw keypoints
 */
static inline void _drawKeypoint(cv::Mat& img, const cv::KeyPoint& p,
                                 const cv::Scalar& color, int flags,
                                 const cv::Point *translate = 0) {
    if (!translate)
        translate = new cv::Point();
    cv::Point center(cvRound((p.pt.x + translate->x)*matcher::DRAWMULTIPLIER),
                     cvRound((p.pt.y + translate->y)*matcher::DRAWMULTIPLIER));

    if (flags & cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS) {
        // KeyPoint::size is a diameter
        int radius = cvRound(p.size/2 * matcher::DRAWMULTIPLIER);

        // draw the circles around keypoints with the keypoints size
        cv::circle(img, center, radius, color, 1, CV_AA,
                   matcher::DRAWSHIFTBITS);

        // draw orientation of the keypoint, if it is applicable
        if (p.angle != -1) {
            float srcAngleRad = p.angle*static_cast<float>(CV_PI)/180.f;
            cv::Point orient(cvRound(cos(srcAngleRad)*radius),
                             cvRound(sin(srcAngleRad)*radius));
            cv::line(img, center, center+orient, color, 1, CV_AA,
                     matcher::DRAWSHIFTBITS);
        }
    } else {
        // draw center with R=3
        int radius = 3 * matcher::DRAWMULTIPLIER;
        cv::circle(img, center, radius, color, 1, CV_AA,
                   matcher::DRAWSHIFTBITS);
    }
}

void MatcherUtils::drawKeypoints(const cv::Mat& image,
                                 const vector<cv::KeyPoint>& keypoints,
                                 cv::Mat& outImage,
                                 const cv::Scalar& _color,
                                 int flags,
                                 const cv::Point *translate) {
    if (!(flags & cv::DrawMatchesFlags::DRAW_OVER_OUTIMG)) {
        if (image.type() == CV_8UC3) {
            image.copyTo(outImage);
        } else if (image.type() == CV_8UC1) {
            cvtColor(image, outImage, CV_GRAY2BGR);
        } else {
            // Incorrect type of input image
            return;
        }
    }

    cv::RNG& rng = cv::theRNG();
    bool isRandColor = _color == cv::Scalar::all(-1);

    vector<cv::KeyPoint>::const_iterator it = keypoints.begin(),
                                         end = keypoints.end();
    for (; it != end; ++it) {
        cv::Scalar color = isRandColor
                         ? cv::Scalar(rng(256), rng(256), rng(256)) : _color;
        _drawKeypoint(outImage, *it, color, flags, translate);
    }
}

/*
 * Helper function to prepare images and draw keypoints
 */
static void _prepareImgAndDrawKeypoints(const cv::Mat& img1,
                                        const vector<cv::KeyPoint>& keypoints1,
                                        const cv::Mat& img2,
                                        const vector<cv::KeyPoint>& keypoints2,
                                        cv::Mat& outImg,
                                        cv::Mat& outImg1,
                                        cv::Mat& outImg2,
                                        const cv::Scalar& singlePointColor,
                                        int flags,
                                        cv::Point *translate) {
    cv::Size size;
    bool horizontal = true;
    if (img1.rows >= img1.cols) {
        size = cv::Size(2*img1.cols, img1.rows);
    } else {
        size = cv::Size(img1.cols, 2*img1.rows);
        horizontal = false;
    }
    if (flags & cv::DrawMatchesFlags::DRAW_OVER_OUTIMG) {
        // outImg has size less than need to draw img1 and img2 together
        if (size.width > outImg.cols || size.height > outImg.rows)
            return;
        outImg1 = outImg(cv::Rect(0, 0, img1.cols, img1.rows));
        outImg2 = outImg(cv::Rect(img1.cols, 0, img1.cols, img1.rows));
    } else {
        outImg.create(size, CV_MAKETYPE(img1.depth(), 3));
        outImg = cv::Scalar::all(0);
        outImg1 = outImg(cv::Rect(0, 0, img1.cols, img1.rows));
        if (horizontal)
            outImg2 = outImg(cv::Rect(img1.cols, 0, img1.cols, img1.rows));
        else
            outImg2 = outImg(cv::Rect(0, img1.rows, img1.cols, img1.rows));

        if (img1.type() == CV_8U)
            cvtColor(img1, outImg1, CV_GRAY2BGR);
        else
            img1.copyTo(outImg1);

        if (img2.type() == CV_8U) {
            cvtColor(img2, outImg2, CV_GRAY2BGR);
        } else {
            cv::Rect rect((img1.cols-img2.cols)/2, (img1.rows-img2.rows)/2,
                          img2.cols, img2.rows);
            *translate = rect.tl();
            cv::Mat roi(outImg2, rect);
            img2.copyTo(roi);
        }
    }

    // draw keypoints
    if (!(flags & cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS)) {
        cv::Mat _outImg1 = outImg(cv::Rect(0, 0, img1.cols, img1.rows));
        MatcherUtils::drawKeypoints(_outImg1, keypoints1, _outImg1,
                                    singlePointColor,
                      flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

        cv::Mat _outImg2 = horizontal
                         ? outImg(cv::Rect(img1.cols, 0, img1.cols, img1.rows))
                         : outImg(cv::Rect(0, img1.rows, img1.cols, img1.rows));
        MatcherUtils::drawKeypoints(_outImg2, keypoints2, _outImg2,
                                 singlePointColor,
                                 flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG,
                                 translate);
    }
}

/*
 * Helper function to draw matches
 */
static inline void _drawMatch(cv::Mat& outImg,  // NOLINT
                              cv::Mat& outImg1,
                              cv::Mat& outImg2,
                              const cv::KeyPoint& kp1,
                              const cv::KeyPoint& kp2,
                              const cv::Scalar& matchColor,
                              int flags, const cv::Point *translate) {
    cv::RNG& rng = cv::theRNG();
    bool isRandMatchColor = matchColor == cv::Scalar::all(-1);
    cv::Scalar color = isRandMatchColor
                     ? cv::Scalar(rng(256), rng(256), rng(256)) : matchColor;

    _drawKeypoint(outImg1, kp1, color, flags);
    _drawKeypoint(outImg2, kp2, color, flags, translate);

    cv::Point2f pt1 = kp1.pt, pt2 = kp2.pt, dpt2;
    if (outImg1.rows >= outImg1.cols)
        dpt2 = cv::Point2f(std::min(pt2.x + outImg1.cols,
                           static_cast<float>(outImg.cols-1)), pt2.y);
    else
        dpt2 = cv::Point2f(pt2.x, std::min(pt2.y + outImg1.rows,
                                  static_cast<float>(outImg.cols-1)));

    cv::line(outImg, cv::Point(cvRound(pt1.x*matcher::DRAWMULTIPLIER),
                               cvRound(pt1.y*matcher::DRAWMULTIPLIER)),
             cv::Point(cvRound((dpt2.x+translate->x)*matcher::DRAWMULTIPLIER),
                       cvRound((dpt2.y+translate->y)*matcher::DRAWMULTIPLIER)),
             color, 1, CV_AA, matcher::DRAWSHIFTBITS);
}

void MatcherUtils::drawMatches(const cv::Mat& img1,
                               const vector<cv::KeyPoint>& keypoints1,
                               const cv::Mat& img2,
                               const vector<cv::KeyPoint>& keypoints2,
                               const vector<cv::DMatch>& matches1to2,
                               cv::Mat& outImg,
                               const cv::Scalar& matchColor,
                               const cv::Scalar& singlePointColor,
                               const vector<char>& matchesMask, int flags) {
    // matchesMask must have the same size as matches1to2
    if (!matchesMask.empty() && matchesMask.size() != matches1to2.size())
        return;

    cv::Mat outImg1, outImg2;
    cv::Point translate;
    _prepareImgAndDrawKeypoints(img1, keypoints1, img2, keypoints2, outImg,
                                outImg1, outImg2, singlePointColor, flags,
                                &translate);

    // draw matches
    for (size_t m = 0; m < matches1to2.size(); m++) {
        int i1 = matches1to2[m].queryIdx;
        int i2 = matches1to2[m].trainIdx;
        if (matchesMask.empty() || matchesMask[m]) {
            const cv::KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
            _drawMatch(outImg, outImg1, outImg2, kp1, kp2, matchColor, flags,
                       &translate);
        }
    }
}
