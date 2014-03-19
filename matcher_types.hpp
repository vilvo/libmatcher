/*
  Copyright (c) 2013, Intel
  All rights reserved.

  Description:
    Matcher types header.
*/

#ifndef matcher_types_H
#define matcher_types_H

#include <sys/types.h>
#include <opencv2/opencv.hpp>  // for cv::Rect
#include <string>
#include <map>
#include <vector>

using std::map;
using std::string;

// Shared structs between Python and C++

namespace matcher {
    /**
     * Matcher configuration containing feature-detection related parameters.
     */
    struct MatcherConfig {
        /**
         * Flag if set to True then OCR engine is enabled.
         */
        bool useOCR;
        /**
         * Flag if set to True then every functionality except for OCR is
         * enabled. If flag is set to False then all further MatcherConfig
         * parameters are ignored.
         */
        bool useOIR;
        // ORB parameters
        /**
         * Maximum number of features to retain.
         */
        int32_t nfeatures;
        /**
         * Pyramid decimation ratio, always greater than 1. Value close to 1
         * means that to cover certain scale range we need more pyramid levels
         * and so the speed will suffer. Value close to 2 increases the speed
         * but degrades feature matching scores dramatically.
         */
        double scaleFactor;
        /**
         * Number of pyramid levels.
         */
        int16_t nlevels;
        /**
         * Size of the border where the features are not detected. It should
         * roughly match the patch size.
         */
        int16_t edgeThreshold;
        /**
         * First level of the pyramid.
         */
        int16_t firstLevel;
        /**
         * The number of points that produce each element of the oriented
         * BRIEF descriptor.
         */
        int16_t WTA_K;
        /**
         * Algorithm used to rank features: HARRIS_SCORE or FAST_SCORE. It is
         * said that the latter produces slightly less stable keypoints, but
         * is a bit faster (according to the OpenCV library documentation).
         */
        int16_t scoreType;
        /**
         * Size of the patch used by the oriented BRIEF descriptor.
         */
        int16_t patchSize;
        // Other parameters
        /**
         * If an absolute distance between k=2 best matches for each descriptor
         * from a query set with train descriptors is lower than this threshold,
         * then such a descriptor is rejected.
         */
        int16_t min_dist_thresh;
        /**
         * If a distance ratio between k=2 best matches for each descriptor from
         * a query set with train descriptors is higher than this threshold,
         * then such a descriptor is rejected.
         */
        double max_nndr_ratio;
        /**
         * Minimum number of inliers required to compute homography.
         */
        int16_t min_inliers;
        /**
         * Maximum allowed reprojection error to treat a point pair (k=2 best
         * matches for each descriptor) as an inlier.
         */
        int16_t ransac_reprojection_thresh;
        /**
         * Flag if set to true then GPU is used as a prefered processing unit
         * if OpenCV is compiled with CUDA support and NVIDIA graphic card is
         * installed in the PC.
         */
        bool gpu_enabled;
        /**
         * Flag that indicates whether this particular instance of Matcher
         * suits for locating (locate is True) or verifying (locate is False).
         */
        bool locate;
        /**
         * Scale invarinat mode ON (true) or OFF (false).
         */
        bool scale_invariant;
    };

    /**
     * Struct to store point coordinates.
     */
    struct Point {
        Point();
        Point(int32_t _x, int32_t _y);
        /**
         * X-coordinate
         */
        int32_t x;
        /**
         * Y-coordinate
         */
        int32_t y;
        /**
         * Overloaded operator << for printing.
         */
        friend std::ostream& operator <<(std::ostream& os, const Point& p);
    };

    /**
     * Struct to store rectangle parameters
     */
    struct Rect {
        Rect();
        explicit Rect(const cv::Rect &rect);
        explicit Rect(const Rect &rect);
        Rect(int32_t _x, int32_t _y, int32_t _w, int32_t _h);
        /**
         * X-coordinate of the top-left corner
         */
        int32_t x;
        /**
         * Y-coordinate of the top-left corner
         */
        int32_t y;
        /**
         * width of the rectangle
         */
        int32_t width;
        /**
         * height of the rectangle
         */
        int32_t height;
        /**
         * Checks whether rectangle is an empty struct.
         * @return True if empty. Otherwise false.
         */
        bool empty() const;
        /**
         * Overloaded operator << for printing.
         */
        friend std::ostream& operator <<(std::ostream& os, const Rect& rect);
    };

    /**
     * Struct of query passed to the Matcher. Since it is a shared struct used
     * by both Python and C/C++ it contains C-strings instead of std::string.
     */
    struct MatchCQuery {
        MatchCQuery();
        explicit MatchCQuery(const MatchCQuery &m);
        /**
         * Path to the screenshot image
         */
        char* screenshot;
        /**
         * Path to the icon image. This image cannot be bigger than screenshot.
         */
        char* icon;
        /**
         * the locate result must be >= threshold to pass.
         */
        int16_t threshold;
        /**
         * location method value (matcher::LocateMethod) or name (string).
         */
        int16_t method;
        /**
         * template name which will act as a region of interest, i.e. roi is
         * located firstly and then template is being located inside that
         * location. The same input parameters are used for both.
         */
        char* roi;
        /**
         * Boundning box that acts as region of interest.
         */
        Rect searcharea;
        /**
         * Scale factor between queried template width (or height) and the
         * width (or height) of the template we expect to find. If set to 0
         * then all possible scales are covered.
         */
        double scale_factor;
        /**
         * result image filename
         */
        char* resultimage;
        /**
         * maximum number of icon locations in the screenshot.
         */
        int16_t maxlocations;
    };

    /**
     * C++ version of MatchCQuery. Uses std::string instead of C-string for
     * convenience.
     */
    struct MatchQuery : public MatchCQuery {
        MatchQuery();
        explicit MatchQuery(const MatchCQuery &mcquery);
        /**
         * Overloaded asignment operator.
         */
        MatchQuery& operator =(const MatchQuery& m);
        /**
         * Path to the screenshot image
         */
        std::string screenshot;
        /**
         * Path to the icon image. This image cannot be bigger than screenshot.
         */
        std::string icon;
        /**
         * template name which will act as a region of interest, i.e. roi is
         * located firstly and then template is being located inside that
         * location. The same input parameters are used for both.
         */
        std::string roi;
        /**
         * result image filename
         */
        std::string resultimage;
    };

    /**
     * Result struct received from Matcher. C-format because Python side
     * utilized ctypes.
     */
    struct MatchCResult {
        explicit MatchCResult();
        /**
         * table of results for each location (according to ``maxlocations``
         * field of matcher::MatchCQuery`)
         */
        int16_t* result;
        /**
         * table of resulted bounding boxes for each location (according to
         * ``maxlocations`` field of matcher::MatchCQuery`)
         */
        Rect* bbox;
        /**
         * table of center points of each location (according to
         * ``maxlocations`` field of matcher::MatchCQuery`)
         */
        Point* center;
        /**
         * result image name
         */
        char* resultimage;
        /**
         * number of inliers
         */
        int32_t nInliers;
        /**
         * number of inliers
         */
        int32_t nMatches;
        /**
         * additional information message from Matcher.
         */
        char* message;
    };

    /**
     * Result struct received from Matcher.
     */
    struct MatchResult : public MatchCResult {
        explicit MatchResult(int16_t maxlocations = 1);
        /**
         * Assigns elements of indices 'idx' from 'm' to the first elements of
         * the current object.
         * @param m   Struct to be copied
         * @param idx Index of elements from 'm' to be copied.
         */
        void operator()(const MatchResult &m, const int leftidx = 0,
                        const int rightidx = 0);
        /**
         * Comparison operator used for sorting
         * @param  i left struct
         * @param  j right struct
         * @return   true if i > j. Otherwise false.
         */
        bool operator() (MatchResult i, MatchResult j);
        /**
         * Clears the content of member vectors.
         */
        void clear();
        /**
         * Resizes member vectors to the given size.
         * @param size new vectors size as number of elements.
         */
        void resize(const unsigned int size);
        /**
         * table of results for each location (according to ``maxlocations``
         * field of matcher::MatchCQuery`)
         */
        std::vector<int16_t> result;
        /**
         * table of resulted bounding boxes for each location (according to
         * ``maxlocations`` field of matcher::MatchCQuery`)
         */
        std::vector<cv::Rect> bbox;
        /**
         * table of center points of each location (according to
         * ``maxlocations`` field of matcher::MatchCQuery`)
         */
        std::vector<Point> center;
        /**
         * result image name
         */
        std::string resultimage;
        /**
         * additional information message from Matcher.
         */
        std::string message;
    };

    // Structs used only in C++

    /**
     * Several phases of feature matching are available for locate. For verify
     * apply only DEFAULT phase.
     */
    struct Feature {
        enum phase {
            /**
             * Use customized ORB parameters provided in constructor.
             */
            DEFAULT,
            /**
             * Use customized ORB params provided in constructor within the
             * bounding box found by Sobel edge detector.
             */
            SOBEL,
            /**
             * Use Opponent Colour Space descriptor in boudning box found
             * by Sobel edge detector.
             */
            OPPONENT,
            /**
             * Raise score (inliers/all ratio)
             */
            RAISESCORE
        };
        /**
         * Map of Feature::phase enmuerator values to their names
         */
        static map<Feature::phase, string> VALUES_TO_DESC;
    };

    /**
     * Flags used in draw() method.
     */
    struct Draw {
        enum flag {
            /**
             * Draws detected keypoints and bounding box if locate is on.
             */
            ONLY_KEYPOINTS,
            /**
             * Draws detected keypoints and matches between left and right
             * image. Draws bounnding box if locate is on.
             */
            DRAW_MATCHES,
            /**
             * Draws black frame in place of right image. Draws bounding box
             * if locate is on.
             */
            NO_REF_IMAGE,
            /**
             * Draws bounding box on the top of the left image
             */
            ONLY_BBOX,
            /**
             *  Like NO_REF_IMAGE but without bounding box.
             */
            NO_BBOX
        };
    };

    /**
     * Icon location methods.
     */
    struct LocateMethod {
        enum type {
            /** Keypoint based feature matching
             */
            FEATURE,
            /**
             * Simple template matching using squared-differences matching
             */
            MATCHTEMPLATE,
            /**
             * Correlation-based template matching with Sobel operator
             * preprocessing (edge filtering).
             */
            SOBEL,
            /**
             * Optical character recognition. Locates single characters, words,
             * sentences and paragraphs.
             */
            OCR
        };
        /**
         * Map of LocateMethod enmuerator values to their names
         */
        static map<int16_t, string> VALUES_TO_NAMES;
    };
}  // namespace matcher
#endif  // matcher_types_H
