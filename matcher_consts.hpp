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

#ifndef matcher_consts_H
#define matcher_consts_H

#include <opencv2/opencv.hpp>  // for cv::Size

#include <map>
#include <string>

using std::string;

namespace matcher {
    // useful constants
    const std::string IMAGETYPE = ".png";
    const std::string RESULTPATH = "/tmp/";

    // OCR related
    const char* const LANGUAGE = "eng";
    const int BLURNESS = 3;  // Gaussian kernel standard deviation
    const double MINSHARP = 0.5;
    const double MAXSHARP = 1.1;
    const double STEPSHARP = 0.1;
    const int MINTHRESH = 10;
    const int MAXTHRESH = 200;
    const int STEPTHRESH = 10;
    const std::string ILLEGALCHARS = "\\/:?\"<>|";
    const int PARAINDENT = 4;  // Number of characters for paragraph identation

    // Matchers specific parameters
    const unsigned int LOWESTEXPECTEDKEYPOINTS = 30;
    const int KNN = 2;  // number of nearest neighbors
    const int NOPP = 3;  // opponent colour space channels count
    const int MAXCONFIDENCE = 100;
    const int SWIPETHRESHOLD = 30;
    const int SWIPETIMEOUT = 3;
    const cv::Size TESTIMAGESIZE(1080, 1920);
    const double AROFFSET = 0.2;  // Aspect ratio offset for validation
    const double SIZEOFFSET = 0.2;  // Size offset for validation
    const double MAXSCALE = 2.0;  // maximum scale for scale-invariant template
                                  // matching
    const cv::Size_<double> MINOCRIMAGE(400, 800);

    // Text string drawing parameters (set for 1920x1080 px resolution).
    // Automatic scaling is done inside Matcher.
    const unsigned MARGIN = 100;  // top and left margin in pixels
    const unsigned TEXTHEIGHT = 40;  // text line height in pixels
    const unsigned MAXLINELENGTH = 38;  // max line length in characters
    const unsigned RECTHEIGHT = 160;  // height of background rect in pixels
    const unsigned FONT = cv::FONT_HERSHEY_PLAIN;
    const double FONTSCALE = 2.0;  // Font scale factor that is multiplied
                                   // by the font-specific base size.
    const double FONTTHICKNESS = 2.0;  // Thickness of the lines used to draw
    const double OPACITY = 0.5;  // opacity of the background rectangle
    const int MAXCROSSSIZE = 20;  // in pixels
    const int DRAWSHIFTBITS = 4;
    const int DRAWMULTIPLIER = 1 << DRAWSHIFTBITS;

    enum errors {
        /* values between 0-100 are used for verification success
           and value indicates confidence of match in percentage.
           All below zero are used for error codes */
        success = 0,
        verification_failed = -1,
        screen_off_identified = -2,
        read_image_err = -3,
        input_entry_err = -4,
        init_matcher_err = -5,
        empty_image_err = -6,
        write_image_err = -13,
        open_file_err = -14,
        write_file_err = -15,
        ocr_err = -16,
        image_conversion_error = -21,
        invalid_configuration = -24
    };

    struct errormap : std::map<int, std::string> {
        errormap() {
            this->operator[](input_entry_err) = "Input MatchEntry is invalid";
            this->operator[](read_image_err) = "Cannot read image from file";
            this->operator[](write_image_err) = "Cannot write image to file";
            this->operator[](open_file_err) = "Cannot open file";
            this->operator[](write_file_err) = "Cannot write to file";
            this->operator[](ocr_err) = "OCR error";
            this->operator[](verification_failed) = "Verification failed";
            this->operator[](screen_off_identified) = "Screen off identified";
        };
        ~errormap() {}
    };
}

#endif  // matcher_consts_H
