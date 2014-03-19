'''
  matcher for computer-vision based SW testing
  Copyright (c) 2012-2014, Intel Corporation.
 
  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU Lesser General Public License,
  version 2.1, as published by the Free Software Foundation.
 
  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.
 
  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
'''

import ctypes
import struct

# Shared structs between Python and C++

class MatcherConfig(ctypes.Structure):
    """
    Matcher configuration containing feature-detection related parameters.
    Flag 'locate' indicates whether this particular instance of Matcher suits
    for locating (locate is True) or verifying (locate is False).

    * **useOCR** -- Flag if set to True then OCR engine is enabled.
    * **useOIR** -- Flag if set to True then every functionality except for OCR
      is enabled. If flag is set to False then all further MatcherConfig
      parameters are ignored.
    * **nfeatures** -- Maximum number of features to retain.
    * **scaleFactor** -- Pyramid decimation ratio, always greater than 1. Value
      close to 1 means that to cover certain scale range we need more
      pyramid levels and so the speed will suffer. Value close to 2 increases
      the speed but degrades feature matching scores dramatically.
    * **nlevels** -- Number of pyramid levels.
    * **edgeThreshold** -- Size of the border where the features are not
      detected. It should roughly match the patch size.
    * **firstLevel** -- First level of the pyramid.
    * **WTA_K** -- The number of points that produce each element of the
      oriented BRIEF descriptor.
    * **scoreType** -- Algorithm used to rank features: HARRIS_SCORE or
      FAST_SCORE. It is said that the latter produces slightly less stable
      keypoints, but is a bit faster (according to the OpenCV library
      documentation).
    * **patchSize** -- Size of the patch used by the oriented BRIEF descriptor.
    * **min_dist_thresh** -- If an absolute distance between k=2 best matches
      for each descriptor from a query set with train descriptors is lower
      than this threshold, then such a descriptor is rejected.
    * **max_nndr_ratio** -- If a distance ratio between k=2 best matches for
      each descriptor from a query set with train descriptors is higher than
      this threshold, then such a descriptor is rejected.
    * **min_inliers** -- Minimum number of inliers required to compute
      homography.
    * **ransac_reprojection_thresh** -- Maximum allowed reprojection error to
      treat a point pair (k=2 best matches for each descriptor) as an inlier.
    * **gpu_enabled** -- Flag if set to true then GPU is used as a prefered
      processing unit if OpenCV is compiled with CUDA support and NVIDIA
      graphic card is installed in the PC.
    * **locate** -- Flag that indicates whether this particular instance of
      Matcher suits for locating (locate is True) or verifying (locate is
      False).
    * **scale_invariant** -- Scale invarinat mode ON (true) or OFF (false).
    """
    _fields_ = [("useOCR", ctypes.c_bool),
                ("useOIR", ctypes.c_bool),
                ("nfeatures", ctypes.c_int32),
                ("scaleFactor", ctypes.c_double),
                ("nlevels", ctypes.c_int16),
                ("edgeThreshold", ctypes.c_int16),
                ("firstLevel", ctypes.c_int16),
                ("WTA_K", ctypes.c_int16),
                ("scoreType", ctypes.c_int16),
                ("patchSize", ctypes.c_int16),
                ("min_dist_thresh", ctypes.c_int16),
                ("max_nndr_ratio", ctypes.c_double),
                ("min_inliers", ctypes.c_int16),
                ("ransac_reprojection_thresh", ctypes.c_int16),
                ("gpu_enabled", ctypes.c_bool),
                ("locate", ctypes.c_bool),
                ("scale_invariant", ctypes.c_bool)]

class Point(ctypes.Structure):
    """ Struct to store point coordinates.

    * **x** -- X-coordinate
    * **y** -- Y-coordinate
    """
    _fields_ = [("x", ctypes.c_int32),
                ("y", ctypes.c_int32)]

class Rect(ctypes.Structure):
    """ Struct to store rectangle parameters

    * **x** -- X-coordinate of the top-left corner
    * **y** -- Y-coordinate of the top-left corner
    * **width** -- width of the rectangle
    * **height** -- height of the rectangel
    """
    _fields_ = [("x", ctypes.c_int32),
                ("y", ctypes.c_int32),
                ("width", ctypes.c_int32),
                ("height", ctypes.c_int32)]

class MatchCQuery(ctypes.Structure):
    """ Struct of query passed to the Matcher.

    * **screenshot** -- Path to the screenshot image
    * **icon** -- Path to the icon image. This image cannot be bigger than
      screenshot.
    * **threshold** -- the locate result must be >= threshold to pass.
    * **method** -- location method value or name.
    * **roi** -- template name which will act as a region of interest, i.e.
      roi is located firstly and then template is being located inside that
      location. The same input parameters are used for both.
    * **searcharea** -- Boundning box that acts as region of interest.
    * **scale_factor** -- Scale factor between queried template width (or
      height) and the width (or height) of the template we expect to find. If
      set to 0 then all possible scales are covered.
    * **resultimage** -- result image filename
    * **maxlocations** -- maximum number of icon locations in the screenshot.
    """
    _fields_ = [("screenshot", ctypes.c_char_p),
                ("icon", ctypes.c_char_p),
                ("threshold", ctypes.c_int16),
                ("method", ctypes.c_int16),
                ("roi", ctypes.c_char_p),
                ("searcharea", Rect),
                ("scale_factor", ctypes.c_double),
                ("resultimage", ctypes.c_char_p),
                ("maxlocations", ctypes.c_int16)]

class MatchCResult(ctypes.Structure):
    """ Result struct received from Matcher.

    * **result** -- table of results for each location (according to
      ``maxlocations`` field of :class:`matcher_types.MatchQuery`)
    * **bbox** -- table of resulted bounding boxes for each location (according
      to ``maxlocations`` field of :class:`matcher_types.MatchQuery`)
    * **center** -- table of center points of each location (according to
      ``maxlocations`` field of :class:`matcher_types.MatchQuery`)
    * **resultimage** -- result image name
    * **nInliers** -- number of inliers
    * **nMatches** -- number of all matches
    * **message** -- additional information message from Matcher.
    """
    _fields_ = [("result", ctypes.POINTER(ctypes.c_int16)),
                ("bbox", ctypes.POINTER(Rect)),
                ("center", ctypes.POINTER(Point)),
                ("resultimage", ctypes.c_char_p),
                ("nIliers", ctypes.c_int32),
                ("nMatches", ctypes.c_int32),
                ("message", ctypes.c_char_p)]

# Structs used only in Python

class LocateMethod:
    """ Icon location methods.
    """
    #: Keypoint based feature matching
    FEATURE = 0
    #: Simple template matching using squared-differences matching
    MATCHTEMPLATE = 1
    #: Correlation-based template matching with Sobel operator preprocessing
    #: (edge filtering).
    SOBEL = 2
    #: Optical character recognition. Locates single characters, words,
    #: sentences and paragraphs.
    OCR = 3

    #: Map of LocateMethod members values to their names
    VALUES_TO_NAMES = {
        0: "FEATURE",
        1: "MATCHTEMPLATE",
        2: "SOBEL",
        3: "OCR",
    }

    #: Map of LocateMethod members names to their values
    NAMES_TO_VALUES = {
        "FEATURE": 0,
        "MATCHTEMPLATE": 1,
        "SOBEL": 2,
        "OCR": 3,
    }
