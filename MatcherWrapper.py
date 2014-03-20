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

from os import path
import ctypes
from matcher_types import MatcherConfig, MatchCQuery, MatchCResult, LocateMethod
from matcher_types import Point, Rect

# default config
dconfig = MatcherConfig(useOCR = True,
                        useOIR = True,
                        nfeatures = 10000,
                        scaleFactor = 1.2,
                        nlevels = 16,
                        edgeThreshold = 9,
                        firstLevel = 0,
                        WTA_K = 2,
                        scoreType = 0,
                        patchSize = 19,
                        min_dist_thresh = 0,
                        max_nndr_ratio = 1.0,
                        min_inliers = 4,
                        ransac_reprojection_thresh = 10,
                        gpu_enabled=False,
                        locate = True,
                        scale_invariant = True)

libmatcher = ctypes.cdll.LoadLibrary('libmatcher.so')

class Matcher(object):
    """Wrapper for shared Matcher library functions
    """
    def __init__(self, mconfig=None):
        if mconfig == None:
            mconfig = dconfig
        self.obj = libmatcher.Matcher_new(ctypes.byref(mconfig))

    def loadImage(self, screenshot):
        """Loads image with path given as 'screenshot' parameter and stores it
        in the current object of Matcher.

        :param screenshot: Path to the image (absolute or relative).
        :type screenshot: str
        :returns: True if loading succeeded, False otherwise.
        """
        return libmatcher.Matcher_loadImage(self.obj, screenshot)

    def unloadImage(self, screenshot):
        """Unloads image with path given as 'screenshot' parameter, i.e. removes
        it from the current object of Matcher.

        :param screenshot: Path to the image (absolute or relative).
        :type screenshot: str
        :returns: True if image was there, False otherwise.
        """
        return libmatcher.Matcher_unloadImage(self.obj, screenshot)

    def match(self, screenshot=None, icon=None, threshold=0, method=None,
              roi=None, searcharea=None, scale_factor=0, resultimage=None,
              maxlocations=1):
        """Processes images according to the input query. According to
        MatcherConfig it can locate icon in the screenshot using different
        location methods or verify icon image to the screenshot.

        :param screenshot: Path to the screenshot image
        :type screenshot: str
        :param icon: Path to the icon image. This image cannot be bigger than
            screenshot.
        :type icon: str
        :param threshold: the locate result must be >= threshold to pass.
        :type threshold: int
        :param method: location method value or name.
        :type method: str or :class:`matcher_types.LocateMethod`
        :param roi: template name which will act as a region of interest, i.e.
            roi is located firstly and then template is being located inside that
            location. The same input parameters are used for both.
        :type roi: str
        :param searcharea: Boundning box that acts as region of interest.
        :type searcharea: Rect
        :param scale_factor: Scale factor between queried template width
            (or height) and the width (or height) of the template we expect to
            find. If set to 0 then all possible scales are covered.
        :type scale_factor: double
        :returns: result struct containing score, bounding box, coordinates of
            the center point and result image name.
        """
        screenshot = str() if screenshot is None else screenshot
        icon = str() if icon is None else icon
        roi = str() if roi is None else roi
        searcharea = Rect() if searcharea is None else searcharea
        locatemethod = self.__getlocatemethod(method)

        mquery = MatchCQuery(screenshot, icon, threshold, locatemethod, roi,
                             searcharea, scale_factor, resultimage, maxlocations)

        results = (ctypes.c_int16 * maxlocations)()
        bboxes = (Rect * maxlocations)()
        centers = (Point * maxlocations)()
        mresult = MatchCResult(results, bboxes, centers)

        libmatcher.Matcher_match(self.obj, ctypes.byref(mquery),
                                 ctypes.byref(mresult))
        return mresult

    def locateCharacters(self, screenshot=None, characters=None, threshold=0,
                         searcharea=None):
        """Locates characters from the given string (characters) using OCR.
        Coordinates of each letter are put to coords list.

        :param screenshot: path to the screenshot image
        :type screenshot: str
        :param characters: input string of characters to be found
        :type screenshot: str
        :param threshold: the locate result must be >= threshold to pass.
        :type screenshot: int
        :param searcharea: Boundning box that acts as region of interest.
        :type screenshot: :class:`matcher_types.Rect`
        :returns: result struct containing score, bounding boxes, coordinates of
            each letter and the result image name.
        """
        noduplicate = ''.join(set(characters))
        N = len(noduplicate)
        locatemethod = self.__getlocatemethod("OCR")

        mquery = MatchCQuery(screenshot, noduplicate, threshold, locatemethod,
                             searcharea=searcharea, maxlocations=N)

        results = (ctypes.c_int16 * 1)()
        bboxes = (Rect * N)()
        centers = (Point * N)()
        mresult = MatchCResult(results, bboxes, centers)

        libmatcher.Matcher_locateCharacters(self.obj,
                                            ctypes.byref(mquery),
                                            ctypes.byref(mresult))
        return mresult

    @staticmethod
    def __getlocatemethod(method):
        """Analyzes locate method object type and returns array with locate
        method value and name.

        :param method: input locate method.
        :type method: str or :class:`matcher_types.LocateMethod`
        :returns: locate method value
        """
        if type(method) is type(str()) and \
        LocateMethod.NAMES_TO_VALUES.has_key(method):
            return LocateMethod.NAMES_TO_VALUES[method]
        elif type(method) is type(int()) and \
        LocateMethod.VALUES_TO_NAMES.has_key(method):
            return method
        return LocateMethod.FEATURE
