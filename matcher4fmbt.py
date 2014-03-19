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

import fmbtgti

import MatcherWrapper

def _area2searcharea(screenshot, area):
    top, left = fmbtgti._intCoords(area[:2], screenshot.size())
    bottom, right = fmbtgti._intCoords(area[2:], screenshot.size())
    return (top, left, bottom, right)

def _resultbbox2bbox(resultbbox):
    return (resultbbox.x,
            resultbbox.y,
            resultbbox.x + resultbbox.width,
            resultbbox.y + resultbbox.height)

class MatcherOcrEngine(fmbtgti.OcrEngine):
    """
    OCR engine specific parameters, can be used in all
    ...OcrText() methods:

          match (float, optional):
                  minimum match score in range [0.0, 1.0].
                  The default is 1.0 (exact match).

          area ((left, top, right, bottom), optional):
                  search from the given area only. Left, top
                  right and bottom are either absolute coordinates
                  (integers) or floats in range [0.0, 1.0]. In the
                  latter case they are scaled to screenshot
                  dimensions. The default is (0.0, 0.0, 1.0, 1.0),
                  that is, search everywhere in the screenshot.
    """
    def __init__(self, *args, **kwargs):
        fmbtgti.OcrEngine.__init__(self, *args, **kwargs)
        mconfig = MatcherWrapper.MatcherConfig(useOCR = True, useOIR = False)
        self._matcher = MatcherWrapper.Matcher(mconfig)

    def _addScreenshot(self, screenshot, **neverMindDefaults):
        self._matcher.loadImage(screenshot.filename())

    def _removeScreenshot(self, screenshot):
        self._matcher.unloadImage(screenshot.filename())

    def _findText(self, screenshot, text, match=1.0, area=(0.0, 0.0, 1.0, 1.0)):
        top, left = fmbtgti._intCoords(area[:2], screenshot.size())
        bottom, right = fmbtgti._intCoords(area[2:], screenshot.size())
        threshold = int(match*100)
        result = self._matcher.match(
            screenshot = screenshot.filename(),
            icon = text,
            threshold = threshold,
            method = "OCR",
            searcharea = _area2searcharea(screenshot, area),
            resultimage = "")
        if result.result[0]/100.0 < match:
            return []
        else:
            return [fmbtgti.GUIItem(
                "OCR text",
                _resultbbox2bbox(result.bbox[0]),
                screenshot = screenshot.filename(),
                ocrFind = text,
                ocrFound = "match:%.2f/%.2f" % (result.result[0]/100.0, match))]

_supportedOirMethods = {
    "template": "MATCHTEMPLATE",
    "feature": "FEATURE",
    "sobel": "SOBEL" }

class MatcherOirEngine(fmbtgti.OirEngine):
    """
    OIR engine specific parameters, can be used in all
    ...OcrBitmap() methods:

          match (float, optional):
                  minimum match score in range [0.0, 1.0].
                  The default is 1.0 (exact match).

          area ((left, top, right, bottom), optional):
                  search from the given area only. Left, top
                  right and bottom are either absolute coordinates
                  (integers) or floats in range [0.0, 1.0]. In the
                  latter case they are scaled to screenshot
                  dimensions. The default is (0.0, 0.0, 1.0, 1.0),
                  that is, search everywhere in the screenshot.

          limit (integer, optional):
                  maximum number of returned matches. The default is
                  1.

          method (string, optional): available matching methods are
                  "template", "feature", and "sobel". The default is
                  "template".

          scale (float, optional):
                  ??? should we provide a range, too: [1.0, 2.0, 0.1]
                  (try 1.0, 1.1, ..., 2.0)

          features (integer, optional):
                  number of features used in feature matching. The
                  default is 10000.

          nlevels (integer, optional):
                  ???

          edgeThreshold (integer, optional):
                  ???

          ???

    """
    def __init__(self, *args, **engineDefaults):
        engineDefaults["match"] = engineDefaults.get("match", 1.0)
        engineDefaults["area"] = engineDefaults.get("area", (0.0, 0.0, 1.0, 1.0))
        engineDefaults["method"] = engineDefaults.get("method", "template")
        engineDefaults["scale"] = engineDefaults.get("scale", 1.0)
        engineDefaults["features"] = engineDefaults.get("features", 10000)
        engineDefaults["nlevels"] = engineDefaults.get("nlevels", 16)
        engineDefaults["edgeThreshold"] = engineDefaults.get("edgeThreshold", 9)
        super(MatcherOirEngine, self).__init__(*args, **engineDefaults)

    def _findBitmap(self, screenshot, bitmap,
                    match=None, area=None, limit=None,
                    method=None, scale=None, features=None,
                    nlevels=None, edgeThreshold=None):
        if scale != None and scale > 0:
            scale_invariant = True
        else:
            scale_invariant = False
        mconfig = MatcherWrapper.MatcherConfig(
            useOCR          = False,
            useOIR          = True,
            nfeatures       = features,
            scaleFactor     = scale,
            nlevels         = nlevels,
            edgeThreshold   = edgeThreshold,
            firstLevel      = 0,
            WTA_K           = 2,
            scoreType       = 0,
            patchSize       = 19,
            min_dist_thresh = 0,
            max_nndr_ratio  = 1.0,
            min_inliers     = 4,
            gpu_enabled     = False,
            locate          = True,
            scale_invariant = scale_invariant,
            ransac_reprojection_thresh = 10)
        matcher = MatcherWrapper.Matcher(mconfig)
        if method not in _supportedOirMethods:
            raise ValueError('Invalid method "%s", use one of "%s"' %
                             '", "'.join(_supportedOirMethods.keys()))

        result = matcher.match(
            screenshot   = screenshot.filename(),
            icon         = bitmap,
            threshold    = int(match*100),
            method       = _supportedOirMethods[method],
            scale_factor = scale,
            searcharea   = _area2searcharea(screenshot, area),
            resultimage  = "")

        if result.result[0] <= 0 or result.result[0] < (match*100):
            return []
        else:
            # TODO: How to get all real matches from the result vector?
            # now there can be only one
            return [fmbtgti.GUIItem(
                "%s match %.2f" % (method, result.result[0] / 100.0),
                _resultbbox2bbox(result.bbox[0]),
                screenshot = screenshot.filename(),
                bitmap = bitmap)]
