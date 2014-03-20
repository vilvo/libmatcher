"""
tests for libmatcher python bindings
Copyright (c) 2014, Intel Corporation.

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
"""

import sys, unittest
sys.path.append('../../')
from MatcherWrapper import Matcher, MatcherConfig, Rect

"""
- Folder for result images is set in matcher_consts.h. Currently it is '/tmp/'.
- threshold is a value between [0-100]
- locate method can be: FEATURE, MATCHTEMPLATE, SOBEL, OCR

Open issues:
- tesseract library prints nasty messages (like "Empty page!!") to the standard
  output.
"""

def printresult(result, maxlocations=1, onlyoneresult=False):
    """ Prints result details to stdout for debugging and analysis """
    printresult = True
    for idx in range(maxlocations):
        if printresult:
            print "Result (%s): " % int(idx+1), result.result[idx]
            printresult = False if onlyoneresult else True
        print "bbox (%s): " % int(idx+1), result.bbox[idx].x, result.bbox[idx].y, \
                        result.bbox[idx].width, result.bbox[idx].height
        print "coord: (%s): " % int(idx+1), result.center[idx].x, result.center[idx].y
    print "resultimage: ", result.resultimage
    print "message: ", result.message
    print '-'*40

mvconfig = MatcherConfig(useOCR = False,
                         useOIR = True,
                         nfeatures = 10000,
                         scaleFactor = 2.0,
                         nlevels = 10,
                         edgeThreshold = 31,
                         firstLevel = 0,
                         WTA_K = 2,
                         scoreType = 0,
                         patchSize = 31,
                         min_dist_thresh = 0,
                         max_nndr_ratio = 1.0,
                         min_inliers = 20,
                         ransac_reprojection_thresh = 10,
                         gpu_enabled=False,
                         locate = False)

class MatcherTest(unittest.TestCase):

    def test_loadunload(self):
        m = Matcher() # could be in setUp() but m shorter than self.m
        self.assertTrue(m.loadImage("../img/homescreen.png"))
        self.assertTrue(m.loadImage("../img/appgrid.png"))
        self.assertTrue(m.loadImage("../img/virtualkeyb_lowercase.png"))
        result = m.match("../img/homescreen.png",
                         "../img/icon_appgrid.png", 100, "FEATURE")
        #printresult(result)
        m.unloadImage("../img/homescreen.png")
        m.unloadImage("../img/appgrid.png")
        m.unloadImage("../img/virtualkeyb_lowercase.png")

    def test_featurematch(self):
        m = Matcher()
        result = m.match("../img/appgrid.png",
                         "../img/icon_clockworktomato.png", 100, "FEATURE",
                         resultimage="/tmp/test.jpg")
        self.assertTrue(result.result[0] == 83)
        result = m.match("../img/appgrid.png",
                         "../img/icon_browser.png", 100, "FEATURE",
                         resultimage="/tmp/test1.jpg")
        self.assertTrue(result.result[0] == 93)
        result = m.match("../img/appgrid.png",
                         "../img/icon_gplus.png", 100, "FEATURE",
                         resultimage="/tmp/test2.jpg")
        self.assertTrue(result.result[0] == 86)
        #printresult(result)

    def test_templatematch(self):
        m = Matcher()
        result = m.match("../img/appgrid.png",
                         "../img/icon_clockworktomato.png",
                         100, "MATCHTEMPLATE")
        self.assertTrue(result.result[0] == 100)
        result = m.match("../img/appgrid.png",
                         "../img/icon_browser.png",
                         100, "MATCHTEMPLATE")
        self.assertTrue(result.result[0] == 99)
        result = m.match("../img/appgrid.png",
                         "../img/icon_gplus.png",
                         100, "MATCHTEMPLATE")
        self.assertTrue(result.result[0] == 99)
        #printresult(result)

    def test_multipletemplatematch(self):
        m = Matcher()
        result = m.match("../img/quick_settings.png", "../img/icon_tickon.png",
                         99, "MATCHTEMPLATE", maxlocations=2, scale_factor=1)
        self.assertTrue(result.result[0] == 99)
        self.assertTrue(result.result[1] == 99)
        #printresult(result, 2)
        result = m.match("../img/quick_settings.png", "../img/icon_tickon.png",
                         99, "SOBEL", maxlocations=2, scale_factor=1)
        self.assertTrue(result.result[0] == 99)
        self.assertTrue(result.result[1] == 99)

    def test_verifyimage(self):
        m = Matcher(mvconfig)
        result = m.match("../img/homescreen.png", "../img/homescreen.png", 0)
        #printresult(result)

    def test_edgedetection(self):
        m = Matcher()
        result = m.match("../img/appgrid.png",
                         "../img/icon_clockworktomato.png", 0, "SOBEL")
        self.assertTrue(result.result[0] == 99)

    def test_negative(self):
        m = Matcher()
        result = m.match(icon="")
        self.assertTrue(result.result[0] == -6)
        result = m.match(None, None)
        self.assertTrue(result.result[0] == -6)
        result = m.match()
        self.assertTrue(result.result[0] == -6)

    def test_OCR_different_fonts(self):
        m = Matcher()
        print "NOTE: tesseract prints noise"
        #m.loadImage("../img/homescreen2.png")
        result = m.match("../img/homescreen2.png", "SONERA", 100, "OCR")
        self.assertTrue(result.result[0] == 83)
        result = m.match("../img/homescreen2.png", "Google", 100, "OCR")
        self.assertTrue(result.result[0] == 100)
        result = m.match("../img/homescreen2.png", "SmartActions", 100, "OCR")
        self.assertTrue(result.result[0] == 100)

    def test_OCR_sentences(self):
        m = Matcher()
        result = m.match("../img/virtualkeyb_lowercase.png",
                         "The conversation will appear here.", 100, "OCR")
        self.assertTrue(result.result[0] == 100)
        result = m.match("../img/virtualkeyb_lowercase.png",
                         "New message", 100, "OCR")
        self.assertTrue(result.result[0] == 100)
        result = m.match("../img/virtualkeyb_lowercase.png",
                         "Compose message", 100, "OCR")
        self.assertTrue(result.result[0] == 100)
        result = m.match("../img/virtualkeyb_lowercase.png",
                         "qwertyuiop", 90, "OCR")
        self.assertTrue(result.result[0] == 90)
        result = m.match("../img/virtualkeyb_lowercase.png",
                         "z", 100, "OCR")
        self.assertTrue(result.result[0] == 100)

    def test_OCR_multiple(self):
        m = Matcher()
        result = m.match("../img/quick_settings.png", "OFF", 100, "OCR", maxlocations=3)
        self.assertTrue(result.result[0] == 100)
        self.assertTrue(result.result[1] == 100)
        self.assertTrue(result.result[2] == 100)
        result = m.match("../img/quick_settings.png", "ON", 100, "OCR", maxlocations=3)
        self.assertTrue(result.result[0] == 100)
        self.assertTrue(result.result[1] == 100)
        self.assertTrue(result.result[2] == 100)

    def test_locate_inside_region_of_interest(self):
        m = Matcher()
        result = m.match("../img/quick_settings.png",
                         "../img/icon_tickon.png", 0, "MATCHTEMPLATE",
                         "../img/roi_bluetooth_settings.png")
        self.assertTrue(result.result[0] == 97)

    def test_locate_in_search_area(self):
        m = Matcher()
        bbox = Rect(100, 500, 300, 400)
        result = m.match("../img/homescreen.png",
                         "../img/icon_appgrid.png", searcharea=bbox)
        self.assertTrue(result.result[0] == 51)

    def test_load_before_matching(self):
        m = Matcher()
        self.assertTrue(m.loadImage("../img/homescreen.png"))
        result = m.match(icon="../img/icon_appgrid.png")
        self.assertTrue(result.result[0] == 51)

    def test_locate_characters(self):
        m = Matcher()
        self.assertTrue(m.loadImage("../img/virtualkeyb_lowercase.png"))
        chars = "qwertyuiopasdfghjklzxcvbnm"
        bbox = m.match(icon="../img/keyboardarea.png").bbox[0]
        result = m.locateCharacters(characters=chars, searcharea=bbox)
        self.assertTrue(result.result[0] == 96)
        #printresult(result, len(''.join(set(chars))), True)

if __name__ == '__main__':
    print "Use printresult() if you want to print match result details"
    unittest.main(verbosity=2)
