# libmatcher

library to match visual information, like areas and text, from images.
Matching results include location data that can be used for example user-interaction testing.

## Building from source and testing

Ubuntu 14.04:

```
sudo apt-get install build-essential cmake libopencv-dev libboost-dev libtesseract-dev tesseract-ocr-eng
cmake .
make
export LD_LIBRARY_PATH=$PWD/lib
cd test/cpp
cmake .
make
./example_*
cd ../python
python test_matcher.py
```

libmatcher requires OpenCV 2.4. For earlier Ubuntu versions please install OpenCV from source.

## Basic usage

Given an image from an Android device homescreen

![](https://raw.github.com/vilvo/libmatcher/master/test/img/homescreen.png)

Basic text recognition:
```python
from MatcherWrapper import Matcher

m = Matcher()
r = m.match('test/img/homescreen.png', "Tampere", threshold=100, method="OCR")
```
produces result image

![](https://raw.github.com/vilvo/libmatcher/master/example/100_homescreen__Tampere_1395905228.png)

## Simple usage with fMBT and Android

```python
import fmbtandroid
from MatcherWrapper import Matcher

# assuming device unlocked, homescreen with Camera-icon
d = fmbtandroid.Device()
s = d.refreshScreenshot()

m = Matcher()
# match camera icon with optical character recognition
r = m.match(s.filename(), "Camera", threshold=100, method="OCR")
assert r.result[0] == 100, "Camera-icon not in homescreen"
# match camera icon (in file "icon_camera.png") with feature-matching
r = m.match(s.filename(), "icon_camera.png", 100, method="FEATURE")
assert r.result[0] == 100, "Camera-icon not in homescreen"
d.tap((r.center[0].x, r.center[0].y))
```

See test/ for more examples
