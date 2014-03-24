libmatcher
==========

Python usage with fMBT:

```
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
