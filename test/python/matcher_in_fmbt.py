"""Example & very manual test of using MatcherOcrEngine as part of fmbt.

Try out:

1. open two terminals

2. write some text in the first terminal, for instance
   $ cat > /dev/null
   HELLO WORLD

                              GOODBYE CRUEL WORLD

3. run in the second terminal:
   $ clear; python matcher_in_fmbt.py "HELLO WORLD" "CRUEL WORLD"

4. see the result:
   $ chromium matcher_in_fmbt.html
"""

import sys
sys.path.append('../../')

import matcher4fmbt
matcher4fmbt.MatcherOcrEngine().register(defaultOcr=True)

import fmbtx11
s = fmbtx11.Screen()
s.enableVisualLog("matcher_in_fmbt.html")

s.refreshScreenshot()

for text in sys.argv[1:]:
    print s.tapOcrText(text, match=0.8)
