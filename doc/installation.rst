************
Installation
************

By ``<MATCHER_ROOT>`` we mean installation directory of Matcher library.

Dependencies
------------

Matcher library depends on ``OpenCV 2.4.3`` and ``tesseract 3.02-2``.

In order to install OpenCV one need to compile from source ::

    wget OpenCV243.tar
    tar xzj OpenCV243.tar
    cd OpenCV243
    cmake .
    make
    sudo make install
    sudo ldconfig

In order to install tesseract ::

    sudo apt-get install tesseract-ocr
    sudo apt-get install libtesseract-dev

Compilation
-----------

Navigate to the ``<MATCHER_ROOT>`` and type: ::

    cmake .
    make

Matcher shared object will be installed to the  ``<MATCHER_ROOT>/lib`` subdirectory.

Generating documentation
------------------------

The following tools are needed to generate documentation:

* sphinx > 1.2
* doxygen > 1.7.6
* breathe > 0.7.5

One can fetch the newest versions by: ::

    sudo apt-get install python-sphinx
    sudo apt-get install doxygen
    sudo pip install breathe

In order to generate documentation from source code, one need to navigate
to ``<MATCHER_ROOT>`` and type: ::

    cmake .
    make docs-html
