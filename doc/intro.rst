************
Introduction
************

.. highlight:: cpp

Matcher library is an not-yet-open-source library created by Intel. The document describes the Matcher 0.1 API, which is essentially a C++ and Python API.

Matcher has a modular structure, which means that the package includes several shared or static libraries. The following modules are available:

 * **Matcher** -  an image processing module that includes linear and non-linear image filtering, geometrical image transformations (resize, affine and perspective warping, generic table-based remapping), color space conversion, histograms, and so on.
 * **MatcherUtils** - the utilities module containing useful basic functions used by Matcher.

The further chapters of the document describe functionality of each module. But first, make sure to get familiar with the common API concepts used thoroughly in the library.

API Concepts
================

``matcher`` Namespace
---------------------

All the Matcher data structures and constants are placed into the ``matcher`` namespace. Therefore, to access this functionality from your code, use the ``matcher::`` specifier or ``using namespace matcher;`` directive:

.. code-block:: c

    #include "matcher/Matcher.hpp"
    ...
    matcher::Rect rect(x, y, width, height);
    ...

or ::

    #include "matcher/Matcher.hpp"
    using namespace matcher;
    ...
    Rect rect(x, y, width, height);
    ...

Some of the current or future Matcher's external names may conflict with OpenCV, STL
or other libraries. In this case, use explicit namespace specifiers to resolve the name conflicts.

In order to use some of the MatcherUtils functions, simply call it:

.. code-block:: c

    #include "matcher/MatcherUtils.hpp"
    ...
    cv::Mat image_left, image_right;
    cv::Mat combined_image = MatcherUtils::combine(image_left, image_right);
    ...

Automatic Memory Management
---------------------------

Matcher handles all the memory automatically.

Everywhere that it is possible ``Ptr<>`` template class is used that is similar to ``std::shared_ptr`` from C++. So, instead of using plain pointers::

   T* ptr = new T(...);

it is used::

   cv::Ptr<T> ptr = new T(...);

That is, ``cv::Ptr<T> ptr`` encapsulates a pointer to a ``T`` instance and a reference counter associated with the pointer. To remove all the references attached to the pointer,
simply call ``release()`` on the pointer: ::

    ptr.relase()

Automatic Allocation of the Output Data
---------------------------------------

Matcher deallocates the memory automatically, as well as automatically allocates the memory for output function parameters most of the time. So, e.g. if a :class:`matcher_types.MatchQuery` has maxlocations field higher than 1, the output arrays in :class:`matcher_types.MatchResult` are automatically allocated or reallocated.

Example: ::

    #include "matcher/Matcher.hpp"
    ...
    // Create match query
    MatchQuery mquery;
    mquery.screenshot = "homescreen.jpg";
    mquery.icon = "appgrid.jpg";
    mquery.method = matcher::LocateMethod::SOBEL;
    mquery.maxlocations = 5;

    // Match icon image with screenshot
    MatchResult *mresult = new MatchResult();
    cv::Ptr<cv::Mat> resultimg = matcher->match(image, mquery, mresult, entry);

    // MatchResult struct contains now results of mquery.maxlocations=5 best matches.
    for (int i = 0; i < mquery.maxlocations; i++) {
        std::cout << mresult->result[i] << std::endl;
        std::cout << mresult->bbox[i] << std::endl;
        std::cout << mresult->center[i] << std::endl;
    }

Error Handling
--------------

Matcher does not leek any exceptions. All exceptions are cought inside and information about
them is included in the message and in the form of the negtive result code.
