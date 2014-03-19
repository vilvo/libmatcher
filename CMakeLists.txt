#
# matcher for computer-vision based SW testing
# Copyright (c) 2012-2014, Intel Corporation.
# 
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU Lesser General Public License,
# version 2.1, as published by the Free Software Foundation.
# 
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
# License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

project(libmatcher)
cmake_minimum_required (VERSION 2.6)
set(LIBRARY_OUTPUT_PATH lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
  "/usr/include/tesseract" "/usr/include/leptonica" "/usr/local/include")
link_directories(${OpenCV_LIB_DIR})

add_library(libmatcher SHARED
    Matcher.cpp
    MatchEntry.cpp
    MatchEntryGPU.cpp
    MatcherUtils.cpp
    OpponentColorDescriptor.cpp
    matcher_types.cpp
    matcher_extern.hpp
    matcher_consts.hpp)

target_link_libraries(libmatcher ${OpenCV_LIBS} lept tesseract)
set_target_properties(libmatcher PROPERTIES OUTPUT_NAME matcher)
install(TARGETS libmatcher LIBRARY DESTINATION lib)


# Generate documentation

macro(add_sphinx_target TARGET_NAME BUILDER COMMENT_STR)
    add_custom_target(${TARGET_NAME}
    COMMAND sphinx-build -b ${BUILDER} . doc/
        WORKING_DIRECTORY doc
        DEPENDS doxygen
        COMMENT ${COMMENT_STR}
    )
endmacro(add_sphinx_target)

add_custom_target(doxygen
    COMMAND doxygen doc/Doxyfile
    COMMENT "Build doxygen xml files used by sphinx/breathe."
)

add_sphinx_target(docs-html
    html
    "Build html documentation"
)