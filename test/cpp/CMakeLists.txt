project(libmatcher_example)
cmake_minimum_required (VERSION 2.6)
set(CMAKE_CXX_FLAGS "-Wall -D=HAVE_CONFIG_H")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "/usr/include/tesseract" "matcher")

link_directories(
	"/usr/local/lib"
	${OpenCV_LIBRARY_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../../lib
)

file(GLOB EXAMPLE_FILES "*.cpp")
foreach(FILE ${EXAMPLE_FILES})
    GET_FILENAME_COMPONENT(FILENAME ${FILE}  NAME_WE)
    add_executable(${FILENAME} ${FILE} )
    target_link_libraries(${FILENAME} matcher ${OpenCV_LIBRARIES})
endforeach()
