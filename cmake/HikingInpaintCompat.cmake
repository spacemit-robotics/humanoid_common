# Pre-4.10 OpenCV misreads masked pixels on crop borders (opencv/opencv#25402).
# Compile only the corrected inpaint routine against the existing core/imgproc;
# do not replace the system OpenCV libraries or interpose their public symbol.
include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")
fetch_thirdparty(
    NAME opencv-4.11.0
    GIT_REPO https://github.com/opencv/opencv.git
    GIT_COMMIT 31b0eeea0b44b370fd0712312df4214d4ae1b158
    OUT_SOURCE_DIR _opencv_src)
set(_photo_dir "${CMAKE_CURRENT_BINARY_DIR}/thirdparty/hiking_photo")
file(MAKE_DIRECTORY "${_photo_dir}")
configure_file("${_opencv_src}/modules/photo/src/inpaint.cpp" "${_photo_dir}/inpaint.cpp" COPYONLY)
# The standalone translation unit only needs public headers, not OpenCV build instrumentation.
file(GENERATE OUTPUT "${_photo_dir}/precomp.hpp"
    CONTENT "#include <opencv2/photo.hpp>\n#include <opencv2/imgproc.hpp>\n#define CV_INSTRUMENT_REGION()\n")
target_sources(humanoid_depth PRIVATE "${_photo_dir}/inpaint.cpp")
set_property(SOURCE "${_photo_dir}/inpaint.cpp" src/perception/depth_stream.cpp
    APPEND PROPERTY COMPILE_DEFINITIONS inpaint=humanoidDepthInpaint411)
install(FILES "${_opencv_src}/LICENSE" "${_opencv_src}/modules/photo/src/inpaint.cpp"
    DESTINATION share/humanoid_common/thirdparty/opencv)
