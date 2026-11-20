# Pre-4.10 OpenCV misreads masked pixels on crop borders (opencv/opencv#25402).
# Compile only the corrected inpaint routine against the existing core/imgproc;
# do not replace the system OpenCV libraries or interpose their public symbol.
include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")
set(_opencv_raw "https://raw.githubusercontent.com/opencv/opencv/31b0eeea0b44b370fd0712312df4214d4ae1b158")
set(_cache_prefix "opencv-inpaint/4.11.0")
set(_old_checkout "${_FETCH_TP_CACHE_ROOT}/opencv-4.11.0")
fetch_thirdparty_file("${_cache_prefix}/inpaint.cpp" "${_opencv_raw}/modules/photo/src/inpaint.cpp"
    15c66e1f742fd8a08590996e80e06895c5355126bd9457ab613be03179019734 _inpaint_src
    "${_old_checkout}/modules/photo/src/inpaint.cpp")
fetch_thirdparty_file("${_cache_prefix}/LICENSE" "${_opencv_raw}/LICENSE"
    cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30 _opencv_license
    "${_old_checkout}/LICENSE")
set(_photo_dir "${CMAKE_CURRENT_BINARY_DIR}/thirdparty/hiking_photo")
file(MAKE_DIRECTORY "${_photo_dir}")
configure_file("${_inpaint_src}" "${_photo_dir}/inpaint.cpp" COPYONLY)
# The standalone translation unit only needs public headers, not OpenCV build instrumentation.
file(GENERATE OUTPUT "${_photo_dir}/precomp.hpp"
    CONTENT "#include <opencv2/photo.hpp>\n#include <opencv2/imgproc.hpp>\n#define CV_INSTRUMENT_REGION()\n")
target_sources(humanoid_depth PRIVATE "${_photo_dir}/inpaint.cpp")
set_property(SOURCE "${_photo_dir}/inpaint.cpp" src/perception/depth_stream.cpp
    APPEND PROPERTY COMPILE_DEFINITIONS inpaint=humanoidDepthInpaint411)
install(FILES "${_opencv_license}" "${_inpaint_src}"
    DESTINATION share/humanoid_common/thirdparty/opencv)
