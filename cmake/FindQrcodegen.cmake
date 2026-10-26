# Fetch the pinned QR Code generator C++ sources into the shared cache.
include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")

fetch_thirdparty(
    NAME qrcodegen/v1.8.0
    GIT_REPO https://github.com/nayuki/QR-Code-generator.git
    GIT_COMMIT 720f62bddb7226106071d4728c292cb1df519ceb
    SUBDIR cpp
    OUT_SOURCE_DIR QRCODEGEN_SOURCE_DIR
)
foreach(_qrcodegen_file IN ITEMS qrcodegen.cpp qrcodegen.hpp)
    if(NOT EXISTS "${QRCODEGEN_SOURCE_DIR}/${_qrcodegen_file}")
        message(FATAL_ERROR "Incomplete qrcodegen sources: ${QRCODEGEN_SOURCE_DIR}")
    endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Qrcodegen REQUIRED_VARS QRCODEGEN_SOURCE_DIR)
