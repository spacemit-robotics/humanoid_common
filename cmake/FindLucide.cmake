# Fetch the pinned browser bundle; Node.js is not required for SDK builds.
include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")

fetch_thirdparty(
    NAME lucide/0.468.0
    ARCHIVE_URL https://registry.npmjs.org/lucide/-/lucide-0.468.0.tgz
    ARCHIVE_SUBDIR package
    OUT_SOURCE_DIR _lucide_source_dir
)
set(LUCIDE_JS "${_lucide_source_dir}/dist/umd/lucide.min.js")
set(LUCIDE_LICENSE "${_lucide_source_dir}/LICENSE")
foreach(_lucide_file IN ITEMS LUCIDE_JS LUCIDE_LICENSE)
    if(NOT EXISTS "${${_lucide_file}}")
        message(FATAL_ERROR "Incomplete Lucide package: ${${_lucide_file}}")
    endif()
endforeach()

file(SHA256 "${LUCIDE_JS}" _lucide_js_sha256)
file(SHA256 "${LUCIDE_LICENSE}" _lucide_license_sha256)
if(NOT _lucide_js_sha256 STREQUAL "3411692820cb8d47543f69496aa25fd603a358f4498046f41c508a5a3342210e" OR
   NOT _lucide_license_sha256 STREQUAL "1e7290b35280a048667bbf0ebabac1c7fd52a75300e8b2946ac165715997f2bc")
    message(FATAL_ERROR "Lucide 0.468.0 package checksum mismatch: ${_lucide_source_dir}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Lucide REQUIRED_VARS LUCIDE_JS LUCIDE_LICENSE)
