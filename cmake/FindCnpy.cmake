# FindCnpy.cmake - 引入 cnpy 源码并构建为静态库（用于 motion_tracking_helper 读取 .npz）
#
# 提供：
#   - target: cnpy  (STATIC，跟随主项目交叉编译，自动 link zlib)
#   - 变量:   CNPY_FOUND
#
# 实现方式：通过 FetchThirdParty.cmake 拉固定 commit 的源码到 cache，
# 然后把 cnpy.cpp 加进本项目构建。
#
# cnpy 仓库：https://github.com/rogersce/cnpy
#   - 纯 C++（cnpy.cpp + cnpy.h，约 300 行）
#   - 唯一外部依赖：zlib（K3 工具链已自带）
#   - 跨架构（x86_64 / rv64）通用，无需架构专用版本

if(DEFINED _FIND_CNPY_LOADED)
    return()
endif()
set(_FIND_CNPY_LOADED ON)

include("${CMAKE_CURRENT_LIST_DIR}/FetchThirdParty.cmake")

fetch_thirdparty(
    NAME cnpy
    GIT_REPO https://github.com/rogersce/cnpy.git
    GIT_COMMIT 4e8810b1a8637695171ed346ce68f6984e585ef4
    OUT_SOURCE_DIR _cnpy_src_dir
)

if(NOT EXISTS "${_cnpy_src_dir}/cnpy.cpp" OR NOT EXISTS "${_cnpy_src_dir}/cnpy.h")
    message(FATAL_ERROR "cnpy 源码不完整: ${_cnpy_src_dir}")
endif()

# 第三方下载目录由不同分支和 worktree 共享，保持只读。将固定版本复制到
# 当前 build tree 后应用仓库内补丁，避免一个构建污染其他构建。
set(_cnpy_build_src_dir "${CMAKE_CURRENT_BINARY_DIR}/thirdparty/cnpy")
file(MAKE_DIRECTORY "${_cnpy_build_src_dir}")
configure_file("${_cnpy_src_dir}/cnpy.cpp"
    "${_cnpy_build_src_dir}/cnpy.cpp" COPYONLY)
configure_file("${_cnpy_src_dir}/cnpy.h"
    "${_cnpy_build_src_dir}/cnpy.h" COPYONLY)

set(_cnpy_patch
    "${CMAKE_CURRENT_LIST_DIR}/patches/cnpy-zip64-selective-load.patch")
execute_process(
    COMMAND git apply --no-index --unidiff-zero --whitespace=nowarn
        "${_cnpy_patch}"
    WORKING_DIRECTORY "${_cnpy_build_src_dir}"
    RESULT_VARIABLE _cnpy_patch_result
    ERROR_VARIABLE _cnpy_patch_error
)
if(NOT _cnpy_patch_result EQUAL 0)
    message(FATAL_ERROR "cnpy 补丁应用失败: ${_cnpy_patch_error}")
endif()

# zlib（cnpy 唯一外部依赖）
find_package(ZLIB REQUIRED)

# 静态库目标
if(NOT TARGET cnpy)
    add_library(cnpy STATIC ${_cnpy_build_src_dir}/cnpy.cpp)
    target_include_directories(cnpy PUBLIC ${_cnpy_build_src_dir})
    target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)
    set_target_properties(cnpy PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

set(CNPY_FOUND TRUE)
message(STATUS "cnpy: patched source at ${_cnpy_build_src_dir}, linked with ZLIB::ZLIB")
