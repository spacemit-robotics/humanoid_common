cmake_minimum_required(VERSION 3.16)
project(humanoid_common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# mm 默认不传 CMAKE_BUILD_TYPE。StateRL runtime benchmark 会执行本包中的
# 控制与推理路径，因此必须与 Release 构建的 RL 库保持一致，不能混入 O0。
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "humanoid_common build type" FORCE)
endif()
message(STATUS "humanoid_common build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib:\$ORIGIN")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# 自定义 Find/Fetch 模块路径（FetchThirdParty / FindCnpy 等）
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# ============================================================================
# 平台检测
# ============================================================================

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(_x86 ON)
else()
    set(_x86 OFF)
endif()

# ============================================================================
# 依赖
# ============================================================================

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# rl 可选（找到则启用 RL 相关 target；否则跳过 behavior_manager / control_runtime / control_sim2sim_runtime / test_behavior）
find_library(RL_LIB NAMES rl
    PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_path(RL_INCLUDE_DIR NAMES rl_service.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)
if(RL_LIB AND RL_INCLUDE_DIR)
    set(BUILD_RL ON)
    message(STATUS "rl: ${RL_LIB}")
else()
    set(BUILD_RL OFF)
    message(STATUS "rl not found in ${CMAKE_INSTALL_PREFIX}, skipping RL-dependent targets (behavior_manager, control_runtime, control_sim2sim_runtime)")
endif()

# mujoco_sim 可选（x86_64 + riscv64/K3 上查找）
# riscv64 上 mujoco_sim 经 gl4es 走 PowerVR 硬件 GLES 渲染（见 components/simulation/mujoco）
set(BUILD_MUJOCO_BACKEND OFF)
if(_x86 OR CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
    find_library(MUJOCO_SIM_LIB NAMES mujoco_sim
        PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
    find_path(MUJOCO_SIM_INCLUDE_DIR NAMES mujoco_sim.h
        PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)
    if(MUJOCO_SIM_LIB AND MUJOCO_SIM_INCLUDE_DIR)
        set(BUILD_MUJOCO_BACKEND ON)
        message(STATUS "mujoco_sim: ${MUJOCO_SIM_LIB}")
    else()
        message(STATUS "mujoco_sim not found in ${CMAKE_INSTALL_PREFIX}")
    endif()
endif()

# whole_body 可选（找到则编入实机整机驱动 backend）
set(BUILD_WHOLE_BODY_BACKEND OFF)
set(DRIVER_RUNTIME_DEFINITIONS)
find_library(WHOLE_BODY_LIB NAMES whole_body
    PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_path(WHOLE_BODY_INCLUDE_DIR NAMES whole_body.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)
if(WHOLE_BODY_LIB AND WHOLE_BODY_INCLUDE_DIR)
    set(BUILD_WHOLE_BODY_BACKEND ON)
    message(STATUS "whole_body: ${WHOLE_BODY_LIB}")
else()
    message(STATUS "whole_body not found in ${CMAKE_INSTALL_PREFIX}")
endif()

# ============================================================================
# 库：robot_base
# ============================================================================

add_library(robot_base SHARED
    src/logging/runtime_logger.cpp
    src/robot_base/robot_base.cpp
    src/robot_base/thread_utils.cpp
)

target_include_directories(robot_base
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/logging>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_base
)

target_link_libraries(robot_base
    PUBLIC  Threads::Threads yaml-cpp
)

# ============================================================================
# 库：transport_executor（含 udp/shm 实现）
# ============================================================================

add_library(transport_executor SHARED
    src/transport/transport_executor/transport_executor.cpp
    src/transport/transport_executor/transport_udp_impl.cpp
    src/transport/transport_executor/transport_shm_impl.cpp
    src/transport/udp/udp_transport.cpp
    src/transport/shm/shm_transport.cpp
)

target_include_directories(transport_executor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/transport_executor
        ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/udp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/shm
)

target_compile_definitions(transport_executor PRIVATE HAS_UDP HAS_SHM)

target_link_libraries(transport_executor
    PUBLIC  robot_base
    PRIVATE rt  # shm_open / mmap
)

# ============================================================================
# 库：behavior_manager（依赖 rl，可选）
# ============================================================================

if(BUILD_RL)
    # cnpy：MJLab policy adapter 读取外置 .npz 参考动作。
    find_package(Cnpy REQUIRED)

    add_library(behavior_manager SHARED
        src/behavior_manager/behavior_fsm.cpp
        src/behavior_manager/behavior_manager.cpp
        src/behavior_manager/rl_state_config.cpp
        src/behavior_manager/policy_adapter/policy_adapter.cpp
        src/behavior_manager/states/state_power_off.cpp
        src/behavior_manager/states/state_damp.cpp
        src/behavior_manager/states/state_home.cpp
        src/behavior_manager/states/state_zero.cpp
        src/behavior_manager/states/state_rl.cpp
        src/behavior_manager/states/state_safety.cpp
    )

    target_include_directories(behavior_manager
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_manager
            ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_manager/policy_adapter
            ${RL_INCLUDE_DIR}
    )

    target_link_libraries(behavior_manager
        PUBLIC  robot_base
        PRIVATE Eigen3::Eigen ${RL_LIB} cnpy
    )
endif()

# ============================================================================
# 安装：基础库 + 公开头文件
# ============================================================================

install(TARGETS robot_base transport_executor
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    include/robot_base.h
    include/transport_executor.h
    DESTINATION include
)

if(BUILD_RL)
    install(TARGETS behavior_manager
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
    install(FILES include/behavior_manager.h DESTINATION include)
endif()

# ============================================================================
# 可执行文件
# ============================================================================

# control_runtime（实机控制：behavior FSM + transport，依赖 rl）
if(BUILD_RL)
    add_executable(control_runtime src/runtime/control_runtime.cpp)
    target_include_directories(control_runtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(control_runtime PRIVATE behavior_manager transport_executor robot_base)
    install(TARGETS control_runtime RUNTIME DESTINATION bin)

    # control_sim2sim_runtime（sim2sim：直接调 RL，无 FSM）
    add_executable(control_sim2sim_runtime src/runtime/control_sim2sim_runtime.cpp)
    target_include_directories(control_sim2sim_runtime PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${RL_INCLUDE_DIR}
    )
    target_link_libraries(control_sim2sim_runtime PRIVATE
        Eigen3::Eigen ${RL_LIB} transport_executor robot_base)
    install(TARGETS control_sim2sim_runtime RUNTIME DESTINATION bin)
endif()

# hmi_runtime（HMI 进程：仅 transport）
add_executable(hmi_runtime src/runtime/hmi_runtime.cpp)
target_include_directories(hmi_runtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hmi_runtime PRIVATE transport_executor robot_base)
install(TARGETS hmi_runtime RUNTIME DESTINATION bin)

# driver_runtime（按 YAML 选择已编译的 MuJoCo / whole_body backend）
if(BUILD_MUJOCO_BACKEND OR BUILD_WHOLE_BODY_BACKEND)
    set(DRIVER_RUNTIME_SOURCES
        src/runtime/driver_runtime.cpp
        src/runtime/driver_backend_factory.cpp
    )
    set(DRIVER_RUNTIME_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/runtime
    )
    set(DRIVER_RUNTIME_LIBRARIES transport_executor robot_base)
    if(BUILD_MUJOCO_BACKEND)
        list(APPEND DRIVER_RUNTIME_SOURCES src/runtime/backends/mujoco_backend.cpp)
        list(APPEND DRIVER_RUNTIME_INCLUDE_DIRS ${MUJOCO_SIM_INCLUDE_DIR})
        list(APPEND DRIVER_RUNTIME_LIBRARIES ${MUJOCO_SIM_LIB})
        list(APPEND DRIVER_RUNTIME_DEFINITIONS HAS_MUJOCO_BACKEND)
    endif()
    if(BUILD_WHOLE_BODY_BACKEND)
        list(APPEND DRIVER_RUNTIME_SOURCES
            src/runtime/backends/whole_body_adapter.cpp
            src/runtime/backends/whole_body_diagnostics.cpp
            src/runtime/backends/whole_body_backend.cpp
        )
        list(APPEND DRIVER_RUNTIME_INCLUDE_DIRS ${WHOLE_BODY_INCLUDE_DIR})
        list(APPEND DRIVER_RUNTIME_LIBRARIES ${WHOLE_BODY_LIB})
        list(APPEND DRIVER_RUNTIME_DEFINITIONS HAS_WHOLE_BODY_BACKEND)
    endif()

    add_executable(driver_runtime ${DRIVER_RUNTIME_SOURCES})
    target_include_directories(driver_runtime PRIVATE
        ${DRIVER_RUNTIME_INCLUDE_DIRS}
    )
    target_compile_definitions(driver_runtime PRIVATE ${DRIVER_RUNTIME_DEFINITIONS})
    target_link_libraries(driver_runtime PRIVATE ${DRIVER_RUNTIME_LIBRARIES})
    target_link_directories(driver_runtime PRIVATE ${CMAKE_INSTALL_PREFIX}/lib)
    install(TARGETS driver_runtime RUNTIME DESTINATION bin)
else()
    message(STATUS "No driver backend found, skipping driver_runtime")
endif()

# ============================================================================
# 示例 / 单模块测试
# ============================================================================

add_executable(test_robot_base example/robot_base/test_robot_base.cpp)
target_include_directories(test_robot_base PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_robot_base PRIVATE robot_base)
install(TARGETS test_robot_base RUNTIME DESTINATION bin)

add_executable(test_policy_command_limits
    example/runtime/test_policy_command_limits.cpp)
target_include_directories(test_policy_command_limits PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/runtime)
target_compile_definitions(test_policy_command_limits PRIVATE
    "POLICY_COMMAND_LIMITS_TEST_CONFIG=\"${CMAKE_CURRENT_SOURCE_DIR}/example/runtime/policy_command_limits.yaml\"")
target_link_libraries(test_policy_command_limits PRIVATE robot_base)
enable_testing()
add_test(NAME policy_command_limits COMMAND test_policy_command_limits)

add_executable(test_transport_executor example/transport_executor/test_transport_executor.cpp)
target_include_directories(test_transport_executor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/transport/shm
)
target_link_libraries(test_transport_executor PRIVATE transport_executor robot_base)
install(TARGETS test_transport_executor RUNTIME DESTINATION bin)

set(DRIVER_RUNTIME_EXAMPLE_CONFIG
    ${CMAKE_CURRENT_SOURCE_DIR}/example/driver_runtime/config_example.yaml)
add_executable(test_driver_runtime
    example/driver_runtime/test_driver_runtime.cpp
    src/runtime/driver_backend_factory.cpp
)
target_include_directories(test_driver_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/runtime
)
if(BUILD_WHOLE_BODY_BACKEND)
    target_sources(test_driver_runtime PRIVATE
        src/runtime/backends/whole_body_adapter.cpp
    )
    target_include_directories(test_driver_runtime PRIVATE
        ${WHOLE_BODY_INCLUDE_DIR}
    )
endif()
target_compile_definitions(test_driver_runtime PRIVATE
    ${DRIVER_RUNTIME_DEFINITIONS}
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_driver_runtime PRIVATE -UNDEBUG)
endif()
target_link_libraries(test_driver_runtime PRIVATE robot_base)
install(TARGETS test_driver_runtime RUNTIME DESTINATION bin)

option(BUILD_TESTS "Build humanoid_common offline tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_test(NAME driver_runtime_backend_selection
        COMMAND test_driver_runtime ${DRIVER_RUNTIME_EXAMPLE_CONFIG})
endif()

if(BUILD_RL)
    add_executable(test_behavior example/behavior_manager/test_behavior.cpp)
    target_include_directories(test_behavior PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${RL_INCLUDE_DIR}
    )
    target_link_libraries(test_behavior PRIVATE behavior_manager transport_executor robot_base)
    install(TARGETS test_behavior RUNTIME DESTINATION bin)

    add_executable(test_policy_adapter
        example/behavior_manager/test_policy_adapter.cpp)
    target_include_directories(test_policy_adapter PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_manager
        ${RL_INCLUDE_DIR}
    )
    target_link_libraries(test_policy_adapter PRIVATE
        behavior_manager robot_base Eigen3::Eigen ${RL_LIB} cnpy)
    install(TARGETS test_policy_adapter RUNTIME DESTINATION bin)

    # StateRL 异步运行时：复用生产 FSM/StateRL/ThreadLoop，以 synthetic
    # 传感器驱动；不包含 transport、关节总线或执行器闭环。
    add_executable(benchmark_humanoid_policy
        example/behavior_manager/benchmark_humanoid_policy.cpp)
    target_include_directories(benchmark_humanoid_policy PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_manager
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ${RL_INCLUDE_DIR}
    )
    target_link_libraries(benchmark_humanoid_policy PRIVATE
        behavior_manager robot_base Eigen3::Eigen ${RL_LIB} cnpy)
    set_target_properties(benchmark_humanoid_policy PROPERTIES
        BUILD_RPATH_USE_ORIGIN ON
    )

    string(TOUPPER "${CMAKE_BUILD_TYPE}" _common_build_type_upper)
    get_directory_property(_common_compile_options COMPILE_OPTIONS)
    string(JOIN " " _common_directory_flags ${_common_compile_options})
    set(_common_benchmark_flags
        "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_common_build_type_upper}} ${_common_directory_flags}")
    string(STRIP "${_common_benchmark_flags}" _common_benchmark_flags)

    # Build metadata may contain quotes, backslashes, semicolons or newlines.
    # Generate a header instead of passing arbitrary text through -D, where it
    # could be re-tokenized by CMake, the build tool or the compiler driver.
    set(_common_benchmark_build_type "${CMAKE_BUILD_TYPE}")
    foreach(_metadata_var
            _common_benchmark_build_type
            _common_benchmark_flags)
        string(REPLACE "\\" "\\\\" ${_metadata_var} "${${_metadata_var}}")
        string(REPLACE "\"" "\\\"" ${_metadata_var} "${${_metadata_var}}")
        string(REPLACE "\n" "\\n" ${_metadata_var} "${${_metadata_var}}")
        string(REPLACE "\r" "\\r" ${_metadata_var} "${${_metadata_var}}")
        string(REPLACE "\t" "\\t" ${_metadata_var} "${${_metadata_var}}")
    endforeach()
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
    configure_file(
        cmake/benchmark_build_metadata.h.in
        generated/benchmark_build_metadata.h
        @ONLY
    )
    install(TARGETS benchmark_humanoid_policy RUNTIME DESTINATION bin)
    install(PROGRAMS scripts/run_benchmark_humanoid_policy.sh
        DESTINATION bin)
endif()
