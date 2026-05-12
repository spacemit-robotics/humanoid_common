cmake_minimum_required(VERSION 3.16)
project(humanoid_common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

# mujoco_sim 可选（仅 x86_64 上查找；找到则启用 driver_runtime，否则跳过）
set(BUILD_DRIVER_SIM OFF)
if(_x86)
    find_library(MUJOCO_SIM_LIB NAMES mujoco_sim
        PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
    find_path(MUJOCO_SIM_INCLUDE_DIR NAMES mujoco_sim.h
        PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)
    if(MUJOCO_SIM_LIB AND MUJOCO_SIM_INCLUDE_DIR)
        set(BUILD_DRIVER_SIM ON)
        message(STATUS "mujoco_sim: ${MUJOCO_SIM_LIB}")
    else()
        message(STATUS "mujoco_sim not found in ${CMAKE_INSTALL_PREFIX}, skipping driver_runtime")
    endif()
endif()

# ============================================================================
# 库：robot_base
# ============================================================================

add_library(robot_base SHARED
    src/robot_base/robot_base.cpp
    src/robot_base/thread_utils.cpp
)

target_include_directories(robot_base
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
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
    # cnpy：用于 motion_tracking_helper 读取 .npz 参考动作（仅 tracking 策略启用时使用）
    find_package(Cnpy REQUIRED)

    add_library(behavior_manager SHARED
        src/behavior_manager/behavior_fsm.cpp
        src/behavior_manager/behavior_manager.cpp
        src/behavior_manager/motion_tracking_helper.cpp
        src/behavior_manager/states/state_power_off.cpp
        src/behavior_manager/states/state_damp.cpp
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
            ${RL_INCLUDE_DIR}
            ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(behavior_manager
        PUBLIC  robot_base
        PRIVATE ${RL_LIB} cnpy
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
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(control_sim2sim_runtime PRIVATE ${RL_LIB} transport_executor robot_base)
    install(TARGETS control_sim2sim_runtime RUNTIME DESTINATION bin)
endif()

# hmi_runtime（HMI 进程：仅 transport）
add_executable(hmi_runtime src/runtime/hmi_runtime.cpp)
target_include_directories(hmi_runtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hmi_runtime PRIVATE transport_executor robot_base)
install(TARGETS hmi_runtime RUNTIME DESTINATION bin)

# driver_runtime（仿真驱动：x86_64 only，链接 mujoco_sim）
if(BUILD_DRIVER_SIM)
    add_executable(driver_runtime src/runtime/driver_runtime.cpp)
    target_include_directories(driver_runtime PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${MUJOCO_SIM_INCLUDE_DIR}
    )
    target_link_libraries(driver_runtime PRIVATE ${MUJOCO_SIM_LIB} transport_executor robot_base)
    # staging/lib 包含 libmujoco.so（mujoco_sim 的传递依赖）
    target_link_directories(driver_runtime PRIVATE ${CMAKE_INSTALL_PREFIX}/lib)
    install(TARGETS driver_runtime RUNTIME DESTINATION bin)
endif()

# ============================================================================
# 示例 / 单模块测试
# ============================================================================

add_executable(test_robot_base example/robot_base/test_robot_base.cpp)
target_include_directories(test_robot_base PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_robot_base PRIVATE robot_base)
install(TARGETS test_robot_base RUNTIME DESTINATION bin)

add_executable(test_transport_executor example/transport_executor/test_transport_executor.cpp)
target_include_directories(test_transport_executor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_transport_executor PRIVATE transport_executor robot_base)
install(TARGETS test_transport_executor RUNTIME DESTINATION bin)

if(BUILD_RL)
    add_executable(test_behavior example/behavior_manager/test_behavior.cpp)
    target_include_directories(test_behavior PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${RL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(test_behavior PRIVATE behavior_manager transport_executor robot_base)
    install(TARGETS test_behavior RUNTIME DESTINATION bin)
endif()
