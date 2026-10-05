cmake_minimum_required(VERSION 3.16)
project(irc_humanoid_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# 설치 위치 예: /usr/local/include/dynamixel_sdk/dynamixel_sdk.h
find_path(DYNAMIXEL_SDK_INCLUDE_DIR dynamixel_sdk/dynamixel_sdk.h)
find_library(DYNAMIXEL_SDK_LIBRARY NAMES dxl_x64_cpp dynamixel_sdk)

add_library(motion_core
    motion_pattern.cpp
    motion_callback.cpp
)
target_include_directories(motion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(motion_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_motion_pattern test_motion_pattern.cpp)
target_link_libraries(test_motion_pattern PRIVATE motion_core)
target_compile_definitions(test_motion_pattern PRIVATE
    TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_executable(test_motion_callback test_motion_callback.cpp)
target_link_libraries(test_motion_callback PRIVATE motion_core)
target_compile_options(test_motion_callback PRIVATE -UNDEBUG)
target_compile_options(test_motion_pattern PRIVATE -UNDEBUG)

add_library(motion_player_core
    robot_motion_player.cpp
)
target_include_directories(motion_player_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(motion_player_core PUBLIC IRC_STEP_GUI_ALIGNMENT_VERSION=20261006)
target_link_libraries(motion_player_core PUBLIC motion_core Threads::Threads)
target_compile_options(motion_player_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME motion_pattern_test COMMAND test_motion_pattern)
add_test(NAME motion_callback_test COMMAND test_motion_callback)

add_executable(gui_trace_probe gui_trace_probe.cpp)
target_link_libraries(gui_trace_probe PRIVATE motion_core)

if(DYNAMIXEL_SDK_INCLUDE_DIR AND DYNAMIXEL_SDK_LIBRARY)
    add_library(robot_control
        robot_motion_player_dynamixel.cpp
        dynamixel_motion_hardware.cpp
        step_dynamixel.cpp
        dynamixel_controller.cpp
    )
    target_include_directories(robot_control PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${DYNAMIXEL_SDK_INCLUDE_DIR}
    )
    target_link_libraries(robot_control PUBLIC
        motion_player_core
        Eigen3::Eigen
        Threads::Threads
        ${DYNAMIXEL_SDK_LIBRARY}
    )
    target_compile_options(robot_control PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(irc_robot main.cpp)
    target_link_libraries(irc_robot PRIVATE robot_control)
    target_compile_options(irc_robot PRIVATE -Wall -Wextra -Wpedantic)
else()
    message(WARNING "DYNAMIXEL SDK not found: irc_robot target will not be built")
endif()

add_executable(gui_execution_probe gui_execution_probe.cpp)
target_link_libraries(gui_execution_probe PRIVATE motion_player_core Threads::Threads)
add_executable(recorded_execution_probe recorded_execution_probe.cpp)
target_link_libraries(recorded_execution_probe PRIVATE motion_player_core Threads::Threads)
add_executable(test_gui_alignment test_gui_alignment.cpp)
target_link_libraries(test_gui_alignment PRIVATE motion_player_core)
target_compile_options(test_gui_alignment PRIVATE -UNDEBUG)
add_test(NAME gui_alignment_contract COMMAND test_gui_alignment ${CMAKE_CURRENT_BINARY_DIR})
