cmake_minimum_required(VERSION 3.8)
project(irc_step_motion_executor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(IRC_STEP_ENABLE_ROBOT_MOTION_SDK
  "Enable inclusion of an externally supplied RobotMotionPlayer SDK" OFF)
set(ROBOT_MOTION_SDK_DIR "" CACHE PATH
  "External RobotMotionPlayer SDK source directory (empty by default)")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
pkg_check_modules(JSON_C REQUIRED IMPORTED_TARGET json-c)

if(IRC_STEP_ENABLE_ROBOT_MOTION_SDK)
  if(NOT ROBOT_MOTION_SDK_DIR)
    message(FATAL_ERROR
      "IRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON requires ROBOT_MOTION_SDK_DIR")
  endif()
  if(NOT IS_DIRECTORY "${ROBOT_MOTION_SDK_DIR}")
    message(FATAL_ERROR
      "ROBOT_MOTION_SDK_DIR is not an existing directory: ${ROBOT_MOTION_SDK_DIR}")
  endif()
  foreach(required_sdk_file
      CMakeLists.txt
      robot_motion_player.hpp
      robot_motion_player.cpp)
    if(NOT EXISTS "${ROBOT_MOTION_SDK_DIR}/${required_sdk_file}")
      message(FATAL_ERROR
        "ROBOT_MOTION_SDK_DIR is missing required file: ${required_sdk_file}")
    endif()
  endforeach()
  message(STATUS
    "SDK inclusion enabled; no SDK backend is instantiated by this package")
  add_subdirectory("${ROBOT_MOTION_SDK_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motion_sdk" EXCLUDE_FROM_ALL)
  if(NOT TARGET robot_control)
    message(FATAL_ERROR
      "External SDK must create a CMake target named robot_control")
  endif()

  add_library(irc_step_motion_sdk_compile_probe SHARED
    src/sdk_compile_probe.cpp)
  target_compile_features(irc_step_motion_sdk_compile_probe PUBLIC cxx_std_17)
  target_include_directories(irc_step_motion_sdk_compile_probe PRIVATE
    "${ROBOT_MOTION_SDK_DIR}")
  target_link_libraries(irc_step_motion_sdk_compile_probe PRIVATE robot_control)
else()
  message(STATUS
    "Building catalog-only mode (RobotMotionPlayer SDK is not included or linked)")
endif()

add_library(catalog_only_core src/catalog_only_core.cpp)
target_compile_features(catalog_only_core PUBLIC cxx_std_17)
target_include_directories(catalog_only_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(catalog_only_core PkgConfig::JSON_C yaml-cpp)

add_library(sdk_executor_core src/sdk_executor_core.cpp)
target_compile_features(sdk_executor_core PUBLIC cxx_std_17)
target_include_directories(sdk_executor_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(sdk_executor_core PUBLIC catalog_only_core)

add_executable(catalog_only_node src/catalog_only_node.cpp)
target_compile_features(catalog_only_node PUBLIC cxx_std_17)
target_link_libraries(catalog_only_node catalog_only_core)
ament_target_dependencies(catalog_only_node ament_index_cpp rclcpp std_msgs)

install(TARGETS catalog_only_core sdk_executor_core catalog_only_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_catalog_only_core test/test_catalog_only_core.cpp)
  if(TARGET test_catalog_only_core)
    target_link_libraries(test_catalog_only_core catalog_only_core)
    target_compile_definitions(test_catalog_only_core PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml"
      TEST_CONTRACT_VECTORS="${CMAKE_CURRENT_SOURCE_DIR}/test/legacy_catalog_contract_vectors.json")
  endif()
  ament_add_gtest(test_sdk_executor_core test/test_sdk_executor_core.cpp)
  if(TARGET test_sdk_executor_core)
    target_include_directories(test_sdk_executor_core PRIVATE test)
    target_link_libraries(test_sdk_executor_core sdk_executor_core)
    target_compile_definitions(test_sdk_executor_core PRIVATE
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()

  function(add_sdk_cmake_guard_test test_name sdk_dir expect_success expected_error)
    add_test(
      NAME ${test_name}
      COMMAND ${CMAKE_COMMAND}
        "-DPACKAGE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/sdk_cmake_tests/${test_name}"
        "-DSDK_DIR=${sdk_dir}"
        "-DEXPECT_SUCCESS=${expect_success}"
        "-DEXPECTED_ERROR=${expected_error}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/test/run_sdk_cmake_case.cmake")
  endfunction()

  add_sdk_cmake_guard_test(
    sdk_cmake_empty_path "" FALSE
    "requires ROBOT_MOTION_SDK_DIR")
  add_sdk_cmake_guard_test(
    sdk_cmake_nonexistent_path
    "${CMAKE_CURRENT_BINARY_DIR}/sdk_path_that_does_not_exist"
    FALSE "is not an existing directory")
  add_sdk_cmake_guard_test(
    sdk_cmake_missing_required_files
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/missing_files_sdk"
    FALSE "missing required file: robot_motion_player.hpp")
  add_sdk_cmake_guard_test(
    sdk_cmake_missing_robot_control_target
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/missing_target_sdk"
    FALSE "must create a CMake target named robot_control")
  add_sdk_cmake_guard_test(
    sdk_cmake_fake_sdk_build
    "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/fake_robot_motion_sdk"
    TRUE "")
  add_test(
    NAME sdk_cmake_disabled_build_has_no_hardware_symbols
    COMMAND ${CMAKE_COMMAND}
      "-DPACKAGE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/sdk_cmake_tests/sdk_disabled"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/run_sdk_disabled_case.cmake")
endif()

ament_export_include_directories(include)
ament_export_libraries(catalog_only_core sdk_executor_core)
ament_package()
