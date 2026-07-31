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
  if(NOT EXISTS "${ROBOT_MOTION_SDK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "ROBOT_MOTION_SDK_DIR must contain the SDK CMakeLists.txt: ${ROBOT_MOTION_SDK_DIR}")
  endif()
  message(STATUS
    "SDK inclusion enabled; no SDK backend is instantiated by this package")
  add_subdirectory("${ROBOT_MOTION_SDK_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/robot_motion_sdk" EXCLUDE_FROM_ALL)
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

add_executable(catalog_only_node src/catalog_only_node.cpp)
target_compile_features(catalog_only_node PUBLIC cxx_std_17)
target_link_libraries(catalog_only_node catalog_only_core)
ament_target_dependencies(catalog_only_node ament_index_cpp rclcpp std_msgs)

install(TARGETS catalog_only_core catalog_only_node
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
      TEST_ALIAS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/motion_aliases.yaml")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(catalog_only_core)
ament_package()
