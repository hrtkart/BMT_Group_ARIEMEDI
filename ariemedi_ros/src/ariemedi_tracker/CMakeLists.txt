cmake_minimum_required(VERSION 3.8)
project(ariemedi_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ARMD_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/armd_sdk")
set(ARMD_INC_DIR "${ARMD_SDK_DIR}/include")
set(ARMD_LIB_DIR "${ARMD_SDK_DIR}/lib/Release")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerPosition.msg"
  "msg/MarkerPlane.msg"
  "msg/Transformation.msg"
  "msg/ToolTrackingData.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${ARMD_INC_DIR}
)

find_library(ARMD_TRACKING_LIB ARMDstereotracking PATHS ${ARMD_LIB_DIR} NO_DEFAULT_PATH REQUIRED)
find_library(ARMD_RECONSTRUCTION_LIB ARMDstereoreconstruction PATHS ${ARMD_LIB_DIR} NO_DEFAULT_PATH REQUIRED)

add_executable(tracker_node src/tracker_node.cpp)
target_include_directories(tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tracker_node PUBLIC c_std_99 cxx_std_17)

rosidl_target_interfaces(tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(tracker_node
  ${ARMD_TRACKING_LIB}
  ${ARMD_RECONSTRUCTION_LIB}
)

ament_target_dependencies(tracker_node
  "rclcpp"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "std_msgs"
)

install(TARGETS tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY armd_sdk/tool/
  DESTINATION share/${PROJECT_NAME}/tool
)
install(DIRECTORY armd_sdk/lib/Release/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(PROGRAMS
  scripts/marker_test_plot.py
  scripts/marker_relative_rotation_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
