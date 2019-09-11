cmake_minimum_required(VERSION 3.5)
project(gps_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############
## Ament ##
############

set(DEPS
#  message_filters
  nav_msgs
  sensor_msgs
  std_msgs
  rclcpp
  rclpy
)

set(MSG_DEPS
  "std_msgs"
)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies(REQUIRED ${DEPS})

#######################################
## Declare ROS messages and services ##
#######################################

set(MSG_FILES
    msg/GPSStatus.msg
    msg/GPSFix.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

###################################################
## Declare things to be passed to other projects ##
###################################################

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
#  ${message_filters_INCLUDE_DIRS}
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

###########
## Build ##
###########

#add_executable(utm_odometry_node src/utm_odometry_node.cpp)
#ament_target_dependencies(utm_odometry_node
#  ${DEPS}
#)
#rosidl_target_interfaces(utm_odometry_node
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)

#############
## Install ##
#############

#install(TARGETS utm_odometry_node
#  DESTINATION lib/${PROJECT_NAME}
#)

install(
  DIRECTORY include/
  DESTINATION include
)

#ament_python_install_module(
#  nodes/fix_translator
#)

INSTALL(
  DIRECTORY nodes/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
