cmake_minimum_required(VERSION 3.5)

project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
  
find_package(rttest)
find_package(tlsf_cpp)

# for now, this program only exists for Linux hosts, since it calls into
# the underlying joystick driver provided by the Linux kernel.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  # Look for <linux/joystick.h>
  include(CheckIncludeFiles)
  check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

  if(HAVE_LINUX_JOYSTICK_H)
    #include_directories(msg/cpp ${catkin_INCLUDE_DIRS})
    add_executable(joy_node src/joy_node_linux.cpp)
    ament_target_dependencies(joy_node
      "rclcpp"
      "sensor_msgs"
      "tlsf_cpp"
      "rttest")
    install(TARGETS joy_node DESTINATION lib/${PROJECT_NAME})
  else()
    message(FATAL_ERROR "no <linux/joystick.h>; can't build joy node")
  endif()

endif()

ament_package()
