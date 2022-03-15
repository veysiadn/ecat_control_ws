cmake_minimum_required(VERSION 3.5)
project(input_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# **********************************************
# CKim - Dependencies (headers and lib files) should be 
#        added here. It can be conveniently set by using 
#        CMake's find_package macro or include_directories
#        and link_directories
# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# CKim - Basic dependency for using rclcpp, standard message, and joystick
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ecat_msgs REQUIRED)
#find_package(dynamixel_sdk REQUIRED)
# find_package(sdl2_vendor REQUIRED)
# find_package(sdl2_custom REQUIRED)

# CKim - Dependency for node generated file
#find_package(input_pkg REQUIRED)
include_directories("include/input_pkg/")
# link_directories("${CMAKE_BINARY_DIR}/my_lib/install/usr/local/lib")
# **********************************************

# **********************************************
# CKim - Custom messages
# Add this package for process msg and srv files
#find_package(rosidl_default_generators REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/HapticCmd.msg"
#  "srv/AddThreeInts.srv"
#)
# **********************************************

# **********************************************
# CKim - Files to be compiled and built
add_executable(hapticNode src/hapticNode.cpp)
ament_target_dependencies(hapticNode rclcpp std_msgs sensor_msgs ecat_msgs)
# **********************************************

# **********************************************
# CKim - Installation of built node
install(TARGETS
  hapticNode
  DESTINATION lib/${PROJECT_NAME})
# **********************************************
target_include_directories(hapticNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
