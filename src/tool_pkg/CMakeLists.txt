cmake_minimum_required(VERSION 3.5)
project(tool_pkg)

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

set(dynamixel_include /usr/local/include/dynamixel_sdk/)
set(dynamixel_lib  /usr/local/lib/libdxl_x64_cpp.so)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(dynamixel_sdk REQUIRED) #CKim - Dynamixel Library
## This is for joystick
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include/tool_pkg)

# Build
add_executable(surgicalToolNode 
               src/DxlMaster.cpp
               src/surgicalToolNode.cpp)
# target_include_directories(surgicalTool PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# CKim - Added dependency
ament_target_dependencies(surgicalToolNode
#  dynamixel_sdk_custom_interfaces
#  dynamixel_sdk
  sensor_msgs 
  rclcpp
)
target_include_directories(surgicalToolNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${dynamixel_include})

target_link_libraries(surgicalToolNode
${dynamixel_lib}
)
install(TARGETS surgicalToolNode
  DESTINATION lib/${PROJECT_NAME})

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