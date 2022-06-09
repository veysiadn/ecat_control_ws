cmake_minimum_required(VERSION 3.5)
project(safety_pkg)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(lifecycle_msgs REQUIRED)
## This is for ecat_pkg
find_package(ecat_msgs REQUIRED)
## This is for joystick
find_package(sensor_msgs REQUIRED)
# This is for lifecycle node manager
find_package(std_msgs REQUIRED)

add_executable(safety_node src/safety_node.cpp)

target_include_directories(safety_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/safety_pkg>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(safety_node
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${ecat_msgs_LIBRARIES}
)

ament_target_dependencies(safety_node  rclcpp rclcpp_lifecycle ecat_msgs sensor_msgs lifecycle_msgs std_msgs)

install(TARGETS safety_node
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
