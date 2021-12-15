cmake_minimum_required(VERSION 3.5)
project(ecat_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
## Compile options for IgH libary and several Linux libraries (e.g lpthread)
add_compile_options(-g -w -Wall -Wextra -Wpedantic -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -lpthread -lrt -Wl,--rpath -Wl,/opt/etherlab/lib)

## Defining paths and libraries to include in the next section.
set(etherlab_include /opt/etherlab/include)
set(etherlab_lib /opt/etherlab/lib/libethercat.so.1.1.0)
set(ecat_node_include ${CMAKE_CURRENT_SOURCE_DIR}/include/ecat_pkg)
set(node_name "ecat_node")

## Finding packages that'll be required for compilation.
## Don't forget to add packages if you use it in your code, otherwise you'll get build errors.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rttest)
find_package(tlsf_cpp)
## This is for custom msg files.
find_package(ecat_msgs REQUIRED)
## This is for joystick
find_package(sensor_msgs REQUIRED)
# This is for lifecycle node manager
find_package(std_msgs REQUIRED)
## Output executable name and requied cpp files for executable
add_executable(ecat_node src/main.cpp
                         src/ecat_node.cpp
                         src/ecat_slave.cpp
                         src/ecat_lifecycle.cpp
                         src/timing.cpp)

## Specifying include directories for ecat_node specifically by using definitions above.
## target include directories adds include directory for specific target executable.
## you can add directory in this way or you can use 
# include_directories(x/y) for general include for whole project..
target_include_directories(ecat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ecat_pkg>
  $<INSTALL_INTERFACE:include>
  ${etherlab_include})

## Specifying libraries by using definitions above.
target_link_libraries(ecat_node
${etherlab_lib}
)
# Add include directories
# include_directories(
#   ${etherlab_include}
#   ${ecat_node_include}
# )
include_directories(
  include
  ${etherlab_include}
  ${ecat_node_include}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  )

add_executable(lifecycle_node_manager src/lifecycle_node_manager.cpp)

target_link_libraries(lifecycle_node_manager
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${ecat_msgs_LIBRARIES}
)
## Don't forget to add dependencies to your build file, 
## Use find_package(x) then add dependecy for x. 
ament_target_dependencies(${node_name}  rclcpp rclcpp_lifecycle ecat_msgs sensor_msgs lifecycle_msgs std_msgs tlsf_cpp)
ament_target_dependencies(lifecycle_node_manager  rclcpp rclcpp_lifecycle ecat_msgs sensor_msgs lifecycle_msgs std_msgs)

install(TARGETS 
  ecat_node
  lifecycle_node_manager
  DESTINATION lib/${PROJECT_NAME})


ament_package()