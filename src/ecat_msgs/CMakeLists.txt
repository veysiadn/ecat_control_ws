cmake_minimum_required(VERSION 3.5)
project(ecat_msgs)

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


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
## Add this part of custom interfaces 
find_package(rosidl_default_generators REQUIRED)
## Define directories
set(msg_files
  "msg/DataReceived.msg"
  "msg/DataSent.msg"
  "msg/GuiButtonData.msg"
  "msg/HapticCmd.msg"
)
## generate interface from your msg files.
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces)

#*************************************************************

ament_package()
