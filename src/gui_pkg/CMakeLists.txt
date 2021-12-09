##############################################################################
# PROJECT
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(gui_pkg)

##############################################################################
# CMAKE | As found in ROS2 dashing distribution
##############################################################################

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

##############################################################################
# DEPENDENCIES | Add package dependencies here
##############################################################################

# Find QT dependencies
    # For QWidgets application add --> Core Gui Widgets
    # For QML application add --> Core Gui Quick
#Example for QWidget
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED) 

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ecat_msgs REQUIRED)
## This is for joystick/Controller_node
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
## This package required for opencv library
find_package(cv_bridge REQUIRED)
  
find_package(rttest)
find_package(tlsf_cpp)


##############################################################################
# QT5 - FILES PATH | AUTO SET  
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB UIS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ui/*.ui
  )
file(GLOB RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  resources/*.qrc
  )
file(GLOB_RECURSE MOC_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  include/${PROJECT_NAME}/*.h *.hpp
  )
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
  src/*.cpp
  )

QT5_ADD_RESOURCES(QT_RESOURCES ${RESOURCES})
QT5_WRAP_UI(QT_UIS ${UIS})
QT5_WRAP_CPP(QT_HEADERS ${MOC_HEADERS})
  

##############################################################################
# COMPILE | Add ament_target dependencies here | Add target link libraries
##############################################################################
set(node_name "gui_node")

## Add executables
add_executable(${node_name}
  ${SOURCES} 
  ${QT_RESOURCES}
  ${QT_UIS}
  ${QT_HEADERS} 
  )
include_directories(
    ${cv_bridge_INCLUDE_DIRS}
)

ament_target_dependencies(${node_name}
  rclcpp
  sensor_msgs
  ecat_msgs
  std_msgs
  cv_bridge
  tlsf_cpp
  rttest
  )

#In case of QML, replace Widgets with Quick
target_link_libraries(${node_name}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  ${cv_bridge_LIBRARIES}
  )

## Install executable
install(TARGETS ${node_name} DESTINATION lib/${PROJECT_NAME})

ament_package()
