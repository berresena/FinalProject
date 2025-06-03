cmake_minimum_required(VERSION 3.0.2)
project(pcl_qt_viewer)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  nav_msgs
)
find_package(Qt5 REQUIRED COMPONENTS 
    Widgets Core Gui OpenGL Quick QuickWidgets
    Concurrent
)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters)
find_package(OpenGL REQUIRED COMPONENTS OpenGL GLX)  
find_package(tf2 REQUIRED) 
find_package(tf2_ros REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) 
set(CMAKE_AUTOUIC ON)  
set(OPENGL_LIBRARIES GL GLU)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})

set(MOC_HEADERS
  include/pcl_qt_viewer/pcl_qt_viewer.h
  include/pcl_qt_viewer/subscribe_pointcloud2message.h
  include/pcl_qt_viewer/pointcloudglwidget.h
  include/pcl_qt_viewer/coverage_planner.h
  include/pcl_qt_viewer/vfh_planner.h
  include/pcl_qt_viewer/navigation_manager.h
  include/pcl_qt_viewer/mapping_manager.h
)


qt5_wrap_cpp(MOC_FILES ${MOC_HEADERS})

qt5_wrap_ui(UI_FILES 
  ui/main_window.ui
)

add_executable(pcl_qt_viewer_node
  src/main.cpp
  src/pcl_qt_viewer_node.cpp
  src/subscribe_pointcloud2message.cpp
  src/pointcloudglwidget.cpp
  src/coverage_planner.cpp
  src/vfh_planner.cpp
  src/navigation_manager.cpp
  src/mapping_manager.cpp
  ${UI_FILES}
  ${MOC_FILES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS} 
  ${Qt5QuickWidgets_INCLUDE_DIRS} 
  ${OpenGL_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(
    /usr/include/x86_64-linux-gnu/qt5
    /usr/include/x86_64-linux-gnu/qt5/QtQuickWidgets
    /usr/include/x86_64-linux-gnu/qt5/QtQuick
)

target_link_libraries(pcl_qt_viewer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::QuickWidgets
  ${OPENGL_LIBRARIES}  
  GL
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)


target_include_directories(pcl_qt_viewer_node PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_conversions pcl_ros
)

install(TARGETS pcl_qt_viewer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)