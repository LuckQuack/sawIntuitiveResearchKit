cmake_minimum_required (VERSION 3.16)
if (POLICY CMP0144)
  cmake_policy (SET CMP0144 NEW)
endif ()
project (dvrk_hrsv_widget VERSION 2.4.0)

# first test for ROS1
set (ROS1_depends roscpp std_msgs diagnostic_msgs sensor_msgs)

find_package (catkin QUIET
  COMPONENTS
  ${ROS1_depends})

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  add_definitions (-DROS1)
  catkin_package (
    CATKIN_DEPENDS ${ROS1_depends})
  include_directories (${catkin_INCLUDE_DIRS})
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    set (ROS_VERSION 2)
    add_definitions (-DROS2)
    set (ROS2_depends rclcpp std_msgs diagnostic_msgs sensor_msgs)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find Qt without cisst dependency - prefer Qt6, fall back to Qt5
find_package (Qt6 QUIET COMPONENTS Widgets Core)
if (Qt6_FOUND)
  set (QT_VERSION 6)
else ()
  find_package (Qt5 REQUIRED COMPONENTS Widgets Core)
  set (QT_VERSION 5)
endif ()

set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories (
  ${catkin_INCLUDE_DIRS}
  ${dvrk_hrsv_widget_SOURCE_DIR}/include
  ${dvrk_hrsv_widget_BINARY_DIR}/include
)

add_executable (hrsv_widget
  include/hrsv_widget.h
  src/hrsv_widget.cpp
  src/main.cpp)

target_link_libraries (
  hrsv_widget
  Qt${QT_VERSION}::Core
  Qt${QT_VERSION}::Widgets
  ${catkin_LIBRARIES})


install (TARGETS hrsv_widget
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

if (ROS_VERSION EQUAL 2)
  ament_target_dependencies (
    hrsv_widget
    ${ROS2_depends}
    )
  ament_package ()
endif ()
