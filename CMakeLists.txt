cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_filter)
##set ( CMAKE_CXX_FLAGS "-ggdb -g")
add_compile_options(-std=c++11 )
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    pcl_ros
    pcl_conversions
    geometry_msgs
    nav_msgs
    filters
    cv_bridge
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
      roscpp
      sensor_msgs
      std_msgs
      message_generation
      pcl_ros
      pcl_conversions
      geometry_msgs
      nav_msgs
      filters
      DEPENDS EIGEN3
)

###########
## Build ##
###########
  include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(filter src/ptc.cpp ${SRCS})
target_link_libraries(filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
add_dependencies(filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
