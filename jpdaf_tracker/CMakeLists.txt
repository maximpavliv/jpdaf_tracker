cmake_minimum_required(VERSION 2.8.3)
project(jpdaf_tracker)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
#  message_filters
#  message_generation
  eigen_conversions
#  diagnostic_updater
  cv_bridge
  darknet_ros_msgs
  image_transport
  jpdaf_tracker_msgs
)

find_package(cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV 3.2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge sensor_msgs darknet_ros_msgs image_transport jpdaf_tracker_msgs 
  DEPENDS EIGEN3 OpenCV
)

# include boost
find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
  include/jpdaf_tracker
)

# build shared library
#add_library(JPDAF_Tracker
#  src/AttitudeESKF.cpp
#  src/*.cpp
#)

set(RELEVANT_LIBRARIES
#  JPDAF_Tracker
  ${catkin_LIBRARIES}
)

# rigorous error checking
add_definitions("-Wall -Werror -DATTITUDE_ESKF_BUILD_MAG -O3 -std=c++11")

# build main filter
add_executable(${PROJECT_NAME}
  src/jpdaf_main.cpp
  src/node.cpp
  src/tracker_param.cpp
  src/hungarian_alg.cpp
  src/track.cpp
  src/kalman.cpp
)
target_link_libraries(${PROJECT_NAME} ${RELEVANT_LIBRARIES} ${OpenCV_LIBRARIES})

# ensures messages are generated before hand
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
