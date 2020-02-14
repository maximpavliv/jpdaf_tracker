cmake_minimum_required(VERSION 2.8.12)
project(jpdaf_tracker_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
    Track.msg
    Tracks.msg
) #added by Max

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
