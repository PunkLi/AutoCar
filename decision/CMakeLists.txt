cmake_minimum_required(VERSION 2.8.3)
project(decision)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  geometry_msgs
  roscpp
  tf
  message_generation
  detect
)
find_package(OpenCV REQUIRED)

add_message_files(
    FILES
    shoot_info.msg
)

generate_messages(
   DEPENDENCIES
   move_base_msgs
   geometry_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decision_node src/decision_node.cpp)

target_link_libraries(decision_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)
