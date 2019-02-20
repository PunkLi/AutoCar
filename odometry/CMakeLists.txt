cmake_minimum_required(VERSION 2.8.3)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_msgs
  serial
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  
)

add_executable(vel2odom_node src/vel2odom.cpp src/vel2odom_node.cpp)
target_link_libraries(vel2odom_node ${catkin_LIBRARIES})

add_executable(tf_node src/tf_tree.cpp

)

target_link_libraries(tf_node ${catkin_LIBRARIES})