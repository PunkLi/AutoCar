cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  roscpp
  sensor_msgs 
  std_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_node
  src/camera_node.cpp
  src/uvc_camera.cpp
)

target_link_libraries(camera_node
  ${catkin_LIBRARIES}    
  ${OpenCV_LIBS}
)