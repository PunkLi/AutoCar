cmake_minimum_required(VERSION 2.8.3)
project(serial)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
    geometry_msgs
)

add_message_files(
    FILES
    car_info.msg
)

generate_messages(DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
)

catkin_package( CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_read_node  src/serial_read_node.cpp)
add_executable(serial_write_node src/serial_write_node.cpp)

target_link_libraries(serial_read_node ${catkin_LIBRARIES})
target_link_libraries(serial_write_node ${catkin_LIBRARIES})
