cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            geometry_msgs
            laser_geometry
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pcl_conversions
            pcl_ros
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf
            visualization_msgs
            voxel_grid
        )

find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# messages
add_message_files(
    DIRECTORY msg
    FILES
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        map_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/Costmap2D.cfg
  cfg/ObstaclePlugin.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
  cfg/VoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    LIBRARIES costmap_2d layers
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pcl_ros
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
        voxel_grid
    DEPENDS
        PCL
        EIGEN3
        Boost
)

add_library(costmap_2d
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
)
add_dependencies(costmap_2d geometry_msgs_generate_messages_cpp)
target_link_libraries(costmap_2d
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
  plugins/voxel_layer.cpp
  src/observation_buffer.cpp
)
target_link_libraries(layers
  costmap_2d
)

add_dependencies(costmap_2d costmap_2d_gencfg)
add_dependencies(layers costmap_2d_gencfg)
add_dependencies(costmap_2d costmap_2d_generate_messages_cpp)

add_executable(costmap_2d_markers src/costmap_2d_markers.cpp)
add_dependencies(costmap_2d_markers visualization_msgs_generate_messages_cpp)
target_link_libraries(costmap_2d_markers
    costmap_2d
    )

add_executable(costmap_2d_cloud src/costmap_2d_cloud.cpp)
add_dependencies(costmap_2d_cloud sensor_msgs_generate_messages_cpp)
target_link_libraries(costmap_2d_cloud
    costmap_2d
    )

add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node
    costmap_2d
    )