cmake_minimum_required(VERSION 2.8.3)
project(amcl)

find_package(catkin REQUIRED
        COMPONENTS
            rosbag
            roscpp
            tf
            dynamic_reconfigure
            nav_msgs
            std_srvs
        )

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    CATKIN_DEPENDS
        rosbag
        roscpp
        dynamic_reconfigure
        tf
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include/amcl include/amcl/map include/amcl/sensors include/amcl/pf)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(amcl_pf
                    src/amcl/pf/pf.c
                    src/amcl/pf/pf_kdtree.c
                    src/amcl/pf/pf_pdf.c
                    src/amcl/pf/pf_vector.c
                    src/amcl/pf/eig3.c
                    src/amcl/pf/pf_draw.c)

add_library(amcl_map
                    src/amcl/map/map.c
                    src/amcl/map/map_cspace.cpp
                    src/amcl/map/map_range.c
                    src/amcl/map/map_store.c
                    src/amcl/map/map_draw.c)

add_library(amcl_sensors
                    src/amcl/sensors/amcl_sensor.cpp
                    src/amcl/sensors/amcl_odom.cpp
                    src/amcl/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)


add_executable(amcl
                       src/amcl_node.cpp)
add_dependencies(amcl amcl_gencfg)

target_link_libraries(amcl
    amcl_sensors amcl_map amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)