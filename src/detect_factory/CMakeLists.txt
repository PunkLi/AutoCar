# - This module support function for taeget detect, such as Robomaster armor.
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

project(detect_factory)

message(STATUS "Configuring Sub_project: detect_factory")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include
)

#detect_factory
add_library(detect_factory
    armor_info.cpp
    armor_detect.cpp
)

target_link_libraries(detect_factory
    ${OpenCV_LIBRARIES}
)