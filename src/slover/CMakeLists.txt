# - This module is one of the software framework, which manage xml files.
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

project(slover)

message(STATUS "Configuring sub_project: slover")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include
)

#param_manager
add_library(slover
    angle_slover.cpp
    armor_recorder.cpp
)

target_link_libraries(slover
    ${OpenCV_LIBRARIES}
)
