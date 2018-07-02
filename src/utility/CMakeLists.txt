# - This is utility tool fo software.
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

project(utility)

message(STATUS "Configuring sub_project: utility")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include
)

add_library(util
    debug_utility.cpp
    video_recoder.cpp
)

target_link_libraries(util
    ${OpenCV_LIBRARIES}
)

target_include_directories(util
  PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)