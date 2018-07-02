# - This Software for Robomaster 2018
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

cmake_minimum_required(VERSION 3.1)

project(armor_svm)

set(CMAKE_BUILE_TYPE RELEASE) 

# using C++11
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
)

add_executable(armor_svm
    main.cpp
    train.cpp
)

target_link_libraries(armor_svm
    ${OpenCV_LIBRARIES}
)


