# - This Software for Robomaster 2018
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

cmake_minimum_required(VERSION 3.1)

# Define project name
project(autocar)

message(STATUS "This is Robomaster Vision program:")

set(CMAKE_BUILE_TYPE RELEASE) 

# using C++11
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

# Find OpenCV
find_package(OpenCV REQUIRED)

message(STATUS "====================================================")
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(CURRENT_PROJET_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CURRENT_PROJET_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CURRENT_PROJET_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CURRENT_PROJET_DIR}/build/lib)

# Add headers location to include paths
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/inlcude
)
message(STATUS "====================================================")
message(STATUS "my include path: ${CMAKE_CURRENT_SOURCE_DIR}/inlcude")
message(STATUS "my library path: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "====================================================")

# find_package( OpenMP REQUIRED)  
# if(OPENMP_FOUND)  
# message("OPENMP FOUND")  
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")  
# endif()  

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/common)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/detect_factory)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/slover)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/driver)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/utility)