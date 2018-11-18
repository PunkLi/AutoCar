# - This module provide hardware support for program, such as usb_camera.
#
# Copyright (c) 2018 Robomaster, Xidian University
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

project(driver)

message(STATUS "Configuring Sub_project: driver")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include
)

#driver
add_library(driver 
    camera_driver.cpp
    LinuxSerial.cpp
    RMVideoCapture.cpp
)

target_link_libraries(driver
    ${OpenCV_LIBRARIES}
)