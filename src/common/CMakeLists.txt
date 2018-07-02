project(common)

message(STATUS "Configuring sub_project: common")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
    ${CURRENT_PROJET_DIR}/include
)

add_executable(autocar 
    main.cpp
    ImageConsProd.cpp
    common_serial.cpp
)

target_include_directories(autocar
  PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CURRENT_PROJET_DIR}/include
)

link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

target_link_libraries(autocar
    ${OpenCV_LIBRARIES}
    detect_factory
    slover 
    driver
    util 
)