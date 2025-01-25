cmake_minimum_required(VERSION 3.10)
project(realsense_interface)

# Include pkg-config module
include(FindPkgConfig)

# Find required packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


# Use pkg-config to find ZeroMQ
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(ZSTD REQUIRED libzstd)


# Add executable
add_executable(${PROJECT_NAME}_send
    send_frames.cpp
    include/config.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}_send PRIVATE
    ${realsense2_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${LZ4_INCLUDE_DIRS}
    ${ZSTD_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_send PRIVATE
    ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
    ${ZMQ_LIBRARIES}
    ${LZ4_LIBRARIES}
    ${ZSTD_LIBRARIES}
    yaml-cpp
)


configure_file(${CMAKE_SOURCE_DIR}/config.yaml ${CMAKE_BINARY_DIR}/config.yaml COPYONLY)