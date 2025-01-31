cmake_minimum_required(VERSION 3.8)
project(realsense_processor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include pkg-config module
include(FindPkgConfig)

# Use pkg-config to find ZeroMQ
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(ZSTD REQUIRED libzstd)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Find threads
find_package(Threads REQUIRED)

# Create executable
add_executable(realsense_processor 
    src/send_frames.cpp
)

# Add include directories
target_include_directories(realsense_processor PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${ZSTD_INCLUDE_DIRS}
    ${LZ4_INCLUDE_DIRS}
)

# Specify dependencies
ament_target_dependencies(realsense_processor
    rclcpp
    sensor_msgs
)

# Link libraries
target_link_libraries(realsense_processor
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${ZMQ_LIBRARIES}
    ${ZSTD_LIBRARIES}
    ${LZ4_LIBRARIES}
    Threads::Threads
)

# Install targets
install(TARGETS
    realsense_processor
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
    DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()