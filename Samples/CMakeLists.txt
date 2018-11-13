# Example CMakeLists.txt
# FindKInectSDK2.cmake copy to CMake\share\cmake-3.5\Modules or same directory as this file

cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project( Kinect_v2_Tutorial )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Settings Kinect SDK v2
include_directories( ${KinectSDK2_INCLUDE_DIRS} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )

# 可按下面的格式添加自己的源文件
add_executable( camera_calibration camera_calibration.cpp )
add_custom_command( TARGET camera_calibration POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "camera_calibration" )
target_link_libraries( camera_calibration ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES} )

# 可按下面的格式添加自己的源文件
add_executable( fast_capture_100_frames fast_capture_100_frames.cpp )
add_custom_command( TARGET fast_capture_100_frames POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "fast_capture_100_frames" )
target_link_libraries( fast_capture_100_frames ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES} )

# 可按下面的格式添加自己的源文件
add_executable( generate_pointcloud_from_depthimage generate_pointcloud_from_depthimage.cpp )
add_custom_command( TARGET generate_pointcloud_from_depthimage POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "generate_pointcloud_from_depthimage" )
target_link_libraries( generate_pointcloud_from_depthimage ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES} )
