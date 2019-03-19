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

# Find Packages
find_package(PCL 1.8 REQUIRED)

# Additional Include Directories
# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )

# Preprocessor Definitions
# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )
add_definitions( -DPCL_NO_PRECOMPILE )

# Additional Library Directories
# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS} )

# 可按下面的格式添加自己的源文件
add_executable( sdk_pointcloud sdk_pointcloud.cpp)
add_custom_command( TARGET sdk_pointcloud POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "sdk_pointcloud" )
target_link_libraries( sdk_pointcloud ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable( kinect_continuous_acquisition kinect_continuous_acquisition.cpp)
add_custom_command( TARGET kinect_continuous_acquisition POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "kinect_continuous_acquisition" )
target_link_libraries( kinect_continuous_acquisition ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
