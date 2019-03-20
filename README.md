# Kinect-v2-Tutorial

- Kinect采集彩色图，深度图，点云
- 彩色图深度图对齐

# 编程环境配置

**依赖项：** Kinect SDK提供了获取传感器数据和相机信息的API；需要用到Opencv的图像数据结构、显示、存储等；PCL中用到点云数据结构，点云存取等；
- Kinect SDK 2.0
- Opencv
- PCL

两种配置方法：
1. VS添加Kinect SDK属性表的方式（不推荐使用，对每个依赖库手动添加属性表比较繁琐）；
2. 使用Cmake  **推荐方式**

由于CMAKE配置方式配置过程简单，项目构建与调整更加灵活，只介绍CMake配置方法；

## 依赖项安装方法

### visual studio
  1. visual studio版本需要等于或高于目标平台。即，你计划用vc14 x64的编译配置，那么vs版本不能低于2015版；
  2. opencv、pcl库的编译选项一定需要和目标编译选项一致。建议下载预编译版本的OPENCV与PCL，注意下载时，选择与目标平台相同的预编译版本；如`PCL-1.8.1-AllInOne-msvc2017-win64.exe`与`opencv-3.4.0-vc14_vc15.exe`均为`vc15 x64`的配置，在用CMAKE编译自己代码时，平台选择应相同。
  ![平台一致性](../Assets/Opencv编译器版本选择.PNG)
### [cmake](https://cmake.org/)
  - 添加Kinect sdk2的cmake配置文件。将Samples目录下的`FindKInectSDK2.cmake`文件复制到Cmake安装目录下的`share\cmake-3.10\Modules\`文件夹中。例如：`D:\CMAKE\share\cmake-3.10\Modules`。

### [Opencv](https://opencv.org/releases.html)
  - 可自己编译或下载预编译版本
  - 注意平台一致性
  - Opencv环境变量
    1. 在系统变量中添加`OpenCV_DIR`，值为build目录下包含`OpenCVConfig.cmake`文件的文件夹路径。例如:`D:\opencv3.2\build_vc15_x64\install`;
    2. 在环境变量`Path`中添加opencv `bin`目录，例如：`D:\opencv3.2\build_vc15_x64\install\bin`
### [PCL](http://unanancyowen.com/en/)
  可自行编译或下载预编译版本。


## cmake工程配置文件CMakeLists.txt写法
```cmake
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
add_executable( camera_calibration camera_calibration.cpp )
add_custom_command( TARGET camera_calibration POST_BUILD ${KinectSDK2_COMMANDS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "camera_calibration" )
target_link_libraries( camera_calibration ${OpenCV_LIBS} ${KinectSDK2_LIBRARIES} )
```
