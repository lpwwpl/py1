cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE "Debug")  # 定义编译类型
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb") # 定义Debug编译参数
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") # 定义Release编译参数
project(OpenCV_Hand_Eye_Cal)


find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${realsense2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} /usr/local/include)

message(STATUS "realsense2 include: ${realsense2_INCLUDE_DIR}")
message(STATUS "realsense2 lib: ${realsense2_LIBRARY}")


add_executable(OpenCV_Hand_Eye_Cal OpenCV_Hand_Eye_Cal.cpp)
target_link_libraries(OpenCV_Hand_Eye_Cal ${realsense2_LIBRARY} ${OpenCV_LIBS})
