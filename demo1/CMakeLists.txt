cmake_minimum_required (VERSION 2.8)

project(TestIndem)


#generate dll
add_executable(${PROJECT_NAME} Indem.cpp)
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 
  FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS}) 
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBRARIES} 
pthread




/home/fang/workspace/indemind/SDK-Linux-master/lib/libindem.a
z
dl
tbb 
cholmod
boost_system boost_filesystem
stdc++fs
slam_imp
g3logger
DBoW2
DLib
protobuf
protobuf-lite
nanomsg
slam
usbdriver
${Boost_LIBRARIES}
handle_fuse
)
