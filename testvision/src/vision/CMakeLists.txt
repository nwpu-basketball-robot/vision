SET(CMAKE_BUILD_TYPE Release)
PROJECT (vision)
cmake_minimum_required(VERSION 3.5)

INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

set(OpenCV_DIR "/home/hisfog/opencv-3.2.0/build")

find_package(OpenCV REQUIRED)
find_package(
catkin REQUIRED COMPONENTS
roscpp
rostime
std_msgs
basketball_msgs
message_generation
geometry_msgs
dynamic_reconfigure
)


include_directories(${OpenCV_INCLUDE_DIRS}
/home/hisfog/testvision/devel/include
${catkin_INCLUDE_DIRS}
/opt/ros/kinetic/include
/home/hisfog/testhokuyo/src/hokuyo_node-indigo-devel/include
/home/hisfog/testhokuyo/src/hokuyo_node-indigo-devel/include/hokuyo_node
${Boost_INCLUDE_DIRS}
/home/hisfog/testvision/hokuyo_node-indigo-devel/src
/home/hisfog/testvision/src/driver_common-indigo-devel/driver_base/include
/home/hisfog/testvision/devel/include
/home/hisfog/testusbcap/include#for wmj::usbcapture
)


add_library(get_rects src/get_rects.cpp)
target_link_libraries(get_rects ${OpenCV_LIBS})

add_library(max_profile src/max_profile.cpp)
target_link_libraries(max_profile ${OpenCV_LIBS})



add_library(range_cut src/range_cut.cpp)
target_link_libraries(range_cut ${OpenCV_LIBS})

add_library(detector src/detector.cpp)
target_link_libraries(detector max_profile range_cut get_rects ${OpenCV_LIBS})


add_library(improcess src/improcess.cpp)
target_link_libraries(improcess ${OpenCV_LIBS})

add_library(network src/network.cpp)
target_link_libraries(network ${OpenCV_LIBS} improcess)


add_executable(mainfunc src/main.cpp)
add_dependencies(mainfunc hokuyo_node_gencfg)
target_link_libraries(mainfunc
${OpenCV_LIBS}
${catkin_LIBRARIES}
max_profile
range_cut
get_rects
detector
libhokuyo
${Boost_LIBRARIES}
Threads::Threads	
darknet
network
UsbCapture
)



