cmake_minimum_required(VERSION 3.1) 
project(vision)            

file(GLOB_RECURSE sources     	include/*.h include/*.hpp src/*.cpp src/*.cu)

set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")

#cuda
find_package(CUDA REQUIRED)

message("-- CUDA version:${CUDA_VERSION}")

if(CUDA_VERSION_MAJOR GREATER 9)
	message("-- CUDA ${CUDA_VERSION_MAJOR} detected")
		set(
		CUDA_NVCC_FLAGS
		${CUDA_NVCC_FLAGS}; 
		-gencode arch=compute_72,code=sm_72
		)
endif()

find_package(OpenCV REQUIRED)
if(Opencv_FOUND)
	message("found opencv")
endif()

find_package(Threads REQUIRED)


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

add_subdirectory(UsbCapture)

include_directories(
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
/opt/ros/melodic/include
/home/basketballrobot/workspace/src/driver_common-indigo-devel

/home/basketballrobot/workspace/src/driver_common-indigo-devel/driver_base/include/
/home/basketballrobot/workspace/src/hokuyo_node-indigo-devel/include/
/home/basketballrobot/workspace/src/vision/UsbCapture/include/
/home/basketballrobot/workspace/devel/include/
/home/basketballrobot/workspace/devel/include/basketball_msgs/
)


#generate detector lib
cuda_add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE include/  ${OpenCV_INCLUDE_DIRS} ${CUDA_TOOLKIT_ROOT_DIR}/include ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
target_link_libraries(detector nvinfer nvinfer_plugin nvcaffe_parser gflags "stdc++fs")
			
add_executable(vision src/main.cpp)
add_dependencies(vision hokuyo_node_gencfg)	  
target_include_directories(vision PRIVATE include/)
target_link_libraries(
vision detector opencv_highgui opencv_imgproc opencv_core opencv_calib3d opencv_dnn
UsbCapture
${OpenCV_LIBS}
${catkin_LIBRARIES}
libhokuyo
${Boost_LIBRARIES}
Threads::Threads
${PYTHON_LIBRARIES}
)



