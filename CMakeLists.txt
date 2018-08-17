CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

SET(CMAKE_BUILD_TYPE Release)
PROJECT(vision CXX)

# Check for c++11 support
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

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)
find_package(freenect2 REQUIRED)
find_package(Caffe REQUIRED)  
add_definitions(${Caffe_DEFINITIONS}) 

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostime
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
  compressed_image_transport
  compressed_depth_image_transport
  kinect2_bridge  
  geometry_msgs
  image_geometry
  message_generation
  basketball_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES vision 
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs
  DEPENDS
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
  ${Caffe_INCLUDE_DIRS}
  /usr/local/cuda/include
)

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()

add_definitions(-DDEPTH_REG_OPENCL)

#add_library(kinect2_receiver src/Receiver.cpp)
#target_link_libraries(kinect2_receiver  
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}  
#  ${kinect2_bridge_LIBRARIES}
#)

add_library(pcl_judge src/PointCloudJudge.cpp)
target_link_libraries(pcl_judge  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES})

add_library(ssd_detector src/ssd_detect.cpp)
target_link_libraries(ssd_detector   ${Caffe_LIBRARIES})

add_library(cylinder_detector src/cylinderFindImage.cpp)
target_link_libraries(cylinder_detector ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(judge src/Judge.cpp)
target_link_libraries(judge
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${catkin_LIBRARIES}
  # kinect2_receiver same effect and more complicate, not using it now
  pcl_judge
  ssd_detector
  cylinder_detector
)
