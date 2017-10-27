CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

SET(CMAKE_BUILD_TYPE Release)
PROJECT(vision CXX)

set(PCL_DIR /usr/local/share/pcl-1.8)
set(OpenCV_DIR /usr/local/share/OpenCV)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED HINTS "/home/huangtairan/libfreenect2/build")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  basketball_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision basketball_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
  DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include_directories ( /home/huangtairan/py-faster-rcnn/caffe-fast-rcnn/include
     /home/huangtairan/py-faster-rcnn/lib/nms
     /home/huangtairan/FasterRCNN-Encapsulation-Cplusplus-master/faster_cxx_lib/lib
     /share/apps/local/include
     /usr/local/include 
     /usr/include/python2.7
     /usr/local/cuda/include  )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()

add_definitions(-DDEPTH_REG_OPENCL)

add_executable(vision src/main.cpp 
            src/Judge.cpp src/Judge.h
            src/PointCloudJudge.cpp src/PointCloudJudge.h
            src/faster_rcnn.cpp     src/faster_rcnn.h
            src/cylinderFindImage.cpp src/cylinderFindImage.h)

target_link_libraries(vision
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${catkin_LIBRARIES}
  /usr/local/cuda/lib64/libOpenCL.so

  /home/huangtairan/py-faster-rcnn/caffe-fast-rcnn/distribute/lib/libcaffe.so  
  /home/huangtairan/catkin_ws/src/vision/lib/libgpu_nms.so
  /usr/lib/x86_64-linux-gnu/libglog.so
  /usr/lib/x86_64-linux-gnu/libboost_system.so
  /usr/lib/x86_64-linux-gnu/libboost_python.so
  /usr/lib/x86_64-linux-gnu/libpython2.7.so
)

add_executable(vision_client src/Client.cpp)
target_link_libraries(vision_client ${catkin_LIBRARIES})

add_executable(camera_vision 
          src/camVision.cpp src/camVision.h
          src/cylinderFindImage.cpp src/cylinderFindImage.h)
target_link_libraries(camera_vision 
               ${catkin_LIBRARIES}
               ${OpenCV_LIBRARIES})

add_executable(camera_vision_client src/camVisionClient.cpp)
target_link_libraries(camera_vision_client ${catkin_LIBRARIES})

