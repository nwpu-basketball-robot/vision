cmake_minimum_required(VERSION 2.8.3)
project(hokuyo_node)

INCLUDE(CheckCXXCompilerFlag)#wyh
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

set(THREADS_PREFER_PTHREAD_FLAG ON)#wyh

find_package(Threads REQUIRED)#wyh

find_package(Boost REQUIRED COMPONENTS system thread)

# Utilities that use log4cxx - getID and getFirmwareVersion
find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    diagnostic_updater
    driver_base
    dynamic_reconfigure
    rosconsole
    roscpp
    self_test
    sensor_msgs
)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Hokuyo.cfg)

# Export with catkin_package
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS rosconsole
    LIBRARIES libhokuyo
)

# Hokuyo library
add_library(libhokuyo src/hokuyo.cpp)
target_link_libraries(libhokuyo ${rosconsole_LIBRARIES})

# hokuyo_node ,now is mainfunc
#add_executable(mainhk src/main.cpp)
#add_dependencies(mainhk ${PROJECT_NAME}_gencfg)#?
#target_link_libraries(mainhk
 # libhokuyo
 # ${Boost_LIBRARIES}
 # ${catkin_LIBRARIES}
 # Threads::Threads
#)

add_executable(getID src/getID.cpp)
target_link_libraries(getID libhokuyo ${LOG4CXX_LIBRARY} ${catkin_LIBRARIES})

add_executable(getFirmwareVersion src/getFirmwareVersion.cpp)
target_link_libraries(getFirmwareVersion
  libhokuyo
  ${LOG4CXX_LIBRARY}
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS libhokuyo DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS getID getFirmwareVersion #mainhk
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES test/TODO test/unplug_test.launch COPYING.lib hokuyo_test.launch hokuyo_test.vcg hokuyo_test_intensity.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
