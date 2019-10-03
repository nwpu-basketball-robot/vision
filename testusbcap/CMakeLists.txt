SET(CMAKE_BUILD_TYPE Release)
PROJECT(testUsbCap)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
/home/hisfog/testusbcap/include
)

add_executable(mainfunc src/main.cpp)
target_link_libraries(mainfunc
${OpenCV_LIBS}
UsbCapture
)
