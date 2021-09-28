cmake_minimum_required(VERSION 2.8.3)
project(larr_ugv)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs  
)

catkin_package(
DEPENDS Eigen
  CATKIN_DEPENDS roscpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC_server ${SRC_server} src/test_tcp_server.cpp)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

set(DCMAKE_build_type debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (test_tcp_server src/test_tcp_server.cpp)
target_link_libraries (test_tcp_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (tcp_server src/main_tcp_server.cpp)
target_link_libraries (tcp_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (keyboard_command src/main_keyboard_command.cpp)
target_link_libraries (keyboard_command ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

#add_executable (tcpip_client ${SRC_client})
#target_link_libraries (tcpip_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

