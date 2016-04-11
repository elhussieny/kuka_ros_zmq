cmake_minimum_required(VERSION 2.8.3)
project(kuka_ros_zmq)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
std_msgs
rviz
cmake_modules
dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)



## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/KUKAcontrol.cfg
 )

	  

catkin_package(
 INCLUDE_DIRS include
   DEPENDS Eigen
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(kuka_interface src/kuka_interface.cpp) 
## Specify libraries to link a library or executable target against

target_link_libraries(kuka_interface ${catkin_LIBRARIES} ${PROJECT_NAME} ${Boost_LIBRARIES} zmq)
add_dependencies(kuka_interface ${PROJECT_NAME}_gencfg)

## Declare a cpp library
add_library(${PROJECT_NAME}
	include/kuka_ros_zmq/KUKAInterface.h
	src/KUKAInterface.cpp
)

set(CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
