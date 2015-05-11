cmake_minimum_required(VERSION 2.8.3)
project(hc_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs
)
find_package(OpenCV 2 REQUIRED COMPONENTS core highgui imgproc)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB hc_vision_src_files src/*.cpp)
file(GLOB hc_algos_src_files 
	Algos/*.cpp 
#	Algos/Balls/*.cpp 
#	Algos/blackGate2/*.cpp
#	Algos/Gate/*.cpp
#	Algos/Path/*.cpp
#	Algos/Shadow/*.cpp
#	Algos/Torpedo/*.cpp
#	Algos/Traffic/*.cpp
	Algos/Utils/*.cpp)


add_executable(hc_vision_src ${hc_algos_src_files} ${hc_vision_src_files})
target_link_libraries(hc_vision_src -ldl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})