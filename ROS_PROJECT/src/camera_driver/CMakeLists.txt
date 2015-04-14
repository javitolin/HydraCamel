cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	cv_bridge
	roscpp
	sensor_msgs
	std_msgs
	image_transport
)
find_package(OpenCV 2 REQUIRED COMPONENTS core highgui imgproc)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/flycapture
)

add_executable(camera_driver_sender src/imageSender.cpp)
target_link_libraries(camera_driver_sender -ldl ${catkin_LIBRARIES} ${OpenCV_LIBS} flycapture)

add_executable(camera_driver_receiver src/imageReceive.cpp)
target_link_libraries(camera_driver_receiver -ldl ${catkin_LIBRARIES} ${OpenCV_LIBS})
