cmake_minimum_required(VERSION 2.8.3)
project(video_pub)

set(OpenCV_DIR "/home/gjx/opencv/opencv-3.4.4/build")   
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
 
  geometry_msgs
  roscpp
  rospy
  std_msgs
  iarc_msgs
   cv_bridge
  image_transport
  rospy
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
 generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${video_pub_SOURCE_DIR}/include/video_pub
)

add_executable(
	video_pub 
	src/video_pub.cpp 
	)

## Specify libraries to link a library or executable target against
target_link_libraries(video_pub
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)