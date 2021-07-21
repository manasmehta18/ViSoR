cmake_minimum_required(VERSION 2.8.3)
project(visor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  image_geometry
  cv_bridge
  tf
  cmake_modules
  message_generation
)

find_package(Eigen)
find_package(OpenCV 3 REQUIRED COMPONENTS core highgui imgproc xfeatures2d)
#find_package(OpenCV 3 REQUIRED PATHS "/opt/opencv/" COMPONENTS core highgui imgproc xfeatures2d)

generate_dynamic_reconfigure_options(
  cfg/stereodom.cfg
)

add_message_files(
  FILES
  kpt.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#link_directories(${OpenCV_LIB_DIR})

add_executable(viodom_node src/viodom_node.cpp)
add_executable(slam_node src/slam_node.cpp)

target_link_libraries(viodom_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Eigen_LIBRARIES}
)

target_link_libraries(slam_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Eigen_LIBRARIES}
)

