cmake_minimum_required(VERSION 3.0.2)
project(point_depth)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES realsense_circle opencv
  CATKIN_DEPENDS cv_bridge roscpp std_msgs sensor_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(${PROJECT_NAME}_node src/point_main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
