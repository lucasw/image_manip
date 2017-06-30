cmake_minimum_required(VERSION 2.8.3)
project(image_manip)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

roslint_cpp()

generate_dynamic_reconfigure_options(
  cfg/Delay.cfg
  cfg/IIRImage.cfg
  cfg/Resize.cfg
  cfg/RotoZoom.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_manip
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(image_manip
  src/iir_image.cpp
  src/image_delay.cpp
  src/resize.cpp
  src/roto_zoom.cpp
  src/utility.cpp)
add_dependencies(image_manip ${PROJECT_NAME}_gencfg)
target_link_libraries(image_manip ${catkin_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(image_manip ${catkin_EXPORTED_LIBRARIES})
endif()

add_executable(iir_image_node src/iir_image_node.cpp)
target_link_libraries(iir_image_node ${catkin_LIBRARIES})

add_executable(image_delay_node src/image_delay_node.cpp)
target_link_libraries(image_delay_node ${catkin_LIBRARIES})

add_executable(resize_node src/resize_node.cpp)
target_link_libraries(resize_node ${catkin_LIBRARIES})

add_executable(roto_zoom_node src/roto_zoom_node.cpp)
target_link_libraries(roto_zoom_node ${catkin_LIBRARIES})

# TODO(lucasw) install
