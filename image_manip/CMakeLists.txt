cmake_minimum_required(VERSION 3.0)
project(image_manip VERSION 0.0.1)

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

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)
roslint_cpp()

generate_dynamic_reconfigure_options(
  cfg/Convert.cfg
  cfg/Delay.cfg
  cfg/IIRImage.cfg
  cfg/ImageDeque.cfg
  cfg/Resize.cfg
  cfg/RotoZoom.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  dynamic_reconfigure
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/convert.cpp
  src/cv_distort_image.cpp
  src/distort.cpp
  src/iir_image.cpp
  src/image_delay.cpp
  src/image_deque.cpp
  src/resize.cpp
  src/roto_zoom.cpp
  src/utility.cpp
  src/utility_ros.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

catkin_install_python(PROGRAMS
  scripts/animated_gif.py
  scripts/image_folder_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(distort_image src/distort_image.cpp)
# target_link_libraries(distort_image ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(convert_node src/convert_node.cpp)
target_link_libraries(convert_node ${catkin_LIBRARIES})

add_executable(iir_image_node src/iir_image_node.cpp)
target_link_libraries(iir_image_node ${catkin_LIBRARIES})

add_executable(image_delay_node src/image_delay_node.cpp)
target_link_libraries(image_delay_node ${catkin_LIBRARIES})

add_executable(image_deque_node src/image_deque_node.cpp)
target_link_libraries(image_deque_node ${catkin_LIBRARIES})

add_executable(resize_node src/resize_node.cpp)
target_link_libraries(resize_node ${catkin_LIBRARIES})

add_executable(roto_zoom_node src/roto_zoom_node.cpp)
target_link_libraries(roto_zoom_node ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
  convert_node
  iir_image_node
  image_delay_node
  image_deque_node
  ${PROJECT_NAME}
  resize_node
  roto_zoom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
