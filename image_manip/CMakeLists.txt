cmake_minimum_required(VERSION 3.5)
project(image_manip)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(example_interfaces REQUIRED)
# TODO(lucasw) if this isn't available then don't build the executables that need it
find_package(imgui_ros REQUIRED)
find_package(internal_pub_sub REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRECTORIES}
  include
)

add_library(imagemanip SHARED
  src/color.cpp
  src/iir_image.cpp
  src/image_deque.cpp
  src/resize.cpp
  src/save_image.cpp
  src/utility.cpp
)
ament_target_dependencies(imagemanip
  "builtin_interfaces"
  "class_loader"
  "cv_bridge"
  "internal_pub_sub"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)
rclcpp_register_node_plugins(imagemanip "image_manip::Color")
rclcpp_register_node_plugins(imagemanip "image_manip::IIRImage")
rclcpp_register_node_plugins(imagemanip "image_manip::ImageDeque")
rclcpp_register_node_plugins(imagemanip "image_manip::Resize")
rclcpp_register_node_plugins(imagemanip "image_manip::SaveImage")
install(TARGETS
  imagemanip
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

####################################################################
# TODO(lucasw) make a cmake macro and a list of all the cpp files
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher
  "cv_bridge"
  "rclcpp"
  "sensor_msgs")
target_link_libraries(image_publisher
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS image_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(roto_zoom src/roto_zoom.cpp src/utility.cpp)
ament_target_dependencies(roto_zoom
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
target_link_libraries(roto_zoom
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS roto_zoom DESTINATION lib/${PROJECT_NAME})

# TODO(lucasw) src/nodes/resize.cpp

add_executable(color src/nodes/color.cpp)
target_link_libraries(color imagemanip)
ament_target_dependencies(color rclcpp)
install(TARGETS color DESTINATION lib/${PROJECT_NAME})

add_executable(iir_image src/nodes/iir_image.cpp)
target_link_libraries(iir_image imagemanip)
ament_target_dependencies(iir_image rclcpp)
install(TARGETS iir_image DESTINATION lib/${PROJECT_NAME})

add_executable(image_deque src/nodes/image_deque.cpp)
target_link_libraries(image_deque imagemanip)
ament_target_dependencies(image_deque rclcpp)
install(TARGETS image_deque DESTINATION lib/${PROJECT_NAME})

add_executable(save_image src/nodes/save_image.cpp)
target_link_libraries(save_image imagemanip)
ament_target_dependencies(save_image rclcpp)
install(TARGETS save_image DESTINATION lib/${PROJECT_NAME})

add_executable(color_imgui_composite src/nodes/color_imgui_composite.cpp)
target_link_libraries(color_imgui_composite imagemanip)
ament_target_dependencies(color_imgui_composite
  imgui_ros
  rclcpp
)
install(TARGETS color_imgui_composite DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/stop_motion_imgui.py
  scripts/color_imgui.py
  scripts/resize_imgui.py
  scripts/setup_roto_gui.py
  scripts/setup_image_pub_gui.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
