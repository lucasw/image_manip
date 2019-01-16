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
find_package(cv_bridge REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# TODO(lucasw) make a cmake macro and a list of all the cpp files
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher
  "rclcpp"
  "sensor_msgs")
target_link_libraries(image_publisher
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS image_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(resize src/resize.cpp src/utility.cpp)
ament_target_dependencies(resize
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(resize
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(resize
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS resize DESTINATION lib/${PROJECT_NAME})

add_executable(roto_zoom src/roto_zoom.cpp src/utility.cpp)
ament_target_dependencies(roto_zoom
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(roto_zoom
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(roto_zoom
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS roto_zoom DESTINATION lib/${PROJECT_NAME})

add_executable(color src/color.cpp src/utility.cpp)
ament_target_dependencies(color
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(color
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(color
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS color DESTINATION lib/${PROJECT_NAME})

add_executable(image_deque src/image_deque.cpp src/utility.cpp)
ament_target_dependencies(image_deque
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(image_deque
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(image_deque
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS image_deque DESTINATION lib/${PROJECT_NAME})

add_executable(iir_image src/iir_image.cpp src/utility.cpp)
ament_target_dependencies(iir_image
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(iir_image
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(iir_image
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS iir_image DESTINATION lib/${PROJECT_NAME})

add_executable(save_image src/save_image.cpp src/utility.cpp)
ament_target_dependencies(save_image
  "cv_bridge"
  "rclcpp"
  "sensor_msgs"
  "std_msgs")
include_directories(save_image
  ${OpenCV_INCLUDE_DIRECTORIES}
  include)
target_link_libraries(save_image
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})
install(TARGETS save_image DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/stop_motion_imgui.py
  scripts/color_imgui.py
  scripts/setup_roto_gui.py
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
