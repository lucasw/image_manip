cmake_minimum_required(VERSION 3.0)
project(image_manip_demo VERSION 0.0.1)

find_package(catkin REQUIRED COMPONENTS
  roslint
)

set(ROSLINT_PYTHON_OPTS "--ignore=''" "1>&2")
roslint_python()

catkin_package()
