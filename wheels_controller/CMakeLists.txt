cmake_minimum_required(VERSION 3.0.2)
project(wheels_controller)


find_package(catkin REQUIRED COMPONENTS
  agrobit_msg
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS agrobit_msg roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_start src/wheels_controller.cpp src/wheels_controller_start.cpp)

target_link_libraries(${PROJECT_NAME}_start
  ${catkin_LIBRARIES}
)


