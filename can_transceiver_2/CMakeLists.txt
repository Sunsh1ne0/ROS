cmake_minimum_required(VERSION 3.0.2)
project(can_transceiver_2)


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

add_executable(${PROJECT_NAME}_start src/can_transceiver_2.cpp src/can_transceiver_2_start.cpp)

add_dependencies(${PROJECT_NAME}_start ${can_transceiver_2_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_start
  ${catkin_LIBRARIES}
)
