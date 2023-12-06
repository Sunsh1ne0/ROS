cmake_minimum_required(VERSION 2.8.3)
project(agrobit_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  all_sensor.msg
  hedge_pos_ang.msg
  wheel_data.msg
  direction_and_speed_wheels.msg
  indication.msg
  mileage.msg
  sensors.msg
  chat.msg
  can_bus.msg
  ecan_e01s_status.msg
  wheels_electrical.msg
  wheels_kinematics.msg
  kinematics_msg.msg
  status_msg.msg
  target_values.msg
  imu_l3gd20.msg
)
generate_messages()

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs message_runtime
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${roscpp_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
