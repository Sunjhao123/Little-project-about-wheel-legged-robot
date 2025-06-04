cmake_minimum_required(VERSION 3.10)
project(sensor_msg)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  FILES
    Noise.msg

)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)