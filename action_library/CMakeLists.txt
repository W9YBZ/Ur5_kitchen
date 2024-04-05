cmake_minimum_required(VERSION 3.0.2)
project(action_library)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  ExecuteActionSequence.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

