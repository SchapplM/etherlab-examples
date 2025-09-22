cmake_minimum_required(VERSION 2.8.3)
project(pcu_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg actionlib_msgs actionlib
)



add_service_files(
  FILES
  SetInt.srv
  SetFloat.srv
  SetString.srv
  GetBond.srv
)

add_action_files(DIRECTORY action FILES AcquireRessource.action)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime actionlib actionlib_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
