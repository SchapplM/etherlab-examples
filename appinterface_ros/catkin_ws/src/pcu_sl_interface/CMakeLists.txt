cmake_minimum_required(VERSION 2.8.3)
project(pcu_sl_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  pcu_common
)

generate_dynamic_reconfigure_options(cfg/model.cfg)

#add_action_files(DIRECTORY action FILES CollectFeatureData.action)
#add_action_files(DIRECTORY action FILES AquireRessource.action)
add_message_files(FILES ControllerState.msg JointsVector.msg)
add_service_files(FILES SetState.srv)


# make sure configure headers are built before any node using them
# add_dependencies(example_node ${PROJECT_NAME}_gencfg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime pcu_common)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_sl_interface src/node.cpp)
target_link_libraries(ros_sl_interface ${catkin_LIBRARIES})
add_dependencies(ros_sl_interface pcu_common_gencpp)

add_executable(test_interface src/test_interface.cpp)
target_link_libraries(test_interface ros_sl_interface ${catkin_LIBRARIES})

#add_executable(test2 src/test.cpp)
#target_link_libraries(test2 ${catkin_LIBRARIES})

install(TARGETS ros_sl_interface test_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/client_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

