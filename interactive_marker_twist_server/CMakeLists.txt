cmake_minimum_required(VERSION 2.8.3)

project(interactive_marker_twist_server)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  roslaunch
  roslint
  visualization_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/marker_server)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}
  PROPERTIES OUTPUT_NAME marker_server PREFIX "")

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/interactive_markers.launch
    DEPENDENCIES marker_server)
  roslint_cpp()
  roslint_add_test()
endif()
