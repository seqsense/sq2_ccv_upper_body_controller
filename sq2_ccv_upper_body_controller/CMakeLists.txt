cmake_minimum_required(VERSION 2.8.3)
project(sq2_ccv_upper_body_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_interface
  nav_msgs
  realtime_tools
  tf
  urdf
  sq2_ccv_roll_pitch_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sq2_ccv_upper_body_controller
#  CATKIN_DEPENDS control_msgs controller_interface dynamic_reconfigure nav_msgs realtime_tools tf urdf
#  DEPENDS system_lib
)

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sq2_ccv_upper_body_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} sq2_ccv_roll_pitch_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(FILES sq2_ccv_upper_body_controller_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

