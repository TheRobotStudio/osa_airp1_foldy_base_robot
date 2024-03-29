cmake_minimum_required(VERSION 2.8.3)
project(airp1_foldy_base_apps)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  rosbag
  roscpp
  roslib
  std_msgs
  tf
  osa_msgs
  osa_common
  visualization_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

#if(0)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  getSlaveCmdArray.srv
  getSlaveCurrentCmd.srv
  getSlaveLockCmd.srv
  switchNode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  osa_msgs
)
#endif()

## Generate dynamic reconfigure options
#generate_dynamic_reconfigure_options(
#  cfg/kdTreeControl.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES osa_control
 CATKIN_DEPENDS rosbag roscpp roslib std_msgs tf osa_msgs osa_common visualization_msgs
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/eigen3)

## Declare a cpp library
#add_library(osa_airp1_foldy_base_apps
#  src/can_layer.cpp
#  src/epos_controller.cpp
#)

## Declare a cpp executable
add_executable(osa_airp1_foldy_joy_conductor src/foldy_joy_conductor.cpp )
add_executable(osa_airp1_foldy_base_manual src/foldy_base_manual.cpp src/kiwi_drive.cpp) 
add_executable(osa_airp1_odometry src/odometry.cpp)
add_executable(osa_airp1_base_controller src/base_controller.cpp)
add_executable(osa_airp1_waldo_joy_teleop src/waldo_joy_teleop.cpp src/kiwi_drive.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(osa_airp1_foldy_joy_conductor osa_control_generate_messages_cpp osa_common)
add_dependencies(osa_airp1_foldy_base_manual osa_control_generate_messages_cpp osa_common)
add_dependencies(osa_airp1_waldo_joy_teleop osa_control_generate_messages_cpp osa_common)
# add_dependencies(example4 osa_control_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(osa_airp1_foldy_joy_conductor ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_airp1_foldy_base_manual ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
target_link_libraries(osa_airp1_odometry ${catkin_LIBRARIES})
target_link_libraries(osa_airp1_base_controller ${catkin_LIBRARIES})
target_link_libraries(osa_airp1_waldo_joy_teleop ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS osa_control osa_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_osa_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
