cmake_minimum_required(VERSION 2.8.3)
project(cooperative_obstacle_avoidance)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  image_geometry

)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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

#  INCLUDE_DIRS include
#  LIBRARIES cooperative_obstacle_avoidance
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(  ${catkin_INCLUDE_DIRS} )

## Declare a cpp library
# add_library(cooperative_obstacle_avoidance
#   src/${PROJECT_NAME}/cooperative_obstacle_avoidance.cpp
# )

## Declare a cpp executable
# add_executable(cooperative_obstacle_avoidance_node src/cooperative_obstacle_avoidance_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cooperative_obstacle_avoidance_node cooperative_obstacle_avoidance_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(cooperative_obstacle_avoidance_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS cooperative_obstacle_avoidance cooperative_obstacle_avoidance_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cooperative_obstacle_avoidance.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(ugv src/ugv.cpp)
add_dependencies(ugv ugv_generate_messages_cpp)
target_link_libraries(ugv ${catkin_LIBRARIES} )

add_executable(uav src/uav.cpp)
add_dependencies(uav uav_generate_messages_cpp)
target_link_libraries(uav ${catkin_LIBRARIES} )

add_executable(ugv_room_hardware src/ugv_room_hardware.cpp)
add_dependencies(ugv_room_hardware ugv_room_hardware_generate_messages_cpp)
target_link_libraries(ugv_room_hardware ${catkin_LIBRARIES} )

add_executable(ugv_room src/ugv_room.cpp)
add_dependencies(ugv_room_hardware ugv_room_generate_messages_cpp)
target_link_libraries(ugv_room ${catkin_LIBRARIES} )

add_executable(uav_room_hardware src/uav_room_hardware.cpp)
add_dependencies(uav_room_hardware uav_room_hardware_generate_messages_cpp)
target_link_libraries(uav_room_hardware ${catkin_LIBRARIES} )

add_executable(uav_room src/uav_room.cpp)
add_dependencies(uav_room_hardware uav_room_generate_messages_cpp)
target_link_libraries(uav_room ${catkin_LIBRARIES} )

add_executable(ugv_homography src/ugv_homography.cpp)
add_dependencies(ugv_homography ugv_homography_generate_messages_cpp)
target_link_libraries(ugv_homography ${catkin_LIBRARIES} )


add_executable(uav_homography src/uav_homography.cpp)
add_dependencies(uav_homography uav_homography_generate_messages_cpp)
target_link_libraries(uav_homography ${catkin_LIBRARIES} )

add_executable(ugv_combined src/ugv_combined.cpp)
add_dependencies(ugv_combined ugv_combined_generate_messages_cpp)
target_link_libraries(ugv_combined ${catkin_LIBRARIES})

add_executable(uav_combined src/uav_combined.cpp)
add_dependencies(uav_combined uav_combined_generate_messages_cpp)
target_link_libraries(uav_combined ${catkin_LIBRARIES} )

add_executable(bearing_angle_avoidance_hardware src/bearing_angle_avoidance_hardware.cpp)
add_dependencies(bearing_angle_avoidance_hardware bearing_angle_avoidance_hardware_generate_messages_cpp)
target_link_libraries(bearing_angle_avoidance_hardware ${catkin_LIBRARIES} )

add_executable(ardrone_hardware src/ardrone_hardware.cpp)
add_dependencies(ardrone_hardware ardrone_hardware_generate_messages_cpp)
target_link_libraries(ardrone_hardware ${catkin_LIBRARIES} )

set(CMAKE_CXX_FLAGS "-std=c++0x  ${CMAKE_CXX_FLAGS}")
