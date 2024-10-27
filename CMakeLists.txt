cmake_minimum_required(VERSION 2.8.3)
project(project_1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
  tf2
  tf2_ros
  message_filters
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   WheelsVel.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  cfg/robotparameters.cfg
)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime message_filters)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocities src/velocities.cpp)
target_link_libraries(velocities ${catkin_LIBRARIES})

add_executable(velocitiesTicks src/velocitiesTicks.cpp)
target_link_libraries(velocitiesTicks ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
add_dependencies(odometry project_1_generate_messages)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_dependencies(odometry ${PROJECT_NAME}_gencfg)
add_dependencies(velocitiesTicks ${PROJECT_NAME}_gencfg)

add_executable(wheelsVel src/wheelsVel.cpp)
add_dependencies(wheelsVel ${catkin_EXPORTED_TARGETS})
add_dependencies(wheelsVel project_1_generate_messages)
target_link_libraries(wheelsVel ${catkin_LIBRARIES})

add_executable(tfBroadcast src/tfBroadcast.cpp)
target_link_libraries(tfBroadcast ${catkin_LIBRARIES})

add_executable(checkParams src/checkParams.cpp)
target_link_libraries(checkParams ${catkin_LIBRARIES})