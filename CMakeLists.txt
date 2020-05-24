cmake_minimum_required(VERSION 2.8.3)
project(alliance)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs 
  geometry_msgs 
  sensor_msgs
)


add_message_files(
  FILES
  message.msg
  motivation.msg
  to_monitor.msg
  to_monitor_l.msg
  task.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)


catkin_package(
   CATKIN_DEPENDS
   message_runtime
   roscpp
   rospy
   std_msgs
   sensor_msgs
   geometry_msgs
   nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CPP_FILES
  src/libs/monitor_lib.cpp
  src/libs/monitor_lib_l.cpp
)

set(H_FILES
  include/alliance/monitor_lib.h
  include/alliance/monitor_lib_l.h
)


catkin_package()

## Build Talker and Listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot_r_alloc src/old/entity_task_alloc.cpp)
target_link_libraries(robot_r_alloc ${catkin_LIBRARIES})
add_dependencies(robot_r_alloc alliance_generate_messages_cpp)

add_executable(monitor src/monitor.cpp ${CPP_FILES})
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor alliance_generate_messages_cpp)

add_executable(monitor_l src/monitor_l.cpp ${CPP_FILES})
target_link_libraries(monitor_l ${catkin_LIBRARIES})
add_dependencies(monitor_l alliance_generate_messages_cpp)

add_executable(alliance_robot0 src/old/alliance_robot1.cpp)
target_link_libraries(alliance_robot0 ${catkin_LIBRARIES})
add_dependencies(alliance_robot0 alliance_generate_messages_cpp)

add_executable(alliance_robot1 src/old/alliance_robot2.cpp)
target_link_libraries(alliance_robot1 ${catkin_LIBRARIES})
add_dependencies(alliance_robot1 alliance_generate_messages_cpp)

add_executable(alliance_robot2 src/old/alliance_robot3.cpp)
target_link_libraries(alliance_robot2 ${catkin_LIBRARIES})
add_dependencies(alliance_robot2 alliance_generate_messages_cpp)
