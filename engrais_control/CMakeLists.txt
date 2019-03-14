cmake_minimum_required(VERSION 2.8.3)
project(engrais_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(robot_control robot_control.cpp
    src/Point.cpp
    src/WeightedPoint.cpp
    src/Utility.cpp
    src/Model.cpp
    src/Pearl.cpp
    src/Ruby_Versions/1_RubyPure.cpp
    src/Ruby_Versions/2_RubyGenetic.cpp
)
target_link_libraries(robot_control ${catkin_LIBRARIES})

add_executable(robot_keyboard_control robot_keyboard_control.cpp
)
target_link_libraries(robot_keyboard_control ${catkin_LIBRARIES})
