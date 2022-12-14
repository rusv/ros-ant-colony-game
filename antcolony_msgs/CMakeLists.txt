cmake_minimum_required(VERSION 3.0.2)
project(antcolony_msgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
   FILES
   Ant.msg
        Action.msg
        Hive.msg
        Order.msg
        Player.msg
        State.msg
        Sugar.msg
)

add_service_files(
   FILES
   Register.srv
   Play.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
