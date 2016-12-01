cmake_minimum_required(VERSION 2.8.3)
project(robotstatus_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES Obstacle.msg         
                 )


generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)
