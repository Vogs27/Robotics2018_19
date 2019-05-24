cmake_minimum_required(VERSION 2.8.3)
project(custom_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy
std_msgs 
message_generation
)

add_message_files(
 FILES
 Num.msg
 floatStamped.msg

)
 
generate_messages(
  DEPENDENCIES
 std_msgs
 )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pub_custom src/pub.cpp)
target_link_libraries(pub_custom ${catkin_LIBRARIES})
add_dependencies(pub_custom ${catkin_EXPORTED_TARGETS})

add_executable(sub_custom src/sub.cpp)
target_link_libraries(sub_custom ${catkin_LIBRARIES})
add_dependencies(sub_custom ${catkin_EXPORTED_TARGETS})


