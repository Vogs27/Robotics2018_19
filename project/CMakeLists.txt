cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
std_msgs 
tf
custom_messages
message_generation
sensor_msgs
)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(my_pub src/pub.cpp)
target_link_libraries(my_pub ${catkin_LIBRARIES})

add_dependencies(my_pub ${catkin_EXPORTED_TARGETS})

