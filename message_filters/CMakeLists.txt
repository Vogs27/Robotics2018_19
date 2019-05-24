cmake_minimum_required(VERSION 2.8.3)
project(message_filters_example)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_filters)


## Declare a catkin package
catkin_package( CATKIN_DEPENDS geometry_msgs message_filters)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(multi_publisher src/pub.cpp)
target_link_libraries(multi_publisher ${catkin_LIBRARIES})

add_executable(filter_subscriber src/sub.cpp)
target_link_libraries(filter_subscriber ${catkin_LIBRARIES})

add_executable(filter_subscriber_policy src/sub_pol.cpp)
target_link_libraries(filter_subscriber_policy ${catkin_LIBRARIES})


