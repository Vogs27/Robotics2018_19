cmake_minimum_required(VERSION 2.8.3)
project (first_project)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf message_generation dynamic_reconfigure) #roslaunch
add_message_files(FILES floatStamped.msg)
generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(cfg/parameters.cfg)
#roslaunch_add_file_check(launch)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(tf_odom_pub src/tf_odom_pub.cpp)
target_link_libraries(tf_odom_pub ${catkin_LIBRARIES})
add_dependencies(tf_odom_pub ${catkin_EXPORTED_TARGETS})
add_dependencies(tf_odom_pub ${PROJECT_NAME}_gencfg)
