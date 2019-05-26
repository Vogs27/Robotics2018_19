cmake_minimum_required(VERSION 2.8.3)
project (first_project)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf message_generation dynamic_reconfigure)
add_message_files(FILES floatStamped.msg)
generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(cfg/parameters.cfg)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(subscriber_sync src/subscriber_sync.cpp)
target_link_libraries(subscriber_sync ${catkin_LIBRARIES})
add_dependencies(subscriber_sync ${catkin_EXPORTED_TARGETS})
add_dependencies(subscriber_sync ${PROJECT_NAME}_gencfg)
