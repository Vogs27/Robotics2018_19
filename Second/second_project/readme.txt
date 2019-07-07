Team members:
- Alessandro Andrea Vogrig (matr. 845461 - cod pers. 10527163)
- Elena Rosalba Schinelli (matr. 844329 - cod pers. 10501436)
________________________________________________________________________________
Package name: second_project
Node name: odom_pub
________________________________________________________________________________
Package structure:
first_project
├── include
│   └── imu_complementary_filter
|       ├── complementary_filter_ros.h
|       └── complementary_filter.h
├── src
│   ├── odom_pub.cpp
│   ├── complementary_filter.cpp
│   ├── complementary_filter_node.cpp
|   └── complementary_filter_ros.cpp
├── launch
│   └── launch_all.launch
├── params
│   └── navsat_transform_param.yaml
├── CMakeLists.txt
├── package.xml
├── project.bag
└── readme.txt
________________________________________________________________________________
File description:
- odom_pub.cpp: our main node. Compute odometry from "speedsteer" topic and publishes under "myTopic/odom"
- launch_all.launch: launch file to launch odom_pub, imu_complementary_filter (cpp files included in the package) and robot localization (sources not included).
- complementary_filter files: files to realize imu complementaty message_filter
- navsat_transform_param.yaml: based on prof. Mentasti's, considering the same source of data, we have used a similar config file for navsat node.
________________________________________________________________________________
How to run our project:
1. Make sure you have installed all dipendencies (imu_tools and robot_localization)
2. Make sure you have the package "robot_localization" compiled and avaiable.
3. Place the package folder in your ROS workspace and compile it with catkin_make
4. Run command "source devel/setup.bash" in your workspace root
5. Run roscore and run the bag file in a separate terminal with rosbag play -l --clock project.bag
6. Start our project with the launch file. roslaunch second_project launch_all.launch
________________________________________________________________________________
