Team members:
- Alessandro Andrea Vogrig (matr. 845461 - cod pers. 10527163)
- Elena Rosalba Schinelli (matr. - cod pers.  )
________________________________________________________________________________
Package name: first_project
Node name: tf_odom_pub
________________________________________________________________________________
Package structure:
first_project
├── cfg
│   └── parameters.cfg
├── msg
│   └── floatedStamped.msg
├── src
│   └── tf_odom_pub.cpp
├── CMakeLists.txt
├── package.xml
└── readme.txt
________________________________________________________________________________
File description:
- parameters.cfg: contains definition of parameters used with dynamic reconfigure
- floatedStamped.msg: contains custom message definition, given with bags
- tf_odom_pub.cpp: our main node. Takes care of syncronizing all the various topics necessary to compute odometry.
    Computes odometry and publishes it via topic and tf.
________________________________________________________________________________
Dynamic reconfigure parameters:
- newX: set new X point (origin or a different value), accepts double
- newY: set new Y point (origin or a different value), accepts double
- odom_mode: set differential drive model (value 0) or ackermann steering model (value 1)

Odom_mode si an enumeration, so if you use rqt_reconfigure tool, only the two avaiable choices with a label will be shown.
________________________________________________________________________________
The root node of tf tree is "world", either for differential drive or ackermann model.
The child of "world" frame is "base_link", considered the odeometry center, in particular, "base_link_differential" for differential drive model and "base_link_ackermann" for ackermann steering model.
There are also two more frames for differential drive model rappresenting the two wheels and called "right_wheel_differential" and "left_wheel_differential", with top frame "base_link_differential".
For the ackermann steering model, there are four more frames, also rappresenting the wheels. They are called "right_rear_wheel_ackermann", "left_rear_wheel_ackermann",
    "left_front_wheel_ackermann" and "right_front_wheel_ackermann", children of "base_link_ackermann".
We also publish odometry with "nav_msgs::Odometry" on "/world" topic.
We publish poses on topic "/custom_pose".
________________________________________________________________________________
Custom message structure:
We publish our poses also on "/custom_pose" topic.
Messages published on this topic ar "customPoses" messages.
They are composed by 3 64 bit float called "x", "y", and "th", rappresenting coordinates and direction of base_link and a string called "model" rappresenting the type of kinematic model used.
________________________________________________________________________________
How to run our project:
1. Place the package folder in your ROS workspace and compile it with catkin_make
2. Run roscore and run the bag file in a separate terminal
3. Start our node with rosrun. Prototipe: rosrun first_project tf_odom_pub

Notes: to show correctly tf in rviz, you must set the static frame name as "world". You can also draw the base_link point using the "odometry" function in rviz and subscribing to "world" topic.
________________________________________________________________________________
Other info:
-We used Eulero method to compute integrals.
-In ackermann model, the pose is not truly reffered to the base_link point but to the center point of the front axis.
