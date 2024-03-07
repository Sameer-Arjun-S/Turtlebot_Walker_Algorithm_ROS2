# ROS2_TurtleBot_Walker
 Walker algorithm inplementation in ROS2 on Turtlebot 
### Author
Sameer Arjun S

### Building the package
```
# Source to ROS2 humble
source /opt/ros/humble/setup.bash
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
git clone <link to repository>
# Navigate to the base directory
cd ..
# Install rosdep dependencies
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select turtlebot3_walker_ros2
# After successfull build source the package
. install/setup.bash
```
### Code to run the program
```
ros2 launch turtlebot3_walker_ros2 launch.py
```
### ROS bag
```
ros2 launch turtlebot3_walker_ros2 launch.py record_topics:=True
# To view the results
ros2 bag info walker_bag_topics_list

# To play the recorded file
cd walker_bag_topics_list
ros2 bag play ["ros bag file name"]
```

### Cpp Check
```
# If you need to install cppcheck, do
sudo apt install cppcheck

# Run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### Cpp Lint
```
# You may need to install cpplint:
sudo apt install cpplint

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### Results
```
sas@sas-virtual-machine:~/ros2_ws/src/turtlebot3_walker$ ros2 bag info walker_bag_topics_list

Files:             walker_bag_topics_list_0.db3
Bag size:          2.7 MiB
Storage id:        sqlite3
Duration:          27.782s
Start:             Nov 29 2023 22:58:51.782 (1701316731.782)
End:               Nov 29 2023 22:59:19.564 (1701316759.564)
Messages:          6642
Topic information: Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /robot_description | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 101 | Serialization Format: cdr
                   Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: 261 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 958 | Serialization Format: cdr
                   Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                   Topic: /imu | Type: sensor_msgs/msg/Imu | Count: 3869 | Serialization Format: cdr
                   Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 599 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 21 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 0 | Serialization Format: cdr
                   Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: 598 | Serialization Format: cdr
                   Topic: /performance_metrics | Type: gazebo_msgs/msg/PerformanceMetrics | Count: 131 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 102 | Serialization Format: cdr
```
### Gazebo results
![Screenshot from 2023-11-29 21-44-17](https://github.com/Sameer-Arjun-S/Turtlebot_Walker_Algorithm_ROS2/assets/112655999/4cedd372-731b-4c8e-b9d9-0c5a8a7d40a1)

