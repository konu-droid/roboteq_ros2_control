# roboteq_ros2_control

A ros2_control package to use roboteq motor controllers for differential drive robots.

*Note: Current the motor velocity is calculated by querying wheel encoder RPM directly from the motor drivers and feed into diff_drive_controller from ros2_control.*

**Subscribers**  
~/cmd_vel [geometry_msgs/msg/TwistStamped]  

**publishers**  
~/odom [nav_msgs::msg::Odometry]  
~/tf [tf2_msgs::msg::TFMessage]

<br>

*For more information on parameters checkout [diff_drive_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)* 