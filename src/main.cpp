// c++ basic header
#include <iostream>
// ros basic header
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// turtlesim node header
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

void position_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    ROS_INFO("msg->linear.x = [%d]", msg->linear.x);
    ROS_INFO("msg->angular.z = [%d]", msg->angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nodeHandle;

    ros::Subscriber position_subscriber = nodeHandle.subscribe("/turtle1/pose", 10, position_callback);

    return 0;
}