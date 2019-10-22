#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void number_callback(const std_msgs::Int32::ConstPtr &msg) {
    ROS_INFO("Recieved [%d]", msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nodeHandle;

    ros::Subscriber subscriber = nodeHandle.subscribe("/numbers", 10, number_callback);

    ros::spin();

    return 0;
}