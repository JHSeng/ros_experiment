// standard header
#include <cstdlib>
#include <iomanip>
// ros header
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
// turtlesim node header
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

int main(int argc, char **argv) {
    // program init
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nodeHandle;

    ros::Publisher publisher = nodeHandle.advertise<std_msgs::Int32>("/numbers", 10);
    ros::Rate rate(10);
    int count = 0;

    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = count;
        ROS_INFO("%d", msg.data);
        publisher.publish(msg);
        ros::spinOnce();
        rate.sleep();
        count++;
    }
    return 0;
}
