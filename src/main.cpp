// c++ basic header
#include <iostream>
// ros basic header
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// turtlesim node header
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

void position_callback(const turtlesim::Pose &msg) {
    // called function sign
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << " position = (" << msg.x << " , " << msg.y << ") * direction = " << msg.theta);
    // data init
    int __curr_turtle_x = msg.x, __curr_turtle_y = msg.y, __curr_turtle_theta = msg.theta;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nodeHandle;

    // get turtle position
    // ros::Subscriber position_subscriber = nodeHandle.subscribe("/turtle1/pose", 1000, position_callback);
    // ros::spin();

    // publish signal to topic
    ros::Publisher move_publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    srand(time(0));
    ros::Rate rate(2);
    while (ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = double(rand()) / double(RAND_MAX);
        msg.angular.z = 2 * double(rand()) / double(RAND_MAX) - 1;
        move_publisher.publish(msg);
        rate.sleep();
    }
    return 0;
}