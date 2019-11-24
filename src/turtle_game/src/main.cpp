// c++ basic header
#include <iostream>
// ros basic header
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
// turtlesim node header
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

geometry_msgs::Twist msg;

void position_callback(const turtlesim::Pose &msg) {
    // called function sign
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << " position = (" << msg.x << " , " << msg.y << ") * direction = " << msg.theta);
    // data init
    int __curr_turtle_x = msg.x, __curr_turtle_y = msg.y, __curr_turtle_theta = msg.theta;

}

void printInfo() {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "msg.linear.x = " << msg.linear.x << " , msg.angular.z = " << msg.angular.z);
}

void goForward(ros::Publisher &move_publisher) {
    for (int i = 1; i <= 50000; i++) {
        msg.linear.x = 1;
        msg.angular.z = 0;
        move_publisher.publish(msg);
        printInfo();
    }
}

void drawRectangle(ros::Publisher &move_publisher) {
    goForward(move_publisher);
    for (int i = 1; i <= 42325; i++) {
        msg.linear.x = 0;
        msg.angular.z = 1;
        move_publisher.publish(msg);
        printInfo();
    }
    goForward(move_publisher);
    for (int i = 1; i <= 42310; i++) {
        msg.linear.x = 0;
        msg.angular.z = 1;
        move_publisher.publish(msg);
        printInfo();
    }
    goForward(move_publisher);
    for (int i = 1; i <= 42200; i++) {
        msg.linear.x = 0;
        msg.angular.z = 1;
        move_publisher.publish(msg);
        printInfo();
    }
    goForward(move_publisher);
    for (int i = 1; i <= 42190; i++) {
        msg.linear.x = 0;
        msg.angular.z = 1;
        move_publisher.publish(msg);
        printInfo();
    }
    for (int i = 1; i <= 20000; i++) {
        msg.linear.x = 1;
        msg.angular.z = 0;
        move_publisher.publish(msg);
        printInfo();
    }
}

void changeBackgroundColor(ros::NodeHandle &nodeHandle) {
    int Red = 255 * double(rand()) / double(RAND_MAX);
    int Green = 255 * double(rand()) / double(RAND_MAX);
    int Bule = 255 * double(rand()) / double(RAND_MAX);
    ros::param::set("background_r", Red);
    ros::param::set("background_g", Green);
    ros::param::set("background_b", Bule);
    ros::ServiceClient clearClient = nodeHandle.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv);
}

void drawCircle(ros::NodeHandle &nodeHandle, ros::Publisher &move_publisher) {
    for (int i = 1; i <= 150000; i++) {
        if (i % 30000 == 0) changeBackgroundColor(nodeHandle);
        msg.linear.x = 1;
        msg.angular.z = 1;
        move_publisher.publish(msg);
        printInfo();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nodeHandle;

    // get turtle position
    // ros::Subscriber position_subscriber = nodeHandle.subscribe("/turtle1/pose", 1000, position_callback);
    // ros::spin();

    // publish signal to topic
    ros::Publisher move_publisher = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    srand(time(0));
    ros::Rate rate(10);

    // 画正方形
    // drawRectangle(move_publisher);

    // 画圆形
    // drawCircle(move_publisher);

    // 边画圆形边改变背景颜色
    while (ros::ok()) {
        drawCircle(nodeHandle, move_publisher);
        rate.sleep();
    }
    return 0;
}