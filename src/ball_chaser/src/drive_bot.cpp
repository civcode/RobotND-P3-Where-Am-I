#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
ball_chaser::DriveToTarget::Response& res) {

    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f, (float)req.linear_x,     (float)req.angular_z);
    
    return true;
}


int main(int argc, char** argv) {
    ros:Init(argc, argv, "drive_bot");

    ros::NodeHandle n;

    motor_command_publisher = n.advertice<geometry_msgs::Twist>("/cmd_vel", 10);

    //todo: drive service

    //todo: 
    

    

    return 0;
}





