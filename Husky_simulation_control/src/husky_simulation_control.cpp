#include<ros/ros.h>
#include <std_msgs/String.h>
// #include<sensor_msgs/JointState.h>
#include<tf/transform_broadcaster.h>
// #include<nav_msgs/Odometry.h>
#include<iostream>
#include"husky_simulation_control/keyboard.h"

// void cmd_velCallback(const geometry_msgs::Twist &twist_aux);


double vel_x = 0.0;
double vel_y = 0.0;
double vel_z = 0.0;

double vel_roll = 0.0;
double vel_pitch = 0.0;
double vel_yaw = 0.0;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"husky_simulation_control");
    ros::NodeHandle node;
    // ros::Subscriber cmd_vel_sub = node.subscribe("cmd_vel",10,cmd_velCallback);
    ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate loop_rate(10);

    while(true)
    {
        geometry_msgs::Twist twist;
        // twist.linear.x = vel_x;
        twist.linear.y = vel_y;
        twist.linear.z = vel_z;
        twist.angular.x = vel_roll;
        twist.angular.y = vel_pitch;
        // twist.angular.z = vel_yaw;
        
        int keynum = scanKeyboard();
        // std::cout<< "key: "<< keynum<<std::endl;
        if (keynum == 97)
        {
            std::cout<< "keynum: "<< keynum << "  key : a"<<std::endl;
            twist.angular.z = 0.3;

        }
        else if (keynum == 115)
        {
            std::cout<< "keynum: "<< keynum << "  key : s"<<std::endl;
            twist.linear.x = -0.7;
        }
        else if (keynum == 100)
        {
            std::cout<< "keynum: "<< keynum << "  key : d"<<std::endl;
            twist.angular.z = -0.3;
        }
        else if (keynum == 119)
        {
            std::cout<< "keynum: "<< keynum << "  key : w"<<std::endl;
            twist.linear.x = 0.7;
        }
        else
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        cmd_vel_pub.publish(twist);
    }

}

// void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
// {
//     geometry_msgs::Twist twist=twist_aux;
//     vel_x = twist.linear.x;
//     vel_y = twist.linear.y;
//     vel_z = twist.linear.z;

//     vel_roll = twist.angular.x;
//     vel_pitch = twist.angular.y;
//     vel_yaw = twist.angular.z;
// }
