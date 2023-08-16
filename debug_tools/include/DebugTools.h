#include <sensor_msgs/PointCloud2.h>
#include<ros/ros.h>

class DebugTools
{
public:
    
    DebugTools(ros::NodeHandle & node,  ros::NodeHandle & private_node);
    ~DebugTools(){}  

private:
   bool ReadLaunchParams(ros::NodeHandle & nodeHandle);
    //the subscirber below is to hear (record) point clouds produced by the Hesai devices
    ros::Subscriber PointCloud2Suber;

    // Topic 
    std::string PointCloud2INTopic;

};