#include <sensor_msgs/PointCloud2.h>
#include<ros/ros.h>

class ParsePointCloud2
{
public:
    
    ParsePointCloud2(ros::NodeHandle & node,  ros::NodeHandle & private_node);
    ~ParsePointCloud2(){}                      

    //process point cloud 
    void HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData);


private:
    //the subscirber below is to hear (record) point clouds produced by the Hesai devices
    ros::Subscriber m_oLaserSuber;

};