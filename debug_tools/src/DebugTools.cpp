#include "DebugTools.h"

 DebugTools:: DebugTools(ros::NodeHandle & node,  ros::NodeHandle & private_node)
{
    // std::cout<<

}


bool DebugTools::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

	nodeHandle.param("PointCloud2InTopic", PointCloud2INTopic, std::string("PointCloud2INTopic"));

	// nodeHandle.param("odom_in_topic", m_sOdomTopic, std::string("/odometry/filtered"));
	// nodeHandle.param("shock_duration", dShockDuration, 8.0);


	return true;

}
