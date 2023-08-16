#include "GtTrans.h"


/*************************************************
Function: GtTrans
Description: constrcution function for GtTrans class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/
GtTrans::GtTrans(ros::NodeHandle & node, 
                        ros::NodeHandle & private_node)
{

    ReadLaunchParams(private_node);
    //subscribe (hear) the point cloud topic from laser on right side 
    m_oLaserSuber = node.subscribe(m_sLaserTopic, 2, &GtTrans::HandlePointClouds, this);
    m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GtTrans::HandleTrajectory, this);
    //subscribe (hear) the odometry information


    //publish
    m_oLaserPub = node.advertise<sensor_msgs::PointCloud2>(m_sLaserOutTopic, 2);

    m_oOdomPub = node.advertise<nav_msgs::Odometry>(m_sOdomOutTopic, 2);

    testCloudPub= node.advertise<sensor_msgs::PointCloud2>("testPointCloud", 2);

}


bool GtTrans::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

	nodeHandle.param("lidar_topic", m_sLaserTopic, std::string("/velodyne_points"));
	nodeHandle.param("lidarout_topic", m_sLaserOutTopic, std::string("/slam_points"));

  nodeHandle.param("odom_topic", m_sOdomTopic, std::string("/odom"));
	nodeHandle.param("odomout_topic", m_sOdomOutTopic, std::string("/slam_odom"));

  // gt pose file path (KITTI  Type) 
	nodeHandle.param("gt_pose_file_path", gt_pose_file_path, std::string(""));
  transform_matrix_list = ReadPosesFile(gt_pose_file_path);

  
	nodeHandle.param("FrameMin", FrameMin, int(0));
	nodeHandle.param("FrameMax", FrameMax,int(-1));
  if(FrameMax == -1) FrameMax = transform_matrix_list.size();


}


std::vector<Eigen::Matrix4d > GtTrans::ReadPosesFile(std::string path)
{
  std::vector<Eigen::Matrix4d > matrix_list;
  Eigen::Matrix4d matrix;
  std::ifstream file(path.c_str());
  std::string line;
  float x;
  int i = 0;
  while (getline(file, line)) {
    std::stringstream ss(line);
    for (int i = 0; i < 12; i++)
    {
      ss >> matrix(i / 4, i % 4);
    }
    matrix(3, 0) = 0; matrix(3, 1) = 0; matrix(3, 2) = 0; matrix(3, 3) = 1;
    //std::cout << matrix << std::endl;
    matrix_list.push_back(matrix);
  }
  file.close();
  return matrix_list;
}


void GtTrans::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData)
{
  // Create a container for the data.  
  sensor_msgs::PointCloud2 vTransposedData;
  sensor_msgs::PointCloud2 testcloudData;

  
  ////a point clouds in PCL type
  pcl::PointCloud<pcl::PointXYZ> vInputCloud;
  pcl::PointCloud<pcl::PointXYZ> vOutputCloud;

  std::cout<< FrameNum<<"  :  ";

  if(FrameNum>=FrameMin && FrameNum <=FrameMax)
  {
      pcl::fromROSMsg(vLaserData, vInputCloud);
      std::cout<<std::endl;
      std::cout<<"transform_matrix size "<<transform_matrix_list.size()<<"       now transform index: "<< FrameNum-300<<std::endl;
      std::cout<< std::scientific <<transform_matrix_list[FrameNum-FrameMin]<<std::endl;

      pcl::transformPointCloud(vInputCloud, vOutputCloud, transform_matrix_list[FrameNum-FrameMin]);


      // for(int i =0;i < vInputCloud.points.size();i++)
      // {
      //   testCloud.push_back(vInputCloud.points[i]);
      // }
      for(int i =0;i < vOutputCloud.points.size();i++)
      {
        testCloud.push_back(vOutputCloud.points[i]);
      }
      // // testCloud += vOutputCloud;
      pcl::toROSMsg(testCloud,testcloudData);

      testcloudData.header.frame_id = vLaserData.header.frame_id;
      testcloudData.header.seq = FrameNum;
      testcloudData.header.stamp = vLaserData.header.stamp;//timestmap is the same with the input
      testCloudPub.publish(testcloudData);


      pcl::toROSMsg(vOutputCloud,vTransposedData);

    //set frame and time
    vTransposedData.header.frame_id = vLaserData.header.frame_id;
    vTransposedData.header.seq = FrameNum;
    vTransposedData.header.stamp = vLaserData.header.stamp;//timestmap is the same with the input
    m_oLaserPub.publish(vTransposedData);
  }
  else{
    std::cout<<std::endl;
  }
FrameNum++;
}


/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &SLAMTrans::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a transposed odometry point to suit the special coordinate system 
Return: none
Others: none
*************************************************/
void GtTrans::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{

  //ROS_ERROR_STREAM("m_fOdomZOffset: "<< m_fOdomZOffset);

  nav_msgs::Odometry oTransposedOdom;
  //count input frames
  // m_iTrajPointNum++;

  //output in a txt file
  //
  oTransposedOdom.header.stamp = oTrajectory.header.stamp;
  // oTransposedOdom.header.frame_id = m_sOdomTargetFrame;
  // oTransposedOdom.child_frame_id = m_sOdomRawFrame;

  oTransposedOdom.pose.pose.position.x = oTrajectory.pose.pose.position.z; 
  oTransposedOdom.pose.pose.position.y = oTrajectory.pose.pose.position.x;
  // oTransposedOdom.pose.pose.position.z = oTrajectory.pose.pose.position.y - m_fOdomZOffset;
  oTransposedOdom.pose.pose.orientation.x = oTrajectory.pose.pose.orientation.z;
  oTransposedOdom.pose.pose.orientation.y = oTrajectory.pose.pose.orientation.x;
  oTransposedOdom.pose.pose.orientation.z = oTrajectory.pose.pose.orientation.y;
  oTransposedOdom.pose.pose.orientation.w = oTrajectory.pose.pose.orientation.w;

  oTransposedOdom.twist.twist.angular.x = oTrajectory.twist.twist.angular.x ;
  oTransposedOdom.twist.twist.angular.y = oTrajectory.twist.twist.angular.y ;
  oTransposedOdom.twist.twist.angular.z = oTrajectory.twist.twist.angular.z ;

  oTransposedOdom.twist.twist.linear.x = oTrajectory.twist.twist.angular.x ;
  oTransposedOdom.twist.twist.linear.y = oTrajectory.twist.twist.angular.y ;
  oTransposedOdom.twist.twist.linear.z = oTrajectory.twist.twist.angular.z ;

  // m_oOdomPub.publish(oTransposedOdom);

  //odom_lider
  nav_msgs::Odometry oCurrOdom = oTransposedOdom;
  //set the position
  // oCurrOdom.pose.pose.position.x = oTransposedOdom.pose.pose.position.x;
  // oCurrOdom.pose.pose.position.y = oTransposedOdom.pose.pose.position.y;
  oCurrOdom.pose.pose.position.z = oCurrOdom.pose.pose.position.z + 0.583;
  // oCurrOdom.header.stamp = oTransposedOdom.header.stamp;

}
