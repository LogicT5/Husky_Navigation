#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
//pcl related
#include "pcl_ros/transforms.h"  
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h> 
#include <pcl/search/kdtree.h> 

//goal point of global path planning
pcl::PointXYZ m_oNodeGoal;
bool NextNodeFlag;
ros::Subscriber m_oOdomSuber;//the subscirber is to hear (record) odometry from gazeb
ros::Publisher m_oGoalPublisher;// goal odometry information publisher
std::vector<Eigen::Matrix4d > nodes_list;
int MaxNode = 0;
int nodeNUM = 0;

std::vector<Eigen::Matrix4d > ReadNodesFile(std::string path)
{
  std::vector<Eigen::Matrix4d > matrix_list;
  Eigen::Matrix4d matrix;
  std::ifstream file(path.c_str());
  std::string line;
  float x;
  while (getline(file, line)) {
    std::stringstream ss(line);
    for (int i = 0; i < 4; i++)
    {
      ss >> matrix(i / 4, i % 4);
    }
    matrix_list.push_back(matrix);
  }
  file.close();
  return matrix_list;
}

void PublishGoalOdom(pcl::PointXYZ & oGoalPoint){

        nav_msgs::Odometry oCurrGoalOdom;
        oCurrGoalOdom.header.stamp = ros::Time::now();
        oCurrGoalOdom.header.frame_id = "odom";

        //set the position
        oCurrGoalOdom.pose.pose.position.x = oGoalPoint.x;
        oCurrGoalOdom.pose.pose.position.y = oGoalPoint.y;
        oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z;
        // oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z + 0.583;
        m_oGoalPublisher.publish(oCurrGoalOdom);
}

void HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z

  if(nodeNUM < nodes_list.size())
  {
    if(abs(oOdomPoint.x - m_oNodeGoal.x) < 0.2)
        if(abs(oOdomPoint.y - m_oNodeGoal.y) < 0.2)
            if(abs(oOdomPoint.z - m_oNodeGoal.z) < 0.2+0.583)
            {
                std::cout<< nodeNUM <<" : Send a new point : X = "<<nodes_list[nodeNUM](0,0)<<"  Y = "<<nodes_list[nodeNUM](0,1)<<std::endl;
                m_oNodeGoal.x =  nodes_list[nodeNUM](0,0);
                m_oNodeGoal.y =  nodes_list[nodeNUM](0,1);
                m_oNodeGoal.z =  0.583;
                nodeNUM++;
            }
    //  std::cout<<"Pubilsh : "<<m_oNodeGoal<<std::endl;
     PublishGoalOdom(m_oNodeGoal);
  }

}

int main(int argc, char** argv){

  ros::init(argc, argv, "PublishNodes");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  //DebugTools Debug(node,privateNode);
  m_oOdomSuber = privateNode.subscribe("/odometry/filtered", 1, &HandleTrajectory);
  m_oGoalPublisher = privateNode.advertise<nav_msgs::Odometry>("goal_odom", 1, true);

  std::string NodesfilesPath;
  privateNode.param("NodesfilesPath", NodesfilesPath, std::string("/home/vcc/husky_ws/src/debug_tools/Nodes/Traj_1307.484000000.txt"));
  nodes_list = ReadNodesFile(NodesfilesPath);

  for(int i =0;i < nodes_list.size();i++)
  {
    std::cout<<i<<" : "<<nodes_list[i](0,0)<<" "<<nodes_list[i](0,1)<<std::endl;
  }

  m_oNodeGoal.x =  0;
  m_oNodeGoal.y =  0;
  m_oNodeGoal.z =  0.583;
  PublishGoalOdom(m_oNodeGoal);

  ros::spin();

  return 0;
}