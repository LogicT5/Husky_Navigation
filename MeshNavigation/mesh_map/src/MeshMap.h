#ifndef MESHMAP_H
#define MESHMAP_H
#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>

//polygon related
#include <visualization_msgs/Marker.h>

//pcl related
#include "pcl_ros/transforms.h"  
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h> 
#include <pcl/search/kdtree.h> 

//vtk
#include <vtkSmartPointer.h>
#include <vtkFeatureEdges.h>
// #include <vtkPolyData.h>
// #include <vtkDiskSource.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkActor.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkNew.h>
#include <vtkExtractEdges.h>

//Eigen
#include <Eigen/Dense>

#include <octomap/octomap.h>
// #include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace mesh_map{

class MeshMap
{
private:

// typedef pcl::PointXYZ PlanPoint;
// typedef pcl::PointNormel PCLPoint;
// typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef octomap::ColorOcTree OcTreeT;

// Trajectory state data. 
typedef struct TrajectoryPoint{
    // The time of the measurement leading to this state (in seconds). //
    ros::Time oTimeStamp;

	//********angles***********
    float roll;    ///< The current roll angle. 
    float pitch;    ///< The current pitch angle.
    float yaw;   ///< The current yaw angle. 

	//********coordinate value****************
    // The global trajectory position in 3D space. 
    pcl::PointXYZ position;
} TrajectoryPoint;

//**********订阅Topic相关变量********
std::string lidarFrame; // 雷达系
std::string baselinkFrame; // 载体系
Eigen::Matrix4d Lidar2BaselinkTF4d; // 雷达系与载体的变换矩阵，作用与两系不同时
// input topics:
ros::Subscriber OdomSub;// odom订阅
ros::Subscriber CloudNormalsSub; // 单帧重建订阅

std::string sub_OdomTopic;  // Odom Topic id


//********发布相关Topic********
std::string pub_CloudFrame; // 发布点云坐标系
std::string pub_MapFrame;
std::string pub_GoalOdomFrame;

std::string pub_nPreseNodeTopic;
std::string pub_nPlanNodeTopic;
std::string pub_nPastNodeTopic;
std::string pub_nGoalNodeTopic;

ros::Publisher debug_CloudPub;

ros::Publisher n_PreseNodePub;
ros::Publisher n_PlanNodePub;// planning nodes publisher for display
ros::Publisher n_PastNodePub;// past nodes publisher for display
ros::Publisher n_GoalNodePub;// goal odometry information publisher

ros::Publisher m_binaryMapPub;
ros::Publisher m_fullMapPub;


//****************变量声明***************************
double MapVoxelsSize = 0.05;
OcTreeT *m_pMeshMapTree;

pcl::PointCloud<pcl::PointXYZ>::Ptr n_pPlanNodeCloud; // boundary point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr n_pPastNodeCloud; // boundary point clouds
pcl::PointXYZ n_GoalNode;// goal point of global path planning

pcl::PointCloud<pcl::PointNormal>::Ptr m_pGroundPN; // segmented ground plane
pcl::PointCloud<pcl::PointNormal>::Ptr m_pNongroundPN; // everything else
pcl::PointCloud<pcl::PointNormal>::Ptr m_pBoundPN;    // boundary point clouds

pcl::PointCloud<pcl::PointNormal>::Ptr n_pPreselectionCloud;

bool n_NextGoalNodeFlag;
// the positions of robot
std::queue<pcl::PointXYZ> m_vOdomViews; // I dont think it is necessary to use a circle vector
std::queue<pcl::PointXYZ> m_vOdomShocks;

public:
    MeshMap(ros::NodeHandle & node, ros::NodeHandle & nodeHandle);
    virtual ~MeshMap();
    
    //Reads and verifies the ROS parameters.
    bool ReadLaunchParams(ros::NodeHandle & nodeHandle);

    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

    void HandleMesh(const visualization_msgs::Marker &oMeshMsgs);
    void HandleCloudNormals(const sensor_msgs::PointCloud2 &oMeshMsgs);

    void MeshBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud);
    void MeshBoundary(pcl::PointCloud<pcl::PointNormal>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointNormal>::Ptr BoundaryCloud);
    double CosineSimilarity(pcl::PointNormal vPoint,pcl::PointCloud<pcl::PointNormal>::Ptr vCloud,std::vector<int> indices );

    void FilterPreselectionPoints(pcl::PointCloud<pcl::PointNormal>::Ptr pGroundPN);
    void FilterTargetPoint();

    void PublishFullOctoMap(octomap::ColorOcTree &m_octree);
    void PublishPreseNodeClouds();
    void PublishPlanNodeClouds();
    void PublishPastNodeClouds();
    void PublishGoalOdom(pcl::PointXYZ & oGoalPoint);


    void PublishPointCloud(ros::Publisher Pub,const pcl::PointCloud<pcl::PointXYZ> & vCloud);
    void PublishPointCloud(ros::Publisher Pub,const pcl::PointCloud<pcl::PointNormal> & vCloud);
    void PublishMesh(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices);

};
    
} 
#endif