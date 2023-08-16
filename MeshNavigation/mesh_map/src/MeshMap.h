#ifndef MESHMAP_H
#define MESHMAP_H
#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>


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

namespace mesh_map{

class MeshMap
{
private:

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

//input topics:
ros::Subscriber m_oOdomSuber;//the subscirber is to hear (record) odometry from gazebo
ros::Subscriber m_sMeshSuber;// the subscirber is to hea frame reconstruction from frame_reconstruction nod
ros::Subscriber m_sCloudNormalsSuber;

std::string m_sOdomTopic;  //the topic name of targeted odometry (robot trajectory)
std::string m_sMeshTopic;//the topic name of targeted frame_reconstruction


//**output topics related**
std::string m_sOutCloudTFId;

std::string m_oTestCloudTopic;
std::string m_oOutMeshTopic;

std::string m_oPlanNodeTopic;
std::string m_oPastNodeTopic;
std::string m_oGoalNodeTopic;

ros::Publisher m_oTestCloudPublisher;
ros::Publisher m_oMeshPublisher;

ros::Publisher m_oPlanNodePublisher;// planning nodes publisher for display
ros::Publisher m_oPastNodePublisher;// past nodes publisher for display
ros::Publisher m_oGoalPublisher;// goal odometry information publisher

//goal point of global path planning
pcl::PointXYZ m_oNodeGoal;

pcl::PointCloud<pcl::PointXYZ>::Ptr m_pBoundCloud;//boundary point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr m_pPlanNodeCloud;//boundary point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr m_pPastNodeCloud;//boundary point clouds

pcl::PointCloud<pcl::PointNormal>::Ptr  PlanCloud;

//the positions of robot
std::queue<pcl::PointXYZ> m_vOdomViews;//I dont think it is necessary to use a circle vector
std::queue<pcl::PointXYZ> m_vOdomShocks;

double VoxelsSize = 0.05;
octomap::ColorOcTree *MeshTree;

bool NextNodeFlag;

public:
    MeshMap(ros::NodeHandle & node, ros::NodeHandle & nodeHandle);
    virtual ~MeshMap();
    
    //Reads and verifies the ROS parameters.
    bool ReadLaunchParams(ros::NodeHandle & nodeHandle);

    void HandleTrajectory(const nav_msgs::Odometry & oTrajectory);

    void HandleMesh(const visualization_msgs::Marker &oMeshMsgs);
    void HandleCloudNormals(const sensor_msgs::PointCloud2 &oMeshMsgs);

    void MeshBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud);
    double CosineSimilarity(pcl::PointNormal vPoint,pcl::PointCloud<pcl::PointNormal>::Ptr vCloud,std::vector<int> indices );


    void PublishPlanNodeClouds();
    void PublishPastNodeClouds();
    void PublishGoalOdom(pcl::PointXYZ & oGoalPoint);


    void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud);
    void PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloud);
    void PublishMeshs(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices);

};
    
}
#endif