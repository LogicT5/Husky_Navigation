#ifndef __MESH_NAVIGATION_H__
#define __MESH_NAVIGATION_H__
#include <string>
#include <ctime>

// ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>

// polygon related
#include <visualization_msgs/Marker.h>

// pcl related
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/boundary.h>

#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
// vtk
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

// Eigen
#include <Eigen/Dense>

#include <octomap/octomap.h>
// #include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "Boundary/Boundary.h"
#include "PathPlanner/AStar_octreemap.h"
#include "PathPlanner/AStar_meshmap.h"
#include "MeshMap.h"

//srv
#include "mesh_navigation/multi_recon.h"

//msg
#include "mesh_navigation/FrameRecon.h"

namespace pcl
{
    struct NavigationPoint
    {
        PCL_ADD_POINT4D;   // xyz + intensity
        PCL_ADD_INTENSITY; //
        PCL_ADD_NORMAL4D;  // normal + curvature
        float CosSimilarity;
        float ExploreScore;
        float DistanceWeight;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::NavigationPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(float, intensity, intensity)(float, CosSimilarity, CosSimilarity)(float, ExploreScore, ExploreScore)(float, DistanceWeight, DistanceWeight))

class MeshNavigation
{
private:
    // typedef pcl::PointXYZ PlanPoint;
    // typedef pcl::PointNormel PCLPoint;
    // typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
    typedef octomap::ColorOcTree OcTreeT;
    typedef pcl::NavigationPoint NavPointType;

    enum PointType
    {
        GROUND = 0,
        NONGROUND,
        BOUND
    };
    // Trajectory state data.
    typedef struct TrajectoryPoint
    {
        // The time of the measurement leading to this state (in seconds). //
        ros::Time oTimeStamp;

        //********angles***********
        double roll;  ///< The current roll angle.
        double pitch; ///< The current pitch angle.
        double yaw;   ///< The current yaw angle.

        //********coordinate value****************
        // The global trajectory position in 3D space.
        pcl::PointXYZ position;
    } TrajectoryPoint;

    MeshNavigation::TrajectoryPoint RobotPose;

    //**********订阅Topic相关变量********
    std::string lidarFrame;             // 雷达系
    std::string baselinkFrame;          // 载体系
    Eigen::Matrix4d Lidar2BaselinkTF4d; // 雷达系与载体的变换矩阵，作用与两系不同时

    // subscrib topics:
    ros::Subscriber OdomSub;         // odom订阅
    ros::Subscriber CloudNormalsSub; // 单帧重建订阅
    ros::Subscriber SingleFrameMeshSub; // 单帧重建订阅
    ros::ServiceClient MultiFrameReconCli;

    std::string sub_OdomTopic; // Odom Topic id

    //********发布相关Topic********
    std::string pub_CloudFrame; // 发布点云坐标系
    std::string pub_SingleFrameMesh;
    std::string pub_MapFrame;
    std::string pub_GoalOdomFrame;

    std::string pub_nPreseNodeTopic;
    std::string pub_nPlanNodeTopic;
    std::string pub_nPastNodeTopic;
    std::string pub_nGoalNodeTopic;

    ros::Publisher n_PreseNodePub;
    ros::Publisher n_PlanNodePub; // planning nodes publisher for display
    ros::Publisher n_PastNodePub; // past nodes publisher for display
    ros::Publisher n_GoalNodePub; // goal odometry information publisher

    ros::Publisher m_binaryMapPub;
    ros::Publisher m_fullMapPub;

    ros::Publisher m_PathOcTreePub;

    tf::TransformBroadcaster tfOdom2BaseLinkOdomFrame;
    tf::TransformBroadcaster tfMap2BaseLinkMapFrame;

    //DEBUG
    ros::NodeHandle *debug_Node;
    ros::Publisher debug_CloudPub;
    ros::Publisher debug_SingReconPCPub;
    ros::Publisher debug_GroundPCPub;
    ros::Publisher debug_NonGroundPCPub;
    ros::Publisher debug_BoundPCPub;
    ros::Publisher debug_MultiReconPCPub;
    ros::Publisher debug_SingleFrameMeshPub;
    ros::Publisher debug_SingleFramePNPub;
    ros::Publisher debug_PathPub;
    int m_iReconstructFrameNum = 0;

    //****************变量声明***************************
    //octree
    double MapVoxelsSize = 0.25;
    OcTreeT *m_pMeshMapTree;
    OcTreeT *PathOcTree;

    //MeshMap
    // MeshMap::Ptr SingleFrameMeshMap;
    // MeshMap test;

    // plan node
    pcl::PointCloud<NavPointType>::Ptr n_pPreselectionCloud;
    pcl::PointCloud<NavPointType>::Ptr n_pPlanNodeCloud; // boundary point clouds
    pcl::PointCloud<NavPointType>::Ptr n_pPastNodeCloud; // boundary point clouds
    pcl::PointXYZ n_GoalNode;                            // goal point of global path planning

    pcl::PointCloud<pcl::PointNormal>::Ptr m_pSingReconPN;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_pGroundPN;    // segmented ground plane
    pcl::PointCloud<pcl::PointNormal>::Ptr m_pNongroundPN; // everything else
    pcl::PointCloud<pcl::PointNormal>::Ptr m_pBoundPN;     // boundary point clouds

    bool n_firstPathPlanFlag;
    bool n_NextGoalNodeFlag;
    // the positions of robot
    std::queue<pcl::PointXYZ> m_vOdomViews; // I dont think it is necessary to use a circle vector
    std::queue<pcl::PointXYZ> m_vOdomShocks;

public:
    MeshNavigation(ros::NodeHandle &node, ros::NodeHandle &nodeHandle);
    ~MeshNavigation();

    // Reads and verifies the ROS parameters.
    bool ReadLaunchParams(ros::NodeHandle &nodeHandle);

    void HandleTrajectory(const nav_msgs::Odometry &oTrajectory);

    void HandleMesh(const mesh_navigation::FrameRecon & SingleMeshMsg);
    void HandleCloudNormals(const sensor_msgs::PointCloud2 &oMeshMsgs);

    void CompareMultiReconVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr MultiReconVoxelPoints);

    void MeshBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud);
    // void MeshBoundary(pcl::PointCloud<pcl::PointNormal>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointNormal>::Ptr BoundaryCloud);
    void MeshBoundary(pcl::PointCloud<pcl::PointNormal>::Ptr verMeshCloud, std::vector<int> vCloudRes, pcl::PointCloud<pcl::PointNormal>::Ptr BoundaryCloud);
    double CosineSimilarity(pcl::PointNormal vPoint, pcl::PointCloud<pcl::PointNormal>::Ptr vCloud, std::vector<int> indices);

    void FilterPreselectionPoints(pcl::PointCloud<pcl::PointNormal>::Ptr pGroundPN);
    bool FilterTargetPoint(pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN);

    void updataMeshMap(pcl::PointNormal point, PointType type);

    void PublishFullOctoMap(octomap::ColorOcTree &m_octree);
    void PublishFullOctoMap(ros::Publisher Pub, octomap::ColorOcTree &m_octree);
    void PublishPreseNodeClouds();
    void PublishPlanNodeClouds();
    void PublishPastNodeClouds();
    void PublishGoalOdom(pcl::PointXYZ &oGoalPoint);

    void PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointXYZ> &vCloud);
    void PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointXYZI> &vCloud);
    void PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointNormal> &vCloud);
    void PublishMesh(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices);
};

#endif