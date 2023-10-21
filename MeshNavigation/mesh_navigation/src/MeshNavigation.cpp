#include "MeshNavigation.h"
#include "tools/ROSPublishMesh.h"
#include "tools/ROSPublishPoints.h"

MeshNavigation::MeshNavigation(ros::NodeHandle &node, ros::NodeHandle &nodeHandle) : 
                                                                                        n_pPlanNodeCloud(new pcl::PointCloud<NavPointType>),
                                                                                        n_pPastNodeCloud(new pcl::PointCloud<NavPointType>),
                                                                                        n_pPreselectionCloud(new pcl::PointCloud<NavPointType>),
                                                                                        m_pMeshMapTree(new OcTreeT(0.05)),
                                                                                        PathOcTree(new OcTreeT(0.05)),
                                                                                        m_pSingReconPN(new pcl::PointCloud<pcl::PointNormal>),
                                                                                        m_pGroundPN(new pcl::PointCloud<pcl::PointNormal>),
                                                                                        m_pNongroundPN(new pcl::PointCloud<pcl::PointNormal>),
                                                                                        m_pBoundPN(new pcl::PointCloud<pcl::PointNormal>),
                                                                                        n_NextGoalNodeFlag(true),
                                                                                        n_firstPathPlanFlag(true)
{
    srand((unsigned)time(NULL));

    // read parameters
    ReadLaunchParams(nodeHandle);
    RobotPose.position = pcl::PointXYZ(0.0,0.0,0.0);
    m_pMeshMapTree->setResolution(MapVoxelsSize);
    PathOcTree->setResolution(MapVoxelsSize);

    // o_AStarPlanner.setROSnodHandle(&nodeHandle);
    // subscribe (hear) the odometry information
    OdomSub = nodeHandle.subscribe(sub_OdomTopic, 1, &MeshNavigation::HandleTrajectory, this);

    SingleFrameMeshSub = nodeHandle.subscribe("/frame_meshs_FrameRecon",1,&MeshNavigation::HandleMesh,this);

    // CloudNormalsSub = nodeHandle.subscribe("/frame_cloudnormals", 1, &MeshNavigation::HandleCloudNormals, this);

    MultiFrameReconCli = nodeHandle.serviceClient<mesh_navigation::multi_recon>("/hash_fusion/GetMultiReconVoxelCloud");
    // m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &MeshNavigation::HandleTrajectory, this);
    // m_oGroundSuber = nodeHandle.subscribe(m_sGroundTopic, 1, &MeshNavigation::HandleGroundClouds, this);
    // m_oBoundSuber = nodeHandle.subscribe(m_sBoundTopic, 1, &MeshNavigation::HandleBoundClouds, this);
    // m_oObstacleSuber = nodeHandle.subscribe(m_sObstacleTopic, 1, &MeshNavigation::HandleObstacleClouds, this);

    debug_Node = &nodeHandle;
    debug_SingReconPCPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_cloud/SingReconPC", 1, true);
    debug_GroundPCPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_cloud/GroundPC", 1, true);
    debug_NonGroundPCPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_cloud/NonGroundPC", 1, true);
    debug_BoundPCPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_cloud/BoundPC", 1, true);
    debug_MultiReconPCPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_cloud/MultiReconPC", 1, true);

    // debug_SingleFrameMeshPub = nodeHandle.advertise<visualization_msgs::Marker>("debug_cloud/SingleFrameMesh", 1, true);
    debug_SingleFrameMeshPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("debug_cloud/SingleFrameMesh", 1, true);
    debug_SingleFramePNPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("debug_cloud/SingleFramePN", 1, true);
    debug_PathPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("debug_cloud/Path", 1, true);


    debug_CloudPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_clouds", 1, true);

    n_PreseNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("presenode_clouds", 1, true);
    n_PlanNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("plannode_clouds", 1, true);
    n_PastNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("pastnode_clouds", 1, true);
    n_GoalNodePub = nodeHandle.advertise<nav_msgs::Odometry>("goal_odom", 1, true);

    m_binaryMapPub = nodeHandle.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);
    m_fullMapPub = nodeHandle.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    m_PathOcTreePub = nodeHandle.advertise<octomap_msgs::Octomap>("PathOcTree_full", 1, true);

//         if (lidarFrame != baselinkFrame)
//         {
//             try
//             {
//                 tf::TransformListener tfListener;
//                 tf::StampedTransform lidar2Baselink; // 雷达系转为载体系
//                 tf::TransformBroadcaster tfOdom2BaseLinkOdomFrame;
//                 tf::TransformBroadcaster tfMap2BaseLinkMapFrame;
//                 // 等待3s
//                 tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(10.0));
//                 // lidar系到baselink系的变换
//                 tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);

//                 // tfOdom2BaseLinkOdomFrame.sendTransform(tf::StampedTransform(lidar2Baselink, ros::Time(0), "odom","base_link_odom"));
//                 // tfMap2BaseLinkMapFrame.sendTransform(tf::StampedTransform(lidar2Baselink, ros::Time(0), "odom","base_link_map"));

//                 // 获取位移和旋转分量
//                 tf::Vector3 translation = lidar2Baselink.inverse().getOrigin();
//                 tf::Quaternion rotation = lidar2Baselink.inverse().getRotation();

//                 // 将位移和旋转转换为Eigen类型
//                 Eigen::Vector3d translation_eigen(translation.x(), translation.y(), translation.z());
//                 Eigen::Quaterniond rotation_eigen(rotation.w(), rotation.x(), rotation.y(), rotation.z());

//                 // 创建变换矩阵
//                 Lidar2BaselinkTF4d.block<3, 3>(0, 0) = rotation_eigen.toRotationMatrix();
//                 Lidar2BaselinkTF4d.block<3, 1>(0, 3) = translation_eigen;
// ///*Debug
// #ifdef DEBUG
//                 std::cout << "\033[31mMeshNavigation DEBUG \033[1m\033[34mlidar2Baselink:";
//                 std::cout << "  Translation: " << lidar2Baselink.getOrigin().getX() << ", " << lidar2Baselink.getOrigin().getY() << ", " << lidar2Baselink.getOrigin().getZ();
//                 std::cout << "  Rotation: " << lidar2Baselink.getRotation().getW() << ", " << lidar2Baselink.getRotation().getX() << ", " << lidar2Baselink.getRotation().getY() << ", " << lidar2Baselink.getRotation().getZ();
//                 std::cout << std::endl;
//                 std::cout << Lidar2BaselinkTF4d << std::endl;
//                 std::cout << "\033[0m" << std::endl; //*/
// #endif
//             }
//             catch (tf::TransformException ex)
//             {
//                 ROS_ERROR("%s", ex.what());
//             }
//         }
}

MeshNavigation::~MeshNavigation(){}

bool MeshNavigation::ReadLaunchParams(ros::NodeHandle &nodeHandle)
{

    // input topic
    nodeHandle.param("odom_in_topic", sub_OdomTopic, std::string("/husky_lio_sam/mapping/odometry"));
    nodeHandle.param<std::string>("lidarFrame", lidarFrame, "base_link");
    nodeHandle.param<std::string>("baselinkFrame", baselinkFrame, "base_link");

    pub_CloudFrame = "odom";
    pub_MapFrame = "map";
    pub_GoalOdomFrame = "odom";
    

    return true;
}

void MeshNavigation::HandleTrajectory(const nav_msgs::Odometry &oTrajectory)
{
    RobotPose.oTimeStamp = oTrajectory.header.stamp;
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x; // z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y; // x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z; // y in loam is z
    RobotPose.position = oOdomPoint;

    // Eigen::Quaterniond quaternion( oTrajectory.pose.pose.orientation.w,
    //                                                     oTrajectory.pose.pose.orientation.x,
    //                                                     oTrajectory.pose.pose.orientation.y,
    //                                                     oTrajectory.pose.pose.orientation.z);
    // Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    // Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // 顺序是 ZYX
    // RobotPose.yaw = euler_angles[0];    // Yaw（偏航角）
    // RobotPose.pitch = euler_angles[1];  // Pitch（俯仰角）
    // RobotPose.roll = euler_angles[2];   // Roll（滚动角）

    // if (lidarFrame != baselinkFrame)
    // {
    //     Eigen::Affine3f Lidar2BaselinkTF3f;
    //     Lidar2BaselinkTF3f.matrix() = Lidar2BaselinkTF4d.cast<float>();
    //     pcl::transformPoint(oOdomPoint, Lidar2BaselinkTF3f);
    // }
    // pcl::transformPointCloud( *pFramePN, *pFramePN, Lidar2BaselinkTF4d);
    if (!n_NextGoalNodeFlag)
    {
        // std::cout<<"OdomPoint: "<<oOdomPoint<<"   GoalPoint: "<<n_GoalNode<<std::endl;
        if (abs(oOdomPoint.x - n_GoalNode.x) < 0.5)
            if (abs(oOdomPoint.y - n_GoalNode.y) < 0.5)
                if (abs(oOdomPoint.z - n_GoalNode.z) < 0.2 + 0.583)
                {
                    // n_pPastNodeCloud->push_back(n_pPlanNodeCloud->points.back());
                    // n_pPlanNodeCloud->clear();
                    // n_pPreselectionCloud->clear();
                    // PublishPastNodeClouds();
                    // PublishPreseNodeClouds();
                    // o_AStarPlanner.setStartNode(octomap::point3d(oOdomPoint.x, oOdomPoint.y, oOdomPoint.z));
                    n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
                }
    }
}

void MeshNavigation::HandleMesh(const mesh_navigation::FrameRecon & SingleMeshMsg)
{   
    MeshMap SingleFrameMeshMap;
    // std::cout << "SingleFrameMeshMap" << std::endl;
    // if(n_NextGoalNodeFlag)
    SingleFrameMeshMap.InitMeshMap(SingleMeshMsg);
    // SingleFrameMeshMap->InitMeshMap(SingleMeshMsg);
    // std::cout << "SingleFrameMeshMap Init Mesh Map" << std::endl;
    // pcl::PointCloud<pcl::PointNormal> CenterPNormal;
    // pcl::moveFromROSMsg(SingleMeshMsg.CenterPNormal,CenterPNormal);
    // ROSPublishMesh::ROSPublishMesh.PublishPolygonMeshAndPolygonNormal(*sFrameMesh,CenterPNormal,"odom", debug_SingleFrameMeshPub);
    // ROSPublishMesh::ROSPublishMesh.PublishPolygonMeshAndPolygonNormal(SingleFrameMeshMap->getPolygonMesh(),SingleFrameMeshMap->getPolygonCenterPNormal(),"odom", debug_SingleFrameMeshPub);
    // ROSPublishPoints::ROSPublishPoints.PublishPointCloudAndNormal(SingleFrameMeshMap.getSingleFramePNormal(),"odom",debug_SingleFramePNPub);
    SingleFrameMeshMap.PublishMeshMap("map",debug_SingleFrameMeshPub);
    // ROSPublishMesh::ROSPublishMesh.PublishPolygonMesh(*sFrameMesh, "odom", debug_SingleFrameMeshPub);



    // std::cout << "planning" << std::endl;
    // std::cout << "end" << std::endl;
    pcl::PointXYZ Position;
    Position = RobotPose.position;
    Position.z = 0.1561;
    if (n_NextGoalNodeFlag && sqrt((Position.x - (-10.051168)) * (Position.x - (-10.051168)) + (Position.y - (-2.031460)) * (Position.y - (-2.031460))) > 0.5)
    {
        // std::cout << "AStar Planner" << std::endl;
        AStarPlanner_meshmap::AStarPlanner o_AStarPlanner;
        // std::cout << "set mesh map" << std::endl;
        o_AStarPlanner.setMeshMap( SingleFrameMeshMap.meshmap);
        std::cout << "set start node" << std::endl;
        o_AStarPlanner.setStartNode(Position);
        // pcl::PointXYZ *endPoint(-10.051168, -2.031460, 0);
        std::cout << "set end node" << std::endl;
        o_AStarPlanner.setEndNode(pcl::PointXYZ(-10.051168, -2.031460, 0.1561));
        o_AStarPlanner.AStarPlanning();
        // if (o_AStarPlanner.Path.size() == 1)
        // {
        //     n_GoalNode.x = o_AStarPlanner.Path[0]->point.x;
        //     n_GoalNode.y = o_AStarPlanner.Path[0]->point.y;
        //     n_GoalNode.z = RobotPose.position.z;
        //     PublishGoalOdom(n_GoalNode);
        // }
        // else if (o_AStarPlanner.Path.size() > 0)
        // {
            // n_GoalNode.x = o_AStarPlanner.Path[o_AStarPlanner.Path.size() -1]->point.x;
            // n_GoalNode.y = o_AStarPlanner.Path[o_AStarPlanner.Path.size() -1]->point.y;
            // n_GoalNode.x = o_AStarPlanner.Path[0]->point.x;
            // n_GoalNode.y = o_AStarPlanner.Path[0]->point.y;
            // n_GoalNode.z = RobotPose.position.z;
            // PublishGoalOdom(n_GoalNode);
            // for(auto && PlanNode:o_AStarPlanner.Path)
            // {
            //     NavPointType PlanPoint;
            //     PlanPoint.x=PlanNode->point.x;
            //     PlanPoint.y=PlanNode->point.y;
            //     PlanPoint.z=PlanNode->point.z;
            //     n_pPlanNodeCloud->push_back(PlanPoint);
            // }
            // o_AStarPlanner.PublishPath("map",debug_PathPub);
        // }
        // else{
            // return;
        // }
         o_AStarPlanner.PublishPath("map",debug_PathPub);
        PublishPlanNodeClouds();
        std::cout << "new Goal Point : " << n_GoalNode << std::endl;
        PublishPointCloud(debug_GroundPCPub, o_AStarPlanner.getOpenSet());
        pcl::PointCloud<pcl::PointXYZ> debug_startandendPoint;
        debug_startandendPoint.push_back(RobotPose.position);
        debug_startandendPoint.push_back(pcl::PointXYZ(-10.051168, -2.031460, 0));
        PublishPointCloud(debug_MultiReconPCPub, debug_startandendPoint);

        n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
    }

    // output
    // std::stringstream sMeshOutputPath;
    // sMeshOutputPath << "/home/vcc/Dense_ROS/MeshNavigation/sf_output/sf_" << std::setw(4) << std::setfill('0') << m_iReconstructFrameNum << "_mesh.ply";
    // std::string mesh_path = sMeshOutputPath.str();
    // pcl::io::savePLYFileBinary(mesh_path, *sFrameMesh);
    // std::stringstream sPcOutputPath;
    // sPcOutputPath << "/home/vcc/Dense_ROS/MeshNavigation/sf_output/sf_" << std::setw(4) << std::setfill('0') << m_iReconstructFrameNum << "_pc.ply";
    // std::string Pc_path = sPcOutputPath.str();
    // pcl::io::savePLYFileBinary(Pc_path, CenterPNormal);
    // m_iReconstructFrameNum++;
}

double MeshNavigation::CosineSimilarity(pcl::PointNormal vPoint, pcl::PointCloud<pcl::PointNormal>::Ptr vCloud, std::vector<int> indices)
{
    Eigen::Vector3d vA, vB;
    double sumCosine = 0.0;
    vA << vPoint.normal_x, vPoint.normal_y, vPoint.normal_z;
    // return vA.dot(vB)/(vA.norm()*vB.norm())
    for (int i = 0; i < indices.size(); ++i)
    {
        vB << vCloud->points[i].normal_x, vCloud->points[i].normal_y, vCloud->points[i].normal_z;
        sumCosine += ((vA.dot(vB) / (vA.norm() * vB.norm())) + 1) / 2;
    }
    return sumCosine / indices.size();
}

void MeshNavigation::HandleCloudNormals(const sensor_msgs::PointCloud2 &oMeshMsgs)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(oMeshMsgs, *pFramePN);

    if (pFramePN->is_dense == false) // false 是填充过后的点云
    // if (pFramePN->is_dense == false && n_NextGoalNodeFlag)
    {
        m_pSingReconPN->clear();
        m_pGroundPN->clear();
        m_pNongroundPN->clear();
        m_pBoundPN->clear();
        m_pMeshMapTree->clear();
        if (lidarFrame != baselinkFrame)
            pcl::transformPointCloud(*pFramePN, *m_pSingReconPN, Lidar2BaselinkTF4d);

        std::vector<int> vCloudRes(m_pSingReconPN->points.size(), 0);
        for (int i; i < m_pSingReconPN->points.size(); i++)
        {
            pcl::PointNormal point = m_pSingReconPN->points[i];
            Eigen::Vector3d Vector_z(0, 0, 1);
            Eigen::Vector3d PointNormal(point.normal_x, point.normal_y, point.normal_z);

            if (point.z < 0.0)
            {
                if (abs(Vector_z.dot(PointNormal) - Vector_z.norm() * PointNormal.norm()) < 0.01)
                {
                    point.z = -0.1651;
                    // point.z = -0.1651 - m_pMeshMapTree->getResolution()/2 ;
                    vCloudRes[i] = 1;
                    m_pGroundPN->push_back(point);

                    updataMeshMap(point, PointType::GROUND);
                }
                else
                {

                    // m_pMeshMapTree->updateInnerOccupancy();
                }
            }
            else
            {
                if (abs(Vector_z.dot(PointNormal) - Vector_z.norm() * PointNormal.norm()) > 0.01)
                {
                    vCloudRes[i] = -1;
                    m_pNongroundPN->push_back(point);

                    updataMeshMap(point, PointType::NONGROUND);
                }
            }
        }

        PublishPointCloud(debug_SingReconPCPub, *m_pSingReconPN);
        // m_pMeshMapTree->updateInnerOccupancy();
        // PublishFullOctoMap(*m_pMeshMapTree);

        // TODO 路径检查
        //选择目标点，并规划路径
        if (n_NextGoalNodeFlag)
        {
            MeshBoundary(m_pSingReconPN, vCloudRes, m_pBoundPN);
            FilterPreselectionPoints(m_pGroundPN);
            if (FilterTargetPoint(m_pGroundPN))
            {
                AStarPlanner_octreemap::AStarPlanner o_AStarPlanner;
                PublishGoalOdom(n_GoalNode);
                PublishPlanNodeClouds();
                PublishPreseNodeClouds();
                PublishPointCloud(debug_GroundPCPub, *m_pGroundPN);
                PublishPointCloud(debug_NonGroundPCPub, *m_pNongroundPN);
                PublishPointCloud(debug_BoundPCPub, *m_pBoundPN);
                o_AStarPlanner.setOctomap(m_pMeshMapTree);
                // if (n_firstPathPlanFlag)
                // {
                o_AStarPlanner.setStartNode(octomap::point3d(RobotPose.position.x, RobotPose.position.y, 0));
                //     n_firstPathPlanFlag = !n_firstPathPlanFlag;
                // }
                
                o_AStarPlanner.setEndNode(octomap::point3d(-10.051168, -2.031460, 0));
                // o_AStarPlanner.setEndNode(octomap::point3d(n_GoalNode.x, n_GoalNode.y, 0));
                o_AStarPlanner.AStarPlanning();
                // o_AStarPlanner.getPath();
                m_pMeshMapTree->updateInnerOccupancy();
                PublishFullOctoMap(*m_pMeshMapTree);
                // pcl::PointCloud<pcl::PointXYZ> PathOcTreeP = o_AStarPlanner.getTreeNodeVoxel();
                // for(auto point:PathOcTreeP.points)
                // {
                //     PathOcTree->updateNode(point.x, point.y, point.z, true);
                //     PathOcTree->integrateNodeColor(point.x, point.y, point.z, 245, 222, 179);
                // }
                // PathOcTree->updateInnerOccupancy();
                //PublishFullOctoMap(m_PathOcTreePub,*PathOcTree);
                o_AStarPlanner.PublishAStarPathOcTree();
                n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
            }
        }
        // m_pMeshMapTree->updateInnerOccupancy();
        PublishFullOctoMap(*m_pMeshMapTree);
        // octomap::ColorOcTree *debugMap = o_AStarPlanner.getOctomap();
        // PublishFullOctoMap(*debugMap);
    }
}

void MeshNavigation::FilterPreselectionPoints(pcl::PointCloud<pcl::PointNormal>::Ptr pGroundPN)
{

    pcl::PointCloud<pcl::PointNormal>::Ptr pBoundaryPN(new pcl::PointCloud<pcl::PointNormal>);
    // MeshBoundary(pGroundPN,pBoundaryPN);
    pBoundaryPN = m_pBoundPN;

    if (n_pPastNodeCloud->points.size() > 0)
        for (auto pastPoint : n_pPastNodeCloud->points)
        {
            pcl::PointNormal point;
            point.x = pastPoint.x;
            point.y = pastPoint.y;
            point.z = pastPoint.z;
            point.normal_x = pastPoint.normal_x;
            point.normal_y = pastPoint.normal_y;
            point.normal_z = pastPoint.normal_z;
            pBoundaryPN->push_back(point);
        }
    pcl::UniformSampling<pcl::PointNormal> us;
    pcl::PointCloud<pcl::PointNormal>::Ptr pSamplingGroundCloud(new pcl::PointCloud<pcl::PointNormal>);
    us.setInputCloud(pGroundPN);
    us.setRadiusSearch(5.0f);
    us.filter(*pSamplingGroundCloud);

    pcl::search::KdTree<pcl::PointNormal> BoundTree;
    BoundTree.setInputCloud(pBoundaryPN);
    float max_dist = 4;
    for (int i = 0; i < pSamplingGroundCloud->points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        pcl::PointNormal point = pSamplingGroundCloud->points[i];
        // point.x = pSamplingGroundCloud->points[i].x;
        // point.y = pSamplingGroundCloud->points[i].y;
        // point.z = pSamplingGroundCloud->points[i].z;
        BoundTree.nearestKSearch(point, 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist)
        {
            double radius = sqrt(pow(point.x - RobotPose.position.x, 2) + pow(point.y - RobotPose.position.x, 2));
            if (radius > 1.5 && radius < 30)
            {
                // pMapCloud->points[i].z = pMapCloud->points[i].z+0.583;
                // pMapCloud->points[i].z = 0.583;
                NavPointType nPoint;
                nPoint.x = pSamplingGroundCloud->points[i].x;
                nPoint.y = pSamplingGroundCloud->points[i].y;
                nPoint.z = pSamplingGroundCloud->points[i].z;
                nPoint.normal_x = pSamplingGroundCloud->points[i].normal_x;
                nPoint.normal_y = pSamplingGroundCloud->points[i].normal_y;
                nPoint.normal_z = pSamplingGroundCloud->points[i].normal_z;
                nPoint.CosSimilarity = 0.0;
                nPoint.ExploreScore = 0.0;
                nPoint.DistanceWeight = radius / 100;
                n_pPreselectionCloud->push_back(nPoint);
            }
        }
    }
}

bool MeshNavigation::FilterTargetPoint(pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN)
{
    // 余弦相似性 计算周围法向量的一致性选点
    double CosSim = 0.0;
    NavPointType goalpoint;
    pcl::search::KdTree<pcl::PointNormal> MapTree;
    MapTree.setInputCloud(pFramePN);
    // std::cout << "FilterTargetPoint" <<std::endl;
    pcl::search::KdTree<pcl::PointXYZ> MultiReconTree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr MultiReconVoxelPoints(new pcl::PointCloud<pcl::PointXYZ>);
    CompareMultiReconVoxel(MultiReconVoxelPoints);
    if (MultiReconVoxelPoints->points.size() > 0)
        MultiReconTree.setInputCloud(MultiReconVoxelPoints);
    else
        return false;

    for (int i = 0; i < n_pPreselectionCloud->points.size(); ++i)
    {
        pcl::PointNormal pointnormel;
        pcl::PointXYZ pointxyz;
        pointxyz.x = pointnormel.x = n_pPreselectionCloud->points[i].x;
        pointxyz.y = pointnormel.y = n_pPreselectionCloud->points[i].y;
        pointxyz.z = pointnormel.z = n_pPreselectionCloud->points[i].z;
        pointnormel.normal_x = n_pPreselectionCloud->points[i].normal_x;
        pointnormel.normal_y = n_pPreselectionCloud->points[i].normal_y;
        pointnormel.normal_z = n_pPreselectionCloud->points[i].normal_z;

        int search_nums = 10;
        std::vector<int> neares_indices(search_nums);
        std::vector<float> neares_sqr_distances(search_nums);
        MapTree.nearestKSearchT(pointnormel, search_nums, neares_indices, neares_sqr_distances);
        double cs = CosineSimilarity(pointnormel, pFramePN, neares_indices);
        n_pPreselectionCloud->points[i].CosSimilarity = cs;

        if (MultiReconVoxelPoints->points.size() > 0)
        {
            MultiReconTree.setInputCloud(MultiReconVoxelPoints);
            float search_radius = 2.0; // 设置搜索半径
            std::vector<int> radius_indices;
            std::vector<float> radius_sqr_distances;
            MapTree.radiusSearchT(pointnormel, search_radius, neares_indices, neares_sqr_distances);
            MultiReconTree.radiusSearchT(pointxyz, search_radius, radius_indices, radius_sqr_distances);
            n_pPreselectionCloud->points[i].ExploreScore = (neares_indices.size() - radius_indices.size()) / neares_indices.size();
        }
        // n_pPreselectionCloud->points[i].intensity = (1 - n_pPreselectionCloud->points[i].ExploreScore) * n_pPreselectionCloud->points[i].CosSimilarity * ( 1 -  n_pPreselectionCloud->points[i].DistanceWeight);
        n_pPreselectionCloud->points[i].intensity = (1 - n_pPreselectionCloud->points[i].ExploreScore) * (1-n_pPreselectionCloud->points[i].CosSimilarity);

        if (n_pPreselectionCloud->points[i].intensity > CosSim)
        {
            CosSim = n_pPreselectionCloud->points[i].intensity;
            n_GoalNode.x = n_pPreselectionCloud->points[i].x;
            n_GoalNode.y = n_pPreselectionCloud->points[i].y;
            n_GoalNode.z = n_pPreselectionCloud->points[i].z + 0.583;
            goalpoint = n_pPreselectionCloud->points[i];
        }
    }
    n_pPlanNodeCloud->push_back(goalpoint);

    return true;
}

void MeshNavigation::CompareMultiReconVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr MultiReconVoxelPoints)
{
    mesh_navigation::multi_recon MultiReconVoxelPoints_srv;

    if (MultiFrameReconCli.call(MultiReconVoxelPoints_srv))
    {
        pcl::fromROSMsg(MultiReconVoxelPoints_srv.response.MultiRecon, *MultiReconVoxelPoints);
        if (lidarFrame != baselinkFrame)
            pcl::transformPointCloud(*MultiReconVoxelPoints, *MultiReconVoxelPoints, Lidar2BaselinkTF4d);
        PublishPointCloud(debug_MultiReconPCPub, *MultiReconVoxelPoints);
    }
    else
    {
        ROS_ERROR("MeshNavigation :  Compare Multi Reconstrction Voxel Error");
    }
}

void MeshNavigation::MeshBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud)
{
    std::vector<double> ver(2, 0);
    std::vector<std::vector<double>> Boundary;
    double max_theta = 0.0, min_theta = 0.0;

    for (int i = 0; i < verMeshCloud->points.size(); i++)
    {
        ver[0] = sqrt(pow(verMeshCloud->points[i].x, 2) + pow(verMeshCloud->points[i].y, 2));
        ver[1] = atan(verMeshCloud->points[i].y / verMeshCloud->points[i].x) * 180 / 3.14;
        Boundary.push_back(ver);
        if (ver[1] < min_theta)
            min_theta = ver[1];
        if (ver[1] > max_theta)
            max_theta = ver[1];
    }

    for (double i = min_theta; i < max_theta; i += 0.5)
    {
        double r = 0.0;
        int index = 0;
        for (int j = 0; j < Boundary.size(); j++)
        {
            if (Boundary[j][1] > i && Boundary[j][1] < i + 0.5)
            {
                if (Boundary[j][0] > r)
                {
                    r = Boundary[j][0];
                    index = j;
                }
            }
        }
        BoundaryCloud->push_back(verMeshCloud->points[index]);
    }
}

void MeshNavigation::MeshBoundary(pcl::PointCloud<pcl::PointNormal>::Ptr verMeshCloud, std::vector<int> vCloudRes, pcl::PointCloud<pcl::PointNormal>::Ptr BoundaryCloud)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pFrameP(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr pFrameN(new pcl::PointCloud<pcl::Normal>);

    for (auto pointPN : *verMeshCloud)
    {
        pcl::PointXYZ point;
        pcl::Normal normal;

        point.x = pointPN.x;
        point.y = pointPN.y;
        point.z = pointPN.z;
        pFrameP->push_back(point);

        normal.normal_x = pointPN.normal_x;
        normal.normal_y = pointPN.normal_y;
        normal.normal_z = pointPN.normal_z;
        pFrameN->push_back(normal);
    }
    // new a boundary class
    Boundary oBounder;
    // input the segment labels
    oBounder.GetSegmentClouds(pFrameP, vCloudRes);
    // compute boundary point
    oBounder.ComputeBoundary();
    // output the boundary cloud
    oBounder.OutputBoundClouds(vCloudRes);

    for (int i = 0; i != vCloudRes.size(); ++i)
    {
        if (vCloudRes[i] == 2)
        {
            BoundaryCloud->push_back(verMeshCloud->points[i]);
        }
        if (vCloudRes[i] == -1)
        {
            if (verMeshCloud->points[i].z < 1.2) // 高度小于husky的点，投影到地面
            {
                pcl::PointNormal point;
                point = verMeshCloud->points[i];
                point.z = 0;
                BoundaryCloud->push_back(point);
            }
        }
    }
}

void MeshNavigation::updataMeshMap(pcl::PointNormal point, PointType type)
{
    octomap::point3d voxel_origin(point.x, point.y, point.z);

    if (type == PointType::GROUND)
    {
        m_pMeshMapTree->updateNode(voxel_origin, true);
        m_pMeshMapTree->integrateNodeColor(point.x, point.y, point.z, 240, 255, 255);
        // octomap::ColorOcTreeNode* voxel_node = m_pMeshMapTree->search(voxel_origin);
        // if (voxel_node != nullptr && m_pMeshMapTree->isNodeOccupied(voxel_node)) {
        // // 设置自定义标记值
        //     voxel_node->setValue(PointType::GROUND);  // 这里的 42 是示例标记值，您可以使用任何您需要的值
        // }
    }
    else if (type == PointType::NONGROUND)
    {
        m_pMeshMapTree->updateNode(voxel_origin, true);
        m_pMeshMapTree->integrateNodeColor(point.x, point.y, point.z, 255, 165, 0);
        // octomap::ColorOcTreeNode* voxel_node = m_pMeshMapTree->search(voxel_origin);
        // if (voxel_node != nullptr && m_pMeshMapTree->isNodeOccupied(voxel_node)) {
        // // 设置自定义标记值
        //     voxel_node->setValue(PointType::NONGROUND);  // 这里的 42 是示例标记值，您可以使用任何您需要的值
        // }
    }
    else if (type == PointType::BOUND)
    {
        m_pMeshMapTree->updateNode(voxel_origin, true);
        m_pMeshMapTree->integrateNodeColor(point.x, point.y, point.z, 255, 165, 0);
        // octomap::ColorOcTreeNode* voxel_node = m_pMeshMapTree->search(voxel_origin);
        // if (voxel_node != nullptr && m_pMeshMapTree->isNodeOccupied(voxel_node)) {
        // // 设置自定义标记值
        //     voxel_node->setValue(PointType::BOUND);  // 这里的 42 是示例标记值，您可以使用任何您需要的值
        // }
    }
}

void MeshNavigation::PublishFullOctoMap(octomap::ColorOcTree &m_octree)
{
    octomap_msgs::Octomap map;
    map.header.frame_id = pub_MapFrame;
    map.header.stamp = ros::Time::now();

    if (octomap_msgs::fullMapToMsg(m_octree, map))
        m_fullMapPub.publish(map);
    else
        ROS_ERROR("Error serializing OctoMap");
}

void MeshNavigation::PublishFullOctoMap(ros::Publisher Pub,octomap::ColorOcTree &m_octree)
{
    octomap_msgs::Octomap map;
    map.header.frame_id = pub_MapFrame;
    map.header.stamp = ros::Time::now();

    if (octomap_msgs::fullMapToMsg(m_octree, map))
        Pub.publish(map);
    else
        ROS_ERROR("Error serializing OctoMap");
}

void MeshNavigation::PublishGoalOdom(pcl::PointXYZ &oGoalPoint)
{

    nav_msgs::Odometry oCurrGoalOdom;
    oCurrGoalOdom.header.stamp = ros::Time::now();
    oCurrGoalOdom.header.frame_id = pub_GoalOdomFrame;

    // set the position
    oCurrGoalOdom.pose.pose.position.x = oGoalPoint.x;
    oCurrGoalOdom.pose.pose.position.y = oGoalPoint.y;
    oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z;
    // oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z + 0.583;

    n_GoalNodePub.publish(oCurrGoalOdom);
}

void MeshNavigation::PublishPreseNodeClouds()
{
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPreselectionCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();

    n_PreseNodePub.publish(CloudData);
}

void MeshNavigation::PublishPlanNodeClouds()
{
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPlanNodeCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();

    n_PlanNodePub.publish(CloudData);
}

void MeshNavigation::PublishPastNodeClouds()
{
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPastNodeCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();
    n_PastNodePub.publish(CloudData);
}

void MeshNavigation::PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointXYZ> &Cloud)
{

    // publish obstacle points
    sensor_msgs::PointCloud2 CloudData;

    pcl::toROSMsg(Cloud, CloudData);

    CloudData.header.frame_id = pub_CloudFrame;

    CloudData.header.stamp = ros::Time::now();

    Pub.publish(CloudData);
}
void MeshNavigation::PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointXYZI> &Cloud)
{

    // publish obstacle points
    sensor_msgs::PointCloud2 CloudData;

    pcl::toROSMsg(Cloud, CloudData);

    CloudData.header.frame_id = pub_CloudFrame;

    CloudData.header.stamp = ros::Time::now();

    Pub.publish(CloudData);
}

void MeshNavigation::PublishPointCloud(ros::Publisher Pub, const pcl::PointCloud<pcl::PointNormal> &Cloud)
{

    // publish obstacle points
    sensor_msgs::PointCloud2 CloudData;

    pcl::toROSMsg(Cloud, CloudData);

    CloudData.header.frame_id = pub_CloudFrame;

    CloudData.header.stamp = ros::Time::now();

    Pub.publish(CloudData);
}

void MeshNavigation::PublishMesh(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices)
{
    ///*
        //new a visual message
        visualization_msgs::Marker oMeshMsgs;

        //define header of message
        oMeshMsgs.header.frame_id = baselinkFrame;
        oMeshMsgs.header.stamp = ros::Time::now();
        oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
        oMeshMsgs.action = visualization_msgs::Marker::ADD;

        oMeshMsgs.scale.x = 1.0;
        oMeshMsgs.scale.y = 1.0;
        oMeshMsgs.scale.z = 1.0;

        oMeshMsgs.pose.position.x = 0.0;
        oMeshMsgs.pose.position.y = 0.0;
        oMeshMsgs.pose.position.z = 0.0;

        oMeshMsgs.pose.orientation.x = 0.0;
        oMeshMsgs.pose.orientation.y = 0.0;
        oMeshMsgs.pose.orientation.z = 0.0;
        oMeshMsgs.pose.orientation.w = 1.0;

        std_msgs::ColorRGBA color;
        color.a = 1;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;

        //convert to publishable message
        for (int k = 0; k < vMeshVertices.points.size(); ++k){

            //temp point
            geometry_msgs::Point oPTemp;
            oPTemp.x = vMeshVertices.points[k].x;
            oPTemp.y = vMeshVertices.points[k].y;
            oPTemp.z = vMeshVertices.points[k].z;

            //color
            oMeshMsgs.points.push_back(oPTemp);
            oMeshMsgs.color = color;

        }//end k

        debug_SingleFrameMeshPub.publish(oMeshMsgs);
    //*/
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "husky_mesh_navigation");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");

    MeshNavigation MeshNavigation(node, privateNode);

    ros::spin();

    return 0;
}