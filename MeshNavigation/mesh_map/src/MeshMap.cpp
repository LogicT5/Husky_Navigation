#include"MeshMap.h"

namespace mesh_map{
MeshMap::MeshMap(ros::NodeHandle & node, ros::NodeHandle & nodeHandle):
                n_pPlanNodeCloud(new pcl::PointCloud<pcl::PointXYZ>),
                n_pPastNodeCloud(new pcl::PointCloud<pcl::PointXYZ>),
                n_pPreselectionCloud(new pcl::PointCloud<pcl::PointNormal>),
                m_pMeshMapTree(new OcTreeT(0.05)),
                m_pGroundPN(new pcl::PointCloud<pcl::PointNormal>),
                m_pNongroundPN(new pcl::PointCloud<pcl::PointNormal>),
                m_pBoundPN(new pcl::PointCloud<pcl::PointNormal>),
                n_NextGoalNodeFlag(true)
{
    srand((unsigned)time(NULL));

	//read parameters
	ReadLaunchParams(nodeHandle);
    m_pMeshMapTree->setResolution(MapVoxelsSize);

    //subscribe (hear) the odometry information
	OdomSub = nodeHandle.subscribe(sub_OdomTopic, 1, &MeshMap::HandleTrajectory, this);
    	
    // m_sMeshSuber = nodeHandle.subscribe(m_sMeshTopic,1,&MeshMap::HandleMesh,this);

    CloudNormalsSub = nodeHandle.subscribe("/frame_cloudnormals",1,&MeshMap::HandleCloudNormals,this);
    

    // m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &MeshMap::HandleTrajectory, this);
	// m_oGroundSuber = nodeHandle.subscribe(m_sGroundTopic, 1, &MeshMap::HandleGroundClouds, this);
	// m_oBoundSuber = nodeHandle.subscribe(m_sBoundTopic, 1, &MeshMap::HandleBoundClouds, this);
	// m_oObstacleSuber = nodeHandle.subscribe(m_sObstacleTopic, 1, &MeshMap::HandleObstacleClouds, this);

    debug_CloudPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("debug_clouds", 1, true);
    n_PreseNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("presenode_clouds",1,true);
    n_PlanNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("plannode_clouds", 1, true);
    n_PastNodePub = nodeHandle.advertise<sensor_msgs::PointCloud2>("pastnode_clouds", 1, true);
    n_GoalNodePub = nodeHandle.advertise<nav_msgs::Odometry>("goal_odom", 1, true);

    m_binaryMapPub = nodeHandle.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);
    m_fullMapPub = nodeHandle.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);

    if(lidarFrame != baselinkFrame)
    {
        try
        {
            tf::TransformListener tfListener;
            tf::StampedTransform lidar2Baselink; // 雷达系转为载体系
            tf::TransformBroadcaster tfOdom2BaseLinkOdomFrame;
            tf::TransformBroadcaster tfMap2BaseLinkMapFrame;
            // 等待3s
            tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(10.0));
            // lidar系到baselink系的变换
            tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);

            // tfOdom2BaseLinkOdomFrame.sendTransform(tf::StampedTransform(lidar2Baselink, ros::Time(0), "odom","base_link_odom"));
            // tfMap2BaseLinkMapFrame.sendTransform(tf::StampedTransform(lidar2Baselink, ros::Time(0), "odom","base_link_map"));       

            // 获取位移和旋转分量
            tf::Vector3 translation =lidar2Baselink.inverse().getOrigin();
            tf::Quaternion rotation = lidar2Baselink.inverse().getRotation();

            // 将位移和旋转转换为Eigen类型
            Eigen::Vector3d translation_eigen(translation.x(), translation.y(), translation.z());
            Eigen::Quaterniond rotation_eigen(rotation.w(), rotation.x(), rotation.y(), rotation.z());

            // 创建变换矩阵
            Lidar2BaselinkTF4d.block<3, 3>(0, 0) = rotation_eigen.toRotationMatrix();
            Lidar2BaselinkTF4d.block<3, 1>(0, 3) = translation_eigen;
            ///*Debug
            #ifdef DEBUG
                std::cout<<"\033[31mMeshNavigation DEBUG \033[1m\033[34mlidar2Baselink:";
                std::cout << "  Translation: " << lidar2Baselink.getOrigin().getX() << ", " << lidar2Baselink.getOrigin().getY() << ", " << lidar2Baselink.getOrigin().getZ();
                std::cout << "  Rotation: " << lidar2Baselink.getRotation().getW()<< ", " << lidar2Baselink.getRotation().getX() << ", " << lidar2Baselink.getRotation().getY() << ", " << lidar2Baselink.getRotation().getZ();
                std::cout <<std::endl;
                std::cout <<Lidar2BaselinkTF4d<<std::endl;
                std::cout << "\033[0m"<<std::endl;//*/
            #endif
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }
}

MeshMap::~MeshMap()
{}

bool MeshMap::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

    //input topic
    nodeHandle.param("odom_in_topic", sub_OdomTopic, std::string("/husky_lio_sam/mapping/odometry"));
    nodeHandle.param<std::string>("lidarFrame", lidarFrame, "base_link");
    nodeHandle.param<std::string>("baselinkFrame", baselinkFrame, "base_link");
    
    if(lidarFrame != baselinkFrame)
    {
        pub_CloudFrame = "base_link_odom";
        pub_MapFrame = "base_link_map";
        pub_GoalOdomFrame = "base_link_odom"; 
    }
    else
    {
        pub_CloudFrame = "odom";
        pub_MapFrame = "map";
        pub_GoalOdomFrame = "odom";
    }

    return true;
}

void MeshMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x; // z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z

    if(lidarFrame != baselinkFrame)
    {
        Eigen::Affine3f Lidar2BaselinkTF3f;
        Lidar2BaselinkTF3f.matrix() = Lidar2BaselinkTF4d.cast<float>();
        pcl::transformPoint(oOdomPoint, Lidar2BaselinkTF3f);
    }
    // pcl::transformPointCloud( *pFramePN, *pFramePN, Lidar2BaselinkTF4d);
    if(!n_NextGoalNodeFlag)
    {
        // std::cout<<"OdomPoint: "<<oOdomPoint<<"   GoalPoint: "<<n_GoalNode<<std::endl;
        if(abs(oOdomPoint.x - n_GoalNode.x) < 0.2)
            if(abs(oOdomPoint.y - n_GoalNode.y) < 0.2)
                if(abs(oOdomPoint.z - n_GoalNode.z) < 0.2+0.583)
                {
                    n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
                    n_pPastNodeCloud->push_back(n_pPlanNodeCloud->points.back());
                    n_pPlanNodeCloud->clear();
                    n_pPreselectionCloud->clear();
                    PublishPastNodeClouds();
                }
    }
}

void MeshMap::HandleMesh(const visualization_msgs::Marker &oMeshMsgs)
{/*
    if(n_NextGoalNodeFlag)
    {
        n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
        pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointNormal>::Ptr pMapCloud(new pcl::PointCloud<pcl::PointNormal>);

        // std::cout<<oMeshMsgs.header.frame_id<<std::endl;

        for (int k = 0; k < oMeshMsgs.points.size(); k += 3){
            int a=k,b=k+1,c=k+2;
            if( oMeshMsgs.points[k].z < 0.1651 && oMeshMsgs.points[k+1].z < 0.1651 && oMeshMsgs.points[k+2].z < 0.1651)
            {
                Eigen::Vector3d v1(oMeshMsgs.points[b].x - oMeshMsgs.points[a].x,oMeshMsgs.points[b].y - oMeshMsgs.points[a].y,oMeshMsgs.points[b].z - oMeshMsgs.points[a].z);
                Eigen::Vector3d v2(oMeshMsgs.points[c].x - oMeshMsgs.points[a].x,oMeshMsgs.points[c].y - oMeshMsgs.points[a].y,oMeshMsgs.points[c].z - oMeshMsgs.points[a].z);
                Eigen::Vector3d v = v1.cross(v2);
                Eigen::Vector3d v0(0,0,1);

                if(v.dot(v0) > 0.01 && v.dot(v0) < 0.1 )
                {
                    // std::cout<< v.dot(v0)  <<std::endl;
                    std::vector<uint32_t> pVertices;
                    pcl::PointXYZ aVertex;
                    pcl::PointXYZ bVertex;
                    pcl::PointXYZ cVertex;

                    aVertex.x = oMeshMsgs.points[a].x;
                    aVertex.y = oMeshMsgs.points[a].y;
                    aVertex.z = oMeshMsgs.points[a].z;
                    // aVertex.z = 0;
                    verMeshCloud->push_back(aVertex);

                    bVertex.x = oMeshMsgs.points[b].x;
                    bVertex.y = oMeshMsgs.points[b].y;
                    bVertex.z = oMeshMsgs.points[b].z;
                    // bVertex.z = 0;
                    verMeshCloud->push_back(bVertex);

                    cVertex.x = oMeshMsgs.points[c].x;
                    cVertex.y = oMeshMsgs.points[c].y;
                    cVertex.z = oMeshMsgs.points[c].z;
                    // cVertex.z  = 0;
                    verMeshCloud->push_back(cVertex);

                    pcl::PointNormal mCentrePoint;
                    mCentrePoint.x = (oMeshMsgs.points[a].x + oMeshMsgs.points[b].x + oMeshMsgs.points[c].x)/3;
                    mCentrePoint.y = (oMeshMsgs.points[a].y + oMeshMsgs.points[b].y + oMeshMsgs.points[c].y)/3;
                    mCentrePoint.z = (oMeshMsgs.points[a].z + oMeshMsgs.points[b].z + oMeshMsgs.points[c].z)/3;
                    // mCentrePoint.z = 0;
                    mCentrePoint.normal_x = v(0);
                    mCentrePoint.normal_y = v(1);
                    mCentrePoint.normal_z = v(2);

                    pMapCloud->push_back(mCentrePoint);
                }

            }// end if
        }//end k
        
        // 目标点进行采样
        pcl::UniformSampling<pcl::PointNormal> us;
        pcl::PointCloud<pcl::PointNormal>::Ptr pSamplingMapCloud(new pcl::PointCloud<pcl::PointNormal>);
        us.setInputCloud(pMapCloud );
        us.setRadiusSearch(3.0f);
        us.filter(*pSamplingMapCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr  vBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
        MeshBoundary(verMeshCloud,vBoundaryCloud);

        pcl::search::KdTree<pcl::PointXYZ> BoundTree;
        BoundTree.setInputCloud(vBoundaryCloud);
        float max_dist = 2.5;
        for(int i =0 ;i <pSamplingMapCloud->points.size();++i)
        {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);

            pcl::PointXYZ point;
            point.x = pSamplingMapCloud->points[i].x;
            point.y = pSamplingMapCloud->points[i].y;
            point.z = pSamplingMapCloud->points[i].z;
            BoundTree.nearestKSearch(point, 1, indices, sqr_distances);
            if (sqr_distances[0] > max_dist )
                if(sqrt(pow(point.x,2)+pow(point.y,2)) > 3)
                {
                    // pMapCloud->points[i].z = pMapCloud->points[i].z+0.583;
                    // pMapCloud->points[i].z = 0.583;
                    PlanCloud->push_back(pSamplingMapCloud->points[i]);
                }
        }
        // std::cout<<"n_NextGoalNodeFlag: "<<n_NextGoalNodeFlag<<std::endl;

        //余弦相似性 计算周围法向量的一致性选点
        double CosSim = 0.0;
        pcl::search::KdTree<pcl::PointNormal> pMapTree;
        pMapTree.setInputCloud(pMapCloud);
        for(int i =0 ;i <PlanCloud->points.size();++i)
        {
            std::vector<int> indices(10);
            std::vector<float> sqr_distances(10);

            pMapTree.nearestKSearchT(PlanCloud->points[i],10,indices,sqr_distances);
            double cs = CosineSimilarity(PlanCloud->points[i],pMapCloud,indices);
            if (cs > CosSim)
            {
                CosSim = cs;
                m_oNodeGoal.x = PlanCloud->points[i].x;
                m_oNodeGoal.y = PlanCloud->points[i].y;
                m_oNodeGoal.z = PlanCloud->points[i].z+0.583;
            }
        }


        // m_oNodeGoal .z = m_oNodeGoal.z +0.583;

        m_pPlanNodeCloud->push_back(m_oNodeGoal);

        // PublishPointCloud(*pSamplingMapCloud);
        PublishPointCloud(*PlanCloud);
        // PublishPointCloud(*pMapCloud);
        // PublishPointCloud(*vBoundaryCloud);
        PublishMeshs(*verMeshCloud);
        PublishPlanNodeClouds();
        PublishGoalOdom(m_oNodeGoal);

        PlanCloud->clear();
    }//*/
}

double MeshMap::CosineSimilarity(pcl::PointNormal vPoint,pcl::PointCloud<pcl::PointNormal>::Ptr vCloud,std::vector<int> indices ){
    Eigen::Vector3d vA, vB;
    double sumCosine=0.0;
    vA << vPoint.normal_x,vPoint.normal_y,vPoint.normal_z;
    // return vA.dot(vB)/(vA.norm()*vB.norm())
    for(int i = 0;i<indices.size();++i)
    {
        vB<<vCloud->points[i].normal_x,vCloud->points[i].normal_y,vCloud->points[i].normal_z;
        sumCosine += vA.dot(vB)/(vA.norm()*vB.norm());
    }
    return sumCosine/indices.size();
}

void MeshMap::HandleCloudNormals(const sensor_msgs::PointCloud2 &oMeshMsgs)
{

    //a point clouds in PCL type
    pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(oMeshMsgs, *pFramePN);
    if(lidarFrame != baselinkFrame)
        pcl::transformPointCloud( *pFramePN, *pFramePN, Lidar2BaselinkTF4d); 

    for(auto point:*pFramePN)
    {
        Eigen::Vector3d Vector_z(0,0,1);
        Eigen::Vector3d PointNormal(point.normal_x, point.normal_y, point.normal_z );

        
        
        if(point.z < 0.005)
        {   
            if(abs(Vector_z.dot(PointNormal) - Vector_z.norm() * PointNormal.norm()) < 0.01)
            {
                m_pGroundPN->push_back(point);

                m_pMeshMapTree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
                m_pMeshMapTree->integrateNodeColor(point.x,point.y,point.z,240,255,255);
                // 更新octomap
                // m_pMeshMapTree->updateInnerOccupancy();
            }  
            else
            {
                
                // m_pMeshMapTree->updateInnerOccupancy();
            }
        }
        else
        {
            if(abs(Vector_z.dot(PointNormal) - Vector_z.norm() * PointNormal.norm()) > 0.01)
            {
                m_pNongroundPN->push_back(point);

                m_pMeshMapTree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
                m_pMeshMapTree->integrateNodeColor(point.x,point.y,point.z,255,165,0);
            }
        }
    }

    m_pMeshMapTree->updateInnerOccupancy();
    PublishFullOctoMap(*m_pMeshMapTree);
    if(n_NextGoalNodeFlag)
    {
        n_NextGoalNodeFlag = !n_NextGoalNodeFlag;
        MeshBoundary(pFramePN,m_pBoundPN);
        FilterPreselectionPoints(m_pGroundPN);
        FilterTargetPoint();
        PublishGoalOdom(n_GoalNode);
    }
    
    PublishPointCloud(debug_CloudPub,*m_pBoundPN);
    PublishPlanNodeClouds();
    PublishPreseNodeClouds();
    
}

void MeshMap::FilterPreselectionPoints(pcl::PointCloud<pcl::PointNormal>::Ptr pGroundPN)
{

    pcl::PointCloud<pcl::PointNormal>::Ptr pBoundaryPN(new pcl::PointCloud<pcl::PointNormal>);
    // MeshBoundary(pGroundPN,pBoundaryPN);
    pBoundaryPN = m_pBoundPN;

    pcl::UniformSampling<pcl::PointNormal> us;
    pcl::PointCloud<pcl::PointNormal>::Ptr pSamplingGroundCloud(new pcl::PointCloud<pcl::PointNormal>);
    us.setInputCloud(pGroundPN );
    us.setRadiusSearch(3.0f);
    us.filter(*pSamplingGroundCloud);

    pcl::search::KdTree<pcl::PointNormal> BoundTree;
    BoundTree.setInputCloud(pBoundaryPN);
    float max_dist = 2.5;
    for(int i =0 ;i <pSamplingGroundCloud->points.size();++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        pcl::PointNormal point = pSamplingGroundCloud->points[i];
        // point.x = pSamplingGroundCloud->points[i].x;
        // point.y = pSamplingGroundCloud->points[i].y;
        // point.z = pSamplingGroundCloud->points[i].z;
        BoundTree.nearestKSearch(point, 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist )
            if(sqrt(pow(point.x,2)+pow(point.y,2)) > 3)
            {
                // pMapCloud->points[i].z = pMapCloud->points[i].z+0.583;
                // pMapCloud->points[i].z = 0.583;
                n_pPreselectionCloud->push_back(pSamplingGroundCloud->points[i]);
            }
    }
}

void MeshMap::FilterTargetPoint()
{
        //余弦相似性 计算周围法向量的一致性选点
        double CosSim = 0.0;
        pcl::search::KdTree<pcl::PointNormal> MapTree;
        MapTree.setInputCloud(m_pGroundPN);
        for(int i =0 ;i <n_pPreselectionCloud->points.size();++i)
        {
            std::vector<int> indices(10);
            std::vector<float> sqr_distances(10);

            MapTree.nearestKSearchT(n_pPreselectionCloud->points[i],10,indices,sqr_distances);
            double cs = CosineSimilarity(n_pPreselectionCloud->points[i],n_pPreselectionCloud,indices);
            if (cs > CosSim)
            {
                CosSim = cs;
                n_GoalNode.x = n_pPreselectionCloud->points[i].x;
                n_GoalNode.y = n_pPreselectionCloud->points[i].y;
                n_GoalNode.z = n_pPreselectionCloud->points[i].z+0.583;
            }
        }
        n_pPlanNodeCloud->push_back(n_GoalNode);
}

void MeshMap::MeshBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud){
    std::vector<double> ver(2,0);
    std::vector<std::vector<double>> Boundary;
    double max_theta = 0.0,min_theta = 0.0;

    for (int i = 0; i <verMeshCloud->points.size(); i++)
	{
        ver[0] = sqrt(pow(verMeshCloud->points[i].x,2)+pow(verMeshCloud->points[i].y,2));
        ver[1] = atan(verMeshCloud->points[i].y/verMeshCloud->points[i].x) * 180 / 3.14;
        Boundary.push_back(ver);
		if (ver[1]<min_theta) min_theta = ver[1];
        if (ver[1]>max_theta) max_theta = ver[1];
	}

    for(double i =min_theta;i<max_theta ;i+=0.5)
    {
        double r=0.0;
        int index = 0;
        for(int j =0;j<Boundary.size();j++)
        {
            if (Boundary[j][1]>i && Boundary[j][1]< i+0.5)
            {
                if(Boundary[j][0] > r)
                {
                    r = Boundary[j][0];
                    index = j;
                }
            }
        }
        BoundaryCloud->push_back(verMeshCloud->points[index]);
	}

}

void MeshMap::MeshBoundary(pcl::PointCloud<pcl::PointNormal>::Ptr verMeshCloud,pcl::PointCloud<pcl::PointNormal>::Ptr BoundaryCloud){
    std::vector<double> ver(2,0);
    std::vector<std::vector<double>> Boundary;
    double max_theta = 0.0,min_theta = 0.0;

    for (int i = 0; i <verMeshCloud->points.size(); i++)
	{
        ver[0] = sqrt(pow(verMeshCloud->points[i].x,2)+pow(verMeshCloud->points[i].y,2));
        ver[1] = atan(verMeshCloud->points[i].y/verMeshCloud->points[i].x) * 180 / 3.14;
        Boundary.push_back(ver);
		if (ver[1]<min_theta) min_theta = ver[1];
        if (ver[1]>max_theta) max_theta = ver[1];
	}

    for(double i =min_theta;i<max_theta ;i+=0.5)
    {
        double r=0.0;
        int index = 0;
        for(int j =0;j<Boundary.size();j++)
        {
            if (Boundary[j][1]>i && Boundary[j][1]< i+0.5)
            {
                if(Boundary[j][0] > r)
                {
                    r = Boundary[j][0];
                    index = j;
                }
            }
        }
        BoundaryCloud->push_back(verMeshCloud->points[index]);
	}

}

void MeshMap::PublishFullOctoMap(octomap::ColorOcTree & m_octree)
{
  octomap_msgs::Octomap map;
  map.header.frame_id = pub_MapFrame;
  map.header.stamp = ros::Time::now();

  if (octomap_msgs::fullMapToMsg(m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

}

void MeshMap::PublishGoalOdom(pcl::PointXYZ & oGoalPoint){

        nav_msgs::Odometry oCurrGoalOdom;
        oCurrGoalOdom.header.stamp = ros::Time::now();
        oCurrGoalOdom.header.frame_id = pub_GoalOdomFrame;

        //set the position
        oCurrGoalOdom.pose.pose.position.x = oGoalPoint.x;
        oCurrGoalOdom.pose.pose.position.y = oGoalPoint.y;
        oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z;
        // oCurrGoalOdom.pose.pose.position.z = oGoalPoint.z + 0.583;

        n_GoalNodePub.publish(oCurrGoalOdom);
}

void MeshMap::PublishPreseNodeClouds(){
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPreselectionCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();

    n_PreseNodePub.publish(CloudData);
}

void MeshMap::PublishPlanNodeClouds(){
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPlanNodeCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();

    n_PlanNodePub.publish(CloudData);
}

void MeshMap::PublishPastNodeClouds(){
    sensor_msgs::PointCloud2 CloudData;
    pcl::toROSMsg(*n_pPastNodeCloud, CloudData);
    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    CloudData.header.frame_id = pub_CloudFrame;
    CloudData.header.stamp = ros::Time::now();
    n_PastNodePub.publish(CloudData);
}

void MeshMap::PublishPointCloud(ros::Publisher Pub,const pcl::PointCloud<pcl::PointXYZ> & Cloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 CloudData;

	pcl::toROSMsg(Cloud, CloudData);

	CloudData.header.frame_id = pub_CloudFrame;

	CloudData.header.stamp = ros::Time::now();

	Pub.publish(CloudData);
}

void MeshMap::PublishPointCloud(ros::Publisher Pub,const pcl::PointCloud<pcl::PointNormal> & Cloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 CloudData;

	pcl::toROSMsg(Cloud, CloudData);

	CloudData.header.frame_id = pub_CloudFrame;

	CloudData.header.stamp = ros::Time::now();

	Pub.publish(CloudData);
}

void MeshMap::PublishMesh(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices){
  /*	
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

	m_oMeshPublisher.publish(oMeshMsgs);
*/
}

}