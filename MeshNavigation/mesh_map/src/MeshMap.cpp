#include"MeshMap.h"

namespace mesh_map{
MeshMap::MeshMap(ros::NodeHandle & node, ros::NodeHandle & nodeHandle):
                m_pBoundCloud(new pcl::PointCloud<pcl::PointXYZ>),
                m_pPlanNodeCloud(new pcl::PointCloud<pcl::PointXYZ>),
                m_pPastNodeCloud(new pcl::PointCloud<pcl::PointXYZ>),
                PlanCloud(new pcl::PointCloud<pcl::PointNormal>),
                MeshTree(new octomap::ColorOcTree(0.05)),
                NextNodeFlag(true)
{
    srand((unsigned)time(NULL));

	//read parameters
	ReadLaunchParams(nodeHandle);
    MeshTree->setResolution(VoxelsSize);

    //subscribe (hear) the odometry information
	// m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &MeshMap::HandleTrajectory, this);
    	
    // m_sMeshSuber = nodeHandle.subscribe(m_sMeshTopic,1,&MeshMap::HandleMesh,this);

    m_sCloudNormalsSuber = nodeHandle.subscribe("/frame_cloudnormals",1,&MeshMap::HandleCloudNormals,this);

    // m_oOdomSuber = nodeHandle.subscribe(m_sOdomTopic, 1, &MeshMap::HandleTrajectory, this);
	// m_oGroundSuber = nodeHandle.subscribe(m_sGroundTopic, 1, &MeshMap::HandleGroundClouds, this);
	// m_oBoundSuber = nodeHandle.subscribe(m_sBoundTopic, 1, &MeshMap::HandleBoundClouds, this);
	// m_oObstacleSuber = nodeHandle.subscribe(m_sObstacleTopic, 1, &MeshMap::HandleObstacleClouds, this);

    m_oTestCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("test_clouds", 1, true);
    m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>("map_meshs", 1, true);
    m_oPlanNodePublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("plannode_clouds", 1, true);
    m_oPastNodePublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("pastnode_clouds", 1, true);
    m_oGoalPublisher = nodeHandle.advertise<nav_msgs::Odometry>("goal_odom", 1, true);
}

MeshMap::~MeshMap()
{}

bool MeshMap::ReadLaunchParams(ros::NodeHandle & nodeHandle) {

    //input topic
    nodeHandle.param("odom_in_topic", m_sOdomTopic, std::string("/odometry/filtered"));
    nodeHandle.param("mesh_in_topic",m_sMeshTopic,std::string("/frame_meshs"));

    //outpt topic 
    nodeHandle.param("map_test_clouds",m_oTestCloudTopic,std::string("/test_clouds"));
    nodeHandle.param("map_mesh_topic",m_oOutMeshTopic,std::string("/map_meshs"));

    //output test point cloud TFID 
    nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("odom"));
    // nodeHandle.param("outcloud_tf_id", m_sOutCloudTFId, std::string("camera_init"));

	return true;
}

void MeshMap::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
    pcl::PointXYZ oOdomPoint;
    oOdomPoint.x = oTrajectory.pose.pose.position.x;//z in loam is x
    oOdomPoint.y = oTrajectory.pose.pose.position.y;//x in loam is y
    oOdomPoint.z = oTrajectory.pose.pose.position.z;//y in loam is z

    if(!NextNodeFlag)
    {
        if(abs(oOdomPoint.x - m_oNodeGoal.x) < 0.2)
            if(abs(oOdomPoint.y - m_oNodeGoal.y) < 0.2)
                if(abs(oOdomPoint.z - m_oNodeGoal.z) < 0.2+0.583)
                {
                    NextNodeFlag = !NextNodeFlag;
                    // std::cout<<"OdomPoint: "<<oOdomPoint<<"   GoalPoint: "<<m_oNodeGoal<<std::endl;
                    m_pPastNodeCloud->push_back(m_pPlanNodeCloud->points.back());
                    m_pPlanNodeCloud->clear();
                    PublishPastNodeClouds();
                }
    }
}

void MeshMap::HandleMesh(const visualization_msgs::Marker &oMeshMsgs)
{
    if(NextNodeFlag)
    {
        NextNodeFlag = !NextNodeFlag;
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
        // std::cout<<"NextNodeFlag: "<<NextNodeFlag<<std::endl;

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
    }
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
    if(NextNodeFlag)
    {
        NextNodeFlag = !NextNodeFlag;
        //a point clouds in PCL type
        pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr pGroundPN(new pcl::PointCloud<pcl::PointNormal>); // segmented ground plane
        pcl::PointCloud<pcl::PointNormal>::Ptr pNongroundPN(new pcl::PointCloud<pcl::PointNormal>); // everything else

        //message from ROS type to PCL type
        pcl::fromROSMsg(oMeshMsgs, *pFramePN);

        for(auto point:*pFramePN)
        {
            if(point.z < 0.1651)
            {
                Eigen::Vector3d Vector_z(0,0,1);
                Eigen::Vector3d PointNormal(point.normal_x,point.normal_y,point.normal_z);
                if(Vector_z.dot(PointNormal) == Vector_z.norm() * PointNormal.norm())
                {
                    pGroundPN->push_back(point);
                }  
            }
        }
        std::cout<< pGroundPN->points.size()<<std::endl;
        PublishPointCloud(*pGroundPN);
    }
}

void MeshMap::PublishGoalOdom(pcl::PointXYZ & oGoalPoint){

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

/*************************************************
Function: PublishPlanNodeClouds
Description: publish unvisited nodes as a point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: none
Output: m_oOPSolver.m_vAllNodes.point - a private variable of class object m_oOPSolver
Return: none
Others: none
*************************************************/
void MeshMap::PublishPlanNodeClouds(){
  //publish obstacle points
    // pcl::PointCloud<pcl::PointXYZ> vCloud;

    // for(int i = 0; i != m_oOPSolver.m_vAllNodes.size(); ++i){
    //    if(!m_oOPSolver.m_vAllNodes[i].visitedFlag){
    //    	    pcl::PointXYZ oNodePoint;
    //    	    oNodePoint.x = m_oOPSolver.m_vAllNodes[i].point.x;
    //         oNodePoint.y = m_oOPSolver.m_vAllNodes[i].point.y;
    //         oNodePoint.z = m_oOPSolver.m_vAllNodes[i].point.z;
    //    	    vCloud.push_back(oNodePoint);

    //    }
    // }

    sensor_msgs::PointCloud2 vCloudData;

    pcl::toROSMsg(*m_pPlanNodeCloud, vCloudData);

    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    vCloudData.header.frame_id = m_sOutCloudTFId;

    vCloudData.header.stamp = ros::Time::now();

    m_oPlanNodePublisher.publish(vCloudData);

}

/*************************************************
Function: PublishPastNodeClouds
Description: Publish visited nodes as a point clouds (mainly used for display and test)
Calls: none
Called By: ComputeConfidence()
Table Accessed: none
Table Updated: none
Input: none
Output: m_oOPSolver.m_vAllNodes.point - a private variable of class object m_oOPSolver
Return: none
Others: none
*************************************************/
void MeshMap::PublishPastNodeClouds(){
  //publish obstacle points

    // pcl::PointCloud<pcl::PointXYZ> vCloud;

    // for(int i = 0; i != m_oOPSolver.m_vAllNodes.size(); ++i){
    //    if(m_oOPSolver.m_vAllNodes[i].visitedFlag){

    //    	    pcl::PointXYZ oNodePoint;
    //    	    oNodePoint.x = m_oOPSolver.m_vAllNodes[i].point.x;
    //         oNodePoint.y = m_oOPSolver.m_vAllNodes[i].point.y;
    //         oNodePoint.z = m_oOPSolver.m_vAllNodes[i].point.z + 0.583;
    //    	    vCloud.push_back(oNodePoint);

    //    }
    // }

    sensor_msgs::PointCloud2 vCloudData;

    pcl::toROSMsg(*m_pPastNodeCloud, vCloudData);

    // vCloudData.header.frame_id = m_oGMer.m_oFeatureMap.getFrameId();
    vCloudData.header.frame_id = m_sOutCloudTFId;

    vCloudData.header.stamp = ros::Time::now();

    m_oPastNodePublisher.publish(vCloudData);

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

void MeshMap::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> & vCloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	m_oTestCloudPublisher.publish(vCloudData);

}

void MeshMap::PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> & vCloud){
  
	//publish obstacle points
	sensor_msgs::PointCloud2 vCloudData;

	pcl::toROSMsg(vCloud, vCloudData);

	vCloudData.header.frame_id = m_sOutCloudTFId;

	vCloudData.header.stamp = ros::Time::now();

	m_oTestCloudPublisher.publish(vCloudData);

}

void MeshMap::PublishMeshs(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices){
  	
  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = m_sOutCloudTFId;
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

}



}
