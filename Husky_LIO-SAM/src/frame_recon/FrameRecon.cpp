#include "FrameRecon.h"

SingleFrameRecon::SingleFrameRecon():FramePNormal(new  pcl::PointCloud<pcl::PointNormal>)
{
    viewPoint.x = 0.0; 
    viewPoint.y = 0.0;
    viewPoint.z = 0.0;
    viewPoint.intensity = 1;

    m_oExplicitBuilder.ClearData();
    m_oExplicitBuilder.HorizontalSectorSize(12);
    m_oExplicitBuilder.SetMultiThread(false);
}

SingleFrameRecon::~SingleFrameRecon(){}


void SingleFrameRecon::setViewPoint(pcl::PointXYZ point){
    viewPoint.x = point.x;
    viewPoint.y = point.y;
    viewPoint.z = point.z;
    viewPoint.intensity = 1;
}

void SingleFrameRecon::FrameRecon(pcl::PointCloud<PointType>::Ptr SingleFramePointCloud){

    m_oExplicitBuilder.setWorkingFrameCount(0);
    m_oExplicitBuilder.SetViewPoint(viewPoint, 0.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ReconPointCloud;
    for (unsigned int i = 0; i < SingleFramePointCloud->points.size();i++)
    {
        pcl::PointXYZI thisPoint;
        thisPoint.x = SingleFramePointCloud->points[i].x;
        thisPoint.y = SingleFramePointCloud->points[i].y;
        thisPoint.z = SingleFramePointCloud->points[i].z;
        thisPoint.intensity = SingleFramePointCloud->points[i].ring;
        ReconPointCloud->push_back(thisPoint);
    }
    m_oExplicitBuilder.FrameReconstruction(*ReconPointCloud, *FramePNormal, 0, 15); // 得到带法向的点云

    for (unsigned int i = 0; i < SingleFramePointCloud->points.size();i++)
    {
        SingleFramePointCloud->points[i].normal_x = FramePNormal->points[i].normal_x;
        SingleFramePointCloud->points[i].normal_y = FramePNormal->points[i].normal_y;
        SingleFramePointCloud->points[i].normal_z = FramePNormal->points[i].normal_z;
    }
}

void SingleFrameRecon::PublishMeshs(ros::NodeHandle & nodeHandle){
  	
  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = "odom";
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
	color.a = 1.0;
	color.r = 200 / 255.f * 1.5f;
	color.g = 128 / 255.f * 1.5f;
	color.b = 54  / 255.f * 1.5f;
	
	color.r = 248 / 255.f;
	color.g = 220 / 255.f;
	color.b = 180 / 255.f;

	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZI> vMeshVertices;

	//get the reconstruted mesh
	m_oExplicitBuilder.OutputAllMeshes(vMeshVertices);

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

    ros::Publisher m_oMeshPublisher;
    m_oMeshPublisher = nodeHandle.advertise<visualization_msgs::Marker>("husky_lio_sam/FrameRecon/SingleFrameRecon", 1, true);
    m_oMeshPublisher.publish(oMeshMsgs);
}