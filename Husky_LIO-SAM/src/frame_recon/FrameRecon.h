#ifndef __FRAAME_RECON_H__
#define __FRAAME_RECON_H__

#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include "pcl_ros/transforms.h"  

#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//polygon related
#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <fusion_msgs/MeshArray.h>

//project related
#include "GHPR.h"
#include "SectorPartition.h"
#include "ExplicitRec.h"
#include "CircularVector.h"
#include "MeshSample.h"

#include "PointType.h"

class SingleFrameRecon{

    pcl::PointXYZI viewPoint;

public:
    SingleFrameRecon();
    ~SingleFrameRecon();

    ExplicitRec m_oExplicitBuilder;
    pcl::PointCloud<pcl::PointNormal>::Ptr FramePNormal;

    void setViewPoint(pcl::PointXYZ point);

    void FrameRecon(pcl::PointCloud<PointType>::Ptr SingleFramePointCloud);

    void PublishMeshs(ros::NodeHandle &nodeHandle);
};

#endif