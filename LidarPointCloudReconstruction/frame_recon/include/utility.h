#pragma once
#ifndef _FRAME_RECON_PARAM_H_
#define _FRAME_RECON_PARAM_H_
#include <string>
#include <ctime>

//ros related
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

//pcl related
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>         
#include <pcl/io/ply_io.h>         
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h>

//polygon related

#include <visualization_msgs/Marker.h>


//project related
#include "GHPR.h"
#include "SectorPartition.h"
#include "ExplicitRec.h"
#include "CircularVector.h"
#include "MeshSample.h"

typedef pcl::PointXYZI PointType;

// Trajectory state data. 
struct RosTimePoint{

	// The time of the measurement leading to this state (in seconds). //
	ros::Time oTimeStamp;

	//********angles***********
	//float roll;    ///< The current roll angle. 
	//float pitch;    ///< The current pitch angle.
	//float yaw;   ///< The current yaw angle. 

	//********coordinate value****************
	// The global trajectory position in 3D space. 
	pcl::PointXYZI oLocation;

};


class ParamServer
{
public:
    bool debug;

// protected:
    //***file related***
    std::string m_sFileHead;

    //full name of output txt that records the point clouds
    std::stringstream m_sOutPCFileName; 

    //whether the file is generated or not
    bool m_bOutPCFileFlag;

    //ouput file
    std::ofstream m_oOutPCFile;

    //output point cloud with normal
    std::stringstream m_sOutPCNormalFileName; 

    //***for input odom topic***
    //the m_oOdomSuber subscirber is to hearinput  odometry topic
    ros::Subscriber m_oOdomSuber;
    //the name of input odometry topic (robot trajectory), 在 reconstruction.launch 中赋值 /slam_odom
    std::string m_sInOdomTopic;

    //***for input point cloud topic***
    //the m_oCloudSuber subscirber is to hear input point cloud topic
    ros::Subscriber m_oCloudSuber;
    //the name of input point cloud topic, 在 reconstruction.launch 中赋值 /slam_points
    std::string m_sInCloudTopic; 


    //***for output cloud topic***
    //output point cloud topic, 在 reconstruction.launch 中赋值 /processed_cloud 目前该节点不存在
    std::string m_sOutCloudTopic;
    //output point cloud frame
    std::string m_sOutCloudTFId;
    //point cloud publisher for test
    ros::Publisher m_oCloudPublisher;

    //***for output mesh***
    //output point cloud topic, 在 reconstruction.launch 中赋值 /frame_meshs，单帧网格的输出
    std::string m_sOutMeshTopic;
    //output point cloud frame
    std::string m_sOutMeshTFId;
    //polygon publisher for test
    ros::Publisher m_oMeshPublisher;

    //displayed point topic
    std::string m_sAdditionalPointTopic = "/additional_points";
    ros::Publisher m_oAdditionalPointPublisher;

    //frame sampling
    int m_iFrameSmpNum;

    //sampling number of input point clouds
    int m_iSampleInPNum;

    //sampling number of sector
    int m_iSectorNum;

    // Lidar ids config, stored in the intensity of point-cloud
    int m_iLidarLineMin;
    int m_iLidarLineMax;

    //**frenquency related**

    float m_fViewZOffset;//z offset of odom to lidar sensor

    //explicit reconstruction
    ExplicitRec m_oExplicitBuilder;

    //circle vector of odom
    CircularVector<RosTimePoint> m_vOdomHistory;

    //How many frames of point cloud have been calculated cumulatively
    unsigned int m_iPCFrameCount;

    //frame count
    unsigned int m_iTrajCount;

    //map point clouds with normals
    //accumulated processed point cloud
    pcl::PointCloud<pcl::PointXYZI> m_vMapPCN;

    //features of map point clouds
    //Features can be specified
    std::vector<float> m_vMapPCFeas;

    double m_dAverageReconstructTime;
    double m_dMaxReconstructTime;

    unsigned int m_iTrajFrameNum;
    int m_iReconstructFrameNum;

    int m_iTotalFrameNum;

    // For lazy node loading mode
    ros::NodeHandle node;
    ros::NodeHandle nodeHandle;

    bool m_bOutputFiles;

    std::ofstream outTimeFile;
    

    ParamServer()
    {
        m_iTrajFrameNum = 0;
        m_dAverageReconstructTime = 0;
        m_iReconstructFrameNum = 0;
        m_dMaxReconstructTime = 0;

        nodeHandle.param<bool>("debug",debug,false);
        // output file name
        node.param<string>("sf_output_path", m_sFileHead, std::string());
        if(m_sFileHead.empty())
            nodeHandle.param<string>("file_output_path", m_sFileHead, std::string(""));
        m_bOutputFiles = !m_sFileHead.empty();

        if(m_bOutputFiles) {

            if(m_sFileHead.back() != '/') m_sFileHead += "/";

            std::stringstream sOutputCommand;
            sOutputCommand << "mkdir -p " << m_sFileHead;
            int retOutputfile = system(sOutputCommand.str().c_str());

            std::stringstream sOutputTimeFileCommand;
            std::string sOutputTimeFile = m_sFileHead+"ReconTime.txt";
            // std::cout<<"OutputTimeFile:  "<<sOutputTimeFile<<std::endl;
            // sOutputTimeFileCommand << "rm " << sOutputTimeFile;
            // std::cout<<"sOutputTimeFileCommand:  "<< sOutputTimeFileCommand.str() <<std::endl;
            // if (access(sOutputTimeFile.c_str(), F_OK) == 0) {system(sOutputTimeFileCommand.str().c_str());}
            // sOutputTimeFileCommand.str("");
            sOutputTimeFileCommand << "touch " << sOutputTimeFile;
            std::cout<<"sOutputTimeFileCommand(touch):  "<< sOutputTimeFileCommand.str() <<std::endl;
            int retOutputTimefile =system(sOutputTimeFileCommand.str().c_str());
            outTimeFile.open(sOutputTimeFile,ios_base::trunc);

            m_sFileHead += "sf_";
        }
        //input odom topic
        nodeHandle.param<string>("odom_in_topic", m_sInOdomTopic, std::string("/odometry/filtered"));

        //input point cloud topic
        nodeHandle.param<string>("cloud_in_topic", m_sInCloudTopic, std::string("/cloud_points"));


        //input odom topic
        nodeHandle.param<string>("cloud_out_topic", m_sOutCloudTopic, std::string("/processed_clouds"));

        //input point cloud topic
        nodeHandle.param<string>("outcloud_tf_id", m_sOutCloudTFId, std::string("camera_init"));

        //input odom topic
        nodeHandle.param<string>("polygon_out_topic", m_sOutMeshTopic, std::string("/processed_clouds"));

        //input point cloud topic
        nodeHandle.param<string>("polygon_tf_id", m_sOutMeshTFId, std::string("camera_init"));

        //point cloud sampling number
        nodeHandle.param<int>("sample_pcframe_num", m_iFrameSmpNum, 1);

        //point cloud sampling number
        nodeHandle.param<int>("sample_inputpoints_num", m_iSampleInPNum, 1);

        nodeHandle.param<int>("lidar_line_min", m_iLidarLineMin, 0);
        nodeHandle.param<int>("lidar_line_max", m_iLidarLineMax, 15);

        //height of viewpoint
        double dViewZOffset;
        nodeHandle.param<double>("viewp_zoffset", dViewZOffset, 0.0);
        m_fViewZOffset = float(dViewZOffset);
        
        //explicit reconstruction related
        //number of sectors
        nodeHandle.param<int>("sector_num", m_iSectorNum, 1);
        m_oExplicitBuilder.HorizontalSectorSize(m_iSectorNum);

        bool bMultiThread;
        nodeHandle.param<bool>("multi_thread", bMultiThread, true);
        m_oExplicitBuilder.SetMultiThread(bMultiThread);

        //count processed point cloud frame
        m_iPCFrameCount = 0;

        //count processed odom frame
        m_iTrajCount = 0;

        //true indicates the file has not been generated
        m_bOutPCFileFlag = true;
    }
};
#endif