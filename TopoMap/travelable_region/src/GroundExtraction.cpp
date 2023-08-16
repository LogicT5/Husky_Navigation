#include "GroundExtraction.h"


/*************************************************
Function: GroundExtraction
Description: constrcution function for GroundExtraction class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: global node,
       privare node
       flag of generating output file
       original frame value
Output: none
Return: none
Others: the HandlePointClouds is the kernel function
*************************************************/
GroundExtraction::GroundExtraction(ros::NodeHandle & node, 
                        ros::NodeHandle & private_node)
                        :m_bFileNmFlag(false),
                         m_iFrames(-1),
                         m_iTrajPointNum(-1){

    //get the value of parameters
    //system parameters
    GetOutputPath(private_node);
    
    //check output file or not
    GetTxtOutputFlag(private_node);

    //point cloud related
    GetSamplingNum(private_node);
    
    //get laser topic name
    GetLaserTopic(private_node);
    
    //get trajectory topic name
    GetOdomTopic(private_node);
    
    //get gp-insac related thresholds
    GetGPINSACThrs(private_node);

    OdomFrame = "odom";
    BaselinkOdomFrame = "base_link_odom";
    // BaselinkOdomFrame = "odom";

    //subscribe (hear) the point cloud topic from laser on right side 
    if(m_sLaserTopic.compare("/frame_cloudnormals") == 0 ){
        m_oLaserSuber = node.subscribe("/frame_cloudnormals", 2, &GroundExtraction::HandleMesh, this);
    }else if(m_sLaserTopic.compare( "/frame_meshs") == 0 ){
        m_oLaserSuber = node.subscribe( "/frame_meshs" , 2, &GroundExtraction::HandleMesh2, this);
    }else{
        m_oLaserSuber = node.subscribe(m_sLaserTopic, 2, &GroundExtraction::HandlePointClouds, this);
    }
    #ifdef DEBUG
        std::cout<<"\033[31m"<<"travelable DEBUG  "<<"\033[35m"<< "Subscribe Point Cloud Topic is "<< m_sLaserTopic<<"\033[0m"<<std::endl;
        debugSubPointCloud = node.advertise<sensor_msgs::PointCloud2>("/travelable/debugSubPointCloud", 2);
    #endif
    //subscribe (hear) the odometry information
    m_oOdomSuber = node.subscribe(m_sOdomTopic, 2, &GroundExtraction::HandleTrajectory, this);
    //publish topic of point clouds 
    m_oGroundPub = node.advertise<sensor_msgs::PointCloud2>("/ground_points", 2);
    //publish topic of boundary clouds 
    m_oBoundPub = node.advertise<sensor_msgs::PointCloud2>("/boundary_points", 2);
    //publish topic of point clouds 
    m_oObstaclePub  = node.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 2);
    //publish odometry with high value
    m_oHighOdomPub = node.advertise<nav_msgs::Odometry>("/odom_lidar", 2);

    m_oMeshPublisher  = node.advertise<visualization_msgs::Marker>("test_mesh", 2);

    Odom2BaselinkOdomTF = Eigen::Matrix4d::Identity();

    if(OdomFrame != BaselinkOdomFrame)
    {
        try
        {
            // 等待3s
            tfListener.waitForTransform(OdomFrame, BaselinkOdomFrame, ros::Time(0), ros::Duration(10.0));
            // lidar系到baselink系的变换
            tfListener.lookupTransform(OdomFrame, BaselinkOdomFrame,  ros::Time(0), Odom2BaselinkOdom);
        //    // 等待3s
        //     tfListener.waitForTransform(BaselinkOdomFrame, OdomFrame, ros::Time(0), ros::Duration(3.0));
        //     // lidar系到baselink系的变换
        //     tfListener.lookupTransform(BaselinkOdomFrame, OdomFrame,  ros::Time(0), Odom2BaselinkOdom);
            
            // 获取位移和旋转分量
            tf::Vector3 translation =Odom2BaselinkOdom.inverse().getOrigin();
            tf::Quaternion rotation = Odom2BaselinkOdom.inverse().getRotation();

            // 将位移和旋转转换为Eigen类型
            Eigen::Vector3d translation_eigen(translation.x(), translation.y(), translation.z());
            Eigen::Quaterniond rotation_eigen(rotation.w(), rotation.x(), rotation.y(), rotation.z());

            // 创建变换矩阵
            Odom2BaselinkOdomTF.block<3, 3>(0, 0) = rotation_eigen.toRotationMatrix();
            Odom2BaselinkOdomTF.block<3, 1>(0, 3) = translation_eigen;

            ///*Debug
            #ifdef DEBUG
                std::cout<<"\033[31mtrav DEBUG \033[1m\033[34mlidar2Baselink:";
                std::cout << "  Translation: " << Odom2BaselinkOdom.getOrigin().getX() << ", " << Odom2BaselinkOdom.getOrigin().getY() << ", " << Odom2BaselinkOdom.getOrigin().getZ();
                std::cout << "  Rotation: " << Odom2BaselinkOdom.getRotation().getW()<< ", " << Odom2BaselinkOdom.getRotation().getX() << ", " << Odom2BaselinkOdom.getRotation().getY() << ", " << Odom2BaselinkOdom.getRotation().getZ();
                std::cout <<std::endl;
                std::cout <<Odom2BaselinkOdomTF<<std::endl;
                std::cout << "\033[0m"<<std::endl;//*/
            #endif

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }
}

/*************************************************
Function: GetOutputPath
Description: inital function for m_sFileHead
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sFileHead
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetOutputPath(ros::NodeHandle & private_node){
        
    std::string sFileHead;

    if(private_node.getParam("output_path", sFileHead)){

      m_sFileHead = sFileHead;

      return true;
                     
      }else{

      m_sFileHead = "./";///<in the default user's path (in usual)

      return false;

    }//end if

}

/*************************************************
Function: GetTxtOutputFlag
Description: inital function for m_sTxtOutFlag
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sTxtOutFlag
Return: none
Others: none
*************************************************/

bool GroundExtraction::GetTxtOutputFlag(ros::NodeHandle & private_node){
        
   bool bTxtOutFlag;

    if(private_node.getParam("txtoutput_flag", bTxtOutFlag)){

      m_bTxtOutFlag = bTxtOutFlag;

      return true;
                     
      }else{

      m_bTxtOutFlag = false;///<don't output result in txt

      return false;

    }//end if

}

/*************************************************
Function: GetLaserTopic
Description: inital function for m_sLaserTopic
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sLaserTopic
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetLaserTopic(ros::NodeHandle & private_node){
        
    std::string sLaserTopic; 

    if(private_node.getParam("lidar_topic", sLaserTopic)){

        m_sLaserTopic = sLaserTopic;

        return true;
                     
       }else{

        m_sLaserTopic = "/velodyne_points";///<velodyne LiDAR due to its popular

        return false;

    }//end if

}

/*************************************************
Function: GetSamplingNum
Description: inital function for m_iSampleNum
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_iSampleNum
Return: none
Others: none
*************************************************/


bool GroundExtraction::GetSamplingNum(ros::NodeHandle & private_node){
        
    int iSampleNum; 

    if(private_node.getParam("sampling_number", iSampleNum)){

        m_iSampleNum =  iSampleNum;

        return true;
                     
       }else{

        m_iSampleNum =  2;///<down sampling under 2 frames

        return false;

    }//end if

}

/*************************************************
Function: GetOdomTopic
Description: inital function for m_sOdomTopic
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: m_sOdomTopic
Return: none
Others: none
*************************************************/
    //trajectory related 
bool GroundExtraction::GetOdomTopic(ros::NodeHandle & private_node){

    std::string sOdomTopic; 

    if(private_node.getParam("traj_topic", sOdomTopic)){

        m_sOdomTopic = sOdomTopic;

        return true;
                     
       }else{

        m_sOdomTopic = "/odom";///<

        return false;

    }//end if

}

/*************************************************
Function: GetGPINSACThrs
Description: inital function for GP-INSAC related thresholds
Calls: Launch file maybe
Called By: GroundExtraction
Table Accessed: none
Table Updated: none
Input: node class with a private node object
Output: thresholds
Return: none
Others: none
*************************************************/
void GroundExtraction::GetGPINSACThrs(ros::NodeHandle & private_node){

     int iSector_num;
     if(private_node.getParam("sector_num", iSector_num))
        m_oGPThrs.iSector_num = iSector_num;

     //seed selection
     double fDisThr;
     if(private_node.getParam("seed_radius", fDisThr))
        m_oGPThrs.fDisThr = float(fDisThr);
    
    double fZLower;
    if(private_node.getParam("seed_lower", fZLower))
        m_oGPThrs.fZLower = float(fZLower);

    double fZUpper;
    if(private_node.getParam("seed_upper", fZUpper))
        m_oGPThrs.fZUpper = float(fZUpper);

    //GP model thresholds
    double dLScale;
    if(private_node.getParam("gp_lscale", dLScale))
        m_oGPThrs.dLScale = dLScale;

    double dSigmaF;
    if(private_node.getParam("gp_sigmaF", dSigmaF))
        m_oGPThrs.dSigmaF = dSigmaF;

    double dSigmaN;
    if(private_node.getParam("gp_sigmaN", dSigmaN))
        m_oGPThrs.dSigmaN = dSigmaN;

    //insac thresholds
    double fModelThr;
    if(private_node.getParam("insac_model", fModelThr))
        m_oGPThrs.fModelThr = float(fModelThr);

    double fDataThr;
    if(private_node.getParam("insac_data", fDataThr))
        m_oGPThrs.fDataThr = float(fDataThr);

}

/*************************************************
Function: OutputGroundPoints
Description: output the result of point clouds in a txt file
Calls: none
Called By: HandlePointClouds
Table Accessed: none
Table Updated: none
Input: vCloud the final result cloud
       oStamp the time stamp of this input point clouds
Output: a point cloud txt file which record ground points only
Return: none
Others: none
*************************************************/
void GroundExtraction::OutputGroundPoints(pcl::PointCloud<pcl::PointXYZ> & vCloud, const ros::Time & oStamp){
  

  if(m_bTxtOutFlag){
    
    //if this is the first time of calling this function
    if(!m_bFileNmFlag){

    //set the current time stamp as a file name
    //full name 
    m_sOutPCFileName << m_sFileHead << "_PC_" << oStamp << ".txt"; 

    m_bFileNmFlag = true;

    }


    //output
    std::ofstream oRecordedFile;
    oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

    for(int i = 0; i != vCloud.size(); ++i ){

      //output in a txt file
      //the storage type of output file is x y z time frames right/left_sensor
      oRecordedFile << vCloud.points[i].x << " "
                    << vCloud.points[i].y << " "
                    << vCloud.points[i].z << " " 
                    << oStamp << " "
                    << std::endl;
    }//end for         

     oRecordedFile.close();

  }//end if(m_bTxtOutFlag)

}


/*************************************************
Function: OutputAllPoints
Description: output all of point clouds with class label in a txt file
Calls: none
Called By: HandlePointClouds
Table Accessed: none
Table Updated: none
Input: pCloud one frame of point clouds in normal coordinate system (z is toward up)
       vRes the label result of each point
       oStamp the time stamp of this input point clouds
Output: a point cloud txt file which record all points with class label
Return: none
Others: none
*************************************************/
void GroundExtraction::OutputAllPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, 
                                          const std::vector<int> & vRes,
                                          const ros::Time & oStamp){
  
  if(m_bTxtOutFlag){

     //if this is the first time of calling this function
     if(!m_bFileNmFlag){

       //set the current time stamp as a file name
       //full name 
       m_sOutPCFileName << m_sFileHead << "_PC_" << oStamp << ".txt"; 

       m_bFileNmFlag = true;
     }

     //output
     std::ofstream oRecordedFile;
     oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);

     for(int i = 0; i != pCloud->points.size(); ++i ){

         //output in a txt file
         //the storage type of output file is x y z time frames right/left_sensor
         oRecordedFile << pCloud->points[i].x << " "
                       << pCloud->points[i].y << " "
                       << pCloud->points[i].z << " " 
                       << vRes[i] << " "
                       << oStamp << " "
                       << std::endl;
      }//end for         

      oRecordedFile.close();

  }//end if m_bTxtOutFlag

}
/*************************************************
Function: HandleRightLaser
Description: a callback function in below: 
             node.subscribe(m_sLaserTopic, 5, &GroundExtraction::HandlePointClouds, this);
Calls: CheckTruthPoint
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void GroundExtraction::HandleMesh(const sensor_msgs::PointCloud2 & vLaserData)
{
  
  //count input frames
  m_iFrames++;
  //ROS_INFO("frame number: %d", m_iFrames);

  if(!(m_iFrames%m_iSampleNum)){

      ////a point clouds in PCL type
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
      ////message from ROS type to PCL type
      	pcl::PointCloud<pcl::PointNormal>::Ptr pFramePN(new pcl::PointCloud<pcl::PointNormal>);
	    //message from ROS type to PCL type
	    pcl::fromROSMsg(vLaserData, *pFramePN);
        pcl::PointCloud<pcl::PointXYZ>::Ptr verBoundCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr pMeshNormal(new pcl::PointCloud<pcl::Normal>);

         //************output value******************
       pcl::PointCloud<pcl::PointXYZ> vGroundCloud;
       sensor_msgs::PointCloud2 vGroundPub;
       pcl::PointCloud<pcl::PointXYZ> vObstacleCloud;
       sensor_msgs::PointCloud2 vObstaclePub;
       pcl::PointCloud<pcl::PointXYZ> vBoundCloud;
       sensor_msgs::PointCloud2 vBoundPub;

        for (int k = 0; k <pFramePN->points.size(); k ++){
            pcl::PointXYZ point;
            point.x = pFramePN->points[k].x;
            point.y = pFramePN->points[k].y;
            point.z = pFramePN->points[k].z;
            
            verMeshCloud->push_back(point);


            Eigen::Vector3d v(pFramePN->points[k].normal_x,pFramePN->points[k].normal_y,pFramePN->points[k].normal_z);
            if(point.z < 0.1651)
            {
                Eigen::Vector3d v0(0,0,1);
                if(v.dot(v0) > -0.01 && v.dot(v0) < 0.3)
                {
                    vGroundCloud.push_back(point);
                }
            }
            else if( point.z > 0.05 )
            {
                vObstacleCloud.push_back(point);
            }

        }

        // for (int k = 0; k <pFramePN->points.size(); k += 3){
        //     int a=k,b=k+1,c=k+2;
        //     pcl::PointXYZ aVertex;
        //     pcl::PointXYZ bVertex;
        //     pcl::PointXYZ cVertex;
        //     aVertex.x = pFramePN->points[a].x;
        //     aVertex.y = pFramePN->points[a].y;
        //     aVertex.z = pFramePN->points[a].z;
            
        //     bVertex.x = pFramePN->points[b].x;
        //     bVertex.y = pFramePN->points[b].y;
        //     bVertex.z = pFramePN->points[b].z;
            
        //     cVertex.x = pFramePN->points[c].x;
        //     cVertex.y = pFramePN->points[c].y;
        //     cVertex.z = pFramePN->points[c].z;
            
        //     if( aVertex.z < 2 && bVertex.z < 2 && cVertex.z < 2)
        //     {
        //         // Eigen::Vector3d v1(bVertex.x - aVertex.x, bVertex.y - aVertex.y,bVertex.z - aVertex.z);
        //         // Eigen::Vector3d v2(cVertex.x - aVertex.x, cVertex.y - aVertex.y,cVertex.z - aVertex.z);
        //         // Eigen::Vector3d v = v1.cross(v2);
                
        //         // pcl::Normal pointNormal;
        //         // pointNormal.normal_x = pFramePN->points[a].normal_x;
        //         // pointNormal.normal_y = pFramePN->points[a].normal_y;
        //         // pointNormal.normal_z = pFramePN->points[a].normal_z;
        //         // pMeshNormal->push_back(pointNormal);
        //         // pointNormal.normal_x = pFramePN->points[b].normal_x;
        //         // pointNormal.normal_y = pFramePN->points[b].normal_y;
        //         // pointNormal.normal_z = pFramePN->points[b].normal_z;
        //         // pMeshNormal->push_back(pointNormal);
        //         // pointNormal.normal_x = pFramePN->points[c].normal_x;
        //         // pointNormal.normal_y = pFramePN->points[c].normal_y;
        //         // pointNormal.normal_z = pFramePN->points[c].normal_z;
        //         // pMeshNormal->push_back(pointNormal);

        //         verMeshCloud->push_back(aVertex);
        //         verMeshCloud->push_back(bVertex);
        //         verMeshCloud->push_back(cVertex);

        //         Eigen::Vector3d va(pFramePN->points[a].normal_x,pFramePN->points[a].normal_y,pFramePN->points[a].normal_z);
        //         Eigen::Vector3d vb(pFramePN->points[b].normal_x,pFramePN->points[b].normal_y,pFramePN->points[b].normal_z);
        //         Eigen::Vector3d vc(pFramePN->points[c].normal_x,pFramePN->points[c].normal_y,pFramePN->points[c].normal_z);

        //         Eigen::Vector3d v((va[0]+vb[0]+vc[0])/3,(va[1]+vb[1]+vc[1])/3,(va[2]+vb[2]+vc[2])/3);

        //          if( aVertex.z < 0.1651 && bVertex.z < 0.1651 && cVertex.z < 0.1651)
        //          {
        //             Eigen::Vector3d v0(0,0,1);
        //             if(v.dot(v0) > -0.01 && v.dot(v0) < 0.2)
        //             {
        //                 vGroundCloud.push_back(aVertex);
        //                 vGroundCloud.push_back(bVertex);
        //                 vGroundCloud.push_back(cVertex);
        //             }
        //          }
        //         else if( aVertex.z > 0.05 && bVertex.z > 0.05 && cVertex.z > 0.05)
        //         {
        //             vObstacleCloud.push_back(aVertex);
        //             vObstacleCloud.push_back(bVertex);
        //             vObstacleCloud.push_back(cVertex);
        //         }
        //     }
        // }//end k

    // // 
    // std::vector<double> ver(2,0);
    // std::vector<std::vector<double>> vBoundary;
    // double max_theta = 0.0,min_theta = 0.0;

    // for (int i = 0; i <verMeshCloud->points.size(); i++)
    //     {
    //     ver[0] = sqrt(pow(verMeshCloud->points[i].x,2)+pow(verMeshCloud->points[i].y,2));
    //     ver[1] = atan(verMeshCloud->points[i].y/verMeshCloud->points[i].x) * 180 / 3.14;
    //     vBoundary.push_back(ver);
    //             if (ver[1]<min_theta) min_theta = ver[1];
    //     if (ver[1]>max_theta) max_theta = ver[1];
    //     }

    // for(double i =min_theta;i<max_theta ;i+=0.2)
    // {
    //     double r=0.0;
    //     int index = 0;
    //     for(int j =0;j<vBoundary.size();j++)
    //     {
    //         if (vBoundary[j][1]>i && vBoundary[j][1]< i+0.2)
    //         {
    //             if(vBoundary[j][0] > r)
    //             {
    //                 r = vBoundary[j][0];
    //                 index = j;
    //             }
    //         }
    //     }
    //     pcl::PointXYZ BoundPoint = verMeshCloud->points[index];
    //     if(BoundPoint.z > -0.01 && BoundPoint.z < 1)
    //     {
    //         BoundPoint.z = 0;
    //         vBoundCloud.push_back(BoundPoint);
    //     }
    // }

          //******************deviding section******************
      //a class to divide point clouds into the given number of sectors
      DivideSector oSectorDivider(m_oGPThrs.iSector_num);
      //compute the corresponding trajectory point
      pcl::PointXYZ oCurrentTrajP;
      
      //if have corresponding trajectory point (viewpoint)
      if( vTrajHistory.size() ){
        //
        oCurrentTrajP = ComputeQueryTraj(vLaserData.header.stamp);
        //
        oSectorDivider.SetOriginPoint(oCurrentTrajP);
      }
         
      //ROS_INFO("Querytime: %f, have trajpoint: %d", .header.stamp.toSec(), bCurTrajPFlag);
      
      //preparation
      std::vector<std::vector<int> > oPointSecIdxs;///<point index reorganization according to sectors
      std::vector<std::vector<GroundFeature> > vGroundFeatures = oSectorDivider.ComputePointSectorIdxs(*verMeshCloud, oPointSecIdxs);
      
      std::vector<std::vector<int> > vAllGroundRes;///<point value according to oPointSecIdxs
      for (int i = 0; i != vGroundFeatures.size(); ++i) {
          std::vector<int> vTmpRes(vGroundFeatures[i].size(),0);
          vAllGroundRes.push_back(vTmpRes);
      }
     //******************deviding section******************
      //***********and adopt GP-INSAC algorithm*************
      //to a sector
      for (int is = 0; is != oPointSecIdxs.size(); ++is) {

          //***********GP algorithm***********
          //new the class of Gaussian Process algorithm class
          GaussianProcessRegression<float> GPR(1, 1);
          
          //paramters
          GPR.SetHyperParams(m_oGPThrs.dLScale, m_oGPThrs.dSigmaF, m_oGPThrs.dSigmaN);

          INSAC GPINSAC(float(m_oGPThrs.dSigmaN), m_oGPThrs.fModelThr, m_oGPThrs.fDataThr);

          //select the initial training input
          GPINSAC.SetSeedThreshold(m_oGPThrs.fDisThr,m_oGPThrs.fZLower,m_oGPThrs.fZUpper);
          GPINSAC.SelectSeeds(vGroundFeatures[is]);

          //***********INSAC algorithm***********
          bool bGrowFlag = true;
          
          //looping until there is not new seed to be involved in current sector
          while(bGrowFlag){
               
               //new training vector - NEW input, NEW means the input does not include old one
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainFeaVec;
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainTruVec;
               GPINSAC.ToAddTrainSamples(vTrainFeaVec, vTrainTruVec, vGroundFeatures[is]);

               //new test vector - NEW output
               std::vector<Eigen::Matrix<float, 1, 1> > vTestFeaVec;
               std::vector<Eigen::Matrix<float, 1, 1> > vTestTruVec;
               GPINSAC.ToAddTestSamples(vTestFeaVec, vTestTruVec, vGroundFeatures[is]);
               
               //training
               GPR.AddTrainingDatas(vTrainFeaVec, vTrainTruVec);
    
               //regression (prediction)
               std::vector<int> vOneLoopLabels;
               for (size_t k = 0; k < vTestFeaVec.size(); k++) {
        
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
                    
                    //below is for parameter optimization only
                    //std::cout <<"vTestFeaVec[k] :"<< vTestFeaVec[k] << std::endl;

                    //regression of new input based on the computed training matrix
                    if (GPR.Regression(vPredValue, vPredVar, vTestFeaVec[k])){
                       
                       //belows are for parameter optimization only
                       //std::cout << "predict" << vPredValue(0) << std::endl;
                       //std::cout << "groundtruth" << vTestTruVec[k](0) << std::endl;
                       //std::cout << "var" << vPredVar(0) << std::endl;

                       //judgement the result is meet the model or not
                       int iPointLabel = GPINSAC.Eval(vTestTruVec[k](0),vPredValue(0), vPredVar(0));
                       vOneLoopLabels.push_back(iPointLabel);

                    }else

                       vOneLoopLabels.push_back(-1);//obstacle
               }
               
               //refresh the remaining unknown points
               if(GPINSAC.AssignIdx(vOneLoopLabels))
                  bGrowFlag = false;
    
          }//end while

          //assigment in current sector
          std::vector<int> vGroundLabels;
          GPINSAC.OutputRes(vGroundLabels);
      
          for (int j = 0; j != vGroundLabels.size(); ++j)
              vAllGroundRes[is][j] = vGroundLabels[j];

      }//end is
  
       //***********boundary extraction***********

      //assigment of result in whole point clouds
      std::vector<int> vCloudRes(verMeshCloud->points.size(),0);
       //assignment in whole point clouds
       for (int is = 0; is != oPointSecIdxs.size(); ++is) {
            for (int j = 0; j != oPointSecIdxs[is].size(); ++j) {
               
                  int iPointIdx = oPointSecIdxs[is][j];
                  vCloudRes[iPointIdx] = vAllGroundRes[is][j];

            }//end for j
       }//end for i

       //new a boundary class
       Boundary oBounder;
       //input the segment labels
       oBounder.GetSegmentClouds(verMeshCloud, vCloudRes);
       //compute boundary point
       oBounder.ComputeBoundary();
       //output the boundary cloud
       oBounder.OutputBoundClouds(vCloudRes);

    for (int i = 0; i != vCloudRes.size(); ++i) {
        //if point is a ground point
        if( vCloudRes[i] == 1){
            //take data
            // vGroundCloud.push_back(verMeshCloud->points[i]);
        //if point is an obstacle point
        }else if(vCloudRes[i] == -1){
            //take data
            // vObstacleCloud.push_back(verMeshCloud->points[i]);
        //if point is an boundary point
        }else if(  vCloudRes[i] == 2)
            //take data
            // verBoundCloud->push_back(verMeshCloud->points[i]);
            vBoundCloud.push_back(verMeshCloud->points[i]);
       }//end for i

    // std::vector<double> ver(2,0);
    // std::vector<std::vector<double>> vBoundary;
    // double max_theta = 0.0,min_theta = 0.0;

    // for (int i = 0; i < verBoundCloud->points.size(); i++)
    //     {
    //     ver[0] = sqrt(pow( verBoundCloud->points[i].x,2)+pow( verBoundCloud->points[i].y,2));
    //     ver[1] = atan( verBoundCloud->points[i].y/ verBoundCloud->points[i].x) * 180 / 3.14;
    //     vBoundary.push_back(ver);
    //             if (ver[1]<min_theta) min_theta = ver[1];
    //     if (ver[1]>max_theta) max_theta = ver[1];
    //     }

    // for(double i =min_theta;i<max_theta ;i+=0.2)
    // {
    //     double r=0.0;
    //     int index = 0;
    //     for(int j =0;j<vBoundary.size();j++)
    //     {
    //         if (vBoundary[j][1]>i && vBoundary[j][1]< i+0.2)
    //         {
    //             if(vBoundary[j][0] > r)
    //             {
    //                 r = vBoundary[j][0];
    //                 index = j;
    //             }
    //         }
    //     }
    //     pcl::PointXYZ BoundPoint =  verBoundCloud->points[index];
    //     if(BoundPoint.z > -0.01 && BoundPoint.z < 1)
    //     {
    //         BoundPoint.z = 0;
    //         vBoundCloud.push_back(BoundPoint);
    //     }
    // }

        // PublishMeshs(vObstacleCloud);
        //PublishMeshs(vGroundCloud);

       //publish ground points
       pcl::toROSMsg(vGroundCloud, vGroundPub);
       vGroundPub.header.frame_id = BaselinkOdomFrame;
       vGroundPub.header.stamp = vLaserData.header.stamp;
       m_oGroundPub.publish(vGroundPub);
       

       //publish ground points
       pcl::toROSMsg(vBoundCloud, vBoundPub);
       vBoundPub.header.frame_id = BaselinkOdomFrame;
       vBoundPub.header.stamp = vLaserData.header.stamp;
       m_oBoundPub.publish(vBoundPub);

       //publish obstacle points
       pcl::toROSMsg(vObstacleCloud, vObstaclePub);
       vObstaclePub.header.frame_id = BaselinkOdomFrame;
       vObstaclePub.header.stamp = vLaserData.header.stamp;
       m_oObstaclePub.publish(vObstaclePub);

  }//end if m_iFrames%m_iSampleNum (down sampling)
}

void GroundExtraction::HandleMesh2(const visualization_msgs::Marker & oMeshData)
{
  
  //count input frames
  m_iFrames++;
  //ROS_INFO("frame number: %d", m_iFrames);

  if(!(m_iFrames%m_iSampleNum)){

      ////a point clouds in PCL type
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
      ////message from ROS type to PCL type
	    //message from ROS type to PCL type
	    //pcl::fromROSMsg(vLaserData, *pFramePN);
        pcl::PointCloud<pcl::PointXYZ>::Ptr verBoundCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr verMeshCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr pMeshNormal(new pcl::PointCloud<pcl::Normal>);

         //************output value******************
       pcl::PointCloud<pcl::PointXYZ> vGroundCloud;
       sensor_msgs::PointCloud2 vGroundPub;
       pcl::PointCloud<pcl::PointXYZ> vObstacleCloud;
       sensor_msgs::PointCloud2 vObstaclePub;
       pcl::PointCloud<pcl::PointXYZ> vBoundCloud;
       sensor_msgs::PointCloud2 vBoundPub;

        for (int k = 0; k < oMeshData.points.size(); k += 3){
            int a=k,b=k+1,c=k+2;
            pcl::PointXYZ aVertex;
            pcl::PointXYZ bVertex;
            pcl::PointXYZ cVertex;
            aVertex.x = oMeshData.points[a].x;
            aVertex.y = oMeshData.points[a].y;
            aVertex.z = oMeshData.points[a].z;
            
            bVertex.x = oMeshData.points[b].x;
            bVertex.y =oMeshData.points[b].y;
            bVertex.z = oMeshData.points[b].z;
            
            cVertex.x =oMeshData.points[c].x;
            cVertex.y = oMeshData.points[c].y;
            cVertex.z = oMeshData.points[c].z;
            
            if( aVertex.z < 2 && bVertex.z < 2 && cVertex.z < 2)
            {
                verMeshCloud->push_back(aVertex);
                verMeshCloud->push_back(bVertex);
                verMeshCloud->push_back(cVertex);
                Eigen::Vector3d v1(bVertex.x - aVertex.x, bVertex.y - aVertex.y,bVertex.z - aVertex.z);
                Eigen::Vector3d v2(cVertex.x - aVertex.x, cVertex.y - aVertex.y,cVertex.z - aVertex.z);
                Eigen::Vector3d v = v1.cross(v2);

                //  if( aVertex.z < 0.1651 && bVertex.z < 0.1651 && cVertex.z < 0.1651)
                 if( aVertex.z < 0.1651-0.9827 && bVertex.z < 0.1651-0.9827 && cVertex.z < 0.1651-0.9827)
                 {
                    Eigen::Vector3d v0(0,0,1);
                    if(v.dot(v0) > -0.01 && v.dot(v0) < 0.2)
                    {
                        vGroundCloud.push_back(aVertex);
                        vGroundCloud.push_back(bVertex);
                        vGroundCloud.push_back(cVertex);
                    }
                 }
                else if( aVertex.z > 0.05-0.9827  && bVertex.z > 0.05-0.9827  && cVertex.z > 0.05-0.9827 )
                {
                    vObstacleCloud.push_back(aVertex);
                    vObstacleCloud.push_back(bVertex);
                    vObstacleCloud.push_back(cVertex);
                }
            }
        }//end k

    // // 
    // std::vector<double> ver(2,0);
    // std::vector<std::vector<double>> vBoundary;
    // double max_theta = 0.0,min_theta = 0.0;

    // for (int i = 0; i <verMeshCloud->points.size(); i++)
    //     {
    //     ver[0] = sqrt(pow(verMeshCloud->points[i].x,2)+pow(verMeshCloud->points[i].y,2));
    //     ver[1] = atan(verMeshCloud->points[i].y/verMeshCloud->points[i].x) * 180 / 3.14;
    //     vBoundary.push_back(ver);
    //             if (ver[1]<min_theta) min_theta = ver[1];
    //     if (ver[1]>max_theta) max_theta = ver[1];
    //     }

    // for(double i =min_theta;i<max_theta ;i+=0.2)
    // {
    //     double r=0.0;
    //     int index = 0;
    //     for(int j =0;j<vBoundary.size();j++)
    //     {
    //         if (vBoundary[j][1]>i && vBoundary[j][1]< i+0.2)
    //         {
    //             if(vBoundary[j][0] > r)
    //             {
    //                 r = vBoundary[j][0];
    //                 index = j;
    //             }
    //         }
    //     }
    //     pcl::PointXYZ BoundPoint = verMeshCloud->points[index];
    //     if(BoundPoint.z > -0.01 && BoundPoint.z < 1)
    //     {
    //         BoundPoint.z = 0;
    //         vBoundCloud.push_back(BoundPoint);
    //     }
    // }

          //******************deviding section******************
      //a class to divide point clouds into the given number of sectors
      DivideSector oSectorDivider(m_oGPThrs.iSector_num);
      //compute the corresponding trajectory point
      pcl::PointXYZ oCurrentTrajP;
      
      //if have corresponding trajectory point (viewpoint)
      if( vTrajHistory.size() ){
        //
        oCurrentTrajP = ComputeQueryTraj(oMeshData.header.stamp);
        //
        oCurrentTrajP.z = oCurrentTrajP.z - 0.9827;
        oSectorDivider.SetOriginPoint(oCurrentTrajP);
      }
         
      //ROS_INFO("Querytime: %f, have trajpoint: %d", .header.stamp.toSec(), bCurTrajPFlag);
      
      //preparation
      std::vector<std::vector<int> > oPointSecIdxs;///<point index reorganization according to sectors
      std::vector<std::vector<GroundFeature> > vGroundFeatures = oSectorDivider.ComputePointSectorIdxs(*verMeshCloud, oPointSecIdxs);
      
      std::vector<std::vector<int> > vAllGroundRes;///<point value according to oPointSecIdxs
      for (int i = 0; i != vGroundFeatures.size(); ++i) {
          std::vector<int> vTmpRes(vGroundFeatures[i].size(),0);
          vAllGroundRes.push_back(vTmpRes);
      }
     //******************deviding section******************
      //***********and adopt GP-INSAC algorithm*************
      //to a sector
      for (int is = 0; is != oPointSecIdxs.size(); ++is) {

          //***********GP algorithm***********
          //new the class of Gaussian Process algorithm class
          GaussianProcessRegression<float> GPR(1, 1);
          
          //paramters
          GPR.SetHyperParams(m_oGPThrs.dLScale, m_oGPThrs.dSigmaF, m_oGPThrs.dSigmaN);

          INSAC GPINSAC(float(m_oGPThrs.dSigmaN), m_oGPThrs.fModelThr, m_oGPThrs.fDataThr);

          //select the initial training input
          GPINSAC.SetSeedThreshold(m_oGPThrs.fDisThr,m_oGPThrs.fZLower,m_oGPThrs.fZUpper);
          GPINSAC.SelectSeeds(vGroundFeatures[is]);

          //***********INSAC algorithm***********
          bool bGrowFlag = true;
          
          //looping until there is not new seed to be involved in current sector
          while(bGrowFlag){
               
               //new training vector - NEW input, NEW means the input does not include old one
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainFeaVec;
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainTruVec;
               GPINSAC.ToAddTrainSamples(vTrainFeaVec, vTrainTruVec, vGroundFeatures[is]);

               //new test vector - NEW output
               std::vector<Eigen::Matrix<float, 1, 1> > vTestFeaVec;
               std::vector<Eigen::Matrix<float, 1, 1> > vTestTruVec;
               GPINSAC.ToAddTestSamples(vTestFeaVec, vTestTruVec, vGroundFeatures[is]);
               
               //training
               GPR.AddTrainingDatas(vTrainFeaVec, vTrainTruVec);
    
               //regression (prediction)
               std::vector<int> vOneLoopLabels;
               for (size_t k = 0; k < vTestFeaVec.size(); k++) {
        
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
                    
                    //below is for parameter optimization only
                    //std::cout <<"vTestFeaVec[k] :"<< vTestFeaVec[k] << std::endl;

                    //regression of new input based on the computed training matrix
                    if (GPR.Regression(vPredValue, vPredVar, vTestFeaVec[k])){
                       
                       //belows are for parameter optimization only
                       //std::cout << "predict" << vPredValue(0) << std::endl;
                       //std::cout << "groundtruth" << vTestTruVec[k](0) << std::endl;
                       //std::cout << "var" << vPredVar(0) << std::endl;

                       //judgement the result is meet the model or not
                       int iPointLabel = GPINSAC.Eval(vTestTruVec[k](0),vPredValue(0), vPredVar(0));
                       vOneLoopLabels.push_back(iPointLabel);

                    }else

                       vOneLoopLabels.push_back(-1);//obstacle
               }
               
               //refresh the remaining unknown points
               if(GPINSAC.AssignIdx(vOneLoopLabels))
                  bGrowFlag = false;
    
          }//end while

          //assigment in current sector
          std::vector<int> vGroundLabels;
          GPINSAC.OutputRes(vGroundLabels);
      
          for (int j = 0; j != vGroundLabels.size(); ++j)
              vAllGroundRes[is][j] = vGroundLabels[j];

      }//end is
  
       //***********boundary extraction***********

      //assigment of result in whole point clouds
      std::vector<int> vCloudRes(verMeshCloud->points.size(),0);
       //assignment in whole point clouds
       for (int is = 0; is != oPointSecIdxs.size(); ++is) {
            for (int j = 0; j != oPointSecIdxs[is].size(); ++j) {
               
                  int iPointIdx = oPointSecIdxs[is][j];
                  vCloudRes[iPointIdx] = vAllGroundRes[is][j];

            }//end for j
       }//end for i

       //new a boundary class
       Boundary oBounder;
       //input the segment labels
       oBounder.GetSegmentClouds(verMeshCloud, vCloudRes);
       //compute boundary point
       oBounder.ComputeBoundary();
       //output the boundary cloud
       oBounder.OutputBoundClouds(vCloudRes);

    for (int i = 0; i != vCloudRes.size(); ++i) {
        //if point is a ground point
        if( vCloudRes[i] == 1){
            //take data
            // vGroundCloud.push_back(verMeshCloud->points[i]);
        //if point is an obstacle point
        }else if(vCloudRes[i] == -1){
            //take data
            // vObstacleCloud.push_back(verMeshCloud->points[i]);
        //if point is an boundary point
        }else if(  vCloudRes[i] == 2)
            //take data
            // verBoundCloud->push_back(verMeshCloud->points[i]);
            vBoundCloud.push_back(verMeshCloud->points[i]);
       }//end for i

    // std::vector<double> ver(2,0);
    // std::vector<std::vector<double>> vBoundary;
    // double max_theta = 0.0,min_theta = 0.0;

    // for (int i = 0; i < verBoundCloud->points.size(); i++)
    //     {
    //     ver[0] = sqrt(pow( verBoundCloud->points[i].x,2)+pow( verBoundCloud->points[i].y,2));
    //     ver[1] = atan( verBoundCloud->points[i].y/ verBoundCloud->points[i].x) * 180 / 3.14;
    //     vBoundary.push_back(ver);
    //             if (ver[1]<min_theta) min_theta = ver[1];
    //     if (ver[1]>max_theta) max_theta = ver[1];
    //     }

    // for(double i =min_theta;i<max_theta ;i+=0.2)
    // {
    //     double r=0.0;
    //     int index = 0;
    //     for(int j =0;j<vBoundary.size();j++)
    //     {
    //         if (vBoundary[j][1]>i && vBoundary[j][1]< i+0.2)
    //         {
    //             if(vBoundary[j][0] > r)
    //             {
    //                 r = vBoundary[j][0];
    //                 index = j;
    //             }
    //         }
    //     }
    //     pcl::PointXYZ BoundPoint =  verBoundCloud->points[index];
    //     if(BoundPoint.z > -0.01 && BoundPoint.z < 1)
    //     {
    //         BoundPoint.z = 0;
    //         vBoundCloud.push_back(BoundPoint);
    //     }
    // }

        // PublishMeshs(vObstacleCloud);
        //PublishMeshs(vGroundCloud);

       //publish ground points
       pcl::toROSMsg(vGroundCloud, vGroundPub);
       vGroundPub.header.frame_id = BaselinkOdomFrame;
       vGroundPub.header.stamp = oMeshData.header.stamp;
       m_oGroundPub.publish(vGroundPub);
       

       //publish ground points
       pcl::toROSMsg(vBoundCloud, vBoundPub);
       vBoundPub.header.frame_id = BaselinkOdomFrame;
       vBoundPub.header.stamp = oMeshData.header.stamp;
       m_oBoundPub.publish(vBoundPub);

       //publish obstacle points
       pcl::toROSMsg(vObstacleCloud, vObstaclePub);
       vObstaclePub.header.frame_id = BaselinkOdomFrame;
       vObstaclePub.header.stamp = oMeshData.header.stamp;
       m_oObstaclePub.publish(vObstaclePub);

  }//end if m_iFrames%m_iSampleNum (down sampling)
}

void GroundExtraction::PublishMeshs(const pcl::PointCloud<pcl::PointXYZ> &vMeshVertices){

        //new a visual message
        visualization_msgs::Marker vLaserData;

        //define header of message
        vLaserData.header.frame_id = BaselinkOdomFrame;
        vLaserData.header.stamp = ros::Time::now();
        vLaserData.type = visualization_msgs::Marker::TRIANGLE_LIST;
        vLaserData.action = visualization_msgs::Marker::ADD;

        vLaserData.scale.x = 1.0;
        vLaserData.scale.y = 1.0;
        vLaserData.scale.z = 1.0;

        vLaserData.pose.position.x = 0.0;
        vLaserData.pose.position.y = 0.0;
        vLaserData.pose.position.z = 0.0;

        vLaserData.pose.orientation.x = 0.0;
        vLaserData.pose.orientation.y = 0.0;
        vLaserData.pose.orientation.z = 0.0;
        vLaserData.pose.orientation.w = 1.0;

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
        vLaserData.points.push_back(oPTemp);
        vLaserData.color = color;

        }//end k

        m_oMeshPublisher.publish(vLaserData);

}



void GroundExtraction::HandlePointClouds(const sensor_msgs::PointCloud2 & vLaserData )
{
  
  //count input frames
  m_iFrames++;
  //ROS_INFO("frame number: %d", m_iFrames);

  if(!(m_iFrames%m_iSampleNum)){

      ////a point clouds in PCL type
      pcl::PointCloud<pcl::PointXYZ> vInputCloud;
      pcl::PointCloud<pcl::PointXYZ> vInputCloud1;
      pcl::PointCloud<pcl::PointXYZ>::Ptr vOneCloud(new pcl::PointCloud<pcl::PointXYZ>);
      ////message from ROS type to PCL type
      pcl::fromROSMsg(vLaserData, vInputCloud1);
      if(OdomFrame != BaselinkOdomFrame)
      {
        pcl::transformPointCloud( vInputCloud1, vInputCloud,Odom2BaselinkOdomTF);        
      }
      
      //get right point clouds from frame output
      for(int i = 0; i != vInputCloud.size(); ++i ){
         
         pcl::PointXYZ oArgPoint;
         oArgPoint.x = vInputCloud.points[i].x;
         oArgPoint.y = vInputCloud.points[i].y;
         oArgPoint.z = vInputCloud.points[i].z;
         vOneCloud->points.push_back(oArgPoint);

      }  

      //******************deviding section******************
      //a class to divide point clouds into the given number of sectors
      DivideSector oSectorDivider(m_oGPThrs.iSector_num);
      //compute the corresponding trajectory point
      pcl::PointXYZ oCurrentTrajP;
      
      //if have corresponding trajectory point (viewpoint)
      if( vTrajHistory.size() ){
        //
        oCurrentTrajP = ComputeQueryTraj(vLaserData.header.stamp);
        //
        oCurrentTrajP.z = oCurrentTrajP.z - 0.9827; //LIO-SAM 雷达系与载体系不一致，变换坐标系
        if(OdomFrame != BaselinkOdomFrame)
        // if(false)
        {      
            pcl::PointCloud<pcl::PointXYZ> InCloud;
            pcl::PointCloud<pcl::PointXYZ> OutCloud;

            InCloud.push_back(oCurrentTrajP);
            pcl::transformPointCloud(InCloud,OutCloud,Odom2BaselinkOdomTF);
            oCurrentTrajP = OutCloud.points[0];
            // oCurrentTrajP.z = oCurrentTrajP.z - 0.9827;
            /*Debug
            #ifdef DEBUG
                std::cout<<"\033[31mtrav DEBUG \033[1m\033[34mlidar2Baselink:"<<std::endl;
                std::cout << "  InCloud.points[0]: " << InCloud.points[0]<<std::endl;
                std::cout << "  OutCloud.points[0]: " << OutCloud.points[0]<<std::endl;
                std::cout << "  oCurrentTrajP: " << oCurrentTrajP;
                std::cout << "\033[0m"<<std::endl;
            #endif//*/

        }

        oSectorDivider.SetOriginPoint(oCurrentTrajP);
      }

    #ifdef DEBUG
        pcl::PointCloud<pcl::PointXYZI>::Ptr debugPointC(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i = 0; i != vInputCloud.size(); ++i ){
            pcl::PointXYZI oArgPoint;
            oArgPoint.x = vInputCloud.points[i].x;
            oArgPoint.y = vInputCloud.points[i].y;
            oArgPoint.z = vInputCloud.points[i].z;
            oArgPoint.intensity = 0;
            debugPointC->points.push_back(oArgPoint);
        } 
        pcl::PointXYZI viewPoint;
        viewPoint.x = oCurrentTrajP.x;
        viewPoint.y = oCurrentTrajP.y;
        viewPoint.z = oCurrentTrajP.z;
        viewPoint.intensity = 1;
        debugPointC->points.push_back(viewPoint);
        sensor_msgs::PointCloud2 pubCloudData;
        pcl::toROSMsg(*debugPointC, pubCloudData);
        pubCloudData.header.frame_id = "odom";
        pubCloudData.header.stamp = ros::Time::now();
        debugSubPointCloud.publish(pubCloudData);
        // ROS_INFO("Querytime: %f, have trajpoint: %d", .header.stamp.toSec(), bCurTrajPFlag);
    #endif
      
      //preparation
      std::vector<std::vector<int> > oPointSecIdxs;///<point index reorganization according to sectors
      std::vector<std::vector<GroundFeature> > vGroundFeatures = oSectorDivider.ComputePointSectorIdxs(*vOneCloud, oPointSecIdxs);
      
      std::vector<std::vector<int> > vAllGroundRes;///<point value according to oPointSecIdxs
      for (int i = 0; i != vGroundFeatures.size(); ++i) {
          std::vector<int> vTmpRes(vGroundFeatures[i].size(),0);
          vAllGroundRes.push_back(vTmpRes);
      }

      //******************deviding section******************
      //***********and adopt GP-INSAC algorithm*************
      //to a sector
      for (int is = 0; is != oPointSecIdxs.size(); ++is) {

          //***********GP algorithm***********
          //new the class of Gaussian Process algorithm class
          GaussianProcessRegression<float> GPR(1, 1);
          
          //paramters
          GPR.SetHyperParams(m_oGPThrs.dLScale, m_oGPThrs.dSigmaF, m_oGPThrs.dSigmaN);

          INSAC GPINSAC(float(m_oGPThrs.dSigmaN), m_oGPThrs.fModelThr, m_oGPThrs.fDataThr);

          //select the initial training input
          GPINSAC.SetSeedThreshold(m_oGPThrs.fDisThr,m_oGPThrs.fZLower,m_oGPThrs.fZUpper);
          GPINSAC.SelectSeeds(vGroundFeatures[is]);

          //***********INSAC algorithm***********
          bool bGrowFlag = true;
          
          //looping until there is not new seed to be involved in current sector
          while(bGrowFlag){
               
               //new training vector - NEW input, NEW means the input does not include old one
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainFeaVec;
               Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> vTrainTruVec;
               GPINSAC.ToAddTrainSamples(vTrainFeaVec, vTrainTruVec, vGroundFeatures[is]);

               //new test vector - NEW output
               std::vector<Eigen::Matrix<float, 1, 1> > vTestFeaVec;
               std::vector<Eigen::Matrix<float, 1, 1> > vTestTruVec;
               GPINSAC.ToAddTestSamples(vTestFeaVec, vTestTruVec, vGroundFeatures[is]);
               
               //training
               GPR.AddTrainingDatas(vTrainFeaVec, vTrainTruVec);
    
               //regression (prediction)
               std::vector<int> vOneLoopLabels;
               for (size_t k = 0; k < vTestFeaVec.size(); k++) {
        
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredValue;
                    Eigen::Matrix<float, Eigen::Dynamic, 1> vPredVar;
                    
                    //below is for parameter optimization only
                    //std::cout <<"vTestFeaVec[k] :"<< vTestFeaVec[k] << std::endl;

                    //regression of new input based on the computed training matrix
                    if (GPR.Regression(vPredValue, vPredVar, vTestFeaVec[k])){
                       
                       //belows are for parameter optimization only
                       //std::cout << "predict" << vPredValue(0) << std::endl;
                       //std::cout << "groundtruth" << vTestTruVec[k](0) << std::endl;
                       //std::cout << "var" << vPredVar(0) << std::endl;

                       //judgement the result is meet the model or not
                       int iPointLabel = GPINSAC.Eval(vTestTruVec[k](0),vPredValue(0), vPredVar(0));
                       vOneLoopLabels.push_back(iPointLabel);

                    }else

                       vOneLoopLabels.push_back(-1);//obstacle
               }
               
               //refresh the remaining unknown points
               if(GPINSAC.AssignIdx(vOneLoopLabels))
                  bGrowFlag = false;
    
          }//end while

          //assigment in current sector
          std::vector<int> vGroundLabels;
          GPINSAC.OutputRes(vGroundLabels);
      
          for (int j = 0; j != vGroundLabels.size(); ++j)
              vAllGroundRes[is][j] = vGroundLabels[j];

      }//end is
  
       //***********boundary extraction***********

      //assigment of result in whole point clouds
      std::vector<int> vCloudRes(vOneCloud->points.size(),0);
       //assignment in whole point clouds
       for (int is = 0; is != oPointSecIdxs.size(); ++is) {
            for (int j = 0; j != oPointSecIdxs[is].size(); ++j) {
               
                  int iPointIdx = oPointSecIdxs[is][j];
                  vCloudRes[iPointIdx] = vAllGroundRes[is][j];

            }//end for j
       }//end for i

       //new a boundary class
       Boundary oBounder;
       //input the segment labels
       oBounder.GetSegmentClouds(vOneCloud, vCloudRes);
       //compute boundary point
       oBounder.ComputeBoundary();
       //output the boundary cloud
       oBounder.OutputBoundClouds(vCloudRes);

       //************output value******************
       pcl::PointCloud<pcl::PointXYZ> vGroundCloud;
       sensor_msgs::PointCloud2 vGroundPub;
       pcl::PointCloud<pcl::PointXYZ> vObstacleCloud;
       sensor_msgs::PointCloud2 vObstaclePub;
       pcl::PointCloud<pcl::PointXYZ> vBoundCloud;
       sensor_msgs::PointCloud2 vBoundPub;
      
       for (int i = 0; i != vCloudRes.size(); ++i) {
                         
                  //if point is a ground point
                  if( vCloudRes[i] == 1){
                       //take data
                       vGroundCloud.push_back(vInputCloud.points[i]);
                  //if point is an obstacle point
                  }else if(vCloudRes[i] == -1){
                      //take data
                      vObstacleCloud.push_back(vInputCloud.points[i]);
                  //if point is an boundary point
                  }else if(  vCloudRes[i] == 2)
                      //take data
                      vBoundCloud.push_back(vInputCloud.points[i]);

       }//end for i

        // if(OdomFrame != BaselinkOdomFrame)
        // {
        //     pcl::transformPointCloud(vGroundCloud,vGroundCloud,Odom2BaselinkOdomTF);
        //     pcl::transformPointCloud(vObstacleCloud,vObstacleCloud,Odom2BaselinkOdomTF);
        //     pcl::transformPointCloud(vBoundCloud,vBoundCloud,Odom2BaselinkOdomTF);
            
        // }

       //publish ground points
       pcl::toROSMsg(vGroundCloud, vGroundPub);
       vGroundPub.header.frame_id = BaselinkOdomFrame;
       vGroundPub.header.stamp = vLaserData.header.stamp;
       m_oGroundPub.publish(vGroundPub);

       //publish bound points
       pcl::toROSMsg(vBoundCloud, vBoundPub);
       vBoundPub.header.frame_id = BaselinkOdomFrame;
       vBoundPub.header.stamp = vLaserData.header.stamp;
       m_oBoundPub.publish(vBoundPub);

       //publish obstacle points
       pcl::toROSMsg(vObstacleCloud, vObstaclePub);
       vObstaclePub.header.frame_id = BaselinkOdomFrame;
       vObstaclePub.header.stamp = vLaserData.header.stamp;
       m_oObstaclePub.publish(vObstaclePub);

       //OutputGroundPoints(vGroundCloud, .header.stamp);
       OutputAllPoints(vOneCloud, vCloudRes, vLaserData.header.stamp);

  }//end if m_iFrames%m_iSampleNum (down sampling)

}



/*************************************************
Function: HandleTrajectory
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but timestamp values 
Return: none
Others: none
*************************************************/
void GroundExtraction::HandleTrajectory(const nav_msgs::Odometry & oTrajectory)
{
  
  
  //std::cout << "m_bTxtOutFlag value: " << m_bTxtOutFlag << std::endl;

  if(m_iTrajPointNum < 0 && m_bTxtOutFlag){

    //set the current time stamp as a file name
    //full name 
    m_sOutTrajFileName << m_sFileHead << "_Traj_" << oTrajectory.header.stamp << ".txt"; 

  }
  
  //count input frames
  m_iTrajPointNum++;
  
  //save the into the memory
  //save the position of trajectory
  TrajectoryPoint oTrajPoint;
  oTrajPoint.position.x = oTrajectory.pose.pose.position.x;//z in loam is x
  oTrajPoint.position.y = oTrajectory.pose.pose.position.y;//x in loam is y
  oTrajPoint.position.z = oTrajectory.pose.pose.position.z;//y in loam is z
  //save record time
  oTrajPoint.oTimeStamp =  oTrajectory.header.stamp;

  vTrajHistory.push(oTrajPoint);

  nav_msgs::Odometry oCurrOdom = oTrajectory;
//   oCurrOdom.header.stamp = ros::Time::now();
  oCurrOdom.header.stamp=  oTrajectory.header.stamp;
  oCurrOdom.header.frame_id = BaselinkOdomFrame;

  //set the position
  //oCurrOdom.pose.pose.position.x = oTrajPoint.position.x;
  //oCurrOdom.pose.pose.position.y = oTrajPoint.position.y;
//   oCurrOdom.pose.pose.position.z = oCurrOdom.pose.pose.position.z - 0.583;
//   oCurrOdom.pose.pose.position.z = oCurrOdom.pose.pose.position.z - 0.9827;
  m_oHighOdomPub.publish(oCurrOdom);

  if(m_bTxtOutFlag){

     //output
     std::ofstream oTrajFile;
     oTrajFile.open(m_sOutTrajFileName.str(), std::ios::out | std::ios::app);

     //output in a txt file
     oTrajFile << oTrajPoint.position.x << " "
              << oTrajPoint.position.y << " "
              << oTrajPoint.position.z << " " 
              << oTrajPoint.oTimeStamp << " "
              << m_iTrajPointNum << " "
              << std::endl;
     oTrajFile.close();

  }//if m_bTxtOutFlag

}

/*************************************************
Function: InterpolateTraj
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
void GroundExtraction::InterpolateTraj(const TrajectoryPoint & oCurrent,
                                                                      const TrajectoryPoint & oPast,
                                                                      const float& ratio,
                                                                      pcl::PointXYZ & oResTraj){
    //The ratio is from the interpolated value to oCurrent value 
    float invRatio = 1 - ratio;
    //p+(c-p)(1-r)
    oResTraj.x = oCurrent.position.x * invRatio + oPast.position.x * ratio;
    oResTraj.y = oCurrent.position.y * invRatio + oPast.position.y * ratio;
    oResTraj.z = oCurrent.position.z * invRatio + oPast.position.z * ratio;

}

/*************************************************
Function: ComputeQueryTraj
Description: a callback function in below:
             m_oOdomSuber = node.subscribe(m_sOdomTopic, 5, &GroundExtraction::HandleTrajectory, this);
Calls: none
Called By: TransformLaserInOdom, which is the construction function
Table Accessed: none
Table Updated: none
Input: rawpoint, a 3d point with pcl point type
Output: a point clouds are almost the same with raw point clouds but only their timestamp values are modified
Return: none
Others: none
*************************************************/
pcl::PointXYZ GroundExtraction::ComputeQueryTraj(const ros::Time & oQueryTime){

    pcl::PointXYZ oResTraj;
    //clear the output
    oResTraj.x = 0.0;
    oResTraj.y = 0.0;
    oResTraj.z = 0.0;
    //index
    int iTrajIdx = 0;
    //time different
    double timeDiff = (oQueryTime - vTrajHistory[iTrajIdx].oTimeStamp).toSec();
    //search the most recent time
    while (iTrajIdx < vTrajHistory.size() - 1 && timeDiff > 0) {
        //increase index
        iTrajIdx++;
        //time different
        timeDiff = (oQueryTime - vTrajHistory[iTrajIdx].oTimeStamp).toSec();
    }

    //if the querytime is out of the stored time section 
    if (iTrajIdx == 0 || timeDiff > 0) {
       //turn back zero
       oResTraj.x = vTrajHistory[iTrajIdx].position.x;
       oResTraj.y = vTrajHistory[iTrajIdx].position.y;
       oResTraj.z = vTrajHistory[iTrajIdx].position.z;

    } else {//if it is between two stored times
       //get the ratio
        //ROS_INFO("Trajtime between: %f and %f", vTrajHistory[iTrajIdx].oTimeStamp.toSec(), vTrajHistory[iTrajIdx - 1].oTimeStamp.toSec());

        float ratio = - timeDiff / (vTrajHistory[iTrajIdx].oTimeStamp - vTrajHistory[iTrajIdx - 1].oTimeStamp).toSec();
        //interpolate an accuracy value
        InterpolateTraj(vTrajHistory[iTrajIdx], vTrajHistory[iTrajIdx - 1], ratio, oResTraj);
    }

    return oResTraj;

}