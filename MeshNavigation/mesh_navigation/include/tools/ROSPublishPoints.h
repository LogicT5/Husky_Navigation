#ifndef __ROS_PUBLISH_POINTS_H__
#define __ROS_PUBLISH_POINTS_H__

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ROSPublishPoints{

void HSVToRGB(float H, float S, float V, float &R, float &G, float &B);
void HSVToRGB(float H, float S, float V, uint8_t &R, uint8_t &G, uint8_t &B);

class ROSPublishPoints
{
public:
    void PublisOnePointAndNormal(const pcl::PointNormal PointNormal,std::string OutTFId,ros::Publisher PointNormalPub)
    {
        visualization_msgs::MarkerArray MarkerArray;
	    constexpr int index = 1e5;

            // make points
            visualization_msgs::Marker oPointMarker;
            oPointMarker.header.frame_id = OutTFId;
            oPointMarker.header.stamp = ros::Time::now();
            oPointMarker.type = visualization_msgs::Marker::POINTS;
            oPointMarker.action = visualization_msgs::Marker::MODIFY;
            oPointMarker.id = index;

            oPointMarker.scale.x = 0.1;
            oPointMarker.scale.y = 0.1;

            oPointMarker.pose.position.x = 0.0;
            oPointMarker.pose.position.y = 0.0;
            oPointMarker.pose.position.z = 0.0;

            oPointMarker.pose.orientation.x = 0.0;
            oPointMarker.pose.orientation.y = 0.0;
            oPointMarker.pose.orientation.z = 0.0;
            oPointMarker.pose.orientation.w = 1.0;

            oPointMarker.color.a = 1;
            oPointMarker.color.r = 0.8;
            oPointMarker.color.g = 0.2;
            oPointMarker.color.b = 0.8;

            geometry_msgs::Point point;
            point.x = PointNormal.x;
            point.y = PointNormal.y;
            point.z = PointNormal.z;
            oPointMarker.points.push_back(point);

            MarkerArray.markers.push_back(oPointMarker);

            // make normals
            visualization_msgs::Marker oNormalMarker;
            oNormalMarker.header.frame_id = OutTFId;
            oNormalMarker.header.stamp = ros::Time::now();
            oNormalMarker.type = visualization_msgs::Marker::LINE_LIST;
            // oNormalMarker.action = visualization_msgs::Marker::MODIFY;
            oNormalMarker.action = visualization_msgs::Marker::ADD;
            oNormalMarker.id = index + 1;

            oNormalMarker.scale.x = 0.05;

            oNormalMarker.pose.position.x = 0.0;
            oNormalMarker.pose.position.y = 0.0;
            oNormalMarker.pose.position.z = 0.0;

            oNormalMarker.pose.orientation.x = 0.0;
            oNormalMarker.pose.orientation.y = 0.0;
            oNormalMarker.pose.orientation.z = 0.0;
            oNormalMarker.pose.orientation.w = 1.0;

            oNormalMarker.color.a = 1;
            oNormalMarker.color.r = 0.8;
            oNormalMarker.color.g = 0.2;
            oNormalMarker.color.b = 0.2;

                
            // geometry_msgs::Point point;
            point.x = PointNormal.x;
            point.y = PointNormal.y;
            point.z = PointNormal.z;
            oNormalMarker.points.push_back(point);

            point.x += PointNormal.normal_x;
            point.y += PointNormal.normal_y;
            point.z += PointNormal.normal_z;
            oNormalMarker.points.push_back(point);
            

            MarkerArray.markers.push_back(oNormalMarker);

            PointNormalPub.publish(MarkerArray);
    }

    // template<class T>
    // void PublishPointCloud(const pcl::PointCloud<T> &Cloud,ros::Publisher PointCloudPub)
    // {
    //     // publish obstacle points
    //     sensor_msgs::PointCloud2 CloudMsg;
    //     pcl::toROSMsg(Cloud, CloudMsg);
    //     CloudMsg.header.frame_id = pub_CloudFrame;
    //     CloudMsg.header.stamp = ros::Time::now();
    //     PointCloudPub.publish(CloudMsg);
    // }
    // template void PublishPointCloud(const pcl::PointCloud<pcl::PointXYZ> &Cloud, ros::Publisher PointCloudPub);
    // template void PublishPointCloud(const pcl::PointCloud<pcl::PointNormal> &Cloud, ros::Publisher PointCloudPub);

    void PublishPointCloudAndNormal(const pcl::PointCloud<pcl::PointNormal> PointsNormal,std::string OutTFId,ros::Publisher PointsNormalPub)
    {
        visualization_msgs::MarkerArray MarkerArray;
	    constexpr int index = 1e5;

            // make points
            visualization_msgs::Marker oPointMarker;
            oPointMarker.header.frame_id = OutTFId;
            oPointMarker.header.stamp = ros::Time::now();
            oPointMarker.type = visualization_msgs::Marker::POINTS;
            oPointMarker.action = visualization_msgs::Marker::MODIFY;
            oPointMarker.id = index;

            oPointMarker.scale.x = 0.1;
            oPointMarker.scale.y = 0.1;

            oPointMarker.pose.position.x = 0.0;
            oPointMarker.pose.position.y = 0.0;
            oPointMarker.pose.position.z = 0.0;

            oPointMarker.pose.orientation.x = 0.0;
            oPointMarker.pose.orientation.y = 0.0;
            oPointMarker.pose.orientation.z = 0.0;
            oPointMarker.pose.orientation.w = 1.0;

            oPointMarker.color.a = 1;
            oPointMarker.color.r = 0.8;
            oPointMarker.color.g = 0.2;
            oPointMarker.color.b = 0.8;

            for(const pcl::PointNormal & oPoint : PointsNormal) {
                geometry_msgs::Point point;
                point.x = oPoint.x;
                point.y = oPoint.y;
                point.z = oPoint.z;
                oPointMarker.points.push_back(point);
            }

            MarkerArray.markers.push_back(oPointMarker);

            // make normals
            visualization_msgs::Marker oNormalMarker;
            oNormalMarker.header.frame_id = OutTFId;
            oNormalMarker.header.stamp = ros::Time::now();
            oNormalMarker.type = visualization_msgs::Marker::LINE_LIST;
            // oNormalMarker.action = visualization_msgs::Marker::MODIFY;
            oNormalMarker.action = visualization_msgs::Marker::ADD;
            oNormalMarker.id = index + 1;

            oNormalMarker.scale.x = 0.05;

            oNormalMarker.pose.position.x = 0.0;
            oNormalMarker.pose.position.y = 0.0;
            oNormalMarker.pose.position.z = 0.0;

            oNormalMarker.pose.orientation.x = 0.0;
            oNormalMarker.pose.orientation.y = 0.0;
            oNormalMarker.pose.orientation.z = 0.0;
            oNormalMarker.pose.orientation.w = 1.0;

            oNormalMarker.color.a = 1;
            oNormalMarker.color.r = 0.8;
            oNormalMarker.color.g = 0.2;
            oNormalMarker.color.b = 0.2;

            for(const pcl::PointNormal & oPoint : PointsNormal) {
                geometry_msgs::Point point;
                point.x = oPoint.x;
                point.y = oPoint.y;
                point.z = oPoint.z;
                oNormalMarker.points.push_back(point);

                point.x += oPoint.normal_x;
                point.y += oPoint.normal_y;
                point.z += oPoint.normal_z;
                oNormalMarker.points.push_back(point);
            }

            MarkerArray.markers.push_back(oNormalMarker);

            PointsNormalPub.publish(MarkerArray);
    }

//     template <class T>
//     void PublishPointCloud(const pcl::PointCloud<T> &vCloud, const std::vector<float> &vFeatures, const std::string &sTopicName, const int iQueueSize)
//     {
//         // check topic
//         if(m_vPublishers.count(sTopicName) == 0)
//         {
//             m_vPublishers[sTopicName] = m_pNodeHandle->advertise<sensor_msgs::PointCloud2>(sTopicName, iQueueSize, true);
//             m_vPublishers[sTopicName].publish(sensor_msgs::PointCloud2());
//         }
//         if(!PublishCheck(sTopicName)) return;

//         //get colors
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColorClouds(new pcl::PointCloud<pcl::PointXYZRGB>);

//         //to each point
//         for (int i = 0; i < vCloud.points.size() && i < vFeatures.size(); ++i){

//             pcl::PointXYZRGB oColorP;
//             oColorP.x = vCloud.points[i].x;
//             oColorP.y = vCloud.points[i].y;
//             oColorP.z = vCloud.points[i].z;

//             float H = abs(vFeatures[i]) * 360.0f;
//             float S = vFeatures[i] < 0.0f ? 0.0f : 1.0f;
//             float V = vFeatures[i] < 0.0f ? 0.5f : 1.0f;

//             HSVToRGB(H, S, V, oColorP.r, oColorP.g, oColorP.b);

//             pColorClouds->points.push_back(oColorP);
//         }

//         pColorClouds->width = 1;

//         pColorClouds->height = vCloud.points.size();

//         //convert to pc2 message
//         sensor_msgs::PointCloud2 vCloudData;

//         pcl::toROSMsg(*pColorClouds, vCloudData);

//         //other informations
//         vCloudData.header.frame_id = m_sFrameId;

//         vCloudData.header.stamp = ros::Time::now();

//         //publish
//     m_vPublishers[sTopicName].publish(vCloudData);
//     }
// template void PublishPointCloud(
//     const pcl::PointCloud<pcl::PointNormal> & vCloud, 
//     const std::vector<float> & vFeatures, 
//     const std::string & sTopicName,
//     const int iQueueSize);

}ROSPublishPoints;

void HSVToRGB(float H, float S, float V, float& R, float& G, float& B) {

    float C = V * S;
    int FixFactor = int(H / 60) & 1;
    float X = C * ((FixFactor ? -1 : 1) * (int(H) % 60) / 60.0 + FixFactor); // X 值的变化随H波动，锯齿状0-1之间周期变化
    float m = V - C;
    switch(int(H) / 60)
    {
        case 1:		R = X;	G = C;	B = 0; break;	// 60  <= H < 120
        case 2:		R = 0;	G = C;	B = X; break;	// 120 <= H < 180
        case 3:		R = 0;	G = X;	B = C; break;	// 180 <= H < 240
        case 4:		R = X;	G = 0;	B = C; break;	// 240 <= H < 300
        case 5:
        case 6:		R = C;	G = 0;	B = X; break;	// 300 <= H < 360
        default:	R = C;	G = X;	B = 0; 		// 0   <= H < 60 or outlier
    }
    R += m;
    G += m;
    B += m;
}

void HSVToRGB(float H, float S, float V, uint8_t& R, uint8_t& G, uint8_t& B) {

    float R_, G_, B_;
    HSVToRGB(H, S, V, R_, G_, B_);
    R = R_ * 255;
    G = G_ * 255;
    B = B_ * 255;
}
}

#endif