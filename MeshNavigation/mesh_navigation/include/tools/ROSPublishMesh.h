#ifndef __ROS_PUBLISH_MESH_H__
#define __ROS_PUBLISH_MESH_H__

#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ROSPublishMesh {

void HSVToRGB(float H, float S, float V, float &R, float &G, float &B);
void HSVToRGB(float H, float S, float V, uint8_t &R, uint8_t &G, uint8_t &B);

class ROSPublishMesh
{
    
    private:
        void InitMarkerMsg(visualization_msgs::Marker& oMeshMsgs, std::string frame_id, int id, float r, float g, float b) {
            oMeshMsgs.header.frame_id = frame_id;
            oMeshMsgs.header.stamp = ros::Time::now();
            oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
            oMeshMsgs.action = visualization_msgs::Marker::MODIFY;
            oMeshMsgs.id = id; 

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

            oMeshMsgs.color.a = 1.0;
            oMeshMsgs.color.r = r;
            oMeshMsgs.color.g = g;
            oMeshMsgs.color.b = b;
        }   
    public: 
        void PublishPolygonMesh(const pcl::PolygonMesh & PolygonMesh,std::string OutMeshTFId,ros::Publisher MarkerArrayPublisher){
            visualization_msgs::MarkerArray PolygonMarkerList;
            pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
            pcl::fromPCLPointCloud2(PolygonMesh.cloud, vPublishCloud);
            visualization_msgs::Marker polygonMarker;            
            //define header of message
            InitMarkerMsg(polygonMarker, OutMeshTFId, 0, 1. , 0.95, 0.2);
            for (int i = 0; i != PolygonMesh.polygons.size(); ++i){

                uint32_t mesh_type = PolygonMesh.polygons[i].vertices.back();
                
                //for each face vertex id
                for (int j = 0; j != 3; ++j){
                    //vertex id in each sector
                    int iVertexIdx =  PolygonMesh.polygons[i].vertices[j];

                    //temp point
                    geometry_msgs::Point oPTemp;
                    oPTemp.x = vPublishCloud.points[iVertexIdx].x;
                    oPTemp.y = vPublishCloud.points[iVertexIdx].y;
                    oPTemp.z = vPublishCloud.points[iVertexIdx].z;
                    polygonMarker.points.push_back(oPTemp);
                }//end k

            }//end j
            PolygonMarkerList.markers.push_back(polygonMarker);
            // MarkerPublisher.publish(polygonMarker);
            MarkerArrayPublisher.publish(PolygonMarkerList);
        }

        void PublishPolygonMeshList(const std::vector<pcl::PolygonMesh> PolygonMeshList,std::string OutMeshTFId,ros::Publisher MarkerArrayPublisher){
            visualization_msgs::MarkerArray PolygonMarkerList;
            int MarkerId = 0;
            for (auto &&PolygonMesh : PolygonMeshList)
            {
                pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
                pcl::fromPCLPointCloud2(PolygonMesh.cloud, vPublishCloud);
                visualization_msgs::Marker PolygonMarker;            
                //define header of message
                InitMarkerMsg(PolygonMarker, OutMeshTFId, MarkerId++, 1. , 0.95, 0.2);
                for (int i = 0; i != PolygonMesh.polygons.size(); ++i){
                    uint32_t mesh_type = PolygonMesh.polygons[i].vertices.back();
                    //for each face vertex id
                    for (int j = 0; j != 3; ++j){
                        //vertex id in each sector
                        int iVertexIdx =  PolygonMesh.polygons[i].vertices[j];
                        //temp point
                        geometry_msgs::Point oPTemp;
                        oPTemp.x = vPublishCloud.points[iVertexIdx].x;
                        oPTemp.y = vPublishCloud.points[iVertexIdx].y;
                        oPTemp.z = vPublishCloud.points[iVertexIdx].z;
                        PolygonMarker.points.push_back(oPTemp);
                    }//end j
                }//end i
                PolygonMarkerList.markers.push_back(PolygonMarker);
            }
            MarkerArrayPublisher.publish(PolygonMarkerList);
        }

        void PublishPolygonMeshAndPolygonNormal(const pcl::PolygonMesh PolygonMesh, pcl::PointCloud<pcl::PointNormal> PolygonCenterPNormal,std::string OutMeshTFId,ros::Publisher MarkerArrayPublisher){
            visualization_msgs::MarkerArray PolygonMarkerList;
	        constexpr int index = 1e5;

            pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
            pcl::fromPCLPointCloud2(PolygonMesh.cloud, vPublishCloud);
            visualization_msgs::Marker polygonMarker;            
            //define header of message
            InitMarkerMsg(polygonMarker, OutMeshTFId, 0, 224/255 , 255/255, 255/255);
            for (int i = 0; i != PolygonMesh.polygons.size(); ++i){

                uint32_t mesh_type = PolygonMesh.polygons[i].vertices.back();
                
                //for each face vertex id
                for (int j = 0; j != 3; ++j){

                    //vertex id in each sector
                    int iVertexIdx =  PolygonMesh.polygons[i].vertices[j];

                    //temp point
                    geometry_msgs::Point oPTemp;
                    oPTemp.x = vPublishCloud.points[iVertexIdx].x;
                    oPTemp.y = vPublishCloud.points[iVertexIdx].y;
                    oPTemp.z = vPublishCloud.points[iVertexIdx].z;
                    polygonMarker.points.push_back(oPTemp);
                }//end k

            }//end j
            PolygonMarkerList.markers.push_back(polygonMarker);

            // make points
            // visualization_msgs::Marker oPointMarker;
            // oPointMarker.header.frame_id = OutMeshTFId;
            // oPointMarker.header.stamp = ros::Time::now();
            // oPointMarker.type = visualization_msgs::Marker::POINTS;
            // oPointMarker.action = visualization_msgs::Marker::MODIFY;
            // oPointMarker.id = index;

            // oPointMarker.scale.x = 0.1;
            // oPointMarker.scale.y = 0.1;

            // oPointMarker.pose.position.x = 0.0;
            // oPointMarker.pose.position.y = 0.0;
            // oPointMarker.pose.position.z = 0.0;

            // oPointMarker.pose.orientation.x = 0.0;
            // oPointMarker.pose.orientation.y = 0.0;
            // oPointMarker.pose.orientation.z = 0.0;
            // oPointMarker.pose.orientation.w = 1.0;

            // oPointMarker.color.a = 1;
            // oPointMarker.color.r = 0.8;
            // oPointMarker.color.g = 0.2;
            // oPointMarker.color.b = 0.8;

            // for(const pcl::PointNormal & oPoint : PolygonCenterPNormal) {
                
            //     geometry_msgs::Point point;
            //     point.x = oPoint.x;
            //     point.y = oPoint.y;
            //     point.z = oPoint.z;
            //     oPointMarker.points.push_back(point);
            // }

            // PolygonMarkerList.markers.push_back(oPointMarker);

            // make normals
            visualization_msgs::Marker oNormalMarker;
            oNormalMarker.header.frame_id = OutMeshTFId;
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

            for(const pcl::PointNormal & oPoint : PolygonCenterPNormal) {
                
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

            PolygonMarkerList.markers.push_back(oNormalMarker);

            MarkerArrayPublisher.publish(PolygonMarkerList);
        }

        
}ROSPublishMesh;


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
} //nemespace
#endif
