#include "MeshMap.h"

void MeshMap::InitMeshMap(const mesh_navigation::FrameRecon & SingleMeshMsg){
    Eigen::Vector3f z_axis_vector(0,0,1);

    for (auto &&partMesh : SingleMeshMsg.ReconMesh)
    {
        for(auto && triangles : partMesh.triangles ){
            pcl::PointNormal CenterPointN;
            CenterPointN.x = 0.0;
            CenterPointN.y = 0.0;
            CenterPointN.z = 0.0;
            CenterPointN.normal_x = 0.0;
            CenterPointN.normal_y = 0.0;
            CenterPointN.normal_z = 0.0;
            pcl::Vertices oVertices;
            std::vector<pcl::PointXYZ> onePolygon;
            // int VerticesNum = 0;
            for (auto && vertices : triangles.vertex_indices)
            {
                onePolygon.push_back(pcl::PointXYZ(partMesh.vertices[vertices].x, partMesh.vertices[vertices].y, partMesh.vertices[vertices].z));

                oVertices.vertices.push_back(VerticesPoints->points.size() + vertices);

                // 多边形中点及面片法向
                CenterPointN.x = CenterPointN.x + partMesh.vertices[vertices].x;
                CenterPointN.y = CenterPointN.y + partMesh.vertices[vertices].y;
                CenterPointN.z = CenterPointN.z + partMesh.vertices[vertices].z;
            }
            CenterPointN.x = CenterPointN.x / float(onePolygon.size());
            CenterPointN.y = CenterPointN.y / float(onePolygon.size());
            CenterPointN.z = CenterPointN.z / float(onePolygon.size());
            Eigen::Vector3f side_ab(onePolygon[1].x - onePolygon[0].x,onePolygon[1].y-onePolygon[0].y,onePolygon[1].z-onePolygon[0].z);
            Eigen::Vector3f side_cb(onePolygon[1].x - onePolygon[2].x,onePolygon[1].y-onePolygon[2].y,onePolygon[1].z-onePolygon[2].z);
            Eigen::Vector3f Normal = -side_ab.cross(side_cb).normalized();
            CenterPointN.normal_x = Normal.x();
            CenterPointN.normal_y = Normal.y();
            CenterPointN.normal_z = Normal.z();

            PolygonCenterPNormal->push_back(CenterPointN);
            Mesh->polygons.push_back(oVertices);
            meshmap.Triangles.push_back(oVertices); // MeshMap 中的面索引

            // 通过法向初步分类mesh中多边形的类型
            if(1.0 - Normal.dot(z_axis_vector) < 0.01) // 向量与Z轴垂直范围
            {
                // PolygonCenterPNormal->push_back(CenterPointN);
                if(onePolygon[0].z > 0.5 && onePolygon[1].z > 0.5 && onePolygon[2].z > 0.5 )
                {
                    meshmap.TriangleLableList.push_back(NONGROUND);
                }
                else{
                    meshmap.TriangleLableList.push_back(GROUND);
                }
            }
            else 
            {
                if(abs(Normal.dot(z_axis_vector)) < 0.05)
                {
                    meshmap.TriangleLableList.push_back(NONGROUND);
                }
                else if(onePolygon[0].z > 0.5 || onePolygon[1].z > 0.5 || onePolygon[2].z > 0.5 )
                {
                    meshmap.TriangleLableList.push_back(NONGROUND);
                }
                else
                {
                    meshmap.TriangleLableList.push_back(BOUND);
                }
            }

        }

        for (auto &&point : partMesh.vertices)
        {
            VerticesPoints->push_back(pcl::PointXYZ(point.x, point.y, point.z));
        }
    }
    pcl::toPCLPointCloud2(*VerticesPoints,  Mesh->cloud);
  
    pcl::fromROSMsg(SingleMeshMsg.FramePNormal,*SingleFramePNormal);
    meshmap.VerticesCloud = *VerticesPoints;
    meshmap.CenterPNormal = *PolygonCenterPNormal;

    // debug 
    pcl::PointNormal z_axis;
    z_axis.x = 0;
    z_axis.y = 0;
    z_axis.z = 0;
    z_axis.normal_x = z_axis_vector.x();
    z_axis.normal_y = z_axis_vector.y();
    z_axis.normal_z = z_axis_vector.z();
    PolygonCenterPNormal->push_back(z_axis);
}

void MeshMap::InitLayerPlugins()
{
    
}


void MeshMap::PublishMeshMap(std::string TFId,ros::Publisher MeshMapPublisher)
{
    visualization_msgs::MarkerArray PolygonMarkerList;
    // pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
    // pcl::fromPCLPointCloud2(meshmap.VerticesCloud, vPublishCloud);
    for (PolygonType polygontype = GROUND; polygontype <= PolygonTypeEnd; polygontype = (PolygonType)(polygontype+1))
    {
        visualization_msgs::Marker polygonMarker;            
        polygonMarker.header.frame_id = "map";
        polygonMarker.header.stamp = ros::Time::now();
        polygonMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        polygonMarker.action = visualization_msgs::Marker::MODIFY;
        polygonMarker.id = polygontype; 

        polygonMarker.scale.x = 1.0;
        polygonMarker.scale.y = 1.0;
        polygonMarker.scale.z = 1.0;

        polygonMarker.pose.position.x = 0.0;
        polygonMarker.pose.position.y = 0.0;
        polygonMarker.pose.position.z = 0.0;

        polygonMarker.pose.orientation.x = 0.0;
        polygonMarker.pose.orientation.y = 0.0;
        polygonMarker.pose.orientation.z = 0.0;
        polygonMarker.pose.orientation.w = 1.0;

        polygonMarker.color = ColorWheel(int(polygontype));
        for (int i = 0; i != meshmap.Triangles.size(); ++i){

            if( meshmap.TriangleLableList[i] == polygontype)
            {
                for (auto && vertices:meshmap.Triangles[i].vertices)
                {
                    geometry_msgs::Point oPTemp;
                    oPTemp.x = meshmap.VerticesCloud.points[vertices].x;
                    oPTemp.y = meshmap.VerticesCloud.points[vertices].y;
                    oPTemp.z = meshmap.VerticesCloud.points[vertices].z;
                    polygonMarker.points.push_back(oPTemp);
                }
            }
        }
        PolygonMarkerList.markers.push_back(polygonMarker);
    }
    MeshMapPublisher.publish(PolygonMarkerList);
}

void MeshMap::PublishMeshMap(std::string TFId,MeshMap::mashMap map,ros::Publisher MeshMapPublisher)
{
    visualization_msgs::MarkerArray PolygonMarkerList;
    // pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
    // pcl::fromPCLPointCloud2(meshmap.VerticesCloud, vPublishCloud);
    for (PolygonType polygontype = GROUND; polygontype <= PolygonTypeEnd; polygontype = (PolygonType)(polygontype+1))
    {
        visualization_msgs::Marker polygonMarker;            
        polygonMarker.header.frame_id = "map";
        polygonMarker.header.stamp = ros::Time::now();
        polygonMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        polygonMarker.action = visualization_msgs::Marker::MODIFY;
        polygonMarker.id = polygontype; 

        polygonMarker.scale.x = 1.0;
        polygonMarker.scale.y = 1.0;
        polygonMarker.scale.z = 1.0;

        polygonMarker.pose.position.x = 0.0;
        polygonMarker.pose.position.y = 0.0;
        polygonMarker.pose.position.z = 0.0;

        polygonMarker.pose.orientation.x = 0.0;
        polygonMarker.pose.orientation.y = 0.0;
        polygonMarker.pose.orientation.z = 0.0;
        polygonMarker.pose.orientation.w = 1.0;

        polygonMarker.color = ColorWheel(int(polygontype));
        for (int i = 0; i != map.Triangles.size(); ++i){

            if( map.TriangleLableList[i] == polygontype)
            {
                for (auto && vertices:map.Triangles[i].vertices)
                {
                    geometry_msgs::Point oPTemp;
                    oPTemp.x = map.VerticesCloud.points[vertices].x;
                    oPTemp.y = map.VerticesCloud.points[vertices].y;
                    oPTemp.z = map.VerticesCloud.points[vertices].z;
                    polygonMarker.points.push_back(oPTemp);
                }
            }
        }
        PolygonMarkerList.markers.push_back(polygonMarker);
    }
    MeshMapPublisher.publish(PolygonMarkerList);
}
