#ifndef __MESH_MAP_H__
#define __MESH_MAP_H__

#include <memory>
// ros related
#include <ros/ros.h>
#include <ros/publisher.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

// polygon related
#include <visualization_msgs/Marker.h>

// pcl related
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/boundary.h>

#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Dense>

//msg
#include <shape_msgs/Mesh.h>
#include "mesh_navigation/FrameRecon.h"

class MeshMap{
private:
    //原始数据 
    pcl::PolygonMesh::Ptr Mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr VerticesPoints;
    pcl::PointCloud<pcl::PointNormal>::Ptr PolygonCenterPNormal;
    pcl::PointCloud<pcl::PointNormal>::Ptr SingleFramePNormal;

public:
    enum PolygonType
    {
        GROUND = 0,
        NONGROUND,
        BOUND
    };
    PolygonType PolygonTypeEnd = BOUND;
    
    struct mashMap
    {
        pcl::PointCloud<pcl::PointXYZ> VerticesCloud;
        pcl::PointCloud<pcl::PointNormal> CenterPNormal;
        std::vector<int> TriangleLableList;
        std::vector<pcl::Vertices> Triangles;
    } ;

    MeshMap::mashMap meshmap;

public:
    MeshMap():  Mesh(new pcl::PolygonMesh),
                VerticesPoints(new pcl::PointCloud<pcl::PointXYZ>),
                PolygonCenterPNormal(new pcl::PointCloud<pcl::PointNormal>),
                SingleFramePNormal(new pcl::PointCloud<pcl::PointNormal>)
                // MapCenterPNormal(new pcl::PointCloud<pcl::PointXYZINormal>)
                {}
    ~MeshMap(){}
    typedef boost::shared_ptr<MeshMap> Ptr;
    typedef boost::shared_ptr<const MeshMap> ConstPtr;

    void InitMeshMap(const mesh_navigation::FrameRecon &SingleMeshMsg);
    void InitLayerPlugins();

    //Get private data
    inline void getPolygonMesh(pcl::PolygonMesh::Ptr mesh) { mesh = Mesh; }
    inline pcl::PolygonMesh getPolygonMesh(){return *Mesh;}

    inline void getMeshVerticesPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr vertices) { vertices = VerticesPoints; }
    inline pcl::PointCloud<pcl::PointXYZ> getMeshVerticesPoints(){return *VerticesPoints;}

    inline void getPolygonCenterPNormal(pcl::PointCloud<pcl::PointNormal>::Ptr CenterPNormal) { CenterPNormal = PolygonCenterPNormal; }
    inline pcl::PointCloud<pcl::PointNormal> getPolygonCenterPNormal() { return *PolygonCenterPNormal; }

    inline void getSingleFramePNormal(pcl::PointCloud<pcl::PointNormal>::Ptr singlePNormal) { singlePNormal = SingleFramePNormal; }
    inline pcl::PointCloud<pcl::PointNormal> getSingleFramePNormal() { return *SingleFramePNormal; }

    //Publish
    void PublishMeshMap(std::string TFId,ros::Publisher MeshMapPublisher);
    void PublishMeshMap(std::string TFId,MeshMap::mashMap map,ros::Publisher MeshMapPublisher);

    std_msgs::ColorRGBA ColorWheel(int color_index)
    {
        std_msgs::ColorRGBA color;
        switch (color_index) {
        case 0:
            color.r = 0;   color.g = 1;   color.b = 0;   color.a=1;//绿色+不透明
            break;
        case 1:
            color.r = 1;   color.g = 0;   color.b = 0;   color.a = 1;//红色+不透明
            break;
        case 2:
            color.r = 0;   color.g = 0;   color.b = 1;   color.a=1;//蓝色+不透明
            break;
        case 3:
            color.r = 1;   color.g = 0;   color.b = 1;   color.a=1;//紫色+不透明
            break;
        case 4:
            color.r = 0;   color.g = 1;   color.b = 1;   color.a=1;//青色+不透明
            break;
        case 5:
            color.r = 1;   color.g = 0;   color.b = 0;   color.a=0.3;//红色+不透明度0.3
            break;
        case 6:
            color.r = 1;   color.g = 0;   color.b = 0;   color.a=0.3;//红色+不透明度0.3
            break;
        case 7:
            color.r = 1;   color.g = 0;   color.b = 0;   color.a=0.3;//红色+不透明度0.3
            break;
    
    }
    return color;
    }
};

#endif 