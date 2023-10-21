#ifndef __ASTART_MESHMAP_H__
#define __ASTART_MESHMAP_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <cstring>
#include <vector>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "MeshMap.h"

#include "tools/ROSPublishMesh.h"

std_msgs::ColorRGBA ColorWheel(int color_index);
void PublishMeshMap(std::string TFId, MeshMap::mashMap map, ros::Publisher MeshMapPublisher);

namespace AStarPlanner_meshmap{

typedef pcl::PointNormal PointType;

class AStarPlanner{
private:
    struct PlanNode{
        PointType point;
        int index;
        float g, h, f;
        bool Passes;
    };

    struct PlanNodeTree{
        PlanNode *p_node;
        std::vector<PlanNodeTree*> p_childNode;
        PlanNodeTree* p_Parent;
    };

    MeshMap::mashMap *meshmap;
    pcl::PointCloud<PointType>::Ptr GroundCloud;
    pcl::search::KdTree<PointType>::Ptr GroundKDTree;

    PlanNode *startNode; 
    PlanNode *endNode; 

    PlanNodeTree *CloseSet;
    std::vector<PlanNode *> OpenSet;


    //Debug
    // ros::NodeHandle *debugNode;
    // ros::Publisher debug_SingleFrameMeshPub;
public:
    std::vector<PlanNode *> Path;

private:
    PlanNodeTree *createPlanNodeTree(PlanNode *node)
    {
        PlanNodeTree *Node = new PlanNodeTree;
        memset(Node, 0, sizeof(PlanNodeTree));
        Node->p_node = node;
        return Node;
    }
    inline float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    }

    void EstimatedCost(PlanNode *current_point)
    {
        current_point->h = pointDistance(current_point->point , endNode->point);
    }

    // 计算当前节点的综合代价
    void ComprehensiveCost(PlanNode *current_node)
    {
        EstimatedCost(current_node);
        current_node->f = current_node->g + current_node->h;
    }

    // template<class T>
    // bool ArePointsEqual(const T& point1, const T& point2)
    // {
    //     return (point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z);
    // }
    bool ArePointsEqual(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
    {
        return (point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z);
    }
   bool ArePointsEqual(const pcl::PointNormal &point1, const pcl::PointNormal &point2)
    {
        return (point1.x == point2.x) && (point1.y == point2.y) && (point1.z == point2.z);
    }
    // 返回开放集合中f值最小的点
    PlanNode *MinFNode()
    {
        std::cout << std::endl;
        std::cout <<"************ Find OpenSent Node ***************" <<std::endl;
        std::cout << "OpenSet size : "<<OpenSet.size()<<std::endl;

        PlanNode *minNode = OpenSet[OpenSet.size() - 1];
        if (OpenSet.size() != 1)
        {
            // for (int i = 0; i < OpenSet.size();i++)
            for (int i = OpenSet.size()-1; i >= 0;i--)
            {
                if (OpenSet[i]->f < minNode->f && OpenSet[i]->Passes == false)
                {   
                    std::cout << "  node_"<<i<<": (" << OpenSet[i]->point.x<<" , " 
                                            << OpenSet[i]->point.y<<" , " 
                                            << OpenSet[i]->point.z<<" ) " 
                                            << "  f: " << OpenSet[i]->f
                                            << "  Passes: "<<OpenSet[i]->Passes << std::endl;
                    minNode = OpenSet[i];
                }
            }
        }
        else if(OpenSet.size() == 1 && OpenSet[0]->Passes == true)
        {
            return NULL;
        }
        std::cout << std::endl;
        return minNode;
    }

    // 返回开放集合中查找到值的索引
    int FindPlanNodeIndexInOpenSet(PlanNode *inquiry_node)
    {

        for (int i = 0; i < OpenSet.size(); i++)
        {
            if (ArePointsEqual(inquiry_node->point , OpenSet[i]->point))
                return i;
        }

        return -1;
    }

    PlanNodeTree *FindMapTreeNode(PlanNodeTree *RootNode, PlanNode *inquiry_node)
    {
        if (ArePointsEqual(RootNode->p_node->point , inquiry_node->point))
        {
            return RootNode;
        }
        else
        {
            if(RootNode->p_childNode.size() != 0)
            {   
                //BFS
                std::queue<PlanNodeTree*> PlanNodeTree_queue;
                PlanNodeTree_queue.push(RootNode);
                while(! PlanNodeTree_queue.empty())
                {
                    PlanNodeTree *currentPlanNodeTree;
                    currentPlanNodeTree = PlanNodeTree_queue.front();
                    PlanNodeTree_queue.pop();
                    for (auto treeNode : currentPlanNodeTree->p_childNode)
                    {
                        PlanNodeTree_queue.push(treeNode);
                        if (ArePointsEqual(treeNode->p_node->point , inquiry_node->point))
                        {
                            return treeNode;
                        }
                    }
                }
                ROS_WARN("No Find the Node");
                return NULL;
            }
            else{
                ROS_WARN(" RootNode's Childs is NULL");
                return NULL;
            }
        }
    }

    // Vector3 a b c triangle vertexs
    // orig is ray original point, dir is direction vector
    bool rayTriangleIntersect(Eigen::Vector3f orig_point, Eigen::Vector3f dir_vector,
                                            Eigen::Vector3f  triangle_a, Eigen::Vector3f  triangle_b, Eigen::Vector3f  triangle_c)
    {
        float t;
        float b1;
        float b2;
        bool isIn = false;
        Eigen::Vector3f E1 = triangle_b - triangle_a;
        Eigen::Vector3f E2 = triangle_c - triangle_a;
        Eigen::Vector3f S = orig_point - triangle_a;
        Eigen::Vector3f S1 = dir_vector.cross(E2);
        Eigen::Vector3f S2 = S.cross(E1);

        // 共同系数
        float coeff = 1.0f / S1.dot(E1);
        t = coeff * S2.dot(E2);
        b1 = coeff * S1.dot(S);
        b2 = coeff * S2.dot(dir_vector);

        // Debug.Log($"t = {t}, b1 = {b1}, b2 = {b2}");

        if (t >= 0 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0)
        {
            isIn = true;
        }

        return isIn;
    }

    
    void getPath(PlanNodeTree *tailTreeNode )
    {
        // MapTreeNode *tailTreeNode = FindMapTreeNode(CloseSet, &endNode);
        // std::cout << "Path: "<< std::endl;

        while (tailTreeNode == NULL)
        {
            Path.push_back(tailTreeNode->p_node);
            // AStar_PathOcTree->setNodeColor(tailTreeNode->Data->node_key,204,0,204);
            std::cout << tailTreeNode->p_node->point.x << " , " 
                           << tailTreeNode->p_node->point.y << " , " 
                           << tailTreeNode->p_node->point.z  << std::endl;
            tailTreeNode = tailTreeNode->p_Parent;
        }

    }

public:
    AStarPlanner(): GroundCloud(new pcl::PointCloud<PointType>),
                    GroundKDTree(new pcl::search::KdTree<PointType>),
                    meshmap(new MeshMap::mashMap){}
    ~AStarPlanner(){}

    void setMeshMap(MeshMap::mashMap map)
    {
        *meshmap = map;
        for (int i = 0; i < map.CenterPNormal.points.size();i++)
        {
            if(map.TriangleLableList[i] == MeshMap::PolygonType::GROUND)
            {
                if(map.CenterPNormal.points[i].z < 0.5)
                    GroundCloud->push_back(map.CenterPNormal.points[i]);
            }
        }
        GroundKDTree->setInputCloud(GroundCloud);
        std::cout << "set MeshMap :  mesh's Num " << meshmap->Triangles.size() << std::endl;
    }

    void setStartNode(PointType point)
    {
        startNode->point = point;
        startNode->g = 0;
        std::cout << "start Node : " << startNode->point << std::endl;
    }
    void setStartNode(pcl::PointXYZ point)
    {
        std::cout << "start Node0 : " << point << std::endl;
        startNode->point.x = point.x;
        startNode->point.y = point.y;
        startNode->point.z = point.z;
        startNode->point.normal_x = 0.0;
        startNode->point.normal_y = 0.0;
        startNode->point.normal_z = 0.0;
        startNode->g = 0;
        std::cout << "start Node : " << startNode->point << std::endl;

    }

    void setEndNode(PointType point)
    {
        endNode->point = point;
        endNode->h = 0;
        GroundCloud->push_back(endNode->point);
        std::cout << "end Node : " << endNode->point << std::endl;
    }
    void setEndNode(pcl::PointXYZ point)
    {
        endNode->point.x = point.x;
        endNode->point.y = point.y;
        endNode->point.z = point.z;
        endNode->point.normal_x = 0.0;
        endNode->point.normal_y = 0.0;
        endNode->point.normal_z = 0.0;
        endNode->h = 0;
        GroundCloud->push_back(endNode->point);
        std::cout << "end Node : " << endNode->point << std::endl;
    }


    void AStarPlanning()
    {
        // if(sqrt((startNode->point.x - endNode->point.x) * (startNode->point.x - endNode->point.x) + (startNode->point.y - endNode->point.y) * (startNode->point.y - endNode->point.y) ) < 0.5)
        // {
        //     return;
        // }
        GroundKDTree->setInputCloud(GroundCloud);

        bool planningFlag = true;
        CloseSet = new PlanNodeTree;
        CloseSet->p_node = startNode;
        CloseSet->p_Parent = NULL;
        OpenSet.push_back(startNode);
        startNode->Passes = true;
        ComprehensiveCost(startNode); // 计算f，h

        PlanNode *current_node = new PlanNode;
        current_node = startNode;
        PlanNodeTree *tailTreeNode = new PlanNodeTree;
        tailTreeNode = CloseSet;

        std::cout << "in While: " << std::endl;
        int t = 0;
        while (t<5)
        // while (OpenSet.size() != 0)
        {
            t++;
            std::cout << "No." << t << " current node: ( " << current_node->point.x << " , " << current_node->point.y << " , " << current_node->point.z << " ) "  << "  g: "<< current_node->g<< std::endl;
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);
            int k = 8;
            int nearPointsNum = 0;
            if(OpenSet.size() < 2)
                nearPointsNum = GroundKDTree->nearestKSearch(current_node->point, k, indices, sqr_distances);// 搜索到最近的八个点，计算代价
            else
                nearPointsNum = GroundKDTree->radiusSearch(current_node->point,current_node->g , indices, sqr_distances);
            
            for (int i = 0; i < nearPointsNum;i++)
            {
                PlanNode *adjacent_node = new PlanNode;
                adjacent_node->point = GroundCloud->points[indices[i]];
                adjacent_node->g = current_node->g + pointDistance(adjacent_node->point , tailTreeNode->p_node->point);
                // std::cout << "near node: (" << adjacent_node->point.x<<" , " << adjacent_node->point.y<<" , " << adjacent_node->point.z<<" ) " << "  g: " << adjacent_node->g << std::endl;

                //检查当前点连接的路径是否穿过了障碍物
                PointType orig_point = current_node->point;
                orig_point.z = 0.1651;
                PointType dir_point = adjacent_node->point;
                dir_point.z = 0.1651;
                Eigen::Vector3f current2adjacent = dir_point.getVector3fMap() - orig_point.getVector3fMap();
                // Eigen::Vector3f current2adjacent = adjacent_node->point.getVector3fMap() - current_node->point.getVector3fMap();

                for(auto &&triangle : meshmap->Triangles)
                {
                    
                    if (rayTriangleIntersect(orig_point.getVector3fMap(), current2adjacent,
                                             meshmap->VerticesCloud[triangle.vertices[0]].getVector3fMap(),
                                             meshmap->VerticesCloud[triangle.vertices[1]].getVector3fMap(),
                                             meshmap->VerticesCloud[triangle.vertices[2]].getVector3fMap()))
                    {
                        continue;
                    }
                }

                // 可导航点 计算代价
                ComprehensiveCost(adjacent_node);

                int adjacent_index = FindPlanNodeIndexInOpenSet(adjacent_node);
                
                if (adjacent_index == -1)// 不在OpenSet中加入OpenSet
                {
                    OpenSet.push_back(adjacent_node);
                    adjacent_node->Passes = false;
                }
                // else //在openset中跟新节点数据
                // {
                //     OpenSet[adjacent_index] = adjacent_node;
                //     // ComprehensiveCost(OpenSet[adjacent_index]);
                // }
                // 加入到CloseSet  
                PlanNodeTree *newChildTreeNode = createPlanNodeTree(adjacent_node);
                newChildTreeNode->p_Parent = tailTreeNode;
                tailTreeNode->p_childNode.push_back(newChildTreeNode);

                //检查目标点是否在当前点的范围内
                // if (ArePointsEqual(adjacent_node->point ,endNode->point))
                if(sqrt((adjacent_node->point.x - endNode->point.x) * (adjacent_node->point.x - endNode->point.x) + (adjacent_node->point.y - endNode->point.y) * (adjacent_node->point.y - endNode->point.y) ) < 0.5)
                {
                    getPath(newChildTreeNode);
                    ros::Duration(0.02).sleep();
                    return;
                }
            }
            // current_node = MinFNode();
            current_node = OpenSet[OpenSet.size() -1];
            if (OpenSet.size() != 1)
            {
                // for (int i = 0; i < OpenSet.size();i++)
                for (int i = OpenSet.size()-1; i >= 0;i--)
                {
                    if (OpenSet[i]->f < current_node->f && OpenSet[i]->Passes == false)
                    {   
                        // std::cout << "  node_"<<i<<": (" << OpenSet[i]->point.x<<" , " 
                        //                         << OpenSet[i]->point.y<<" , " 
                        //                         << OpenSet[i]->point.z<<" ) " 
                        //                         << "  f: " << OpenSet[i]->f
                        //                         << "  Passes: "<<OpenSet[i]->Passes << std::endl;
                        current_node = OpenSet[i];
                    }
                }
            }
            else if(OpenSet.size() == 1 && OpenSet[0]->Passes == true)
            {
                current_node = NULL;
            }
            tailTreeNode = FindMapTreeNode(CloseSet, current_node);

            if(current_node == NULL  || tailTreeNode == NULL)
            {
                return;
            }
            int current_index = FindPlanNodeIndexInOpenSet(current_node);
            // OpenSet[current_index]->Passes = true;
            current_node->Passes = true;

            // for (int i = 0; i < OpenSet.size();i++)
            // {
            //     std::cout << "  OpenSet node "<<i<<": (" << OpenSet[i]->point.x<<" , " 
            //                                              << OpenSet[i]->point.y<<" , " 
            //                                              << OpenSet[i]->point.z<<" ) " 
            //                                              << "  f: " << OpenSet[i]->f
            //                                              << "  Passes: "<<OpenSet[i]->Passes << std::endl;
            // }

            std::cout << "  New Current Node: " << current_node->point.x << " , " << current_node->point.y << " , " << current_node->point.z << "  in OpenSet Index: "<< current_index <<std::endl;
            std::cout << "  Tree Min Node: " << tailTreeNode->p_node->point.x << " , " << tailTreeNode->p_node->point.y << " , " << tailTreeNode->p_node->point.z << std::endl;
            std::cout << "  OpenSet SIZE :  "<<OpenSet.size() << std::endl;

            std::cout << std::endl;
        }
        std::cout << "***** END ASTART PLANNER WHILE ****** " << std::endl;
        std::cout << "PATH SIZE :  "<<Path.size() << std::endl;
        Path.push_back(current_node);
    }

public:
    pcl::PointCloud<pcl::PointXYZI> getOpenSet()
    {
        pcl::PointCloud<pcl::PointXYZI> openSetCloud;
        for(auto && node:OpenSet)
        {
            pcl::PointXYZI point;
            point.x = node->point.x;
            point.y = node->point.y;
            point.z = node->point.z = 0.156;
            if(node->Passes == true)
                point.intensity = 1;
            else
                point.intensity = 0;
            openSetCloud.push_back(point);
        }
        return openSetCloud;
    }

    void PublishPath(std::string TFId,ros::Publisher PathPublisher)
    {
        if(Path.size() > 0)
        {
            visualization_msgs::MarkerArray PathMarkerList;
            constexpr int index = 1e5;

            visualization_msgs::Marker PointMarker;
            PointMarker.header.frame_id = "map";
            PointMarker.header.stamp = ros::Time::now();
            PointMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            PointMarker.action = visualization_msgs::Marker::MODIFY;
            PointMarker.id = index;

            PointMarker.scale.x = 2.0;
            PointMarker.scale.y = 2.0;
            PointMarker.scale.z = 2.0;

            PointMarker.pose.position.x = 0.0;
            PointMarker.pose.position.y = 0.0;
            PointMarker.pose.position.z = 0.0;

            PointMarker.pose.orientation.x = 0.0;
            PointMarker.pose.orientation.y = 0.0;
            PointMarker.pose.orientation.z = 0.0;
            PointMarker.pose.orientation.w = 1.0;

            PointMarker.color = ColorWheel(int(4)); // 点是青色
            for (int i = Path.size() - 1; i > 0;i--)
            {
                geometry_msgs::Point oPTemp;
                oPTemp.x = Path[i]->point.x;
                oPTemp.y = Path[i]->point.y;
                oPTemp.z = Path[i]->point.z;
                PointMarker.points.push_back(oPTemp);
            }

            PathMarkerList.markers.push_back(PointMarker);

            visualization_msgs::Marker SideMarker;
            SideMarker.header.frame_id = "map";
            SideMarker.header.stamp = ros::Time::now();
            SideMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            SideMarker.action = visualization_msgs::Marker::MODIFY;
            SideMarker.id = index+1;

            SideMarker.scale.x = 2.0;
            SideMarker.scale.y = 2.0;
            SideMarker.scale.z = 2.0;

            SideMarker.pose.position.x = 0.0;
            SideMarker.pose.position.y = 0.0;
            SideMarker.pose.position.z = 0.0;

            SideMarker.pose.orientation.x = 0.0;
            SideMarker.pose.orientation.y = 0.0;
            SideMarker.pose.orientation.z = 0.0;
            SideMarker.pose.orientation.w = 1.0;

            SideMarker.color = ColorWheel(int(3)); // 路径用紫色
            for (int i = Path.size() - 1; i > 1;i--)
            {
                geometry_msgs::Point oPTemp;
                oPTemp.x = Path[i]->point.x;
                oPTemp.y = Path[i]->point.y;
                oPTemp.z = Path[i]->point.z;

                oPTemp.x += Path[i-1]->point.x;
                oPTemp.y += Path[i-1]->point.y;
                oPTemp.z += Path[i-1]->point.z;
                SideMarker.points.push_back(oPTemp);
            }

            PathMarkerList.markers.push_back(SideMarker);
            PathPublisher.publish(PathMarkerList);
        }
        else
        {
            return;
        }
    }
};
}

void PublishMeshMap(std::string TFId,MeshMap::mashMap map,ros::Publisher MeshMapPublisher)
{
    visualization_msgs::MarkerArray PolygonMarkerList;
    // pcl::PointCloud<pcl::PointXYZ> vPublishCloud;
    // pcl::fromPCLPointCloud2(meshmap.VerticesCloud, vPublishCloud);
    for (MeshMap::PolygonType polygontype = MeshMap::GROUND; polygontype <= MeshMap::BOUND; polygontype = (MeshMap::PolygonType)(polygontype+1))
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

std_msgs::ColorRGBA ColorWheel(int color_index)
    {
        std_msgs::ColorRGBA color;
        switch (color_index) {
        case 0:
            color.r = 1;   color.g = 0;   color.b = 0;   color.a = 1;//红色+不透明
            break;
        case 1:
            color.r = 0;   color.g = 1;   color.b = 0;   color.a=1;//绿色+不透明
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
#endif