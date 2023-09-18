#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstring>
#include <vector>
#include <queue>
namespace AStarPlanner
{
    struct MapNode
    {
        // octomap::ColorOcTreeNode *node;
        octomap::point3d node_coord;
        octomap::OcTreeKey node_key;
        int g, h, f;
    };

    struct MapTreeNode
    {
        MapNode *Data;
        std::vector<MapTreeNode *> pChild_list;
        MapTreeNode *pParent;
    };

    MapTreeNode *createMapTreeNode(MapNode *nodedata)
    {
        MapTreeNode *Node = new MapTreeNode;
        memset(Node, 0, sizeof(MapTreeNode));
        Node->Data = nodedata;
        return Node;
    }

    const int direct_cost = 10;
    const int diagonal_cost = 14;
    enum direct // 方向
    {
        p_up,     // 上
        p_down,   // 下
        p_left,   // 左
        p_right,  // 右
        p_l_up,   // 左上
        p_r_up,   // 右上
        p_l_down, // 左下
        p_r_down  // 右下
    };
    // 定义八个相邻体素的相对索引
    //              上 下 左 右 左上 右上 左下 右下
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    int dz[] = {0, 0, 0, 0, 0, 0, 0, 0};

    class AStarPlanner
    {
        MapTreeNode *CloseSet;
        std::vector<MapNode *> OpenSet;
        std::vector<bool> PathPasses;

        octomap::ColorOcTree *Map;

        MapNode startNode;
        MapNode endNode;

        // nav_msgs::Path AStarPath;
        ros::NodeHandle ROSnodeHandle;

        ros::Publisher AStar_PathOcTreePub;
        octomap::ColorOcTree *AStar_PathOcTree;

    public:
        AStarPlanner():Map(new octomap::ColorOcTree(0.05)),
                                AStar_PathOcTree(new octomap::ColorOcTree(0.25))
        {
            // setStartNode(octomap::point3d(0, 0, 0.583));
            AStar_PathOcTreePub = ROSnodeHandle.advertise<octomap_msgs::Octomap>("/AStarPath/PathOcTree_full", 1, true);

        }
        // void setROSnodHandle(ros::NodeHandle *nodeHandle)
        // {
        //     ROSnodeHandle = nodeHandle;
        // }

        void setOctomap(octomap::ColorOcTree *map)
        {
            std::cout << "setMap " << map->getResolution() << std::endl;
            Map->setResolution(map->getResolution());
            // AStar_PathOcTree->setResolution(map->getResolution());
            Map = map;
        }

        octomap::ColorOcTree *getOctomap()
        {
            return Map;
        }

        void setStartNode(octomap::point3d node)
        {
            startNode.node_coord = node;
            startNode.node_key = Map->coordToKey(node);
            startNode.g = 0;
            // Map->updateNode(node, true);
            // Map->integrateNodeColor(node.x(), node.y(), node.z(), 0, 0, 255);
            std::cout << "startNode: " << startNode.node_coord.x() << " , " << startNode.node_coord.y() << " , " << startNode.node_coord.z() << std::endl;
        }

        void setEndNode(octomap::point3d node)
        {
            endNode.node_coord = node;
            endNode.node_key = Map->coordToKey(node);
            endNode.h = 0;
            // Map->updateNode(node, true);
            // Map->integrateNodeColor(node.x(), node.y(), node.z(), 0, 255, 0);
            std::cout << "endNode:   " << endNode.node_coord.x() << " , " << endNode.node_coord.y() << " , " << endNode.node_coord.z() << std::endl;
        }

        // void ActualCost(MapNode current_node)
        // {
        //     current_node.g = direct_cost * abs(startNode.node_coord.x() - current_node.node_coord.x()) + direct_cost * abs(startNode.node_coord.y() - current_node.node_coord.y());
        // }

        void EstimatedCost(MapNode *current_node)
        {
            current_node->h = direct_cost * abs(endNode.node_key[0] - current_node->node_key[0]) + direct_cost * abs(endNode.node_key[1] - current_node->node_key[1]);
        }

        // 计算当前节点的综合代价
        void ComprehensiveCost(MapNode *current_node)
        {
            EstimatedCost(current_node);
            current_node->f = current_node->g + current_node->h;
        }

        // 返回开放集合中f值最小的点
        MapNode *MinFNode()
        {
            MapNode *minNode = OpenSet[OpenSet.size() - 1];
            if (OpenSet.size() != 1)
            {
                // for (int i = 0; i < OpenSet.size();i++)
                for (int i =  OpenSet.size()-1; i >0;i--)
                {
                    if (OpenSet[i]->f < minNode->f && PathPasses[i] == false)
                    {
                        minNode = OpenSet[i];
                    }
                }
            }
            else if(OpenSet.size() ==1 && PathPasses[0] == true)
            {
                return NULL;
            }

            return minNode;
        }

        // 返回开放集合中查找到值的索引
        int FindMapNodeIndexInOpenSet(MapNode *inquiry_node)
        {

            for (int i = 0; i < OpenSet.size(); i++)
            {
                if (inquiry_node->node_key == OpenSet[i]->node_key)
                    return i;
            }

            return -1;
        }

        MapTreeNode *FindMapTreeNode(MapTreeNode *RootNode, MapNode *inquiry_node)
        {
            if (RootNode->Data->node_coord == inquiry_node->node_coord)
            {
                return RootNode;
            }
            else
            {
                if(RootNode->pChild_list.size() != 0)
                {   
                    //BFS
                    std::queue<MapTreeNode*> MapTreeNode_queue;
                    MapTreeNode_queue.push(RootNode);
                    while(! MapTreeNode_queue.empty())
                    {
                        MapTreeNode *currentMapTreeNode;
                        currentMapTreeNode = MapTreeNode_queue.front();
                        MapTreeNode_queue.pop();
                        for (auto treeNode : currentMapTreeNode->pChild_list)
                        {
                            MapTreeNode_queue.push(treeNode);
                             if (treeNode->Data->node_coord == inquiry_node->node_coord)
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

        void PrintMapTreeNode(MapTreeNode *RootNode)
        {
            if (RootNode->pChild_list.size() == 0)
                std::cout << RootNode->Data->node_key.k<< " ";
            else
            {
                for (auto *node : RootNode->pChild_list)
                {
                    PrintMapTreeNode(node);
                }
            }

        }

        void getPath(MapTreeNode *tailTreeNode )
        {
            // MapTreeNode *tailTreeNode = FindMapTreeNode(CloseSet, &endNode);
            // std::cout << "Path: "<< std::endl;

            while (tailTreeNode->pParent != NULL)
            {
                AStar_PathOcTree->setNodeColor(tailTreeNode->Data->node_key,204,0,204);
                // std::cout << tailTreeNode->Data->node_key[0] << " , " << tailTreeNode->Data->node_key[1] << " , " <<tailTreeNode->Data->node_key[2] << std::endl;
                tailTreeNode = tailTreeNode->pParent;
            }
        }

        pcl::PointCloud<pcl::PointXYZ> getTreeNodeVoxel()
        {
            pcl::PointCloud<pcl::PointXYZ> TreeNodeVoxel;
            for (int i = 0; i < OpenSet.size(); i++)
            {
                pcl::PointXYZ Point;
                Point.x = OpenSet[i]->node_coord.x();
                Point.y = OpenSet[i]->node_coord.y();
                Point.z = OpenSet[i]->node_coord.z();
                TreeNodeVoxel.push_back(Point);
            }
            return TreeNodeVoxel;
        }

        void MapUpdataNode(octomap::point3d node)
        {
            // Map->updateNode(node, true);
            // Map->integrateNodeColor(node.x(), node.y(), node.z(), 255, 1, 0);
        }

        void MapUpdataNode(octomap::OcTreeKey nodekey)
        {
            // Map->updateNode(nodekey, true);
            // Map->integrateNodeColor(nodekey, 255, 1, 0);
        }

        void PublishFullOctoMap(ros::Publisher Pub,octomap::ColorOcTree *m_octree)
        {
            octomap_msgs::Octomap map;
            map.header.frame_id = "odom";
            // map.header.frame_id = "base_link_map";
            map.header.stamp = ros::Time::now();

            if (octomap_msgs::fullMapToMsg(*m_octree, map))
                Pub.publish(map);
            else
                ROS_ERROR("Error serializing OctoMap");
        }
        void PublishAStarPathOcTree()
        {
            PublishFullOctoMap(AStar_PathOcTreePub, AStar_PathOcTree);
        }

        void AStarPlanning()
        {
            bool planningFlag = true;
            CloseSet = createMapTreeNode(&startNode);
            CloseSet->pParent = NULL;
            OpenSet.push_back(&startNode);
            PathPasses.push_back(true);
            ComprehensiveCost(&startNode); // 计算f，h


            MapNode *current_node = new MapNode();
            *current_node = startNode;
            MapTreeNode *tailTreeNode = new MapTreeNode();
            tailTreeNode = CloseSet;

            AStar_PathOcTree->updateNode(startNode.node_key,true);
            AStar_PathOcTree->setNodeColor(startNode.node_key,0,0,255);
            AStar_PathOcTree->updateNode(endNode.node_key,true);
            AStar_PathOcTree->setNodeColor(endNode.node_key,0,255,0);
        
            int t = 500;
            // while (t > 0)
            while (OpenSet.size() != 0)
            {
                t--;
                // MapNode *current_node = MinFNode();
                // current_node->node_key = Map->coordToKey(current_node->node_coord);
                std::cout << std::endl; 
                std::cout << "current_node: "<< current_node->node_coord.x() << " , " << current_node->node_coord.y() << " , " << current_node->node_coord.z();
                std::cout << "  current_node_key: " << current_node->node_key[0] << " , " << current_node->node_key[1] << " , " << current_node->node_key[2] << std::endl;

                for (int i = 0; i < 8; i++)
                {
                    MapNode *adjacent_node = new MapNode();
                    
                    adjacent_node->node_key[0] = current_node->node_key[0] + dx[i];
                    adjacent_node->node_key[1] = current_node->node_key[1] + dy[i];
                    adjacent_node->node_key[2] = current_node->node_key[2] + dz[i];
                    std::cout << "  adjacent_key_" << i << ": " << adjacent_node->node_key[0] << " , " << adjacent_node->node_key[1] << " , " << adjacent_node->node_key[2] ;
                    // Map->updateNode(adjacent_node->node_key, true);
                    // Map->integrateNodeColor(adjacent_node->node_key, 245, 222, 179);

                    switch (i)
                    {
                        case direct::p_up:
                            // adjacent_node->g = current_node->g + direct_cost;
                            // break;
                        case direct::p_down:
                            // adjacent_node->g = current_node->g + direct_cost;
                            // break;
                        case direct::p_left:
                            // adjacent_node->g = current_node->g + direct_cost;
                            // break;
                        case direct::p_right:
                            adjacent_node->g = current_node->g + direct_cost;
                            break;
                        case direct::p_l_up:
                            // adjacent_node->g = current_node->g + diagonal_cost;
                            // break;
                        case direct::p_l_down:
                            // adjacent_node->g = current_node->g + diagonal_cost;
                            // break;
                        case direct::p_r_up:
                            // adjacent_node->g = current_node->g + diagonal_cost;
                            // break;
                        case direct::p_r_down:
                            adjacent_node->g = current_node->g + diagonal_cost;
                            break;
                    }
                    // // 判断这个体素是不是障碍物
                    if (Map->search(adjacent_node->node_key) != nullptr && Map->isNodeOccupied(Map->search(adjacent_node->node_key)))
                    {
                        // 是则跳过这个体素
                    //    std::cout << std::endl;
                        continue;
                    }

                    // 将相邻键转换为坐标 即相邻的体素
                    adjacent_node->node_coord = Map->keyToCoord(adjacent_node->node_key);
                    ComprehensiveCost(adjacent_node); // 计算f，h
                    std::cout <<" f: "<< adjacent_node->f <<" g: "<< adjacent_node->g <<" h: "<< adjacent_node->h <<std::endl;
                    
                    // 不在OpenSet中加入OpenSet
                    int adjacent_index = FindMapNodeIndexInOpenSet(adjacent_node);
                    if (adjacent_index == -1)
                    {
                        OpenSet.push_back(adjacent_node);
                        PathPasses.push_back(false);
                    }
                    else
                    {
                        OpenSet[adjacent_index] = adjacent_node;
                        // ComprehensiveCost(OpenSet[adjacent_index]);
                    }
                    // 加入到CloseSet  
                    MapTreeNode *newChildTreeNode = createMapTreeNode(adjacent_node);
                    newChildTreeNode->pParent = tailTreeNode;
                    tailTreeNode->pChild_list.push_back(newChildTreeNode);
                    // if (adjacent_node->node_coord == endNode.node_coord)
                    //     // break;
                    // std::cout << std::endl;
                    if (adjacent_node->node_key== endNode.node_key)
                    {
                        std::cout << "Get Path: " << std::endl;
                        std::cout << "  endNode: " << endNode.node_key[0] << " , " << endNode.node_key[1] << " , " << endNode.node_key[2] << std::endl;
                        getPath(newChildTreeNode);
                        ros::Duration(0.02).sleep();
                        PublishFullOctoMap(AStar_PathOcTreePub, AStar_PathOcTree);
                        return;
                    }

                } // end i
                
                // ComprehensiveCost(OpenSet[current_index]);
                // OpenSet.erase(OpenSet.begin() + current_index);

                // MapNode *current_node = MinFNode();
                current_node = MinFNode();
                tailTreeNode = FindMapTreeNode(CloseSet, current_node);

                if(current_node == NULL  || tailTreeNode == NULL)
                {
                    return;
                }
                int current_index = FindMapNodeIndexInOpenSet(current_node);
                PathPasses[current_index] = true;

                std::cout << "  OpenSet: "<<std::endl;
                for (int i = 0; i < OpenSet.size();i++)
                {
                    AStar_PathOcTree->updateNode(OpenSet[i]->node_key,true);
                    AStar_PathOcTree->setNodeColor(OpenSet[i]->node_key,169 ,169 ,169);
                    if(OpenSet[i]->h ==10)
                    {
                        AStar_PathOcTree->updateNode(OpenSet[i]->node_key,true);
                        AStar_PathOcTree->setNodeColor(OpenSet[i]->node_key,255,0,255);
                    }
                    if(PathPasses[i] == true)
                    {
                        AStar_PathOcTree->setNodeColor(OpenSet[i]->node_key, 0, 255, 255);
                        // OpenSet.erase(OpenSet.begin() + i);
                        // PathPasses.erase(PathPasses.begin() + i);
                    }
                    else
                    {
                        // std::cout << "  "<<i<<": " << OpenSet[i]->node_key[0] << " , " << OpenSet[i]->node_key[1] << " , " << OpenSet[i]->node_key[2];
                        // std::cout <<"  f: "<<OpenSet[i]->f <<" g: "<< OpenSet[i]->g <<" h: "<< OpenSet[i]->h <<std::endl;
                    }
                }
                std::cout << std::endl;
                // std::cout << "  Tree_Min_node: " << tailTreeNode->Data->node_key[0] << " , " << tailTreeNode->Data->node_key[1] << " , " << tailTreeNode->Data->node_key[2] << std::endl;
                std::cout << "  Min_node: " << current_node->node_key[0] << " , " << current_node->node_key[1] << " , " << current_node->node_key[2] << std::endl;
                std::cout << "  f: "<<current_node->f <<" g: "<<current_node->g <<" h: "<< current_node->h <<" current_index: "<<current_index<<std::endl;

                // AStar_PathOcTree->updateNode(current_node->node_key,true);
                AStar_PathOcTree->setNodeColor(current_node->node_key,255,0,0);
                AStar_PathOcTree->setNodeColor(startNode.node_key,0,0,255);
                
                AStar_PathOcTree->updateInnerOccupancy();
                PublishFullOctoMap(AStar_PathOcTreePub, AStar_PathOcTree);
                ros::Duration(0.02).sleep();
                PublishFullOctoMap(AStar_PathOcTreePub, AStar_PathOcTree);
            } // end while

            // PublishFullOctoMap(AStar_PathOcTreePub, AStar_PathOcTree);

            std::cout <<"Pub AStar_PathOcTreePub"<< t << std::endl;
        }
    }; 
}
