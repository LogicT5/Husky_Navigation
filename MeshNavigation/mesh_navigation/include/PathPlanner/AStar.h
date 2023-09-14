#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstring>
#include <vector>
namespace AStarPlanner
{
    struct MapNode
    {
        // octomap::ColorOcTreeNode *node;
        octomap::point3d node_coord;
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

    public:
        AStarPlanner() {}

        void setOctomap(octomap::ColorOcTree *map)
        {

            Map = new octomap::ColorOcTree(map->getResolution());
            Map = map;
        }

        void setStartNode(octomap::point3d node)
        {
            startNode.node_coord = node;
            startNode.g = 0;
            Map->updateNode(node, true);
            Map->integrateNodeColor(node.x(), node.y(), node.z(), 0, 0, 255);
        }

        void setEndNode(octomap::point3d node)
        {
            endNode.node_coord = node;
            endNode.h = 0;
            Map->updateNode(node, true);
            Map->integrateNodeColor(node.x(), node.y(), node.z(), 0, 255, 0);
        }

        // void ActualCost(MapNode current_node)
        // {
        //     current_node.g = direct_cost * abs(startNode.node_coord.x() - current_node.node_coord.x()) + direct_cost * abs(startNode.node_coord.y() - current_node.node_coord.y());
        // }

        void EstimatedCost(MapNode *current_node)
        {
            current_node->h = direct_cost * abs(endNode.node_coord.x() - current_node->node_coord.x()) + direct_cost * abs(endNode.node_coord.y() - current_node->node_coord.y());
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
            MapNode *minNode = OpenSet[0];
            if (OpenSet.size() != 1)
            {
                for (auto *node : OpenSet)
                {
                    minNode = node->f < minNode->f ? node : minNode;
                }
            }
            return minNode;
        }

        // 返回开放集合中查找到值的索引
        int FindMapNodeIndexInOpenSet(MapNode *inquiry_node)
        {

            for (int i = 0; i < OpenSet.size(); i++)
            {
                if (inquiry_node->node_coord == OpenSet[i]->node_coord)
                    return i;
            }

            return -1;
        }
        MapTreeNode *FindMapTreeNodeInCloseSet(MapTreeNode *RootNode, MapNode *inquiry_node)
        {
            if (RootNode->Data->node_coord == inquiry_node->node_coord)
                return RootNode;
            else
            {
                for (auto *node : RootNode->pChild_list)
                {
                    return FindMapTreeNodeInCloseSet(node, inquiry_node);
                }
            }
        }

        void getPath()
        {
            MapTreeNode *tailTreeNode = FindMapTreeNodeInCloseSet(CloseSet, &endNode);
            while (tailTreeNode->pParent != NULL)
            {
                // geometry_msgs::PoseStamped AStarPoseStamped;
                // AStarPoseStamped.pose.position.x = tailTreeNode->Data->node_coord.x;
                // AStarPoseStamped.pose.position.y = tailTreeNode->Data->node_coord.y;
                // AStarPoseStamped.pose.position.z = tailTreeNode->Data->node_coord.z;

                // AStarPath.p
                Map->updateNode(tailTreeNode->Data->node_coord, true);
                Map->integrateNodeColor(tailTreeNode->Data->node_coord.x(), tailTreeNode->Data->node_coord.y(), tailTreeNode->Data->node_coord.z(), 255, 1, 0);
                tailTreeNode = tailTreeNode->pParent;
            }
        }

        void AStarPlanning()
        {
            CloseSet = createMapTreeNode(&startNode);
            CloseSet->pParent = NULL;
            OpenSet.push_back(&startNode);
            PathPasses.push_back(true);

            MapNode *current_node = &startNode;
            MapTreeNode *tailTreeNode = CloseSet;

            while (OpenSet.size() != 0)
            {
                // MapNode *current_node = MinFNode();

                for (int i = 0; i < 8; i++)
                {
                    octomap::OcTreeKey current_node_key = Map->coordToKey(current_node->node_coord);
                    MapNode *adjacent_node;
                    octomap::OcTreeKey adjacent_key;
                    adjacent_key[0] = current_node_key[0] + dx[i];
                    adjacent_key[1] = current_node_key[1] + dy[i];
                    adjacent_key[2] = current_node_key[2] + dz[i];

                    switch (i)
                    {
                    case direct::p_up:
                        adjacent_node->g = current_node->g + direct_cost;
                        break;
                    case direct::p_down:
                        adjacent_node->g = current_node->g + direct_cost;
                        break;
                    case direct::p_left:
                        adjacent_node->g = current_node->g + direct_cost;
                        break;
                    case direct::p_right:
                        adjacent_node->g = current_node->g + direct_cost;
                        break;
                    case direct::p_l_up:
                        adjacent_node->g = current_node->g + diagonal_cost;
                        break;
                    case direct::p_l_down:
                        adjacent_node->g = current_node->g + diagonal_cost;
                        break;
                    case direct::p_r_up:
                        adjacent_node->g = current_node->g + diagonal_cost;
                        break;
                    case direct::p_r_down:
                        adjacent_node->g = current_node->g + diagonal_cost;
                        break;
                    default:
                        break;
                    }
                    // 判断这个体素是不是障碍物
                    if (Map->search(adjacent_key) != nullptr && Map->isNodeOccupied(Map->search(adjacent_key)))
                    {
                        // 是则跳过这个体素
                        continue;
                    }

                    // 将相邻键转换为坐标 即相邻的体素
                    adjacent_node->node_coord = Map->keyToCoord(adjacent_key);
                    ComprehensiveCost(adjacent_node); // 计算f，h

                    // 不在OpenSet中加入OpenSet
                    int adjacent_index = FindMapNodeIndexInOpenSet(adjacent_node);
                    if (adjacent_index == -1)
                    {
                        OpenSet.push_back(adjacent_node);
                        PathPasses.push_back(false);
                    }
                    // else if(PathPasses[adjacent_index] == false && OpenSet[adjacent_index]->g > adjacent_node->g)
                    // {}
                    // 加入到CloseSet
                    MapTreeNode *newChildTreeNode = createMapTreeNode(adjacent_node);
                    newChildTreeNode->pParent = tailTreeNode;
                    tailTreeNode->pChild_list.push_back(newChildTreeNode);

                    MapNode *current_node = MinFNode();
                    tailTreeNode = FindMapTreeNodeInCloseSet(CloseSet, current_node);

                    if (adjacent_node->node_coord == endNode.node_coord)
                        break;
                } // end i
            }     // end while
        }
    };
}
