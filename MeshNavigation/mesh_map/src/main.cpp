#include "MeshMap.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "husky_mesh_map");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  
  mesh_map::MeshMap meshMapping(node,privateNode);

  ros::spin();

  return 0;
}