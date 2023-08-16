#include "DebugTools.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "DebugTools");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  DebugTools Debug(node,privateNode);

  ros::spin();

  return 0;
}