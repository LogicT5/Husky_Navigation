#include "HuskySLAMTrans.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "husky_slam_trans");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  SLAMTrans SLAMTransfor(node,privateNode);

  ros::spin();

  return 0;
}

