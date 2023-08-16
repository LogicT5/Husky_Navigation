#include "GtTrans.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "Gt_transfor");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
 
  GtTrans GtTransfor(node,privateNode);

  ros::spin();

  return 0;
}

