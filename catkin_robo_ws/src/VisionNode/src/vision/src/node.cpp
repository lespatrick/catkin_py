#include "ros/ros.h"
#include <iostream>
#include "VisionNode.h"

using namespace std;

int main(int argc, char **argv) {
  //initialize node
  ros::init(argc, argv, "vision_node");
  
  VisionNode vn;

  ros::spin();

  return 0;
}