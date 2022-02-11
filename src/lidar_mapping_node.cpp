#include "lidar_mapping.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_mapping");
  LidarMapping node;
  ros::spin();
  return 0;
}
