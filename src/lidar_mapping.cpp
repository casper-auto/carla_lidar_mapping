#include "lidar_mapping.h"

LidarMapping::LidarMapping() : nh_(), private_nh_("~") {
  // param server
  private_nh_.param<bool>("new_scan", new_scan_, true);
  private_nh_.param<double>("vehicle_radius", vehicle_radius_, 8.0);
  private_nh_.param<int>("cloud_resolution", cloud_resolution_, 5000);

  // initialize subscribers
  point_cloud_sub_ = nh_.subscribe("/carla/ego_vehicle/lidar", 1, &LidarMapping::pointCloudCallback, this);
  localization_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &LidarMapping::localizationCallback, this);

  // initialize publishers
  point_cloud_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_map", 1);

  // createTimer
  timer_ = nh_.createTimer(ros::Duration(0.1), &LidarMapping::timerCallback, this);

  // smart pointers
  scaned_cloud_ptr_ = boost::make_shared<PointCloudT>();
}

LidarMapping::~LidarMapping() {}

void LidarMapping::pointCloudCallback(const sensor_msgs::PointCloud2::Ptr msg) {
  ROS_INFO("Point cloud received ...");
  pcl::fromROSMsg(*msg, *scaned_cloud_ptr_);
}

void LidarMapping::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_INFO("Odometry received ...");

  // update current_pose_.position
  current_pose_.position.x = msg->pose.pose.position.x;
  current_pose_.position.y = msg->pose.pose.position.y;
  current_pose_.position.z = msg->pose.pose.position.z;

  // quaternion to rpy
  geometry_msgs::Quaternion geo_quat = msg->pose.pose.orientation;

  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(geo_quat, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // update current_pose_.rotation
  current_pose_.rotation.roll = roll;
  current_pose_.rotation.pitch = pitch;
  current_pose_.rotation.yaw = yaw;
}

void LidarMapping::timerCallback(const ros::TimerEvent &e) {

  scanTransformation(scaned_cloud_ptr_);

  //////////////////////////////////////////////////////////////////////////
  // Mapping Visualization
  //////////////////////////////////////////////////////////////////////////
  saved_map_.header.frame_id = "map";
  point_cloud_map_pub_.publish(saved_map_);
}

///////////////////////////////////////////////////////////
// Magic happens here: Transformation
///////////////////////////////////////////////////////////

void LidarMapping::scanTransformation(const PointCloudT::Ptr scan_cloud) {
  // Set transform to pose using transform3D()
  Eigen::Matrix4d transform = transform3D(
      current_pose_.rotation.yaw, current_pose_.rotation.pitch, current_pose_.rotation.roll,
      current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);

  for (auto point : *scan_cloud) {
    // Don't include points touching ego
    if ((point.x * point.x +
         point.y * point.y +
         point.z * point.z) >
        vehicle_radius_) {
      Eigen::Vector4d local_point(point.x, point.y, point.z, 1);
      // Multiply local_point by transform
      Eigen::Vector4d transform_point = transform * local_point;
      pcl_cloud_.points.push_back(PointT(transform_point[0], transform_point[1], transform_point[2]));
    }
  }

  PointCloudT::Ptr cloud_filtered = downsamplePointCloud(pcl_cloud_);
  pcl::toROSMsg(*cloud_filtered.get(), saved_map_);

  // scan resolution
  if (pcl_cloud_.points.size() > cloud_resolution_) {
    new_scan_ = false;
    // savePointCloudMap(cloud_filtered);
  }

  double distanceR_res = 5.0; // CANDO: Can modify this value
  double time_res = 1.0;      // CANDO: Can modify this value
  auto current_time = std::chrono::system_clock::now();
  if (!new_scan_) {
    std::chrono::duration<double> prev_scan_seconds = current_time - prev_scan_time_;
    if (prev_scan_seconds.count() > time_res && minDistance(Point(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z), scan_poses_) > distanceR_res) {
      scan_poses_.push_back(Point(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z));
      new_scan_ = true;
    }
  }

  prev_scan_time_ = std::chrono::system_clock::now();
}

/////////////////////////////////////////////////////
// Other methods: mostly are helper functions
/////////////////////////////////////////////////////

Eigen::Matrix4d LidarMapping::transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt) {

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

  matrix(0, 3) = xt;
  matrix(1, 3) = yt;
  matrix(2, 3) = zt;

  matrix(0, 0) = cos(yaw) * cos(pitch);
  matrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
  matrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
  matrix(1, 0) = sin(yaw) * cos(pitch);
  matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
  matrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
  matrix(2, 0) = -sin(pitch);
  matrix(2, 1) = cos(pitch) * sin(roll);
  matrix(2, 2) = cos(pitch) * cos(roll);

  return matrix;
}

// angle around z axis
Eigen::Quaternionf LidarMapping::getQuaternion(float theta) {
  Eigen::Matrix3f rotation_mat;
  rotation_mat << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  Eigen::Quaternionf q(rotation_mat);
  return q;
}

Pose LidarMapping::getPose(Eigen::Matrix4d matrix) {
  Pose pose(Point(matrix(0, 3), matrix(1, 3), matrix(2, 3)),
            Rotate(atan2(matrix(2, 1), matrix(2, 2)),
                   atan2(-matrix(2, 0), sqrt(matrix(2, 1) * matrix(2, 1) +
                                             matrix(2, 2) * matrix(2, 2))),
                   atan2(matrix(1, 0), matrix(0, 0))));
  return pose;
}

double LidarMapping::getDistance(Point p1, Point p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

double LidarMapping::minDistance(Point p1, std::vector<Point> points) {
  if (points.size() > 0) {
    double dist = getDistance(p1, points[0]);
    for (int index = 1; index < points.size(); index++) {
      double new_dist = getDistance(p1, points[index]);
      if (new_dist < dist) {
        dist = new_dist;
      }
    }
    return dist;
  }
  return -1;
}

void LidarMapping::print4x4Matrix(const Eigen::Matrix4d &matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1),
         matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1),
         matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1),
         matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3),
         matrix(2, 3));
}

void LidarMapping::print4x4Matrixf(const Eigen::Matrix4f &matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1),
         matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1),
         matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1),
         matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3),
         matrix(2, 3));
}

PointCloudT::Ptr LidarMapping::downsamplePointCloud(const PointCloudT& pcl_cloud) {
  PointCloudT::Ptr cloud(new PointCloudT);
  *cloud = pcl_cloud;
  cloud->width = cloud->points.size();
  cloud->height = 1;

  // Downsample the map point cloud using a voxel filter
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  double filterRes = 0.5;
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloud_filtered);

  return cloud_filtered;
}

void LidarMapping::savePointCloudMap(const PointCloudT::Ptr cloud_filtered) {
  // save the point cloud map
  pcl::io::savePCDFileASCII("town_map.pcd", *cloud_filtered);
  ROS_INFO("Saved pcd map.");
}
