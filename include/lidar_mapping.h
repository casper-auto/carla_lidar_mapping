// system
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <thread>

// eigen
#include <Eigen/Geometry>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// ros
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double pi = M_PI;


struct Point {
  double x, y, z;

  Point() : x(0), y(0), z(0) {}

  Point(double setX, double setY, double setZ) : x(setX), y(setY), z(setZ) {}

  void Print() {
    std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
  }
};

struct Rotate {
  double roll, pitch, yaw;

  Rotate() : roll(0), yaw(0), pitch(0) {}

  Rotate(double setRoll, double setPitch, double setYaw)
      : roll(setRoll), pitch(setPitch), yaw(setYaw) {}

  void Print() {
    std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
  }
};

struct Pose {

  Point position;
  Rotate rotation;

  Pose() : position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)) {}

  Pose(Point setPos, Rotate setRotation)
      : position(setPos), rotation(setRotation) {}

  Pose operator-(const Pose &p) {
    Pose result(Point(position.x - p.position.x, position.y - p.position.y,
                      position.z - p.position.z),
                Rotate(rotation.yaw - p.rotation.yaw,
                       rotation.pitch - p.rotation.pitch,
                       rotation.roll - p.rotation.roll));
    return result;
  }
};

class LidarMapping {
public:
  LidarMapping();
  ~LidarMapping();

private:
  // ros nodehandler
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros subscribers
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber localization_sub_;

  // ros publishers
  ros::Publisher point_cloud_map_pub_;

  // ros timer
  ros::Timer timer_;

  // vars
  bool new_scan_;
  double vehicle_radius_;
  int cloud_resolution_;
  // ros::Time prev_scan_time_;
  std::chrono::time_point<std::chrono::system_clock> prev_scan_time_;
  Pose current_pose_;
  std::vector<Point> scan_poses_;
  PointCloudT::Ptr scaned_cloud_ptr_;
  PointCloudT pcl_cloud_; // point cloud map before filtering
  sensor_msgs::PointCloud2 saved_map_;


  // callback functions
  void pointCloudCallback(const sensor_msgs::PointCloud2::Ptr msg);
  void localizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent &e);

  // core method
  void scanTransformation(const PointCloudT::Ptr scan_cloud);

  // methods
  Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt);
  Eigen::Quaternionf getQuaternion(float theta);
  Pose getPose(Eigen::Matrix4d matrix);
  double getDistance(Point p1, Point p2);
  double minDistance(Point p1, std::vector<Point> points);
  void print4x4Matrix(const Eigen::Matrix4d &matrix);
  void print4x4Matrixf(const Eigen::Matrix4f &matrix);
  PointCloudT::Ptr downsamplePointCloud(const PointCloudT& pcl_cloud);
  void savePointCloudMap(const PointCloudT::Ptr cloud_filtered);
};
