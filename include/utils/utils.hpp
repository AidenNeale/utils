#pragma once

#include <filesystem>  // or #include <filesystem> for C++17 and up
#include <iomanip>     // for std::setw and std::setfill
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace utils {
struct UtilParams {
  // Default constructor with default values
  UtilParams(const int& mean_k = 9, const int& multi_thresh = 3, const double& outlier_radius = 25,
             const double& voxel_leaf_size = 2e-2, const std::string& save_directory = "/tmp/pcds",
             const double& mls_search_radius = 0.05, const int& mls_poly_order = 2,
             const std::vector<double>& cropbox_min = {-1.0, -1.0, -1.0},
             const std::vector<double>& cropbox_max = {1.0, 1.0, 1.0})
      : mean_k_(mean_k),
        multi_thresh_(multi_thresh),
        mls_poly_order_(mls_poly_order),
        outlier_radius_(outlier_radius),
        voxel_leaf_size_(voxel_leaf_size),
        mls_search_radius_(mls_search_radius),
        save_directory_(save_directory),
        cropbox_min_(cropbox_min),
        cropbox_max_(cropbox_max) {
  }

  int mean_k_, multi_thresh_, mls_poly_order_;
  double outlier_radius_, voxel_leaf_size_, mls_search_radius_;
  std::string save_directory_;
  std::vector<double> cropbox_min_, cropbox_max_;
};

class Utils {
 public:
  typedef std::shared_ptr<Utils> Ptr;

  Utils();

  Utils(const UtilParams& params);

  ~Utils() = default;

  void laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan,
                              sensor_msgs::msg::PointCloud2& pointcloud);
  sensor_msgs::msg::PointCloud2
  laserscanToPointcloud2(const sensor_msgs::msg::LaserScan& laserscan);

  void filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
                        const std::string& filter_name = "voxel");
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                        sensor_msgs::msg::PointCloud2& cloud_out,
                        const std::string& filter_name = "voxel");
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
                        const std::string& filter_name = "voxel");

  void savePointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);
  void savePointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const std::string& save_directory);
  void savePointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const std::string& save_directory, const std::string& name);
  void savePointcloud(const pcl::PointCloud<pcl::PointNormal>& cloud,
                      const std::string& save_directory, const std::string& name);

  void makeDirectory(const std::string& directory);

  void convertRos2Pcl(const sensor_msgs::msg::PointCloud2& cloud_in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
  void convertRos2Pcl(const sensor_msgs::msg::PointCloud2& cloud_in,
                      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out);

  void convertPcl2Ros(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                      sensor_msgs::msg::PointCloud2& cloud_out);

  geometry_msgs::msg::Quaternion multiplyQuaternions(const geometry_msgs::msg::Quaternion& q1,
                                                     const geometry_msgs::msg::Quaternion& q2);

  geometry_msgs::msg::TransformStamped eulerToQuaternions(const double xyz[3]);
  geometry_msgs::msg::TransformStamped eulerToQuaternions(const std::vector<double>& xyz);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> loadPCDs(const std::string& folder_path);

  std::vector<std::string> getPcdPath(const std::string& folder_path);

  void saveTrajectory(const std::vector<std::vector<double>>& curve, const std::string& filename);

 protected:
  UtilParams params_;

 private:
  void initialize();

  double getDistance(const pcl::PointXYZI& p);

  void statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
  void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
  void movingLeastSquares(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
  void distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
  void cropBoxFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);

  int i_, j_;

  laser_geometry::LaserProjection projector_;

  pcl::PCDWriter writer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr mls_cloud_;
};
}  // namespace utils
