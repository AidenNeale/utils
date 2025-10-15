#include "utils/pointcloud_from_laserscans.hpp"

namespace fs = std::filesystem;

namespace pcd_from_laserscan {
PointcloudFromLaserscan::PointcloudFromLaserscan() {
  yaml_dir_ = ament_index_cpp::get_package_share_directory("utils") +
              "/config/pointcloud_from_laserscan.yaml";

  config_ = YAML::LoadFile(yaml_dir_);
  pcl_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  laserscan_topic_ = config_["scan_topic"].as<std::string>();
  save_directory_ = config_["save_directory"].as<std::string>();
  bag_root_folder_ = config_["bag_root_folder"].as<std::string>();

  ros2bag_reader_ = std::make_shared<readbag::Ros2bagReader>();
  bag_folders_ = ros2bag_reader_->getBagFiles(bag_root_folder_);

  pointcloudFromLaserscan();
}

void PointcloudFromLaserscan::pointcloudFromLaserscan() {
  for (const std::string& f : bag_folders_) {
    std::time_t sec;
    rclcpp::Time timestamp;
    std::filesystem::path path(f);
    params_.save_directory_ = save_directory_ + path.filename().string() + "/pcds/";
    utilities_ = std::make_shared<utils::Utils>(params_);
    laserscans_ = ros2bag_reader_->readMsg<sensor_msgs::msg::LaserScan>(f, laserscan_topic_);

    for (const auto& ls : laserscans_) {
      std::ostringstream oss;
      timestamp = ls.header.stamp;
      sec = static_cast<time_t>(timestamp.seconds());
      uint32_t nsec = timestamp.nanoseconds() % 1'000'000'000;

      oss << std::put_time(std::localtime(&sec), "%Y-%m-%d %H:%M:%S") << "." << std::setw(9)
          << std::setfill('0') << nsec;
      std::string datetime_str = oss.str();

      auto pc_msg = utilities_->laserscanToPointcloud2(ls);
      utilities_->convertRos2Pcl(pc_msg, pcl_cloud_);
      utilities_->savePointcloud(*pcl_cloud_, params_.save_directory_, datetime_str);
    }
  }
}
}  // namespace pcd_from_laserscan

int main(int, char**) {
  pcd_from_laserscan::PointcloudFromLaserscan::Ptr pcd_from_ls =
      std::make_shared<pcd_from_laserscan::PointcloudFromLaserscan>();
  std::cout << "Processing completed successfully." << std::endl;
  return 0;
}