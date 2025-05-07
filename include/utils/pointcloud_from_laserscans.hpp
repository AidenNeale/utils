#pragma once

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utils/utils.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <utils/ros2bag_reader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/point_cloud.h>
#include <sstream>
#include <chrono>
#include <iomanip>

namespace pcd_from_laserscan
{
    class PointcloudFromLaserscan
    {
    public:
        typedef std::shared_ptr<PointcloudFromLaserscan> Ptr;
        PointcloudFromLaserscan();
        ~PointcloudFromLaserscan() = default;

    private:
        readbag::Ros2bagReader::Ptr ros2bag_reader_;

        utils::UtilParams params_;
        utils::Utils::Ptr utilities_;

        YAML::Node config_;

        std::vector<std::string> bag_folders_;
        std::string save_directory_, bag_root_folder_, laserscan_topic_, yaml_dir_;

        std::vector<sensor_msgs::msg::LaserScan> laserscans_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_;

        bool pointcloudFromLaserscan();
    };
}

