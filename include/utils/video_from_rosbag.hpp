#pragma once

#include "utils/ros2bag_reader.hpp"

#include <filesystem>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <yaml-cpp/yaml.h>

namespace video_from_rosbag {
class VideoFromRosbag {
 public:
  typedef std::shared_ptr<VideoFromRosbag> Ptr;
  VideoFromRosbag();
  ~VideoFromRosbag() = default;

 private:
  readbag::Ros2bagReader::Ptr ros2bag_reader_;
  cv_bridge::CvImagePtr cv_ptr_;

  YAML::Node config_;

  std::vector<std::string> bag_folders_;
  std::string save_directory_, bag_root_folder_, image_topic_, yaml_dir_;

  std::vector<sensor_msgs::msg::Image> ros_images_;
  int fps_;

  bool convertBagToVideo();
};
}  // namespace video_from_rosbag
