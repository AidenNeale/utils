#pragma once

#include <string>
#include <filesystem>

#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <utils/ros2bag_reader.hpp>

namespace video_from_rosbag
{
    class VideoFromRosbag
    {
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
}
