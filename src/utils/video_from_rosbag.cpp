#include <utils/video_from_rosbag.hpp>

namespace fs = std::filesystem;

namespace video_from_rosbag
{
    /* 
        Logic:
        * Give a folder, say ros2_bags
        * ros2_bags contain several other folders, say bag_1, bag_2, bag_3....bag_n
        * Each of these bag_i folders contain a single metadata.yaml file and several .db3 
        or .mcap files produced as a result of splitting ros2 bags with the duration argument
        * This will look for the image_topic specified through the yaml file and start converting
          that to a video using opencv
    */
    VideoFromRosbag::VideoFromRosbag()
    {
        yaml_dir_ = ament_index_cpp::get_package_share_directory("utils") + "/config/video_from_rosbag.yaml";

        config_ = YAML::LoadFile(yaml_dir_);

        fps_ = config_["fps"].as<int>();
        image_topic_ = config_["image_topic"].as<std::string>();
        save_directory_ = config_["save_directory"].as<std::string>();
        bag_root_folder_ = config_["bag_root_folder"].as<std::string>();

        ros2bag_reader_ = std::make_shared<readbag::Ros2bagReader>();
        bag_folders_ = ros2bag_reader_->getBagFiles(bag_root_folder_);

        convertBagToVideo();
    }
    
    bool VideoFromRosbag::convertBagToVideo()
    {
        try
        {
            // VideoWriter setup
            cv::VideoWriter video_writer;

            // Loop through the bag files
            for (const std::string &f : bag_folders_)
            {
                bool video_initialized = false;
                std::filesystem::path path(f);
                std::string video_save_directory = save_directory_ + path.filename().string() + "/" + path.filename().string() +".mp4";

                ros_images_ = ros2bag_reader_->readMsg<sensor_msgs::msg::Image>(f, image_topic_);

                for (const auto &image_msg : ros_images_)
                {
                    // Convert to OpenCV Mat
                    cv::Mat cv_image;
                    try
                    {
                        cv_ptr_ = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
                        cv_image = cv_ptr_->image;

                        // std::cout << "Successfully converted image to OpenCV format\n";
                    }
                    catch (cv_bridge::Exception &e)
                    {
                        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
                    }

                    // Initialize VideoWriter on the first frame
                    if (!video_initialized)
                    {
                        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPG codec
                        cv::Size frame_size(cv_image.cols, cv_image.rows);
                        video_writer.open(video_save_directory, fourcc, fps_, frame_size, true);
                        if (!video_writer.isOpened())
                        {
                            std::cerr << "Exception: VideoWriter could not be opened.\n";
                            return false;
                        }
                        video_initialized = true;
                    }

                    // Write frame to video
                    video_writer.write(cv_image);
                }

                // Release VideoWriter
                if (video_initialized)
                {
                    video_writer.release();
                    std::cout << "Video saved to " << video_save_directory << "\n";
                }
                else
                {
                    std::cout << "No frames were written to the video.\n";
                    return false;
                }
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << "\n";
            return false;
        }
    }

}
int main(int argc, char **argv)
{
    video_from_rosbag::VideoFromRosbag::Ptr rosbag_to_video = std::make_shared<video_from_rosbag::VideoFromRosbag>();
    return 0;
}
