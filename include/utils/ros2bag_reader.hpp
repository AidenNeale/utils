#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace readbag {
class Ros2bagReader {
 public:
  typedef std::shared_ptr<Ros2bagReader> Ptr;

  Ros2bagReader();

  ~Ros2bagReader() = default;

  template <typename T>
  std::vector<T> readMsg(const std::string& bag_folder, const std::string& topic_name,
                         const int& nth = 1);

  std::vector<std::string> getBagFiles(const std::string& root_folder);

 private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  rosbag2_storage::StorageOptions storage_options_;
};

template <typename T>
std::vector<T> Ros2bagReader::readMsg(const std::string& bag_folder, const std::string& topic_name,
                                      const int& nth) {
  /*
      bag_file: path to the ros2 bag folder
      topic_name: the specific topic name that is to be returned as a ros2 msg
      For example, this will read every nth msg under the topic_name in a ros2bag, stores in an
     array and returns it.
  */
  storage_options_.uri = bag_folder;
  storage_options_.storage_id = "sqlite3";
  reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options_);
  std::cout << "Trying to open bag at path: " << bag_folder << std::endl;
  reader_->open(storage_options_);
  std::cout << "Opened: " << bag_folder << std::endl;

  std::vector<T> msg_array;
  int message_count = 0;
  std::shared_ptr<T> ros_msg = std::make_shared<T>();
  rclcpp::Serialization<T> serialization;

  while (reader_->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

    if (msg->topic_name == topic_name) {
      if (message_count % nth == 0) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization.deserialize_message(&serialized_msg, ros_msg.get());

        msg_array.emplace_back(*ros_msg);
      }
      message_count++;
    }
  }
  std::cout << "\033[1;33m finished reading bag msg \033[0m\n";  // print out in yellow
  std::cout << "\033[0 \033[0m\n";                               // back to normal text
  return msg_array;
}
}  // namespace readbag