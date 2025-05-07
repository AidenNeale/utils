#include <utils/ros2bag_reader.hpp>

namespace fs = std::filesystem;

namespace readbag
{
    Ros2bagReader::Ros2bagReader()
    {
    }

    std::vector<std::string> Ros2bagReader::getBagFiles(const std::string &root_folder)
    {
        /*
            Provided a root_folder, this will check inside all the folders inside the root_folder
            for files of extension ".db3" which is the default ros2 bag extension
         */
        std::vector<std::string> bag_folders;

        for (const auto &entry : fs::directory_iterator(root_folder))
        {
            if (fs::is_directory(entry.path()))
            {
                for (const auto &sub_entry : fs::directory_iterator(entry.path()))
                {
                    if (sub_entry.path().extension() == ".db3" || sub_entry.path().extension() == ".mcap")
                    {
                        bag_folders.push_back(entry.path().string());
                        break;
                    }
                }
            }
        }


        std::sort(bag_folders.begin(), bag_folders.end());
        return bag_folders;
    }
}
