#include <utils/utils.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace utils
{

    Utils::Utils() : params_(UtilParams())
    {
        initialize();
    }

    Utils::Utils(const UtilParams &params) : params_(params)
    {
        initialize();
    }

    void Utils::initialize()
    {
        makeDirectory(params_.save_directory_);
        mls_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        j_ = 1;
        i_ = 1;
    }

    void Utils::makeDirectory(const std::string &directory)
    {
        if (!fs::is_directory(directory) || !fs::exists(directory))
        {                                    // Check if directory exists
            fs::create_directory(directory); // create directory
        }
    }

    void Utils::convertRos2Pcl(const sensor_msgs::msg::PointCloud2 &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        pcl::fromROSMsg(cloud_in, *cloud_out);
    }

    void Utils::convertRos2Pcl(const sensor_msgs::msg::PointCloud2 &cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_out)
    {
        pcl::fromROSMsg(cloud_in, *cloud_out);
    }

    void Utils::convertPcl2Ros(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, sensor_msgs::msg::PointCloud2 &cloud_out)
    {
        pcl::toROSMsg(*cloud_in, cloud_out);
    }

    void Utils::distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        auto condition = [this](const pcl::PointXYZI &p)
        {
            return getDistance(p) > params_.outlier_radius_;
        };
        cloud_in->erase(std::remove_if(cloud_in->begin(), cloud_in->end(), condition), cloud_in->end());

        cloud_out = cloud_in;
    }

    void Utils::movingLeastSquares(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        // Moving Least Squares: https://pointclouds.org/documentation/tutorials/resampling.html
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;

        mls.setInputCloud(cloud_in);
        mls.setPolynomialOrder(params_.mls_poly_order_);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::RANDOM_UNIFORM_DENSITY);
        mls.setPointDensity(15);
        mls.setSqrGaussParam(0.021025);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(params_.mls_search_radius_);
        mls.process(*mls_cloud_);
        *cloud_out = *mls_cloud_;
    }

    void Utils::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        // Downsample pointcloud: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(cloud_in);
        voxel_grid_filter.setLeafSize(params_.voxel_leaf_size_, params_.voxel_leaf_size_, params_.voxel_leaf_size_);
        voxel_grid_filter.filter(*cloud_out);
    }

    void Utils::cropBoxFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        pcl::CropBox<pcl::PointXYZI> cropbox_filter(true);
        cropbox_filter.setNegative(true);
        cropbox_filter.setInputCloud(cloud_in);
        Eigen::Vector4f min_pt(params_.cropbox_min_[0], params_.cropbox_min_[1], params_.cropbox_min_[2], 1.0);
        Eigen::Vector4f max_pt(params_.cropbox_max_[0], params_.cropbox_max_[1], params_.cropbox_max_[2], 1.0);
        cropbox_filter.setMin(min_pt);
        cropbox_filter.setMax(max_pt);
        cropbox_filter.filter(*cloud_out);
    }

    void Utils::statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        /*
        Statistical Outlier removal: https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html
        The resulting cloud_out contains all points of cloud_in that have an average distance to their 8 nearest neighbors that is below the computed threshold
        Using a standard deviation multiplier of 1.0 and assuming the average distances are normally distributed there is a 84.1% chance that a point will be an inlier
        */
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> stat_outlier_remover(true);
        stat_outlier_remover.setInputCloud(cloud_in);
        stat_outlier_remover.setMeanK(params_.meanK_);
        stat_outlier_remover.setStddevMulThresh(params_.multi_thresh_);
        stat_outlier_remover.filter(*cloud_out);
    }

    void Utils::filterPointCloud(const sensor_msgs::msg::PointCloud2 &cloud_in,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,
                                 const std::string &filter_name)
    {
        // Radius filter: remove points which are outside a sphere of radius `outlier_radius_`
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        convertRos2Pcl(cloud_in, pcl_pointcloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pcl_pointcloud, *pcl_pointcloud, indices);

        if (filter_name == "cropbox")
        {
            cropBoxFilter(pcl_pointcloud, cloud_out);
        }
        else if (filter_name == "voxel")
        {
            voxelGridFilter(pcl_pointcloud, cloud_out);
        }
        else if (filter_name == "sor")
        {
            statisticalOutlierRemoval(pcl_pointcloud, cloud_out);
        }
        else if (filter_name == "distance")
        {
            distanceFilter(pcl_pointcloud, cloud_out);
        }
        else if (filter_name == "mls")
        {
            movingLeastSquares(pcl_pointcloud, cloud_out);
        }
    }

    void Utils::filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
                                 sensor_msgs::msg::PointCloud2 &cloud_out_ros,
                                 const std::string &filter_name)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        filterPointCloud(cloud_in, cloud_out, filter_name);
        convertPcl2Ros(cloud_out, cloud_out_ros);
    }

    void Utils::filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,
                                 const std::string &filter_name)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

        if (filter_name == "cropbox")
        {
            cropBoxFilter(cloud_in, cloud_out);
        }
        else if (filter_name == "voxel")
        {
            voxelGridFilter(cloud_in, cloud_out);
        }
        else if (filter_name == "sor")
        {
            statisticalOutlierRemoval(cloud_in, cloud_out);
        }
        else if (filter_name == "distance")
        {
            distanceFilter(cloud_in, cloud_out);
        }
        else if (filter_name == "mls")
        {
            movingLeastSquares(cloud_in, cloud_out);
        }
    }

    double Utils::getDistance(const pcl::PointXYZI &p)
    {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    void Utils::savePointcloud(const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        std::ostringstream ss;
        ss << std::setw(5) << std::setfill('0') << i_;
        std::string save_name = params_.save_directory_ + "pcd_map_" + ss.str() + ".pcd";
        if (!cloud.empty())
        {
            writer_.writeBinary(save_name, cloud);
        }
        ++i_;
        // pcl::io::savePCDFileASCII (save_name, cloud);
    }

    void Utils::savePointcloud(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::string &save_directory)
    {
        makeDirectory(save_directory);
        std::ostringstream ss;
        ss << std::setw(5) << std::setfill('0') << j_;
        std::string save_name = save_directory + "reg_pcds_" + ss.str() + ".pcd";
        if (!cloud.empty())
        {
            writer_.writeBinary(save_name, cloud);
        }
        ++j_;
    }

    void Utils::savePointcloud(const pcl::PointCloud<pcl::PointXYZI> &cloud, const std::string &save_directory, const std::string &name)
    {
        makeDirectory(save_directory);
        std::string save_name = save_directory + name + ".pcd";
        if (!cloud.empty())
        {
            try
            {
                writer_.writeBinary(save_name, cloud);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    void Utils::savePointcloud(const pcl::PointCloud<pcl::PointNormal> &cloud, const std::string &save_directory, const std::string &name)
    {
        makeDirectory(save_directory);
        std::string save_name = save_directory + name + ".pcd";
        if (!cloud.empty())
        {
            writer_.writeBinary(save_name, cloud);
        }
    }

    void Utils::laserscanToPointcloud2(const sensor_msgs::msg::LaserScan &laserscan,
                                       sensor_msgs::msg::PointCloud2 &lasercloud)
    {
        projector_.projectLaser(laserscan, lasercloud); // convert laserscan to pointcloud
    }

    sensor_msgs::msg::PointCloud2 Utils::laserscanToPointcloud2(const sensor_msgs::msg::LaserScan &laserscan)
    {
        sensor_msgs::msg::PointCloud2 lasercloud;
        projector_.projectLaser(laserscan, lasercloud); // convert laserscan to pointcloud

        return lasercloud;
    }

    geometry_msgs::msg::Quaternion Utils::multiplyQuaternions(const geometry_msgs::msg::Quaternion &q1,
                                                              const geometry_msgs::msg::Quaternion &q2)
    {
        geometry_msgs::msg::Quaternion result;

        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

        return result;
    }

    geometry_msgs::msg::TransformStamped Utils::eulerToQuaternions(const double xyz[3])
    {
        tf2::Quaternion q;
        q.setRPY(xyz[0], xyz[1], xyz[2]);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        transformStamped.transform.translation.x = 0;

        return transformStamped;
    }

    geometry_msgs::msg::TransformStamped Utils::eulerToQuaternions(const std::vector<double> xyz)
    {
        double rot[3];
        for (int i = 0; i < xyz.size(); ++i)
        {
            rot[i] = xyz[i];
        }
        return eulerToQuaternions(rot);
    }
}
