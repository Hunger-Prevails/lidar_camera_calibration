# include "calibrator.hpp"
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

void CheckerboardCalibrator::calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) {
    std::cout << "To calibrate against checkerboard target..." << std::endl;
    return;
}
