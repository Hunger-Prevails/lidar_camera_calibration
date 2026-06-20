# include "calibrator.hpp"
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

void DiamondCalibrator::calibrate(std::vector<fs::path> image_paths, std::vector<fs::path> point_cloud_paths, Eigen::Matrix3d& intrinsics) {
    std::cout << "To calibrate against diamond target..." << std::endl;
    return;
}

void CylinderCalibrator::calibrate(std::vector<fs::path> image_paths, std::vector<fs::path> point_cloud_paths, Eigen::Matrix3d& intrinsics) {
    std::cout << "To calibrate against cylinder target..." << std::endl;
    return;
}
