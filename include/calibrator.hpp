# pragma once

# include <vector>
# include <Eigen/Dense>
# include <filesystem>

# include <pcl/PCLPointCloud2.h>
# include <pcl/PCLPointField.h>
# include <pcl/io/pcd_io.h>

# include "utils.hpp"

namespace fs = std::filesystem;

class Calibrator {
protected:
    fs::path write_path;

    static void append_as_column(std::vector<ScalarColumn>& columns, const pcl::PCLPointField& field);
    static std::vector<ScalarColumn> makeCanonicalColumns(const pcl::PCLPointCloud2& cloud);

    pcl::PCLPointCloud2 load_point_cloud(const fs::path& pcd_path);

    EigenCloud to_eigen(const pcl::PCLPointCloud2& cloud);

public:
    Calibrator(fs::path write_path);
    virtual ~Calibrator() = default;

    virtual void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) = 0;
};

class CheckerboardCalibrator : public Calibrator {
public:
    Eigen::Vector3d sphere_center;
    double sphere_radius;

public:
    CheckerboardCalibrator(fs::path write_path, Eigen::Vector3d sphere_center, double sphere_radius);
    ~CheckerboardCalibrator() override = default;

    void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) override;
};
