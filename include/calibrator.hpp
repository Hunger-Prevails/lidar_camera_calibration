# pragma once

# include <vector>
# include <Eigen/Dense>
# include <filesystem>

# include <pcl/PCLPointCloud2.h>
# include <pcl/PCLPointField.h>
# include <pcl/io/pcd_io.h>

# include "eigen_cloud.hpp"
# include "eigen_cloud_view.hpp"
# include "plane_model.hpp"
# include "utils.hpp"

namespace fs = std::filesystem;

class Calibrator {
protected:
    fs::path write_path;

    static void append_as_column(std::vector<ScalarColumn>& columns, const pcl::PCLPointField& field);
    static std::vector<ScalarColumn> makeCanonicalColumns(const pcl::PCLPointCloud2& cloud);

    pcl::PCLPointCloud2 load_point_cloud(const fs::path& pcd_path);

    std::shared_ptr<const EigenCloud> to_eigen(const pcl::PCLPointCloud2& cloud);

public:
    Calibrator(fs::path write_path);
    virtual ~Calibrator() = default;

    virtual void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) = 0;
};

class CheckerboardCalibrator : public Calibrator {
protected:
    Eigen::Vector3d sphere_center;
    double sphere_radius;
    double threshold_inliers;
    double ransac_min_area;
    std::size_t max_planes;
    std::size_t min_inliers;
    std::size_t ransac_iterations;
    std::uint32_t random_seed;

    std::vector<PlaneCandidate> extract_plane_candidates(std::shared_ptr<const EigenCloud> cloud) const;

public:
    CheckerboardCalibrator(
        fs::path write_path_,
        Eigen::Vector3d sphere_center_,
        double sphere_radius_,
        double threshold_inliers_,
        double ransac_min_area_,
        std::size_t max_planes_,
        std::size_t min_inliers_,
        std::size_t ransac_iterations_,
        std::uint32_t random_seed_
    );
    ~CheckerboardCalibrator() override = default;

    void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) override;
};
