# include <Eigen/Dense>
# include <indicators/progress_bar.hpp>
# include <pcl/PCLPointCloud2.h>
# include <pcl/PCLPointField.h>
# include <pcl/io/pcd_io.h>

# include <algorithm>
# include <cmath>
# include <cstdint>
# include <cstring>
# include <filesystem>
# include <fstream>
# include <iostream>
# include <random>
# include <stdexcept>
# include <string>
# include <vector>
# include <unordered_set>
# include <opencv2/opencv.hpp>

# include "calibrator.hpp"

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using RandomState = std::mt19937;

void Calibrator::append_as_column(std::vector<ScalarColumn>& columns, const pcl::PCLPointField& field) {
    const std::uint32_t count = std::max<std::uint32_t>(field.count, 1);
    const std::size_t scalar_size = datatypeSize(field.datatype);

    for (std::uint32_t element = 0; element < count; ++element) {
        auto byte_offset = static_cast<std::size_t>(field.offset) + element * scalar_size;

        std::string column_name = field.name;

        if (count > 1) column_name += "[" + std::to_string(element) + "]";

        columns.push_back(ScalarColumn(column_name, byte_offset, field.datatype));
    }
}

std::vector<ScalarColumn> Calibrator::makeCanonicalColumns(const pcl::PCLPointCloud2& cloud) {
    std::vector<ScalarColumn> columns;

    auto coord_field_names = EigenCloud::get_coord_field_names();

    for (const auto& name : coord_field_names) Calibrator::append_as_column(columns, find_field(cloud, name));

    for (const auto& field : cloud.fields) {
        if (coord_field_names.count(field.name) > 0) continue;

        Calibrator::append_as_column(columns, field);
    }
    return columns;
}

pcl::PCLPointCloud2 Calibrator::load_point_cloud(const fs::path& pcd_path) {
    pcl::PCLPointCloud2 cloud;

    const int ret = pcl::io::loadPCDFile(pcd_path.string(), cloud);
    if (ret < 0) {
        throw std::runtime_error("Failed to load PCD file: " + pcd_path.string());
    }

    return cloud;
}

std::shared_ptr<const EigenCloud> Calibrator::to_eigen(const pcl::PCLPointCloud2& cloud) {
    const auto columns = Calibrator::makeCanonicalColumns(cloud);

    const std::size_t num_cols = columns.size();
    const std::size_t num_points = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);

    auto column_names = std::vector<std::string>();
    column_names.reserve(num_cols);

    for (const auto& column : columns) column_names.push_back(column.name);

    RowMatrixXd values(static_cast<Eigen::Index>(num_points), static_cast<Eigen::Index>(num_cols));

    const std::uint8_t* data = cloud.data.data();

    for (std::size_t row = 0; row < cloud.height; ++row) {
        const std::uint8_t* row_ptr = data + row * cloud.row_step;

        for (std::size_t col = 0; col < cloud.width; ++col) {
            const std::size_t source_index = row * cloud.width + col;

            const std::uint8_t* point_ptr = row_ptr + col * cloud.point_step;

            for (std::size_t j = 0; j < num_cols; ++j) {
                const auto& column = columns[j];

                values(
                    static_cast<Eigen::Index>(source_index),
                    static_cast<Eigen::Index>(j)
                ) = readColumn(point_ptr, column);
            }
        }
    }
    return std::make_shared<const EigenCloud>(std::move(values), std::move(column_names));
}

Calibrator::Calibrator(fs::path write_path) : write_path(std::move(write_path)) {
    if (!fs::exists(this->write_path)) {
        fs::create_directories(this->write_path);
    }
}

CheckerboardCalibrator::CheckerboardCalibrator(
    fs::path write_path_,
    Eigen::Vector3d sphere_center_,
    double sphere_radius_,
    double threshold_inliers_,
    double ransac_min_area_,
    std::size_t max_planes_,
    std::size_t min_inliers_,
    std::size_t ransac_iterations_,
    std::uint32_t random_seed_
) :
    Calibrator(std::move(write_path_)),
    sphere_center(sphere_center_),
    sphere_radius(sphere_radius_),
    threshold_inliers(threshold_inliers_),
    ransac_min_area(ransac_min_area_),
    max_planes(max_planes_),
    min_inliers(min_inliers_),
    ransac_iterations(ransac_iterations_),
    random_seed(random_seed_) {}

std::vector<PlaneCandidate> CheckerboardCalibrator::extract_plane_candidates(std::shared_ptr<const EigenCloud> cloud) const {
    if (!cloud) {
        throw std::runtime_error("extractPlaneCandidates: null input cloud");
    }

    EigenCloudView remainder_view = EigenCloudView::from_eigen_cloud(cloud);

    std::vector<PlaneCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(max_planes));

    RandomState random_state(random_seed);

    while (candidates.size() < max_planes) {
        std::uniform_int_distribution<Eigen::Index> dist(
            0,
            remainder_view.size() - 1
        );

        std::optional<PlaneCandidate> best_candidate;

        for (int iter = 0; iter < ransac_iterations; ++iter) {
            const Eigen::Index a = dist(random_state);
            const Eigen::Index b = dist(random_state);
            const Eigen::Index c = dist(random_state);

            if (a == b || a == c || b == c) {
                continue;
            }

            const Eigen::Vector3d p0 = remainder_view.xyz_at(a);
            const Eigen::Vector3d p1 = remainder_view.xyz_at(b);
            const Eigen::Vector3d p2 = remainder_view.xyz_at(c);

            if (!p0.allFinite() || !p1.allFinite() || !p2.allFinite()) {
                continue;
            }

            const auto candidate_plane = PlaneModel::fit_from_triplet(p0, p1, p2, ransac_min_area);

            if (!candidate_plane.has_value()) continue;

            auto inliers_view = remainder_view.compute_inlier_view(
                candidate_plane.value(), threshold_inliers
            );

            if (!best_candidate.has_value() || inliers_view.size() > best_candidate->inliers.size()) {
                best_candidate = PlaneCandidate{
                    .plane = std::move(candidate_plane.value()),
                    .inliers = std::move(inliers_view),
                };
            }
        }

        if (!best_candidate.has_value()) break;

        auto final_plane = best_candidate->inliers.fit_plane();

        auto final_inliers = remainder_view.compute_inlier_view(final_plane, threshold_inliers);

        if (final_inliers.size() < min_inliers) break;

        remainder_view = remainder_view.subtract(final_inliers);

        candidates.push_back(
            PlaneCandidate{
                .plane = std::move(final_plane),
                .inliers = std::move(final_inliers),
            }
        );
    }
    return candidates;
}

void CheckerboardCalibrator::calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) {
    indicators::ProgressBar bar{
        indicators::option::MaxProgress{image_cloud_pairs.size()},
        indicators::option::PrefixText{"=> Loads and crops point clouds:"},
        indicators::option::Start{"["},
        indicators::option::Fill{"="},
        indicators::option::Lead{">"},
        indicators::option::End{"]"},
        indicators::option::ShowPercentage{true},
    };
    for (const auto& [image_path, point_cloud_path] : image_cloud_pairs) {
        bar.tick();

        auto write_path = this->write_path / image_path.stem();

        if (!fs::exists(write_path)) fs::create_directories(write_path);

        auto point_cloud = load_point_cloud(point_cloud_path);

        auto eigen_cloud = to_eigen(point_cloud);

        eigen_cloud->export_to(write_path / (point_cloud_path.stem().string() + ".pcd"));

        auto cropped_cloud = std::make_shared<const EigenCloud>(eigen_cloud->sphere_crop(sphere_center, sphere_radius));

        cropped_cloud->summary();
        cropped_cloud->export_to(write_path / (point_cloud_path.stem().string() + "_cropped.pcd"));

        auto plane_candidates = extract_plane_candidates(cropped_cloud);

        std::cout << "=> extracts " << plane_candidates.size() << " plane candidates in " << image_path.stem().string() << std::endl;

        for (size_t i = 0; i < plane_candidates.size(); ++i) {
            const auto plane_cloud = plane_candidates[i].inliers.to_eigen_cloud();

            plane_cloud.export_to(write_path / (point_cloud_path.stem().string() + "_plane_" + std::to_string(i) + ".pcd"));
        }

        auto image = cv::imread(image_path.string(), cv::IMREAD_COLOR);

        cv::imwrite((write_path / image_path.filename()).string(), image);
    }
}
