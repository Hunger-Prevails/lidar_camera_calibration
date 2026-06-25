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
# include <stdexcept>
# include <string>
# include <vector>
# include <unordered_set>
# include <opencv2/opencv.hpp>

# include "calibrator.hpp"
# include "eigen_cloud.hpp"
# include "utils.hpp"

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

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

EigenCloud Calibrator::to_eigen(const pcl::PCLPointCloud2& cloud) {
    const auto columns = Calibrator::makeCanonicalColumns(cloud);

    const std::size_t num_cols = columns.size();
    const std::size_t num_points = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);

    EigenCloud result;

    result.column_names.reserve(num_cols);

    for (const auto& column : columns) result.column_names.push_back(column.name);

    result.values.resize(
        static_cast<Eigen::Index>(num_points),
        static_cast<Eigen::Index>(num_cols)
    );

    const std::uint8_t* data = cloud.data.data();

    for (std::size_t row = 0; row < cloud.height; ++row) {
        const std::uint8_t* row_ptr = data + row * cloud.row_step;

        for (std::size_t col = 0; col < cloud.width; ++col) {
            const std::size_t source_index = row * cloud.width + col;

            const std::uint8_t* point_ptr = row_ptr + col * cloud.point_step;

            for (std::size_t j = 0; j < num_cols; ++j) {
                const auto& column = columns[j];

                result.values(
                    static_cast<Eigen::Index>(source_index),
                    static_cast<Eigen::Index>(j)
                ) = readColumn(point_ptr, column);
            }
        }
    }
    return result;
}

Calibrator::Calibrator(fs::path write_path) : write_path(std::move(write_path)) {
    if (!fs::exists(this->write_path)) {
        fs::create_directories(this->write_path);
    }
}

CheckerboardCalibrator::CheckerboardCalibrator(
    fs::path write_path, Eigen::Vector3d sphere_center, double sphere_radius
) : Calibrator(std::move(write_path)), sphere_center(sphere_center), sphere_radius(sphere_radius) {}

void CheckerboardCalibrator::calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) {
    indicators::ProgressBar bar{
        indicators::option::MaxProgress{image_cloud_pairs.size()},
        indicators::option::PrefixText{"=> Loads and crops point clouds:"},
        indicators::option::Start{"["},
        indicators::option::Fill{"="},
        indicators::option::Lead{">"},
        indicators::option::End{"]"},
    };
    for (const auto& [image_path, point_cloud_path] : image_cloud_pairs) {
        bar.tick();

        auto write_path = this->write_path / image_path.stem();

        if (!fs::exists(write_path)) fs::create_directories(write_path);

        auto point_cloud = load_point_cloud(point_cloud_path);

        auto eigen_cloud = to_eigen(point_cloud);

        eigen_cloud.export_to(write_path / (point_cloud_path.stem().string() + ".pcd"));

        auto cropped_cloud = eigen_cloud.sphere_crop(sphere_center, sphere_radius);

        cropped_cloud.export_to(write_path / (point_cloud_path.stem().string() + "_cropped.pcd"));

        auto image = cv::imread(image_path.string(), cv::IMREAD_COLOR);

        cv::imwrite((write_path / image_path.filename()).string(), image);
    }
}
