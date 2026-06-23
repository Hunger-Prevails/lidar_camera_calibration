# include <Eigen/Dense>

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

# include "calibrator.hpp"
# include "utils.hpp"

namespace fs = std::filesystem;

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

const std::unordered_set<std::string> Calibrator::core_fields{"x", "y", "z"};

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

    for (const auto& name : Calibrator::core_fields) {
        Calibrator::append_as_column(columns, find_field(cloud, name));
    }
    for (const auto& field : cloud.fields) {
        if (Calibrator::core_fields.count(field.name) > 0) continue;

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

    result.source_indices.resize(num_points);

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
                ) = readScalar(
                    point_ptr + column.byte_offset,
                    column.datatype
                );
            }
            result.source_indices.push_back(source_index);
        }
    }
    return result;
}

CheckerboardCalibrator::CheckerboardCalibrator(
    Eigen::Vector3d sphere_center, double sphere_radius) : sphere_center(sphere_center), sphere_radius(sphere_radius) {}

void CheckerboardCalibrator::calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) {
    for (const auto& [image_path, point_cloud_path] : image_cloud_pairs) {
        auto point_cloud = load_point_cloud(point_cloud_path);

        std::cout << "PCL point cloud:" << std::endl;
        std::cout << "=> width: " << point_cloud.width << std::endl;
        std::cout << "=> height: " << point_cloud.height << std::endl;
        std::cout << "=> point_step: " << point_cloud.point_step << std::endl;
        std::cout << "=> row_step: " << point_cloud.row_step << std::endl;
        std::cout << "=> fields: ";

        for (const auto& field : point_cloud.fields) {
            std::cout << " " << field.name;
        }
        std::cout << std::endl;

        const auto cropped = to_eigen(point_cloud);

        cropped.summary();
    }
}
