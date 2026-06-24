# pragma once

# include <unordered_map>
# include <unordered_set>
# include <Eigen/Dense>
# include <pcl/PCLPointCloud2.h>
# include <pcl/PCLPointField.h>
# include <pcl/io/pcd_io.h>
# include <stdexcept>
# include <cstdint>
# include <cstring>
# include <iostream>
# include <iomanip>
# include <fstream>
# include <filesystem>

namespace fs = std::filesystem;

Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> argwhere(const Eigen::Array<bool, Eigen::Dynamic, 1>& mask);

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct EigenCloud {
protected:
    static const std::vector<std::pair<std::string_view, Eigen::Index>> COORD_FIELDS;

public:
    RowMatrixXd values;
    std::vector<std::string> column_names;

    static const std::unordered_set<std::string_view> get_coord_field_names();
    static const std::unordered_map<std::string_view, Eigen::Index> get_index_map();

    void summary() const;
    void export_to(const fs::path& path) const;

    EigenCloud sphere_crop(
        const Eigen::Vector3d& center,
        double radius
    ) const;
};

struct ScalarColumn {
    std::string name;
    std::size_t byte_offset = 0;
    std::uint8_t datatype = 0;

    ScalarColumn(std::string name, std::size_t byte_offset, std::uint8_t datatype);
};

template <typename T>
T loadUnaligned(const std::uint8_t* ptr) {
    T value{};
    std::memcpy(&value, ptr, sizeof(T));
    return value;
}

double readScalar(const std::uint8_t* ptr, std::uint8_t datatype);

double readColumn(const std::uint8_t* ptr, const ScalarColumn& column);

std::size_t datatypeSize(std::uint8_t datatype);

const pcl::PCLPointField& find_field(
    const pcl::PCLPointCloud2& cloud,
    const std::string_view& name
);
