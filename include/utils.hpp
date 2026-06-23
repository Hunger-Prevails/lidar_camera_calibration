# pragma once

# include <Eigen/Dense>
# include <pcl/PCLPointCloud2.h>
# include <pcl/PCLPointField.h>
# include <pcl/io/pcd_io.h>
# include <stdexcept>
# include <cstdint>
# include <cstring>

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct EigenCloud {
    RowMatrixXd values;
    std::vector<std::string> column_names;

    void summary() const;
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
    const std::string& name
);
