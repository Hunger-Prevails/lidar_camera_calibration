# include "utils.hpp"

namespace fs = std::filesystem;

ScalarColumn::ScalarColumn(std::string name, std::size_t byte_offset, std::uint8_t datatype)
    : name(std::move(name)), byte_offset(byte_offset), datatype(datatype) {};

Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> argwhere(const Eigen::Array<bool, Eigen::Dynamic, 1>& mask) {
    Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> indices(mask.count());

    Eigen::Index dst = 0;
    for (Eigen::Index i = 0; i < mask.size(); ++i) {
        if (mask(i)) {
            indices(dst++) = i;
        }
    }

    return indices;
}

double readScalar(const std::uint8_t* ptr, std::uint8_t datatype) {
    switch (datatype) {
        case pcl::PCLPointField::INT8:
            return static_cast<double>(loadUnaligned<std::int8_t>(ptr));

        case pcl::PCLPointField::UINT8:
            return static_cast<double>(loadUnaligned<std::uint8_t>(ptr));

        case pcl::PCLPointField::INT16:
            return static_cast<double>(loadUnaligned<std::int16_t>(ptr));

        case pcl::PCLPointField::UINT16:
            return static_cast<double>(loadUnaligned<std::uint16_t>(ptr));

        case pcl::PCLPointField::INT32:
            return static_cast<double>(loadUnaligned<std::int32_t>(ptr));

        case pcl::PCLPointField::UINT32:
            return static_cast<double>(loadUnaligned<std::uint32_t>(ptr));

        case pcl::PCLPointField::FLOAT32:
            return static_cast<double>(loadUnaligned<float>(ptr));

        case pcl::PCLPointField::FLOAT64:
            return loadUnaligned<double>(ptr);

        default:
            throw std::runtime_error(
                "Unsupported PCLPointField datatype: " +
                std::to_string(static_cast<int>(datatype)));
    }
}

double readColumn(const std::uint8_t* ptr, const ScalarColumn& column) {
    return readScalar(ptr + column.byte_offset, column.datatype);
}

std::size_t datatypeSize(std::uint8_t datatype) {
    switch (datatype) {
        case pcl::PCLPointField::INT8:
        case pcl::PCLPointField::UINT8:
            return 1;

        case pcl::PCLPointField::INT16:
        case pcl::PCLPointField::UINT16:
            return 2;

        case pcl::PCLPointField::INT32:
        case pcl::PCLPointField::UINT32:
        case pcl::PCLPointField::FLOAT32:
            return 4;

        case pcl::PCLPointField::FLOAT64:
            return 8;

        default:
            throw std::runtime_error(
                "Unsupported PCLPointField datatype: " +
                std::to_string(static_cast<int>(datatype)));
    }
}

const pcl::PCLPointField& find_field(
    const pcl::PCLPointCloud2& cloud,
    const std::string_view& name
) {
    for (const auto& field : cloud.fields) {
        if (field.name == name) {
            return field;
        }
    }
    throw std::runtime_error("Required field not found: " + std::string(name));
}
