# include "utils.hpp"

namespace fs = std::filesystem;

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

const std::vector<std::pair<std::string_view, Eigen::Index>> EigenCloud::COORD_FIELDS{
    {"x", 0},
    {"y", 1},
    {"z", 2}
};

const std::unordered_set<std::string_view> EigenCloud::get_coord_field_names() {
    std::unordered_set<std::string_view> result;
    result.reserve(COORD_FIELDS.size());

    for (const auto& [name, _] : COORD_FIELDS) {
        result.insert(name);
    }

    return result;
}

const std::unordered_map<std::string_view, Eigen::Index> EigenCloud::get_index_map() {
    std::unordered_map<std::string_view, Eigen::Index> result;
    result.reserve(COORD_FIELDS.size());

    for (const auto& [name, index] : COORD_FIELDS) {
        result.emplace(name, index);
    }

    return result;
}

void EigenCloud::summary() const {
    std::cout << "Eigen point cloud:" << std::endl;
    std::cout << "=> rows: " << this->values.rows() << std::endl;
    std::cout << "=> cols: " << this->values.cols() << std::endl;
    std::cout << "=> columns:";

    for (const auto& name : this->column_names) {
        std::cout << " " << name;
    }

    std::cout << std::endl;

    if (this->values.rows() > 0) {
        std::cout << "=> first row: " << this->values.row(0) << std::endl;
    }
}

void EigenCloud::export_to(const fs::path& path) const {
    if (values.cols() != static_cast<Eigen::Index>(column_names.size())) {
        throw std::runtime_error(
            "values.cols() does not match column_names.size()"
        );
    }
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open output file");

    const Eigen::Index num_points = values.rows();
    const Eigen::Index num_fields = values.cols();

    out << "VERSION 0.7" << std::endl;

    out << "FIELDS";
    for (const auto& name : column_names) {
        out << ' ' << name;
    }
    out << std::endl;

    out << "SIZE";
    for (Eigen::Index j = 0; j < num_fields; ++j) {
        out << " 4";
    }
    out << std::endl;

    out << "TYPE";
    for (Eigen::Index j = 0; j < num_fields; ++j) {
        out << " F";
    }
    out << std::endl;

    out << "COUNT";
    for (Eigen::Index j = 0; j < num_fields; ++j) {
        out << " 1";
    }
    out << std::endl;

    out << "WIDTH " << num_points << std::endl;
    out << "HEIGHT 1" << std::endl;
    out << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    out << "POINTS " << num_points << std::endl;
    out << "DATA ascii" << std::endl;

    out << std::fixed << std::setprecision(6);

    for (Eigen::Index i = 0; i < num_points; ++i) {
        for (Eigen::Index j = 0; j < num_fields; ++j) {
            if (j > 0) {
                out << ' ';
            }
            out << static_cast<float>(values(i, j));
        }
        out << std::endl;
    }
}

EigenCloud EigenCloud::sphere_crop(
    const Eigen::Vector3d& center,
    double radius
) const {
    auto index_map = EigenCloud::get_index_map();

    const Eigen::Index x_col = index_map.at("x");
    const Eigen::Index y_col = index_map.at("y");
    const Eigen::Index z_col = index_map.at("z");

    const auto dx = values.col(x_col).array() - center.x();
    const auto dy = values.col(y_col).array() - center.y();
    const auto dz = values.col(z_col).array() - center.z();

    const Eigen::Array<bool, Eigen::Dynamic, 1> mask = (dx.square() + dy.square() + dz.square()) <= radius * radius;

    auto to_keep = argwhere(mask);

    EigenCloud result;

    result.column_names = column_names;
    result.values = values(to_keep, Eigen::placeholders::all);

    return result;
}

ScalarColumn::ScalarColumn(std::string name, std::size_t byte_offset, std::uint8_t datatype)
    : name(std::move(name)), byte_offset(byte_offset), datatype(datatype) {};

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
