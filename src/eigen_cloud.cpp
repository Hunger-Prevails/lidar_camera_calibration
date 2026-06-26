# include "eigen_cloud.hpp"
# include "utils.hpp"

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

EigenCloud::EigenCloud(RowMatrixXd values_, std::vector<std::string> column_names_)
    : values(std::move(values_)), column_names(std::move(column_names_)) {}

void EigenCloud::summary() const {
    std::cout << std::endl;
    std::cout << "=> points: [" << this->values.rows() << " x " << this->values.cols() << "]" << std::endl;
    std::cout << "=> columns:";

    for (const auto& name : this->column_names) std::cout << " " << name;

    std::cout << std::endl;
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

Eigen::Vector3d EigenCloud::xyz_at(Eigen::Index row) const {
    auto index_map = EigenCloud::get_index_map();

    const Eigen::Index x_col = index_map.at("x");
    const Eigen::Index y_col = index_map.at("y");
    const Eigen::Index z_col = index_map.at("z");

    return Eigen::Vector3d{
        values(row, x_col),
        values(row, y_col),
        values(row, z_col)
    };
}

EigenCloud EigenCloud::select_rows(const std::vector<Eigen::Index>& rows) const {
    EigenCloud result;
    result.column_names = column_names;

    if (rows.empty()) {
        result.values.resize(0, values.cols());
        return result;
    }

    const Eigen::Map<const Eigen::Array<Eigen::Index, Eigen::Dynamic, 1>> row_indices(
        rows.data(), static_cast<Eigen::Index>(rows.size())
    );

    result.values = values(row_indices, Eigen::placeholders::all);

    return result;
}
