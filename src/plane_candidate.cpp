# include <Eigen/Dense>
# include <stdexcept>

# include "plane_candidate.hpp"

Eigen::ArrayXd PlaneCandidate::distances_to_plane(const EigenCloudView& view) const {
    if (!view.cloud) {
        throw std::runtime_error("null cloud pointer");
    }

    if (view.rows.empty()) {
        return Eigen::ArrayXd{};
    }

    const auto index_map = EigenCloud::get_index_map();

    const Eigen::Index x_col = index_map.at("x");
    const Eigen::Index y_col = index_map.at("y");
    const Eigen::Index z_col = index_map.at("z");

    const Eigen::Map<const Eigen::Array<Eigen::Index, Eigen::Dynamic, 1>>
        row_indices(
            view.rows.data(),
            static_cast<Eigen::Index>(view.rows.size())
        );

    const auto& values = view.cloud->values;

    Eigen::ArrayXd distances =
        values(row_indices, x_col).array() * plane.normal.x()
      + values(row_indices, y_col).array() * plane.normal.y()
      + values(row_indices, z_col).array() * plane.normal.z()
      - plane.rho;

    return distances;
}
