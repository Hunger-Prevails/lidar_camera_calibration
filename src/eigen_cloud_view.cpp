# include "eigen_cloud_view.hpp"
# include "utils.hpp"

EigenCloudView::EigenCloudView(
    std::shared_ptr<const EigenCloud> cloud_, std::vector<Eigen::Index> rows_
) : cloud(std::move(cloud_)), rows(std::move(rows_)) {}

Eigen::Index EigenCloudView::size() const {
    return static_cast<Eigen::Index>(rows.size());
}

Eigen::Vector3d EigenCloudView::xyz_at(Eigen::Index local_row) const {
    if (cloud == nullptr) {
        throw std::runtime_error("null cloud pointer");
    }

    const Eigen::Index global_row =
        rows[static_cast<std::size_t>(local_row)];

    return cloud->xyz_at(global_row);
}

EigenCloud EigenCloudView::to_eigen_cloud() const {
    if (cloud == nullptr) {
        throw std::runtime_error("null cloud pointer");
    }

    return cloud->select_rows(rows);
}

EigenCloudView EigenCloudView::from_eigen_cloud(std::shared_ptr<const EigenCloud> cloud) {
    if (!cloud) {
        throw std::runtime_error("input cloud is null");
    }

    std::vector<Eigen::Index> rows(static_cast<std::size_t>(cloud->values.rows()));

    std::iota(rows.begin(), rows.end(), Eigen::Index{0});

    return EigenCloudView(std::move(cloud), std::move(rows));
}

Eigen::ArrayXd EigenCloudView::compute_distances_to(const PlaneModel& plane) const {
    if (!cloud) {
        throw std::runtime_error("null cloud pointer");
    }

    if (rows.empty()) {
        return Eigen::ArrayXd{};
    }

    const auto index_map = EigenCloud::get_index_map();

    const Eigen::Index x_col = index_map.at("x");
    const Eigen::Index y_col = index_map.at("y");
    const Eigen::Index z_col = index_map.at("z");

    const Eigen::Map<const Eigen::Array<Eigen::Index, Eigen::Dynamic, 1>> row_indices(
        rows.data(), static_cast<Eigen::Index>(rows.size())
    );

    const auto& values = cloud->values;

    Eigen::ArrayXd distances =
        values(row_indices, x_col).array() * plane.normal.x()
      + values(row_indices, y_col).array() * plane.normal.y()
      + values(row_indices, z_col).array() * plane.normal.z()
      - plane.rho;

    return distances;
}

EigenCloudView EigenCloudView::compute_inlier_view(const PlaneModel& plane, double threshold) const {
    if (!cloud) {
        throw std::runtime_error("null cloud pointer");
    }

    const auto distances = compute_distances_to(plane);

    const Eigen::Array<bool, Eigen::Dynamic, 1> mask = distances.abs() <= threshold;

    auto to_keep = argwhere(mask);

    std::vector<Eigen::Index> result_rows(static_cast<std::size_t>(to_keep.size()));

    for (std::size_t i = 0; i < to_keep.size(); ++i) {
        result_rows[i] = rows[static_cast<std::size_t>(to_keep[i])];
    }

    return EigenCloudView(cloud, std::move(result_rows));
}

EigenCloudView EigenCloudView::subtract(const EigenCloudView& other) const {
    if (!cloud) {
        throw std::runtime_error("null cloud pointer");
    }

    if (cloud != other.cloud) {
        throw std::runtime_error("clouds must be the same for subtraction");
    }

    std::unordered_set<Eigen::Index> other_rows_set(other.rows.begin(), other.rows.end());

    size_t count_diff = 0;

    for (const auto& row : rows) if (other_rows_set.count(row) == 0) count_diff++;

    std::vector<Eigen::Index> result_rows;

    result_rows.reserve(count_diff);

    for (const auto& row : rows) if (other_rows_set.count(row) == 0) result_rows.push_back(row);

    return EigenCloudView(cloud, std::move(result_rows));
}

PlaneModel EigenCloudView::fit_plane() const {
    if (!cloud) {
        throw std::runtime_error("null cloud pointer");
    }

    const auto index_map = EigenCloud::get_index_map();

    const Eigen::Index x_col = index_map.at("x");
    const Eigen::Index y_col = index_map.at("y");
    const Eigen::Index z_col = index_map.at("z");

    const Eigen::Map<const Eigen::Array<Eigen::Index, Eigen::Dynamic, 1>> row_indices(
        rows.data(), static_cast<Eigen::Index>(rows.size())
    );

    RowMatrixXd points(rows.size(), 3);
    points.col(0) = cloud->values(row_indices, x_col);
    points.col(1) = cloud->values(row_indices, y_col);
    points.col(2) = cloud->values(row_indices, z_col);

    return PlaneModel::fit_from_points(points);
}
