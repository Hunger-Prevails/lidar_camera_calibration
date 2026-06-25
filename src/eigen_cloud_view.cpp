# include "eigen_cloud_view.hpp"

Eigen::Index EigenCloudView::size() const {
    return static_cast<Eigen::Index>(rows.size());
}

Eigen::Vector3d EigenCloudView::xyzAt(Eigen::Index local_row) const {
    if (cloud == nullptr) {
        throw std::runtime_error("null cloud pointer");
    }

    const Eigen::Index global_row =
        rows[static_cast<std::size_t>(local_row)];

    return cloud->xyzAt(global_row);
}

EigenCloud EigenCloudView::toEigenCloud() const {
    if (cloud == nullptr) {
        throw std::runtime_error("null cloud pointer");
    }

    return cloud->select_rows(rows);
}

EigenCloudView EigenCloudView::fromEigenCloud(std::shared_ptr<const EigenCloud> cloud) {
    if (!cloud) {
        throw std::runtime_error("input cloud is null");
    }

    EigenCloudView view;
    view.cloud = std::move(cloud);
    view.rows.resize(static_cast<std::size_t>(view.cloud->values.rows()));

    std::iota(view.rows.begin(), view.rows.end(), Eigen::Index{0});

    return view;
}
