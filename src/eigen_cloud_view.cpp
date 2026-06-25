# include "eigen_cloud_view.hpp"

Eigen::Index EigenCloudView::size() const {
    return static_cast<Eigen::Index>(rows.size());
}

Eigen::Vector3d EigenCloudView::xyzAt(Eigen::Index local_row) const {
    if (cloud == nullptr) {
        throw std::runtime_error("EigenCloudView: null cloud pointer");
    }

    const Eigen::Index global_row =
        rows[static_cast<std::size_t>(local_row)];

    return cloud->xyzAt(global_row);
}

EigenCloud EigenCloudView::toEigenCloud() const {
    if (cloud == nullptr) {
        throw std::runtime_error("EigenCloudView: null cloud pointer");
    }

    return cloud->select_rows(rows);
}
