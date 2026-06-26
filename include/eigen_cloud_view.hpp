# pragma once

# include <Eigen/Dense>

# include "eigen_cloud.hpp"
# include "plane_model.hpp"

struct EigenCloudView {
    std::shared_ptr<const EigenCloud> cloud;
    std::vector<Eigen::Index> rows;

    EigenCloudView() = delete;
    EigenCloudView(std::shared_ptr<const EigenCloud> cloud_, std::vector<Eigen::Index> rows_);

    Eigen::Index size() const;

    Eigen::Vector3d xyz_at(Eigen::Index local_row) const;

    EigenCloud to_eigen_cloud() const;

    static EigenCloudView from_eigen_cloud(std::shared_ptr<const EigenCloud> cloud);

    Eigen::ArrayXd compute_distances_to(const PlaneModel& plane) const;

    EigenCloudView compute_inlier_view(const PlaneModel& plane, double threshold) const;

    EigenCloudView subtract(const EigenCloudView& other) const;

    PlaneModel fit_plane() const;
};
