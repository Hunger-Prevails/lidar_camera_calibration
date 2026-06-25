# pragma once

# include <Eigen/Dense>

# include "eigen_cloud.hpp"
# include "plane_model.hpp"

struct EigenCloudView {
    std::shared_ptr<const EigenCloud> cloud;
    std::vector<Eigen::Index> rows;

    Eigen::Index size() const;

    Eigen::Vector3d xyzAt(Eigen::Index local_row) const;

    EigenCloud toEigenCloud() const;

    static EigenCloudView fromEigenCloud(std::shared_ptr<const EigenCloud> cloud);
};
