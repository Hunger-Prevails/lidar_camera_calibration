# pragma once

# include <Eigen/Dense>

# include "eigen_cloud_view.hpp"
# include "plane_model.hpp"

struct PlaneCandidate {
    PlaneModel plane;

    EigenCloudView inliers;

    double rms_error = 0.0;

    Eigen::ArrayXd distances_to_plane(const EigenCloudView& view) const;
};
