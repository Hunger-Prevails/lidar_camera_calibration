# pragma once

# include <Eigen/Dense>

# include "eigen_cloud_view.hpp"
# include "plane_model.hpp"

struct PlaneCandidate {
    PlaneModel plane;

    EigenCloudView inliers;

    double rms_error = 0.0;
};
