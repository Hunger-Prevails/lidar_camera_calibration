# pragma once

# include <Eigen/Dense>

struct PlaneModel {
    Eigen::Vector3d normal;
    double rho = 0.0;

    double signedDistance(const Eigen::Vector3d& p) const;

    double absDistance(const Eigen::Vector3d& p) const;
};
