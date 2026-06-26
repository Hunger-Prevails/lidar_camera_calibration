# pragma once

# include <Eigen/Dense>
# include <optional>

struct PlaneModel {
    Eigen::Vector3d normal;
    double rho = 0.0;

    double signedDistance(const Eigen::Vector3d& p) const;

    double absDistance(const Eigen::Vector3d& p) const;

    static PlaneModel fit_from_points(const Eigen::MatrixXd& points);

    static std::optional<PlaneModel> fit_from_triplet(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        double min_area
    );
};
