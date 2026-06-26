# include "plane_model.hpp"

double PlaneModel::signedDistance(const Eigen::Vector3d& p) const {
    return normal.dot(p) - rho;
}

double PlaneModel::absDistance(const Eigen::Vector3d& p) const {
    return std::abs(signedDistance(p));
}

PlaneModel PlaneModel::fit_from_points(const Eigen::MatrixXd& points) {
    if (points.rows() < 3) {
        throw std::runtime_error("At least 3 points are required to fit a plane.");
    }

    Eigen::Vector3d centroid = points.colwise().mean();

    Eigen::MatrixXd centered_points = points.rowwise() - centroid.transpose();

    Eigen::Matrix3d covariance = centered_points.transpose() * centered_points;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

    if (solver.info() != Eigen::Success) throw std::runtime_error("Eigen decomposition failed.");

    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();

    double rho = normal.dot(centroid);

    return PlaneModel{normal, rho};
}
