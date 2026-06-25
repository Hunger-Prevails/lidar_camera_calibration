# include "plane_model.hpp"

double PlaneModel::signedDistance(const Eigen::Vector3d& p) const {
    return normal.dot(p) - rho;
}

double PlaneModel::absDistance(const Eigen::Vector3d& p) const {
    return std::abs(signedDistance(p));
}
