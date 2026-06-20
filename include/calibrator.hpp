# include <vector>
# include <Eigen/Dense>
# include <filesystem>

namespace fs = std::filesystem;

class Calibrator {
public:
    Calibrator() = default;
    virtual ~Calibrator() = default;

    virtual void calibrate(std::vector<fs::path> image_paths, std::vector<fs::path> point_cloud_paths, Eigen::Matrix3d& intrinsics) = 0;
};

class DiamondCalibrator : public Calibrator {
public:
    DiamondCalibrator() = default;
    ~DiamondCalibrator() override = default;

    void calibrate(std::vector<fs::path> image_paths, std::vector<fs::path> point_cloud_paths, Eigen::Matrix3d& intrinsics) override;
};

class CylinderCalibrator : public Calibrator {
public:
    CylinderCalibrator() = default;
    ~CylinderCalibrator() override = default;

    void calibrate(std::vector<fs::path> image_paths, std::vector<fs::path> point_cloud_paths, Eigen::Matrix3d& intrinsics) override;
};
