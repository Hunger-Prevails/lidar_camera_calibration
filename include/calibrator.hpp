# include <vector>
# include <Eigen/Dense>
# include <filesystem>

namespace fs = std::filesystem;

class Calibrator {
public:
    Calibrator() = default;
    virtual ~Calibrator() = default;

    virtual void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) = 0;
};

class CheckerboardCalibrator : public Calibrator {
public:
    CheckerboardCalibrator() = default;
    ~CheckerboardCalibrator() override = default;

    void calibrate(std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs, Eigen::Matrix3d& intrinsics) override;
};
