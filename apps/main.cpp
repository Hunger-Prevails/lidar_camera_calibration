# include <iostream>
# include <filesystem>
# include <fstream>
# include <stdexcept>
# include <memory>
# include <Eigen/Dense>
# include <nlohmann/json.hpp>
# include <cxxopts.hpp>

# include "calibrator.hpp"

namespace fs = std::filesystem;

using json = nlohmann::json;


std::vector<fs::path> get_files_in_directory(const fs::path& directory) {
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }
    return files;
}


Eigen::Matrix3d load_intrinsics(const fs::path& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::invalid_argument("Failed to open file: " + filepath.string());
    }

    json matrix_data;
    file >> matrix_data;
    file.close();

    if (!matrix_data.is_object()) {
        throw std::invalid_argument("Invalid JSON format: expected an object");
    }

    if (!matrix_data.contains("K") || !matrix_data["K"].is_array()) {
        throw std::invalid_argument("Invalid JSON format: expected 'K' to be an intrinsics array");
    }

    Eigen::Matrix3d matrix;

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            matrix(i, j) = matrix_data["K"][i * 3 + j].get<double>();
        }
    }
    return matrix;
}


enum class Mode {Diamond, Cylinder};

Mode parse_mode(const std::string& text) {
    if (text == "Diamond") {
        return Mode::Diamond;
    }
    if (text == "Cylinder") {
        return Mode::Cylinder;
    }

    throw std::invalid_argument("invalid mode: " + text);
}

int main(int argc, char *argv[]) {
    cxxopts::Options options("lidar-camera-calibration", "options to configure lidar-camera calibration pipeline");

    options.add_options()("mode", "Calibration mode", cxxopts::value<std::string>());
    options.add_options()("images", "Path to the image to process", cxxopts::value<fs::path>());
    options.add_options()("point-clouds", "Path to the point cloud to process", cxxopts::value<fs::path>());
    options.add_options()("intrinsics", "Path to the intrinsics file", cxxopts::value<fs::path>());
    options.add_options()("write-path", "Path to write outputs to", cxxopts::value<fs::path>()->default_value("outputs"));

    auto args = options.parse(argc, argv);

    auto mode = parse_mode(args["mode"].as<std::string>());

    std::unique_ptr<Calibrator> calibrator;

    if (mode == Mode::Diamond) {
        calibrator = std::make_unique<DiamondCalibrator>();
    } else {
        calibrator = std::make_unique<CylinderCalibrator>();
    }

    auto image_paths = get_files_in_directory(args["images"].as<fs::path>());
    auto point_cloud_paths = get_files_in_directory(args["point-clouds"].as<fs::path>());

    auto intrinsics = load_intrinsics(args["intrinsics"].as<fs::path>());

    calibrator->calibrate(image_paths, point_cloud_paths, intrinsics);
    return 0;
}
