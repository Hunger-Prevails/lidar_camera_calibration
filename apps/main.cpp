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


std::vector<std::pair<fs::path, fs::path>> load_image_cloud_pairs(const fs::path& dataset_path, const fs::path& metadata_path) {
    std::ifstream file(metadata_path);
    if (!file.is_open()) {
        throw std::invalid_argument("Failed to open file: " + metadata_path.string());
    }

    json metadata;
    file >> metadata;
    file.close();

    std::vector<std::pair<fs::path, fs::path>> image_cloud_pairs;

    if (!metadata.contains("selected_pairs") || !metadata["selected_pairs"].is_array()) {
        throw std::invalid_argument("Invalid JSON format: expected 'selected_pairs' to be an array");
    }

    for (const auto& pair : metadata["selected_pairs"]) {
        if (!pair.contains("image") || !pair["image"].contains("file")) {
            throw std::invalid_argument("Invalid JSON format: each pair must contain field 'image'");
        }
        auto image_path = dataset_path / pair["image"]["file"].get<fs::path>();

        if (!fs::exists(image_path)) {
            throw std::invalid_argument("Image file does not exist: " + image_path.string());
        }

        if (!pair.contains("pointcloud") || !pair["pointcloud"].contains("file")) {
            throw std::invalid_argument("Invalid JSON format: each pair must contain field 'pointcloud'");
        }
        auto cloud_path = dataset_path / pair["pointcloud"]["file"].get<fs::path>();

        if (!fs::exists(cloud_path)) {
            throw std::invalid_argument("Point cloud file does not exist: " + cloud_path.string());
        }

        image_cloud_pairs.push_back(std::make_pair(image_path, cloud_path));
    }

    return image_cloud_pairs;
}

int main(int argc, char *argv[]) {
    cxxopts::Options options("lidar-camera-calibration", "options to configure lidar-camera calibration pipeline");

    options.add_options()
        ("dataset", "Path to the dataset directory", cxxopts::value<fs::path>())
        ("metadata", "Path to the metadata file", cxxopts::value<fs::path>())
        ("intrinsics", "Path to the intrinsics file", cxxopts::value<fs::path>())
        ("write-path", "Path to write outputs to", cxxopts::value<fs::path>()->default_value("outputs"))
        ("center", "Spherical crop center in LiDAR frame: x,y,z", cxxopts::value<std::vector<double>>()->default_value("0.0,0.0,0.0"))
        ("radius", "Spherical crop radius in meters", cxxopts::value<double>()->default_value("2.0"))
        ("help", "Print help");

    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    if (!args.count("dataset") || !args.count("metadata") || !args.count("intrinsics")) {
        std::cerr << "Error: --dataset, --metadata, and --intrinsics are required arguments." << std::endl;
        std::cout << options.help() << std::endl;
        return 1;
    }

    auto center = args["center"].as<std::vector<double>>();

    Eigen::Vector3d center_eigen(center[0], center[1], center[2]);

    auto calibrator = std::make_unique<CheckerboardCalibrator>(center_eigen, args["radius"].as<double>());

    auto dataset_path = args["dataset"].as<fs::path>();
    auto metadata_path = args["metadata"].as<fs::path>();

    auto image_cloud_pairs = load_image_cloud_pairs(dataset_path, metadata_path);

    auto intrinsics = load_intrinsics(args["intrinsics"].as<fs::path>());

    calibrator->calibrate(image_cloud_pairs, intrinsics);
    return 0;
}
