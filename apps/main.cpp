# include <iostream>
# include <filesystem>
# include <fstream>
# include <stdexcept>
# include <memory>
# include <Eigen/Dense>
# include <nlohmann/json.hpp>

namespace fs = std::filesystem;

using json = nlohmann::json;


Eigen::Matrix3d load_intrinsics(const fs::path& filepath, const std::string& camera) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::invalid_argument("Failed to open file: " + filepath.string());
    }

    json matrix_data;
    file >> matrix_data;
    file.close();

    Eigen::Matrix3d matrix;

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            matrix(i, j) = matrix_data[camera][i][j].get<double>();
        }
    }
    return matrix;
}

int main(int argc, char *argv[])
{
    return 0;
}
