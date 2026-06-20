find_package(cxxopts CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG 3.12.0 REQUIRED)

add_executable(main main.cpp)
if(MSVC)
    target_compile_options(main PRIVATE /EHsc)
endif()
target_link_libraries(main PRIVATE calibrator cxxopts::cxxopts Eigen3::Eigen nlohmann_json::nlohmann_json)
