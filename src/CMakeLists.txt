find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(indicators CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED COMPONENTS COMMON IO FILTERS)

add_library(
    calibrator
        calibrator.cpp
        eigen_cloud.cpp
        eigen_cloud_view.cpp
        plane_candidate.cpp
        plane_model.cpp
        utils.cpp
)
target_include_directories(calibrator PUBLIC ${CMAKE_SOURCE_DIR}/include)
if(MSVC)
    target_compile_options(calibrator PRIVATE /EHsc)
endif()
target_link_libraries(
    calibrator PRIVATE
        Ceres::ceres
        Eigen3::Eigen
        opencv_core
        opencv_features2d
        opencv_imgproc
        opencv_imgcodecs
        opencv_calib3d
        indicators::indicators
        pcl_COMMON
        pcl_IO
        pcl_FILTERS
)
