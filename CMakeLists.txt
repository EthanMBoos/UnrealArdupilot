cmake_minimum_required(VERSION 3.28)
project(UnrealVehicleDynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
  eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz
  URL_HASH SHA256=b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(eigen)

# UnrealBuildTool cannot consume CMake dependencies directly. Stage Eigen in
# the plugin so both builds compile the same three core sources.
set(UVD_UNREAL_EIGEN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/unreal/Plugins/UnrealVehicleDynamics/ThirdParty/Eigen")
add_custom_target(stage_unreal ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UVD_UNREAL_EIGEN_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${eigen_SOURCE_DIR}/Eigen" "${UVD_UNREAL_EIGEN_DIR}/Eigen")
