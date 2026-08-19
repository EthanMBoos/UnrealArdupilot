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
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(eigen nlohmann_json)

# UnrealBuildTool cannot consume CMake dependencies directly. Stage Eigen in
# the plugin so both builds compile the same shared model sources.
set(UVD_UNREAL_EIGEN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/unreal/Plugins/UnrealVehicleDynamics/ThirdParty/Eigen")
add_custom_target(stage_unreal ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UVD_UNREAL_EIGEN_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${eigen_SOURCE_DIR}/Eigen" "${UVD_UNREAL_EIGEN_DIR}/Eigen")

add_library(uvd_core STATIC
  core/src/fixed_wing.cpp
  core/src/geodesy.cpp
  core/src/integrator.cpp
  core/src/rigid_body.cpp)
target_include_directories(uvd_core PUBLIC core/include)
target_link_libraries(uvd_core PUBLIC Eigen3::Eigen)

add_executable(uvd
  cli/analysis.cpp
  cli/config.cpp
  cli/main.cpp
  cli/simulation.cpp)
target_link_libraries(uvd PRIVATE uvd_core nlohmann_json::nlohmann_json)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(uvd_core PRIVATE -Wall -Wextra -Wpedantic
    -Wno-deprecated-declarations)
  target_compile_options(uvd PRIVATE -Wall -Wextra -Wpedantic
    -Wno-deprecated-declarations)
endif()
