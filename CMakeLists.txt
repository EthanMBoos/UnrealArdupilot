cmake_minimum_required(VERSION 3.28)
project(UnrealVehicleDynamics VERSION 0.1.0 LANGUAGES CXX)

# Resolve renamed-repository symlinks before embedding paths in the executable.
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" UVD_SOURCE_ROOT)

execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE UVD_GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git status --porcelain WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE UVD_GIT_STATUS ERROR_QUIET)
if(NOT UVD_GIT_COMMIT)
  set(UVD_GIT_COMMIT "unknown")
endif()
if(UVD_GIT_STATUS)
  set(UVD_GIT_DIRTY 1)
else()
  set(UVD_GIT_DIRTY 0)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

option(UVD_BUILD_TESTS "Build the headless core test suite" ON)
option(UVD_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)

set(FETCHCONTENT_QUIET OFF)
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
FetchContent_Declare(
  json_schema_validator
  URL https://github.com/pboettch/json-schema-validator/archive/refs/tags/2.4.0.tar.gz
  URL_HASH SHA256=24cbb114609cc9b43d4018b8d03e082ff5d2f26f5dce8bd36538097267b63af9
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_Declare(
  cli11
  URL https://github.com/CLIUtils/CLI11/archive/refs/tags/v2.6.2.tar.gz
  URL_HASH SHA256=c6ea6b2e5608b3ea8617999bd5f47420c71b2ebdb8dc4767c1034d1da5785711
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

set(JSON_VALIDATOR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(JSON_VALIDATOR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen nlohmann_json json_schema_validator cli11)

set(UVD_UNREAL_EIGEN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/unreal/Plugins/UnrealVehicleDynamics/ThirdParty/Eigen")
add_custom_target(uvd_stage_unreal_dependencies ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${UVD_UNREAL_EIGEN_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${eigen_SOURCE_DIR}/Eigen" "${UVD_UNREAL_EIGEN_DIR}/Eigen"
  COMMENT "Staging pinned Eigen headers for the Unreal plugin")

add_library(uvd_core STATIC
  core/src/fixed_wing.cpp
  core/src/rigid_body.cpp)
target_include_directories(uvd_core PUBLIC core/include)
target_compile_features(uvd_core PUBLIC cxx_std_23)
target_link_libraries(uvd_core PUBLIC Eigen3::Eigen)

add_executable(uvd
  app/analysis.cpp
  app/comparison.cpp
  app/configuration.cpp
  app/main.cpp
  app/simulation.cpp)
target_link_libraries(uvd PRIVATE
  uvd_core
  CLI11::CLI11
  nlohmann_json::nlohmann_json
  nlohmann_json_schema_validator)
target_compile_definitions(uvd PRIVATE UVD_SOURCE_DIR="${UVD_SOURCE_ROOT}"
  UVD_GIT_COMMIT="${UVD_GIT_COMMIT}" UVD_GIT_DIRTY=${UVD_GIT_DIRTY})

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  set(UVD_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_compile_options(uvd_core PRIVATE ${UVD_WARNINGS})
  target_compile_options(uvd PRIVATE ${UVD_WARNINGS})
  # Eigen 3.4.1 triggers libc++ C++23 deprecation and NEON signedness warnings.
  target_compile_options(uvd_core PRIVATE -Wno-deprecated-declarations -Wno-sign-conversion)
  target_compile_options(uvd PRIVATE -Wno-deprecated-declarations -Wno-sign-conversion)
  if(UVD_WARNINGS_AS_ERRORS)
    target_compile_options(uvd_core PRIVATE -Werror)
    target_compile_options(uvd PRIVATE -Werror)
  endif()
endif()

if(UVD_BUILD_TESTS)
  set(BUILD_TESTING ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.15.0.tar.gz
    URL_HASH SHA256=9650c55e497759cc39b977e45524bc8acb15256061c112080916ab6cb0b1ea66
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(catch2)
  include(CTest)
  include(Catch)
  add_executable(uvd_tests tests/test_core.cpp)
  target_link_libraries(uvd_tests PRIVATE uvd_core Catch2::Catch2WithMain)
  target_compile_definitions(uvd_tests PRIVATE UVD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(uvd_tests PRIVATE ${UVD_WARNINGS}
      -Wno-deprecated-declarations -Wno-sign-conversion)
    # Catch2 uses __COUNTER__, which Clang 22 diagnoses as a future C2y
    # extension under -Wpedantic even though it is supported by the compiler.
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND
       CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 22)
      target_compile_options(uvd_tests PRIVATE -Wno-c2y-extensions)
    endif()
    if(UVD_WARNINGS_AS_ERRORS)
      target_compile_options(uvd_tests PRIVATE -Werror)
    endif()
  endif()
  catch_discover_tests(uvd_tests)
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(NAME uvd_headless_acceptance
      COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/headless_acceptance.py
        --uvd $<TARGET_FILE:uvd>)
  endif()
  add_test(NAME uvd_validate_aircraft COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/examples/aircraft/aerosonde.json)
  add_test(NAME uvd_validate_run COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/headless.json)
  add_test(NAME uvd_validate_unreal_run COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/unreal_ardupilot.json)
  add_test(NAME uvd_validate_unreal_smoke COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/unreal_smoke.json)
  add_test(NAME uvd_validate_unreal_u0 COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/unreal_u0.json)
  add_test(NAME uvd_reject_headless_sitl COMMAND uvd sitl ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/headless.json --preflight)
  set_tests_properties(uvd_reject_headless_sitl PROPERTIES WILL_FAIL TRUE)
  add_test(NAME uvd_reject_smoke_sitl COMMAND uvd sitl ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/unreal_smoke.json --preflight)
  set_tests_properties(uvd_reject_smoke_sitl PROPERTIES WILL_FAIL TRUE)
  add_test(NAME uvd_reject_controller_unreal COMMAND uvd unreal ${CMAKE_CURRENT_SOURCE_DIR}/examples/runs/unreal_ardupilot.json --preflight)
  set_tests_properties(uvd_reject_controller_unreal PROPERTIES WILL_FAIL TRUE)
  add_test(NAME uvd_reject_duplicate_json COMMAND uvd validate ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/duplicate.json)
  set_tests_properties(uvd_reject_duplicate_json PROPERTIES WILL_FAIL TRUE)
endif()

install(TARGETS uvd uvd_core)
install(DIRECTORY core/include/ DESTINATION include)
install(DIRECTORY schemas/ DESTINATION share/uvd/schemas)
install(DIRECTORY examples/ DESTINATION share/uvd/examples)
