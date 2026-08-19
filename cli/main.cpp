#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "config.hpp"

namespace fs = std::filesystem;
using uvd::cli::Json;

namespace {

void usage() {
  std::cerr << "usage:\n"
            << "  uvd simulate <run.json> [--duration seconds] [--dt seconds] "
               "[--output trajectory.csv]\n"
            << "  uvd evaluate <run.json> [--input sample.json|-]\n"
            << "  uvd trim <run.json> [--output result.json]\n"
            << "  uvd linearize <run.json> [--output result.json]\n";
}

std::string require_value(int& index, int argc, char** argv,
                          std::string_view option) {
  if (++index >= argc) {
    throw std::runtime_error(std::string(option) + " requires a value");
  }
  return argv[index];
}

std::optional<fs::path> parse_output(int start, int argc, char** argv) {
  std::optional<fs::path> output;
  for (int i = start; i < argc; ++i) {
    const std::string_view option = argv[i];
    if (option == "--output") {
      output = require_value(i, argc, argv, option);
    } else {
      throw std::runtime_error("unknown option: " + std::string(option));
    }
  }
  return output;
}

void emit(const Json& value, const std::optional<fs::path>& output) {
  if (output) {
    uvd::cli::write_json(*output, value);
  }
  std::cout << value.dump(2) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      usage();
      return 2;
    }
    const std::string command = argv[1];
    const uvd::cli::RunConfig run = uvd::cli::load_run(argv[2]);

    if (command == "simulate") {
      uvd::cli::SimulationOptions options;
      for (int i = 3; i < argc; ++i) {
        const std::string_view option = argv[i];
        if (option == "--duration") {
          options.duration_s = std::stod(require_value(i, argc, argv, option));
        } else if (option == "--dt") {
          options.dt_s = std::stod(require_value(i, argc, argv, option));
        } else if (option == "--output") {
          options.output = require_value(i, argc, argv, option);
        } else {
          throw std::runtime_error("unknown option: " + std::string(option));
        }
      }
      emit(uvd::cli::simulate(run, options), std::nullopt);
      return 0;
    }

    if (command == "evaluate") {
      std::optional<Json> input;
      for (int i = 3; i < argc; ++i) {
        const std::string_view option = argv[i];
        if (option != "--input") {
          throw std::runtime_error("unknown option: " + std::string(option));
        }
        const std::string path = require_value(i, argc, argv, option);
        if (path == "-") {
          Json value;
          std::cin >> value;
          input = std::move(value);
        } else {
          input = uvd::cli::load_json(path);
        }
      }
      emit(uvd::cli::evaluate(run, input), std::nullopt);
      return 0;
    }

    const auto output = parse_output(3, argc, argv);
    if (command == "trim") {
      emit(uvd::cli::trim(run), output);
      return 0;
    }
    if (command == "linearize") {
      emit(uvd::cli::linearize(run), output);
      return 0;
    }
    usage();
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "uvd: " << error.what() << '\n';
    return 1;
  }
}
