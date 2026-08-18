#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "app.hpp"

namespace fs = std::filesystem;

namespace uvd::app {
namespace {

struct CsvTable {
  std::vector<std::string> column_names;
  std::map<std::uint64_t, std::vector<double>> rows_by_tick;
};

std::string timestamp_id() {
  const auto value =
      std::chrono::system_clock::now().time_since_epoch().count();
  return std::to_string(value);
}

std::vector<std::string> split_csv_row(const std::string& text) {
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, ',')) {
    values.push_back(value);
  }
  return values;
}

CsvTable read_csv(const fs::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read CSV: " + path.string());
  }

  CsvTable table;
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("empty CSV");
  }
  table.column_names = split_csv_row(line);
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = split_csv_row(line);
    if (fields.size() != table.column_names.size()) {
      throw std::runtime_error("inconsistent CSV row");
    }
    std::vector<double> row;
    row.reserve(fields.size());
    for (const auto& field : fields) {
      row.push_back(std::stod(field));
    }
    table.rows_by_tick.emplace(static_cast<std::uint64_t>(row.front()),
                               std::move(row));
  }
  return table;
}

Json compare_tables(const CsvTable& left, const CsvTable& right) {
  if (left.column_names != right.column_names) {
    throw std::runtime_error("signal columns differ");
  }

  Json metrics = Json::object();
  std::vector<double> squared_error_sum(left.column_names.size());
  std::vector<double> maximum_error(left.column_names.size());
  std::size_t common_tick_count = 0;
  for (const auto& [tick, left_row] : left.rows_by_tick) {
    const auto right_row = right.rows_by_tick.find(tick);
    if (right_row == right.rows_by_tick.end()) {
      continue;
    }
    ++common_tick_count;
    for (std::size_t column = 1; column < left_row.size(); ++column) {
      const double difference = left_row[column] - right_row->second[column];
      squared_error_sum[column] += difference * difference;
      maximum_error[column] =
          std::max(maximum_error[column], std::abs(difference));
    }
  }
  if (common_tick_count == 0) {
    throw std::runtime_error("runs have no common ticks");
  }

  for (std::size_t column = 1; column < left.column_names.size(); ++column) {
    metrics[left.column_names[column]] = {
        {"rms", std::sqrt(squared_error_sum[column] /
                          static_cast<double>(common_tick_count))},
        {"max_abs", maximum_error[column]}};
  }
  return {{"schema_version", 1},
          {"common_ticks", common_tick_count},
          {"signals", metrics}};
}

void write_comparison_svg(const fs::path& path, const Json& comparison) {
  std::ostringstream svg;
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" "
         "height=\"500\"><rect width=\"100%\" height=\"100%\" "
         "fill=\"white\"/><text x=\"20\" y=\"30\" font-family=\"sans-serif\" "
         "font-size=\"20\">Run comparison: maximum absolute error</text>";
  int row = 0;
  for (const auto& [name, value] : comparison.at("signals").items()) {
    if (row >= 20) {
      break;
    }
    const double error = value.at("max_abs");
    const double width = std::min(500.0, 60.0 * std::log10(1.0 + error * 1e9));
    const int y = 55 + row * 21;
    svg << "<text x=\"20\" y=\"" << y
        << "\" font-family=\"monospace\" font-size=\"12\">" << name
        << "</text><rect x=\"300\" y=\"" << y - 12 << "\" width=\"" << width
        << "\" height=\"12\" fill=\"#3979b8\"/><text x=\"810\" y=\"" << y
        << "\" font-family=\"monospace\" font-size=\"11\">" << std::scientific
        << error << "</text>";
    ++row;
  }
  svg << "</svg>\n";
  write_text(path, svg.str());
}

void write_aligned_csv(const fs::path& path, const CsvTable& left,
                       const CsvTable& right) {
  std::ofstream aligned(path);
  if (!aligned) {
    throw std::runtime_error("cannot write aligned CSV: " + path.string());
  }
  aligned << "tick";
  for (std::size_t column = 1; column < left.column_names.size(); ++column) {
    aligned << ",left_" << left.column_names[column] << ",right_"
            << left.column_names[column] << ",delta_"
            << left.column_names[column];
  }
  aligned << '\n';
  aligned << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto& [tick, left_row] : left.rows_by_tick) {
    const auto right_row = right.rows_by_tick.find(tick);
    if (right_row == right.rows_by_tick.end()) {
      continue;
    }
    aligned << tick;
    for (std::size_t column = 1; column < left_row.size(); ++column) {
      aligned << ',' << left_row[column] << ',' << right_row->second[column]
              << ',' << left_row[column] - right_row->second[column];
    }
    aligned << '\n';
  }
  if (!aligned) {
    throw std::runtime_error("failed writing aligned CSV: " + path.string());
  }
}

}  // namespace

void run_compare(const fs::path& left_path, const fs::path& right_path,
                 const std::optional<fs::path>& output) {
  const fs::path left_signals =
      fs::is_directory(left_path) ? left_path / "signals.csv" : left_path;
  const fs::path right_signals =
      fs::is_directory(right_path) ? right_path / "signals.csv" : right_path;
  const CsvTable left = read_csv(left_signals);
  const CsvTable right = read_csv(right_signals);
  const Json comparison = compare_tables(left, right);
  const fs::path directory =
      output.value_or(fs::path("comparison_" + timestamp_id()));
  fs::create_directories(directory);

  write_aligned_csv(directory / "aligned.csv", left, right);
  write_text(directory / "metrics.json", comparison.dump(2) + "\n");
  write_comparison_svg(directory / "summary.svg", comparison);
  std::cout << directory << '\n';
}

void run_replay(const fs::path& bundle_path) {
  const fs::path bundle = fs::weakly_canonical(bundle_path);
  const RunConfig run = load_run(bundle / "resolved_config.json");
  const SimulationResult replay = simulate(run, bundle / "replay", true);
  const Json comparison =
      compare_tables(read_csv(bundle / "signals.csv"),
                     read_csv(replay.bundle / "signals.csv"));

  double maximum_error = 0.0;
  for (const auto& [name, value] : comparison.at("signals").items()) {
    static_cast<void>(name);
    maximum_error = std::max(maximum_error, value.at("max_abs").get<double>());
  }
  Json report = comparison;
  report["byte_identical"] = read_text(bundle / "signals.csv") ==
                             read_text(replay.bundle / "signals.csv");
  report["passed"] = maximum_error == 0.0;
  write_text(replay.bundle / "results/replay_comparison.json",
             report.dump(2) + "\n");
  std::cout << (replay.bundle / "results/replay_comparison.json") << '\n';
  if (maximum_error != 0.0) {
    throw std::runtime_error("replay differs from original run");
  }
}

}  // namespace uvd::app
