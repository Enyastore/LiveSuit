#include "normalizer.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <filesystem>

namespace eye_tracker {

Normalizer::Normalizer(const std::string& extreme_file) {
    extreme_file_path_ = resolve_path(extreme_file);
    load_extremes();
}

std::string Normalizer::resolve_path(const std::string& filename) const {
    // Try the current working directory first (for absolute paths or direct files)
    if (filename.empty() || filename[0] == '/') {
        return filename;
    }

    // Check if the file exists in the current directory
    if (std::filesystem::exists(filename)) {
        return std::filesystem::absolute(filename).string();
    }

    // Use __FILE__ based directory resolution at runtime - this is for when
    // the .so is loaded from the eye_tracker package directory
    // The Python bindings will set the path correctly at runtime
    return filename;
}

void Normalizer::load_extremes() {
    extremes_.clear();

    try {
        YAML::Node config = YAML::LoadFile(extreme_file_path_);
        if (!config) {
            std::cerr << "[Normalizer] Extreme file empty or not found" << std::endl;
            return;
        }
        for (const auto& entry : config) {
            std::string key = entry.first.as<std::string>();
            auto vec = entry.second;
            if (vec.IsSequence() && vec.size() >= 3) {
                extremes_[key] = {
                    vec[0].as<double>(),
                    vec[1].as<double>(),
                    vec[2].as<double>()
                };
            }
        }
        std::cerr << "[Normalizer] Loaded " << extremes_.size() << " extreme vectors" << std::endl;
    } catch (const YAML::BadFile&) {
        std::cerr << "[Normalizer] Extreme file not found: " << extreme_file_path_ << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Normalizer] Error loading extremes: " << e.what() << std::endl;
    }
}

void Normalizer::reload() {
    load_extremes();
}

void Normalizer::set_extreme(const std::string& side, const std::string& direction,
                             const std::vector<double>& vector) {
    std::string key = side + "_" + direction;
    extremes_[key] = vector;

    try {
        YAML::Node config;
        // Load existing file if present
        try {
            config = YAML::LoadFile(extreme_file_path_);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        config[key] = YAML::Node(vector);
        std::ofstream fout(extreme_file_path_);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
    } catch (const std::exception& e) {
        std::cerr << "[Normalizer] Error saving extreme: " << e.what() << std::endl;
    }
}

void Normalizer::clear_extremes(const std::string& side) {
    // Remove keys matching side_*
    for (auto it = extremes_.begin(); it != extremes_.end(); ) {
        if (it->first.rfind(side + "_", 0) == 0) {
            it = extremes_.erase(it);
        } else {
            ++it;
        }
    }

    try {
        YAML::Node config;
        try {
            config = YAML::LoadFile(extreme_file_path_);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        // Remove keys from YAML node
        std::vector<std::string> keys_to_remove;
        for (const auto& entry : config) {
            std::string k = entry.first.as<std::string>();
            if (k.rfind(side + "_", 0) == 0) {
                keys_to_remove.push_back(k);
            }
        }
        for (const auto& k : keys_to_remove) {
            config.remove(k);
        }

        std::ofstream fout(extreme_file_path_);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
        std::cerr << "[Normalizer] Cleared " << keys_to_remove.size()
                  << " extreme vectors for " << side << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Normalizer] Error clearing extremes: " << e.what() << std::endl;
    }
}

NormalizeResult Normalizer::normalize(const std::string& side,
                                      const std::vector<double>& gaze_rotated) const {
    NormalizeResult result;

    if (gaze_rotated.size() != 3) {
        return result;
    }

    std::string inner_key = side + "_inner";
    std::string outer_key = side + "_outer";
    std::string up_key = side + "_up";
    std::string down_key = side + "_down";

    // Check for missing extremes
    std::vector<std::string> expected = {inner_key, outer_key, up_key, down_key};
    for (const auto& k : expected) {
        if (extremes_.find(k) == extremes_.end()) {
            // Extract direction part after "side_"
            auto pos = k.find('_');
            if (pos != std::string::npos) {
                result.missing.push_back(k.substr(pos + 1));
            } else {
                result.missing.push_back(k);
            }
        }
    }

    if (!result.missing.empty()) {
        return result;
    }

    auto [eye_x, eye_y] = orthogonal_project(
        gaze_rotated,
        extremes_.at(inner_key),
        extremes_.at(outer_key),
        extremes_.at(up_key),
        extremes_.at(down_key)
    );

    result.eye_x = eye_x;
    result.eye_y = eye_y;
    return result;
}

std::tuple<std::optional<double>, std::optional<double>>
Normalizer::orthogonal_project(const std::vector<double>& current,
                                const std::vector<double>& inner,
                                const std::vector<double>& outer,
                                const std::vector<double>& up,
                                const std::vector<double>& down) {
    // Compute centroid of the four extreme points
    double cx = inner[0] + outer[0] + up[0] + down[0];
    double cy = inner[1] + outer[1] + up[1] + down[1];
    double cz = inner[2] + outer[2] + up[2] + down[2];
    double c_len = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (c_len < 1e-12) {
        return {std::nullopt, std::nullopt};
    }
    cx /= c_len; cy /= c_len; cz /= c_len;

    // Compute ex axis (inner - outer, orthogonalized to centroid)
    double raw_ex_x = inner[0] - outer[0];
    double raw_ex_y = inner[1] - outer[1];
    double raw_ex_z = inner[2] - outer[2];
    double d_ex = raw_ex_x * cx + raw_ex_y * cy + raw_ex_z * cz;
    double ex_x = raw_ex_x - d_ex * cx;
    double ex_y = raw_ex_y - d_ex * cy;
    double ex_z = raw_ex_z - d_ex * cz;
    double ex_len = std::sqrt(ex_x * ex_x + ex_y * ex_y + ex_z * ex_z);
    if (ex_len < 1e-12) {
        return {std::nullopt, std::nullopt};
    }
    ex_x /= ex_len; ex_y /= ex_len; ex_z /= ex_len;

    // Compute ey = centroid × ex (cross product)
    double ey_x = cy * ex_z - cz * ex_y;
    double ey_y = cz * ex_x - cx * ex_z;
    double ey_z = cx * ex_y - cy * ex_x;
    double ey_len = std::sqrt(ey_x * ey_x + ey_y * ey_y + ey_z * ey_z);
    if (ey_len < 1e-12) {
        return {std::nullopt, std::nullopt};
    }
    ey_x /= ey_len; ey_y /= ey_len; ey_z /= ey_len;

    // Dot product helper
    auto dot = [](const std::vector<double>& a, double bx, double by, double bz) -> double {
        return a[0] * bx + a[1] * by + a[2] * bz;
    };

    double outer_x_proj = dot(outer, ex_x, ex_y, ex_z);
    double inner_x_proj = dot(inner, ex_x, ex_y, ex_z);
    double down_y_proj  = dot(down,  ey_x, ey_y, ey_z);
    double up_y_proj    = dot(up,    ey_x, ey_y, ey_z);
    double cur_x = dot(current, ex_x, ex_y, ex_z);
    double cur_y = dot(current, ey_x, ey_y, ey_z);

    // Clamp-map function
    auto clamp_map = [](double val, double lo, double hi) -> std::optional<double> {
        if (hi - lo < 1e-12) {
            return 0.0;
        }
        val = std::max(lo, std::min(val, hi));
        return 2.0 * (val - lo) / (hi - lo) - 1.0;
    };

    return {
        clamp_map(cur_x, outer_x_proj, inner_x_proj),
        clamp_map(cur_y, down_y_proj, up_y_proj)
    };
}

} // namespace eye_tracker