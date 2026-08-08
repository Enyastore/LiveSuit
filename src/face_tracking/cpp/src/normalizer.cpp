 #include "normalizer.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <filesystem>

namespace eye_tracker {

Normalizer::Normalizer(const std::string& refs_file) {
    refs_file_path_ = resolve_path(refs_file);
    load_refs();
}

//帮助函数：保存参考值到YAML文件
static void save_yaml_file(const std::string& filepath,
                           const std::unordered_map<std::string, std::vector<double>>& extreme_vectors,
                           const std::string& side, const std::string& ref_type, double ref_value) {
    try {
        YAML::Node config;
        try {
            config = YAML::LoadFile(filepath);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        // 写入现有的极值向量
        for (const auto& [key, vec] : extreme_vectors) {
            config[key] = YAML::Node(vec);
        }

        //更新开闭度参考值
        std::string ref_key = side + "_" + ref_type;
        config[ref_key] = ref_value;

        std::ofstream fout(filepath);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
    } catch (const std::exception& e) {
        std::cerr << "[归一化器] 保存YAML时出错: " << e.what() << std::endl;
    }
}

void Normalizer::set_openness_ref(const std::string& side, const std::string& ref_type, double value) {
    // 先更新内存缓存（与 set_extreme_vectors 更新 extreme_vectors_ 的行为一致）
    if (ref_type == "open") {
        cached_open_ref_[side] = value;
    } else if (ref_type == "close") {
        cached_close_ref_[side] = value;
    }

    save_yaml_file(refs_file_path_, extreme_vectors_, side, ref_type, value);
    std::cerr << "[归一化器] 保存了 " << side << "_" << ref_type << " = " << value << std::endl;
}

void Normalizer::clear_openness_ref(const std::string& side) {
    // 先清除内存缓存
    cached_open_ref_.erase(side);
    cached_close_ref_.erase(side);

    try {
        YAML::Node config;
        try {
            config = YAML::LoadFile(refs_file_path_);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        //清除 {side}_open 和 {side}_close
        std::vector<std::string> keys_to_remove;
        for (const auto& entry : config) {
            std::string k = entry.first.as<std::string>();
            if (k == side + "_open" || k == side + "_close") {
                keys_to_remove.push_back(k);
            }
        }
        for (const auto& k : keys_to_remove) {
            config.remove(k);
        }

        std::ofstream fout(refs_file_path_);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
        std::cerr << "[归一化器] 清除了 " << side << " 的开闭参考值" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[归一化器] 清除开闭参考值时出错: " << e.what() << std::endl;
    }
}

std::optional<double> Normalizer::get_openness_ref(const std::string& side, const std::string& ref_type) const {
    if (ref_type == "open") {
        auto it = cached_open_ref_.find(side);
        if (it != cached_open_ref_.end()) {
            return it->second;
        }
    } else if (ref_type == "close") {
        auto it = cached_close_ref_.find(side);
        if (it != cached_close_ref_.end()) {
            return it->second;
        }
    }
    return std::nullopt;
}

//处理路径绝对路径与相对路径
std::string Normalizer::resolve_path(const std::string& filename) const {
    //如果用户给的是相对路径，并且当前目录下能找到文件，就把相对路径变成绝对路径，方便后续使用；
    //如果找不到，就信任调用者传入的路径本身。
    if (filename.empty() || filename[0] == '/') {
        return filename;
    }

    if (std::filesystem::exists(filename)) {
        return std::filesystem::absolute(filename).string();
    }
    return filename;
}

void Normalizer::load_refs() {
    extreme_vectors_.clear();
    cached_open_ref_.clear();
    cached_close_ref_.clear();

    try {
        YAML::Node config = YAML::LoadFile(refs_file_path_);
        if (!config) {
            std::cerr << "[归一化器] 参考值文件为空或未找到" << std::endl;
            return;
        }
        for (const auto& entry : config) {
            std::string key = entry.first.as<std::string>();
            auto vec = entry.second;
            if (vec.IsSequence() && vec.size() >= 3) {
                extreme_vectors_[key] = {
                    vec[0].as<double>(),
                    vec[1].as<double>(),
                    vec[2].as<double>()
                };
                continue;
            }

            // 解析开度参考值标量键：{side}_open / {side}_close
            if (!vec.IsScalar()) continue;
            static const std::string open_suffix = "_open";
            static const std::string close_suffix = "_close";
            if (key.size() > open_suffix.size() &&
                key.compare(key.size() - open_suffix.size(), open_suffix.size(), open_suffix) == 0) {
                std::string side = key.substr(0, key.size() - open_suffix.size());
                if (!side.empty()) cached_open_ref_[side] = vec.as<double>();
            } else if (key.size() > close_suffix.size() &&
                       key.compare(key.size() - close_suffix.size(), close_suffix.size(), close_suffix) == 0) {
                std::string side = key.substr(0, key.size() - close_suffix.size());
                if (!side.empty()) cached_close_ref_[side] = vec.as<double>();
            }
        }
        std::cerr << "[归一化器] 加载了 " << extreme_vectors_.size() << " 个注视极值向量，"
                  << cached_open_ref_.size() << " 个开参考，"
                  << cached_close_ref_.size() << " 个闭参考" << std::endl;
    } catch (const YAML::BadFile&) {
        std::cerr << "[归一化器] 参考文件未找到：" << refs_file_path_ << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[归一化器] 加载参考值时出错：" << e.what() << std::endl;
    }
}

void Normalizer::reload() {
    load_refs();
}

void Normalizer::set_extreme_vectors(const std::string& side, const std::string& direction,
                                     const std::vector<double>& vector) {
    std::string key = side + "_" + direction;
    extreme_vectors_[key] = vector;

    try {
        YAML::Node config;
        // 加载当前文件（如果存在）
        try {
            config = YAML::LoadFile(refs_file_path_);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        config[key] = YAML::Node(vector);
        std::ofstream fout(refs_file_path_);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
    } catch (const std::exception& e) {
        std::cerr << "[归一化器] 保存注视极值向量时出错： " << e.what() << std::endl;
    }
}

void Normalizer::clear_extreme_vectors(const std::string& side) {
    // 移除四个注视极值向量
    std::vector<std::string> extreme_vector_directions = {"inner", "outer", "up", "down"};
    for (auto it = extreme_vectors_.begin(); it != extreme_vectors_.end(); ) {
        bool is_extreme_vector = false;
        for (const auto& dir : extreme_vector_directions) {
            if (it->first == side + "_" + dir) {
                is_extreme_vector = true;
                break;
            }
        }
        if (is_extreme_vector) {
            it = extreme_vectors_.erase(it);
        } else {
            ++it;
        }
    }

    try {
        YAML::Node config;
        try {
            config = YAML::LoadFile(refs_file_path_);
        } catch (...) {
            config = YAML::Node(YAML::NodeType::Map);
        }

        //移除四个注视极值向量对应的键
        std::vector<std::string> keys_to_remove;
        for (const auto& entry : config) {
            std::string k = entry.first.as<std::string>();
            for (const auto& dir : extreme_vector_directions) {
                if (k == side + "_" + dir) {
                    keys_to_remove.push_back(k);
                    break;
                }
            }
        }
        for (const auto& k : keys_to_remove) {
            config.remove(k);
        }

        std::ofstream fout(refs_file_path_);
        if (fout.is_open()) {
            fout << config;
            fout.close();
        }
        std::cerr << "清除了" << keys_to_remove.size()
                  << side << "眼的注视极值向量" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Normalizer] Error clearing extreme vectors: " << e.what() << std::endl;
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

    // 检查缺失的注视极值向量
    std::vector<std::string> expected = {inner_key, outer_key, up_key, down_key};
    for (const auto& k : expected) {
        if (extreme_vectors_.find(k) == extreme_vectors_.end()) {
            // 提取 "side_" 后面的方向部分
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
        extreme_vectors_.at(inner_key),
        extreme_vectors_.at(outer_key),
        extreme_vectors_.at(up_key),
        extreme_vectors_.at(down_key)
    );

    result.eye_x = eye_x;
    result.eye_y = eye_y;
    return result;
}


//========================
//由于眼睛的运动，up与down向量的差向量并不正交于left于right向量的差向量。
//算法的目的是基于准确的横轴（left-right）放置纵轴，并用up于down在其上的投影标定最高点/最低点。
//便于归一化输出eye_x、eye_y参数。
//========================
std::tuple<std::optional<double>, std::optional<double>>
Normalizer::orthogonal_project(const std::vector<double>& current,
                                const std::vector<double>& inner,
                                const std::vector<double>& outer,
                                const std::vector<double>& up,
                                const std::vector<double>& down) {
    // 计算四个极值向量的质心
    double cx = inner[0] + outer[0] + up[0] + down[0];
    double cy = inner[1] + outer[1] + up[1] + down[1];
    double cz = inner[2] + outer[2] + up[2] + down[2];
    double c_len = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (c_len < 1e-12) {
        return {std::nullopt, std::nullopt};
    }
    cx /= c_len; cy /= c_len; cz /= c_len;

    // 计算横轴
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

    // 计算纵轴 = 质心 × 横轴 (叉积)
    double ey_x = cy * ex_z - cz * ex_y;
    double ey_y = cz * ex_x - cx * ex_z;
    double ey_z = cx * ex_y - cy * ex_x;
    double ey_len = std::sqrt(ey_x * ey_x + ey_y * ey_y + ey_z * ey_z);
    if (ey_len < 1e-12) {
        return {std::nullopt, std::nullopt};
    }
    ey_x /= ey_len; ey_y /= ey_len; ey_z /= ey_len;

    // 点乘工具函数（注视向量模长已化为1，避免反复计算模长）
    auto dot = [](const std::vector<double>& a, double bx, double by, double bz) -> double {
        return a[0] * bx + a[1] * by + a[2] * bz;
    };

    double outer_x_proj = dot(outer, ex_x, ex_y, ex_z);
    double inner_x_proj = dot(inner, ex_x, ex_y, ex_z);
    double down_y_proj  = dot(down,  ey_x, ey_y, ey_z);
    double up_y_proj    = dot(up,    ey_x, ey_y, ey_z);
    double cur_x = dot(current, ex_x, ex_y, ex_z);
    double cur_y = dot(current, ey_x, ey_y, ey_z);

    // 钳制映射（将映射值限制在 [-1, 1] 范围内）
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